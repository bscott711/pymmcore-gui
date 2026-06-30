"""Interactive dual-ROI selection widget for multi-ROI cameras (e.g. PVCAM).

The user snaps a full-chip image, draws the first ROI, then adds a second ROI that is
**locked to the first's size** and only repositioned.  Applying calls
``CMMCorePlus.setMultiROI`` so the camera reads out only those two regions.

The drawing surface reuses the interactive multi-rectangle machinery shipped in
``pymmcore_widgets.control._rois`` (``SceneROIManager`` + ``RectangleROI``).  The
vispy canvas is configured with a y-flipped pan/zoom camera so that world coordinates
equal sensor pixel coordinates (data row 0 at the top), which keeps the
rectangle-to-ROI mapping straightforward.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING, Any

import numpy as np
from pymmcore_plus import CMMCorePlus
from pymmcore_widgets.control._rois.roi_manager import SceneROIManager
from pymmcore_widgets.control._rois.roi_model import RectangleROI
from PyQt6.QtWidgets import (
    QLabel,
    QMessageBox,
    QPushButton,
    QToolBar,
    QVBoxLayout,
    QWidget,
)
from superqt import QIconifyIcon
from vispy import scene
from vispy.scene.visuals import Image

from pymmcore_gui._roi_utils import (
    PixelROI,
    apply_dual_roi,
    clear_roi,
    physical_camera_labels,
    read_snapped_frame,
    rect_to_pixel_roi,
)

if TYPE_CHECKING:
    from pymmcore_widgets.control._rois.roi_model import ROI
    from PyQt6.QtGui import QShowEvent

# ROIs whose width/height differ from the reference by less than this (in pixels)
# are considered equal — avoids fighting sub-pixel float drift during dragging.
_SIZE_EPS = 0.5


class DualRoiWidget(QWidget):
    """Draw two equal-size ROIs and apply them to the current camera as a multi-ROI.

    Parameters
    ----------
    parent : QWidget | None
        Optional parent widget.
    mmcore : CMMCorePlus | None
        Optional core instance; defaults to ``CMMCorePlus.instance()``.
    """

    def __init__(
        self, parent: QWidget | None = None, *, mmcore: CMMCorePlus | None = None
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Dual ROI")
        self._mmc = mmcore or CMMCorePlus.instance()

        # Full sensor size (w, h) captured from the last full-chip snap; used to clamp
        # ROIs inside the chip when applying.
        self._sensor_wh: tuple[int, int] | None = None
        # Locked ROI size (w, h) in pixels once the second ROI has been added.
        self._reference_size: tuple[float, float] | None = None
        # Guard against re-entrant model edits while normalizing ROI sizes.
        self._normalizing = False
        # vispy Image visual (untyped); None until the first snap.
        self._image: Any = None

        # Tracks whether the (deferred) first snap has run.
        self._snapped_once = False

        # --- vispy canvas: y-flipped so world coords == sensor pixel coords --------
        # show=True ensures the GL context is live before any visual is created.  With
        # show=False an Image visual built during __init__ never initializes its
        # texture and renders blank, which is why the first snap is also deferred to
        # showEvent (below) — by then the canvas is on screen and ready to draw.
        self._canvas = scene.SceneCanvas(show=True)
        self._view = self._canvas.central_widget.add_view()
        self._view.camera = scene.PanZoomCamera(aspect=1, flip=(False, True, False))

        self.roi_manager = SceneROIManager(self._canvas)
        # Tie lifetime to this widget so nothing leaks after close.
        self.roi_manager.setParent(self)
        # The ROI manager was built for stage-tiling ROIs and generates a grid plan
        # per ROI (which requires a field-of-view size).  Here ROIs are plain camera
        # pixel regions, so set a huge FOV: every ROI is then smaller than it and no
        # grid is generated.
        self._fov = (1.0e12, 1.0e12)
        self.roi_manager.update_fovs(self._fov)
        model = self.roi_manager.roi_model
        model.rowsInserted.connect(self._on_model_changed)
        model.rowsRemoved.connect(self._on_model_changed)
        model.dataChanged.connect(self._on_data_changed)
        # When the rectangle tool finishes a box it flips back to "select" mode; use
        # that as the cue to auto-add the size-locked second ROI (so the user doesn't
        # have to find a button).
        self.roi_manager.modeChanged.connect(self._on_mode_changed)

        # --- toolbar: mode actions + all controls, always visible at the top --------
        # (Putting the buttons here rather than in a bottom row guarantees they can't
        # be clipped off-screen when the dock is short.)
        toolbar = QToolBar()
        self._rect_action = None
        for act in self.roi_manager.mode_actions.actions():
            if act.data() in ("select", "create-rect"):
                toolbar.addAction(act)
                if act.data() == "create-rect":
                    self._rect_action = act
        toolbar.addSeparator()

        self._snap_btn = QPushButton(QIconifyIcon("mdi-light:camera"), "Snap")
        self._snap_btn.clicked.connect(self._on_snap_clicked)
        self._add_btn = QPushButton(QIconifyIcon("mdi:plus-box-outline"), "Add 2nd ROI")
        self._add_btn.clicked.connect(self._add_matching_roi)
        self._clear_boxes_btn = QPushButton("Clear boxes")
        self._clear_boxes_btn.clicked.connect(self.roi_manager.clear)
        self._apply_btn = QPushButton(QIconifyIcon("mdi:crop", color="green"), "Apply")
        self._apply_btn.clicked.connect(self._apply)
        self._reset_btn = QPushButton("Reset to full chip")
        self._reset_btn.clicked.connect(self._reset_full_chip)
        for b in (self._snap_btn, self._add_btn, self._clear_boxes_btn):
            toolbar.addWidget(b)
        toolbar.addSeparator()
        for b in (self._reset_btn, self._apply_btn):
            toolbar.addWidget(b)

        self._info = QLabel()
        self._info.setWordWrap(True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(toolbar)
        layout.addWidget(self._canvas.native, 1)
        layout.addWidget(self._info)

        # Start in rectangle mode so the user can immediately draw the first ROI.
        self.roi_manager.mode = "create-rect"
        self._update_controls()

    def showEvent(self, a0: QShowEvent | None) -> None:
        """Snap the first background frame once the canvas is on screen.

        Deferred from ``__init__`` so the GL context exists when the Image visual is
        created — otherwise the texture never initializes and the canvas stays blank.
        """
        super().showEvent(a0)
        if not self._snapped_once:
            self._snapped_once = True
            self._snap()

    def _on_mode_changed(self, mode: str) -> None:
        """Auto-add the matching second ROI right after the first is drawn."""
        if (
            mode == "select"
            and self.roi_manager.roi_model.rowCount() == 1
            and self._reference_size is None
        ):
            self._add_matching_roi()

    # ------------------------------------------------------------------
    # Camera image background
    # ------------------------------------------------------------------

    def _on_snap_clicked(self, _checked: bool = False) -> None:
        """Snap triggered by the user via the Snap button (errors shown in a dialog)."""
        self._snap(interactive=True)

    def _snap(self, *, interactive: bool = False) -> None:
        """Snap a full-chip frame and show it as the drawing background.

        Parameters
        ----------
        interactive : bool
            When True (the user clicked *Snap*), failures are reported in a dialog.
            During construction / programmatic refresh they are reported in the info
            label only, so opening the widget never raises a modal popup.
        """
        mmc = self._mmc
        if not mmc.getCameraDevice():
            self._update_controls()
            return
        try:
            # Snapping while a sequence is running leaves the snap buffer unread, so
            # stop any live/sequence acquisition first (mirroring the Snap action).
            if mmc.isSequenceRunning():
                mmc.stopSequenceAcquisition()
            img = self._grab_background()
        except Exception as exc:  # pragma: no cover - hardware/runtime errors
            if interactive:
                QMessageBox.warning(self, "Snap failed", str(exc))
            self._info.setText(f"Snap failed: {exc}.  Press Snap to retry.")
            return
        self._set_background(img)

    def _grab_background(self) -> np.ndarray:
        """Return a full-chip frame to draw on, via the app's normal snap path.

        Selection happens against the full sensor, so any active ROI is cleared first.
        For a composite (``Multi Camera``) device we use the **exact path the working
        Snap/Live preview uses**: ``snapImage()`` then read the frame back from the
        circular buffer via ``getLastImage()`` (:func:`read_snapped_frame`).  We do
        **not** use ``getImage(0)`` here — its raw per-channel buffer is empty on the
        composite, so it raises *"Camera image buffer read failed"*.  As a last
        resort (e.g. the demo composite, whose buffer can stay empty) we snap the
        first physical camera directly.
        """
        mmc = self._mmc
        if not self._is_multi_camera():
            if mmc.isMultiROIEnabled():
                clear_roi(mmc)
            return np.asarray(mmc.snap())

        labels = physical_camera_labels(mmc)
        clear_roi(mmc, cameras=labels)
        mmc.snapImage()
        try:
            return np.asarray(read_snapped_frame(mmc))
        except Exception:
            return self._snap_single(labels[0])

    def _snap_single(self, label: str) -> np.ndarray:
        """Snap one physical camera, restore the active device, return the image."""
        mmc = self._mmc
        original = mmc.getCameraDevice()
        try:
            mmc.setCameraDevice(label)
            return np.asarray(mmc.snap())
        finally:
            mmc.setCameraDevice(original)

    def _set_background(self, img: np.ndarray) -> None:
        # A composite snap can come back stacked as (C, H, W); show the first plane.
        if img.ndim == 3 and img.shape[-1] not in (3, 4):
            img = img[0]
        if self._image is not None:
            self._image.parent = None
        self._image = Image(
            img,
            cmap="grays",
            clim="auto",
            parent=self._view.scene,
        )
        # Draw the image *behind* the ROI visuals (lower order draws first).
        self._image.order = -1
        h, w = img.shape[-2], img.shape[-1]
        self._sensor_wh = (int(w), int(h))
        self._view.camera.set_range(x=(0, w), y=(0, h), margin=0)
        self._canvas.update()

    # ------------------------------------------------------------------
    # ROI model helpers
    # ------------------------------------------------------------------

    def _rois(self) -> list[ROI]:
        model = self.roi_manager.roi_model
        return [model.index(r).internalPointer() for r in range(model.rowCount())]

    def _add_matching_roi(self) -> None:
        """Add a second ROI identical in size to the first, offset below it."""
        rois = self._rois()
        if len(rois) != 1:
            return
        left, top, right, bottom = rois[0].bbox()
        w, h = right - left, bottom - top
        self._reference_size = (w, h)
        # Place the second ROI one ROI-height below, clamped on apply.
        gap = max(1.0, 0.1 * h)
        new_top = top + h + gap
        roi2 = RectangleROI(
            top_left=(left, new_top),
            bot_right=(left + w, new_top + h),
            fov_size=self._fov,
        )
        self.roi_manager.add_roi(roi2)
        self.roi_manager.mode = "select"
        self._update_controls()

    def _enforce_size(self, roi: ROI, w: float, h: float) -> None:
        """Force *roi* to be a ``w`` x ``h`` rectangle anchored at its top-left."""
        verts = np.asarray(roi.vertices)
        x0, y0 = verts.min(axis=0)
        target = np.array(
            [[x0, y0], [x0, y0 + h], [x0 + w, y0 + h], [x0 + w, y0]], dtype=np.float32
        )
        cur_w = float(verts[:, 0].max() - verts[:, 0].min())
        cur_h = float(verts[:, 1].max() - verts[:, 1].min())
        if abs(cur_w - w) > _SIZE_EPS or abs(cur_h - h) > _SIZE_EPS:
            roi.vertices = target
            model = self.roi_manager.roi_model
            idx = model.index_of(roi)
            model.dataChanged.emit(idx, idx, [model.VERTEX_ROLE])

    # ------------------------------------------------------------------
    # Model signal handlers
    # ------------------------------------------------------------------

    def _on_model_changed(self, *args: object) -> None:
        if self.roi_manager.roi_model.rowCount() < 2:
            self._reference_size = None
        self._update_controls()
        self._update_readout()

    def _on_data_changed(self, *args: object) -> None:
        # Keep both ROIs locked to the reference size (second ROI is move-only).
        if (
            not self._normalizing
            and self._reference_size is not None
            and self.roi_manager.roi_model.rowCount() >= 2
        ):
            self._normalizing = True
            try:
                for roi in self._rois():
                    self._enforce_size(roi, *self._reference_size)
            finally:
                self._normalizing = False
        self._update_readout()

    # ------------------------------------------------------------------
    # Apply / reset
    # ------------------------------------------------------------------

    def _current_pixel_rois(self) -> list[PixelROI]:
        max_w, max_h = self._sensor_wh or (None, None)
        rois: list[PixelROI] = []
        for roi in self._rois():
            left, top, right, bottom = roi.bbox()
            rois.append(
                rect_to_pixel_roi(
                    (left, top), (right, bottom), max_w=max_w, max_h=max_h
                )
            )
        return rois

    def _apply(self) -> None:
        rois = self._current_pixel_rois()
        if len(rois) != 2:
            return
        mmc = self._mmc
        # For a composite device, fan the same ROIs out to every physical camera so
        # each Kinetix reads out identically (equal sizes keep the composite valid).
        cams = physical_camera_labels(mmc) if self._is_multi_camera() else None
        try:
            if mmc.isSequenceRunning():
                mmc.stopSequenceAcquisition()
            apply_dual_roi(mmc, rois, cameras=cams)
        except (RuntimeError, ValueError) as exc:
            # On partial failure, restore a consistent full-chip state so the
            # composite device doesn't end up with mismatched per-camera sizes.
            if cams:
                with suppress(Exception):
                    clear_roi(mmc, cameras=cams)
            QMessageBox.warning(self, "Could not apply ROIs", str(exc))
            return
        # setMultiROI emits no roiSet; refresh the live preview by snapping. For a
        # single camera snap() emits imageSnapped and shows the new composite; on a
        # composite device snap()/getImage() return NULL, so leave the refresh to the
        # user's normal Snap/Live in the main window.
        if cams is None:
            with suppress(Exception):
                mmc.snap()
        self._update_readout()

    def _reset_full_chip(self) -> None:
        cams = physical_camera_labels(self._mmc) if self._is_multi_camera() else None
        with suppress(Exception):
            clear_roi(self._mmc, cameras=cams)
        self._snap()

    # ------------------------------------------------------------------
    # UI state
    # ------------------------------------------------------------------

    def _has_camera(self) -> bool:
        return bool(self._mmc.getCameraDevice())

    def _is_multi_camera(self) -> bool:
        """True if the active camera is a composite (e.g. ``Multi Camera``) device.

        ``setMultiROI`` acts on the current camera only; a composite device that
        fans out to several physical cameras cannot itself take a multi-ROI, so this
        is surfaced to the user as actionable guidance rather than silently failing.
        """
        try:
            return self._mmc.getNumberOfCameraChannels() > 1
        except Exception:  # pragma: no cover - defensive
            return False

    def _reports_multiroi(self) -> bool:
        """Advisory only — the camera's self-reported multi-ROI capability flag."""
        try:
            return bool(self._mmc.isMultiROISupported())
        except Exception:  # pragma: no cover - defensive
            return False

    def _update_controls(self) -> None:
        n = self.roi_manager.roi_model.rowCount()
        has_cam = self._has_camera()
        if self._rect_action is not None:
            self._rect_action.setEnabled(has_cam and n == 0)
        self._add_btn.setEnabled(has_cam and n == 1)
        # Apply is gated only on having a camera and two ROIs — NOT on the unreliable
        # isMultiROISupported() flag.  The camera itself reports any real rejection.
        self._apply_btn.setEnabled(has_cam and n == 2)
        self._reset_btn.setEnabled(has_cam)
        self._update_readout()

    def _status_prefix(self) -> str:
        """Leading status/guidance line shown above the ROI readout."""
        if not self._has_camera():
            return "No camera loaded."
        cam = self._mmc.getCameraDevice()
        if self._is_multi_camera():
            cams = ", ".join(physical_camera_labels(self._mmc))
            return (
                f"Active camera {cam!r} is a composite device — Apply will set both "
                f"ROIs on each physical camera ({cams})."
            )
        if not self._reports_multiroi():
            return (
                f"Camera {cam!r} does not advertise multi-ROI support; Apply will "
                "attempt it anyway (this flag is often a false negative)."
            )
        return f"Camera {cam!r} — ready."

    def _update_readout(self) -> None:
        prefix = self._status_prefix()
        rois = self._current_pixel_rois() if self._has_camera() else []
        if not rois:
            hint = "Draw the first ROI, then add a matching second ROI."
            self._info.setText(f"{prefix}\n{hint}")
            return
        parts = [
            f"ROI {i + 1}: x={x} y={y} w={w} h={h}"
            for i, (x, y, w, h) in enumerate(rois)
        ]
        self._info.setText(f"{prefix}\n" + "   |   ".join(parts))
