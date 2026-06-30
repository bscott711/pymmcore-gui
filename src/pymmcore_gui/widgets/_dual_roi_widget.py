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
    QHBoxLayout,
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
    rect_to_pixel_roi,
)

if TYPE_CHECKING:
    from pymmcore_widgets.control._rois.roi_model import ROI

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

        # --- vispy canvas: y-flipped so world coords == sensor pixel coords --------
        self._canvas = scene.SceneCanvas(show=False)
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

        # --- toolbar (Select / Rectangle modes) -----------------------------------
        toolbar = QToolBar()
        self._rect_action = None
        for act in self.roi_manager.mode_actions.actions():
            if act.data() in ("select", "create-rect"):
                toolbar.addAction(act)
                if act.data() == "create-rect":
                    self._rect_action = act

        # --- buttons ----------------------------------------------------------------
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

        btn_row = QHBoxLayout()
        for b in (self._snap_btn, self._add_btn, self._clear_boxes_btn):
            btn_row.addWidget(b)
        btn_row.addStretch()
        for b in (self._reset_btn, self._apply_btn):
            btn_row.addWidget(b)

        self._info = QLabel()
        self._info.setWordWrap(True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(toolbar)
        layout.addWidget(self._canvas.native, 1)
        layout.addWidget(self._info)
        layout.addLayout(btn_row)

        # Start in rectangle mode so the user can immediately draw the first ROI.
        self.roi_manager.mode = "create-rect"
        self._snap()
        self._update_controls()

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
            # Select against a still, full-chip frame: stop any live/sequence
            # acquisition first (mirroring the Snap action), then drop any active ROI.
            # Snapping while a sequence is running leaves the snap buffer unread and
            # raises "Camera image buffer not read".
            if mmc.isSequenceRunning():
                mmc.stopSequenceAcquisition()
            if mmc.isMultiROIEnabled():
                clear_roi(mmc)
            img = mmc.snap()
        except Exception as exc:  # pragma: no cover - hardware/runtime errors
            if interactive:
                QMessageBox.warning(self, "Snap failed", str(exc))
            self._info.setText(f"Snap failed: {exc}.  Press Snap to retry.")
            return
        self._set_background(np.asarray(img))

    def _set_background(self, img: np.ndarray) -> None:
        if self._image is not None:
            self._image.parent = None
        self._image = Image(
            img,
            cmap="grays",
            clim="auto",
            parent=self._view.scene,
        )
        # Draw the image behind the ROI visuals.
        self._image.order = 10
        h, w = img.shape[-2], img.shape[-1]
        self._sensor_wh = (int(w), int(h))
        self._view.camera.set_range(x=(0, w), y=(0, h), margin=0)

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
        try:
            apply_dual_roi(self._mmc, rois)
        except (RuntimeError, ValueError) as exc:
            QMessageBox.warning(self, "Could not apply ROIs", str(exc))
            return
        # setMultiROI emits no roiSet; snap so live previews refresh to the composite.
        # Use snap() (not snapImage()) so the imageSnapped event fires and previews update.
        with suppress(Exception):
            self._mmc.snap()
        self._update_readout()

    def _reset_full_chip(self) -> None:
        with suppress(Exception):
            clear_roi(self._mmc)
        self._snap()

    # ------------------------------------------------------------------
    # UI state
    # ------------------------------------------------------------------

    def _supported(self) -> bool:
        if not self._mmc.getCameraDevice():
            return False
        try:
            return bool(self._mmc.isMultiROISupported())
        except Exception:  # pragma: no cover - camera w/o multi-ROI support
            return False

    def _update_controls(self) -> None:
        n = self.roi_manager.roi_model.rowCount()
        supported = self._supported()
        if self._rect_action is not None:
            self._rect_action.setEnabled(n == 0)
        self._add_btn.setEnabled(n == 1)
        self._apply_btn.setEnabled(supported and n == 2)
        self._reset_btn.setEnabled(supported)
        if not supported:
            self._info.setText("The current camera does not support multiple ROIs.")

    def _update_readout(self) -> None:
        if not self._supported():
            return
        rois = self._current_pixel_rois()
        if not rois:
            self._info.setText("Draw the first ROI, then add a matching second ROI.")
            return
        parts = [
            f"ROI {i + 1}: x={x} y={y} w={w} h={h}"
            for i, (x, y, w, h) in enumerate(rois)
        ]
        self._info.setText("   |   ".join(parts))
