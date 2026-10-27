"""Draw the image-focus-lock box on an MDA viewer.

The lock watches one box on one (camera, laser preset, slice), stored in
``SettingsV1.image_focus.box`` in **full-sensor pixels** (x = column, y = row).
An MDA viewer shows the camera's hardware ROI starting at array index 0, and its
ROI overlay (``ndv.ArrayViewer.roi``) is in array indices, so the two differ by
the camera's ROI origin. :func:`bbox_to_focus_box` and :func:`focus_box_to_bbox`
convert between them (pure functions); :class:`ViewerFocusBox` wires a
checkable "Focus box" button, the ROI overlay and the settings together for one
viewer.

The ROI is ndv's own: a rectangle the user drags and resizes in the canvas. It
changes the model on every mouse move, so the new box is only written to the
settings once it has been still for :data:`_COMMIT_DELAY_MS`.
"""

from __future__ import annotations

import logging
from contextlib import suppress
from typing import TYPE_CHECKING

from ndv.models import RectangularROIModel

from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._qt.QtCore import QObject, QTimer
from pymmcore_gui._settings import FocusBoxSettings, SettingsV1
from pymmcore_gui._spectral_channel_handler import camera_roi
from pymmcore_gui.asi_z_stack.image_focus_events import focus_lock_signals

if TYPE_CHECKING:
    import ndv
    import useq
    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._qt.QtWidgets import QAbstractButton

logger = logging.getLogger(__name__)

#: A bounding box as ndv stores it: ``((x1, y1), (x2, y2))`` in array indices.
BBox = tuple[tuple[float, float], tuple[float, float]]

#: How long the ROI must sit still before the box is written to the settings.
_COMMIT_DELAY_MS = 500


def focus_box_to_bbox(box: FocusBoxSettings, origin: tuple[int, int] = (0, 0)) -> BBox:
    """The viewer ROI (array indices) for *box* on a frame starting at *origin*.

    *origin* is the camera's hardware-ROI ``(x, y)`` on the sensor, subtracted
    because the viewer's array starts there.
    """
    x, y, w, h = box.rect
    ox, oy = origin
    return ((x - ox, y - oy), (x - ox + w, y - oy + h))


def bbox_to_focus_box(
    bbox: BBox,
    origin: tuple[int, int],
    *,
    camera: str,
    preset: str,
    slice_index: int,
    frame_size: tuple[int, int] | None = None,
) -> FocusBoxSettings | None:
    """The box to store for a viewer ROI, or ``None`` if nothing is left of it.

    The ROI is rounded to whole pixels and, if *frame_size* (``(width,
    height)``) is given, clipped to the frame; *origin* is then added to give
    full-sensor coordinates (the inverse of :func:`focus_box_to_bbox`).
    """
    (x1, y1), (x2, y2) = bbox
    x1, x2 = sorted((round(x1), round(x2)))
    y1, y2 = sorted((round(y1), round(y2)))
    if frame_size is not None:
        width, height = frame_size
        x1, x2 = max(x1, 0), min(x2, width)
        y1, y2 = max(y1, 0), min(y2, height)
    if x2 <= x1 or y2 <= y1:
        return None
    ox, oy = origin
    return FocusBoxSettings(
        camera=camera,
        preset=preset,
        slice_index=int(slice_index),
        rect=(x1 + ox, y1 + oy, x2 - x1, y2 - y1),
    )


def default_bbox(frame_size: tuple[int, int]) -> BBox:
    """A box a quarter of the frame in each direction, centred in it."""
    width, height = frame_size
    w, h = max(width // 4, 1), max(height // 4, 1)
    x, y = (width - w) // 2, (height - h) // 2
    return ((x, y), (x + w, y + h))


class ViewerFocusBox(QObject):
    """Let the user draw the focus box on one MDA viewer.

    Parameters
    ----------
    viewer : ndv.ArrayViewer
        The viewer showing one camera's frames of *sequence*.
    sequence : useq.MDASequence
        The running sequence; its channel list names the preset behind each
        viewer ``c`` index.
    camera_label : str
        The physical camera the viewer shows (``""`` for a single-camera run).
    button : QAbstractButton
        A checkable button: checked shows the ROI (drag it to move, drag a
        handle to resize), unchecked hides it.
    mmcore : CMMCorePlus
        For the camera's hardware ROI (origin and frame size).
    parent : QObject | None
        Owner; the controller lives and dies with it.
    """

    def __init__(
        self,
        viewer: ndv.ArrayViewer,
        sequence: useq.MDASequence,
        camera_label: str,
        button: QAbstractButton,
        mmcore: CMMCorePlus,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._viewer = viewer
        self._channels = [ch.config for ch in sequence.channels]
        self._button = button
        self.camera = camera_label or physical_camera_labels(mmcore)[0]
        # Fixed now, while the run's ROI is the camera's ROI: the viewer's
        # array starts here even if the ROI is changed later.
        roi = camera_roi(mmcore, self.camera)
        self.origin: tuple[int, int] = (roi[0], roi[1]) if roi else (0, 0)
        self.frame_size: tuple[int, int] | None = (roi[2], roi[3]) if roi else None

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(_COMMIT_DELAY_MS)
        self._timer.timeout.connect(self.commit)

        self._model: RectangularROIModel | None = None
        self._settling = False  # ignore the model events our own writes cause

        button.setToolTip(
            "Show the image focus lock box. Drag it over a bright, sharp "
            "structure, resize with the corner handles. The box is saved for "
            "the displayed slice and channel once you stop moving it."
        )
        button.toggled.connect(self._on_toggled)

        # The saved box, if it is for this camera, is shown at once.
        saved = SettingsV1.instance().image_focus.box
        if saved is not None and saved.camera == self.camera:
            self._show(focus_box_to_bbox(saved, self.origin))
            button.setChecked(True)

        # Follow "Clear box" / a new box from another viewer; drop the link
        # when this controller goes (psygnal does not know about Qt lifetimes).
        callback = self._on_settings_box_changed
        focus_lock_signals.box_changed.connect(callback)
        self.destroyed.connect(
            lambda *_: suppress_disconnect(focus_lock_signals.box_changed, callback)
        )

    # -- viewer state --------------------------------------------------

    def selection(self) -> tuple[str, int] | None:
        """``(preset, slice_index)`` of what the viewer displays now.

        The preset is the sequence's channel config at the viewer's ``c``
        index (the index ``event.index["c"]`` the display store writes by);
        slice is its ``z`` index. An axis the viewer lacks counts as 0.
        ``None`` if the sequence has no channel to name a preset.
        """
        index = dict(self._viewer.display_model.current_index)
        c, z = index.get("c", 0), index.get("z", 0)
        c = c if isinstance(c, int) else 0
        z = z if isinstance(z, int) else 0
        if not 0 <= c < len(self._channels):
            return None
        return self._channels[c], z

    def _size(self) -> tuple[int, int] | None:
        if self.frame_size is not None:
            return self.frame_size
        wrapper = self._viewer.data_wrapper
        shape = getattr(getattr(wrapper, "data", None), "shape", None)
        return (int(shape[-1]), int(shape[-2])) if shape else None

    # -- ROI overlay ---------------------------------------------------

    def _show(self, bbox: BBox) -> None:
        """Make the ROI visible at *bbox* (creating it if needed)."""
        self._settling = True
        try:
            if self._model is None:
                self._viewer.roi = RectangularROIModel(visible=True, bounding_box=bbox)
                self._model = self._viewer.roi  # the viewer's own, not a copy
                assert self._model is not None
                self._model.events.bounding_box.connect(self._on_bbox_changed)
            else:
                self._model.bounding_box = bbox
                self._model.visible = True
        finally:
            self._settling = False

    def _hide(self) -> None:
        if self._model is not None:
            self._settling = True
            try:
                self._model.visible = False
            finally:
                self._settling = False

    def _on_toggled(self, checked: bool) -> None:
        if not checked:
            self._timer.stop()
            self._hide()
            return
        if self._model is not None:
            self._settling = True
            try:
                self._model.visible = True
            finally:
                self._settling = False
            return
        saved = SettingsV1.instance().image_focus.box
        if saved is not None and saved.camera == self.camera:
            self._show(focus_box_to_bbox(saved, self.origin))
        elif (size := self._size()) is not None:
            self._show(default_bbox(size))
            self.commit()  # the default box is a box; saving it is the point
        else:
            self._button.setChecked(False)  # no frame yet, nothing to draw on

    def _on_bbox_changed(self, *_: object) -> None:
        if not self._settling:
            self._timer.start()

    # -- settings ------------------------------------------------------

    def commit(self) -> FocusBoxSettings | None:
        """Write the ROI as the focus box (displayed slice and channel).

        Returns the stored box, or ``None`` if there is no ROI, no channel to
        name a preset, or nothing is left of the ROI after clipping.
        """
        self._timer.stop()
        if self._model is None or not self._model.visible:
            return None
        if (sel := self.selection()) is None:
            logger.warning("Focus box not saved: the sequence has no channel")
            return None
        preset, slice_index = sel
        box = bbox_to_focus_box(
            self._model.bounding_box,
            self.origin,
            camera=self.camera,
            preset=preset,
            slice_index=slice_index,
            frame_size=self._size(),
        )
        if box is None:
            logger.warning("Focus box not saved: the ROI is outside the frame")
            return None
        settings = SettingsV1.instance()
        settings.image_focus.box = box
        settings.flush()
        self._emit_box_changed()
        return box

    def _emit_box_changed(self) -> None:
        # Our own write: other listeners (the dock) refresh, we skip ourselves.
        self._settling = True
        try:
            focus_lock_signals.box_changed.emit()
        except Exception:
            logger.warning("A focus box listener raised.", exc_info=True)
        finally:
            self._settling = False

    def _on_settings_box_changed(self) -> None:
        """The box was cleared or redrawn elsewhere: follow it."""
        if self._settling or self._model is None:
            return
        saved = SettingsV1.instance().image_focus.box
        if saved is None or saved.camera != self.camera:
            self._button.setChecked(False)
            return
        self._show(focus_box_to_bbox(saved, self.origin))
        self._button.setChecked(True)


def suppress_disconnect(signal: object, callback: object) -> None:
    """Disconnect *callback* from a psygnal *signal*, ignoring a missing link."""
    with suppress(Exception):
        signal.disconnect(callback)  # type: ignore[attr-defined]
