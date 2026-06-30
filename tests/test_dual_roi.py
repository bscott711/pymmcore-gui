from __future__ import annotations

from typing import TYPE_CHECKING, Any, ClassVar

import numpy as np
import pytest
import useq

from pymmcore_gui._multi_camera_handler import MultiCameraHandler
from pymmcore_gui._roi_utils import (
    apply_dual_roi,
    clear_roi,
    rect_to_pixel_roi,
    rois_from_multiroi_tuple,
    safe_split_composite,
    split_composite,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus


# --------------------------------------------------------------------------- #
# split_composite
# --------------------------------------------------------------------------- #


def _labeled_frame(shape: tuple[int, int]) -> np.ndarray:
    """Frame whose pixel values encode their flat index (for content checks)."""
    return np.arange(int(np.prod(shape)), dtype=np.uint32).reshape(shape)


def test_split_composite_bounding_box() -> None:
    # The exact layout produced by the demo camera (measured): two 100x80 ROIs at
    # y=20 and y=200 pack into a (260, 100) bounding box.
    rois = [(10, 20, 100, 80), (10, 200, 100, 80)]
    frame = _labeled_frame((260, 100))
    out = split_composite(frame, rois)
    assert set(out) == {0, 1}
    assert out[0].shape == (80, 100)
    assert out[1].shape == (80, 100)
    # ROI 0 sits at rows 0:80, ROI 1 at rows 180:260 (y - ymin).
    np.testing.assert_array_equal(out[0], frame[0:80, 0:100])
    np.testing.assert_array_equal(out[1], frame[180:260, 0:100])


def test_split_composite_horizontal_tight_pack() -> None:
    # ROIs far apart on the chip but composite is tightly concatenated along x.
    rois = [(10, 5, 100, 80), (400, 5, 100, 80)]
    frame = _labeled_frame((80, 200))
    out = split_composite(frame, rois)
    np.testing.assert_array_equal(out[0], frame[:, 0:100])
    np.testing.assert_array_equal(out[1], frame[:, 100:200])


def test_split_composite_vertical_tight_pack() -> None:
    rois = [(10, 5, 100, 80), (10, 500, 100, 80)]
    frame = _labeled_frame((160, 100))
    out = split_composite(frame, rois)
    np.testing.assert_array_equal(out[0], frame[0:80, :])
    np.testing.assert_array_equal(out[1], frame[80:160, :])


def test_split_composite_single_roi_is_passthrough() -> None:
    frame = _labeled_frame((50, 60))
    out = split_composite(frame, [(0, 0, 60, 50)])
    assert list(out) == [0]
    np.testing.assert_array_equal(out[0], frame)


def test_split_composite_unknown_layout_raises_and_safe_returns_none() -> None:
    rois = [(10, 20, 100, 80), (10, 200, 100, 80)]
    bad = _labeled_frame((123, 77))  # matches no known layout
    with pytest.raises(ValueError, match="Cannot determine multi-ROI packing"):
        split_composite(bad, rois)
    with pytest.warns(RuntimeWarning):
        assert safe_split_composite(bad, rois) is None


# --------------------------------------------------------------------------- #
# rect_to_pixel_roi / rois_from_multiroi_tuple
# --------------------------------------------------------------------------- #


def test_rect_to_pixel_roi_normalizes_and_rounds() -> None:
    # corners given bottom-right first -> still normalized to positive w/h
    # (round() is banker's rounding: round(80.5) == 80)
    assert rect_to_pixel_roi((110.4, 100.6), (10.2, 20.1)) == (10, 20, 100, 80)


def test_rect_to_pixel_roi_clamps_to_sensor() -> None:
    x, y, w, h = rect_to_pixel_roi((480, 480), (600, 600), max_w=512, max_h=512)
    assert x + w <= 512
    assert y + h <= 512


def test_rois_from_multiroi_tuple() -> None:
    rois = rois_from_multiroi_tuple([10, 10], [20, 200], [100, 100], [80, 80])
    assert rois == [(10, 20, 100, 80), (10, 200, 100, 80)]


# --------------------------------------------------------------------------- #
# apply_dual_roi / clear_roi  (lightweight fake core)
# --------------------------------------------------------------------------- #


class _FakeCore:
    def __init__(self, supported: bool = True) -> None:
        self._supported = supported
        self.multi_calls: list[Any] = []
        self.set_roi_calls: list[Any] = []
        self._roi = (0, 0, 512, 512)

    def isMultiROISupported(self) -> bool:
        return self._supported

    def getCameraDevice(self) -> str:
        return "Cam"

    def setMultiROI(self, xs, ys, ws, hs) -> None:
        self.multi_calls.append((xs, ys, ws, hs))

    def clearROI(self) -> None:
        self._roi = (0, 0, 512, 512)

    def getROI(self, label: str) -> tuple[int, int, int, int]:
        return self._roi

    def setROI(self, label, x, y, w, h) -> None:
        self.set_roi_calls.append((label, x, y, w, h))


def test_apply_dual_roi_calls_set_multi_roi() -> None:
    core = _FakeCore(supported=True)
    apply_dual_roi(core, [(10, 20, 100, 80), (10, 200, 100, 80)])  # type: ignore[arg-type]
    assert core.multi_calls == [([10, 10], [20, 200], [100, 100], [80, 80])]


def test_apply_dual_roi_attempts_even_when_flag_false() -> None:
    # isMultiROISupported() is advisory only: a False flag must NOT block the attempt
    # (the demo cam / some Kinetix configs report False yet accept setMultiROI).
    core = _FakeCore(supported=False)
    apply_dual_roi(core, [(0, 0, 10, 10)])  # type: ignore[arg-type]
    assert core.multi_calls == [([0], [0], [10], [10])]


def test_apply_dual_roi_propagates_device_rejection() -> None:
    core = _FakeCore(supported=False)

    def _raise(*_a: Any, **_k: Any) -> None:
        raise RuntimeError("multi-ROI not available on this device")

    core.setMultiROI = _raise  # type: ignore[assignment]
    with pytest.raises(RuntimeError, match="rejected the multi-ROI request"):
        apply_dual_roi(core, [(0, 0, 10, 10)])  # type: ignore[arg-type]


def test_apply_dual_roi_validates_input() -> None:
    core = _FakeCore(supported=True)
    with pytest.raises(ValueError, match="At least one"):
        apply_dual_roi(core, [])  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="positive"):
        apply_dual_roi(core, [(0, 0, 0, 10)])  # type: ignore[arg-type]


def test_clear_roi_reapplies_full_chip() -> None:
    core = _FakeCore()
    clear_roi(core)  # type: ignore[arg-type]
    assert core.set_roi_calls == [("Cam", 0, 0, 512, 512)]


# --------------------------------------------------------------------------- #
# MultiCameraHandler per-ROI splitting
# --------------------------------------------------------------------------- #


class _RecordingWriter:
    instances: ClassVar[dict[str, _RecordingWriter]] = {}

    def __init__(self, path: str) -> None:
        self.path = path
        self.frames: list[np.ndarray] = []
        _RecordingWriter.instances[path] = self

    def sequenceStarted(self, seq, meta=None) -> None:
        pass

    def frameReady(self, frame, event, meta) -> None:
        self.frames.append(np.asarray(frame))

    def sequenceFinished(self, seq) -> None:
        pass


@pytest.fixture()
def fake_writers(monkeypatch: pytest.MonkeyPatch) -> type[_RecordingWriter]:
    _RecordingWriter.instances.clear()
    monkeypatch.setattr(
        "pymmcore_gui._multi_camera_handler.handler_for_path", _RecordingWriter
    )
    return _RecordingWriter


def test_handler_splits_composite_into_per_roi_writers(
    mmcore: CMMCorePlus, fake_writers: type[_RecordingWriter]
) -> None:
    handler = MultiCameraHandler("exp.ome.tiff", mmcore=mmcore)
    multi_roi = ([10, 10], [20, 200], [100, 100], [80, 80])
    meta = {"image_infos": [{"camera_label": "Camera", "multi_roi": multi_roi}]}
    seq = useq.MDASequence(time_plan={"interval": 0, "loops": 1})

    handler.sequenceStarted(seq, meta)
    composite = _labeled_frame((260, 100))
    handler.frameReady(
        composite, useq.MDAEvent(index={"t": 0}), {"camera_device": "Camera"}
    )

    keys = set(handler._writers)
    assert any(k.endswith("_roi0") for k in keys)
    assert any(k.endswith("_roi1") for k in keys)
    roi0 = handler._writers[next(k for k in keys if k.endswith("_roi0"))]
    roi1 = handler._writers[next(k for k in keys if k.endswith("_roi1"))]
    assert roi0.frames[0].shape == (80, 100)
    assert roi1.frames[0].shape == (80, 100)
    np.testing.assert_array_equal(roi0.frames[0], composite[0:80, 0:100])
    np.testing.assert_array_equal(roi1.frames[0], composite[180:260, 0:100])


def test_handler_single_roi_is_unchanged(
    mmcore: CMMCorePlus, fake_writers: type[_RecordingWriter]
) -> None:
    handler = MultiCameraHandler("exp.ome.tiff", mmcore=mmcore)
    seq = useq.MDASequence(time_plan={"interval": 0, "loops": 1})
    handler.sequenceStarted(seq, {})  # no multi_roi
    frame = _labeled_frame((512, 512))
    handler.frameReady(
        frame, useq.MDAEvent(index={"t": 0}), {"camera_device": "Camera"}
    )
    # exactly one writer (the per-camera one), no _roiN split
    assert not any(k.endswith(("_roi0", "_roi1")) for k in handler._writers)
    (writer,) = handler._writers.values()
    np.testing.assert_array_equal(writer.frames[0], frame)


# --------------------------------------------------------------------------- #
# DualRoiWidget (interactive, vispy)
# --------------------------------------------------------------------------- #


def test_dual_roi_widget_size_lock_and_apply(
    qtbot, mmcore: CMMCorePlus, monkeypatch: pytest.MonkeyPatch
) -> None:
    from pymmcore_widgets.control._rois.roi_model import RectangleROI

    from pymmcore_gui.widgets._dual_roi_widget import DualRoiWidget

    widget = DualRoiWidget(mmcore=mmcore)
    qtbot.addWidget(widget)

    # Draw the first ROI (100 x 80).  Real drawing goes through the canvas event
    # filter which sets fov_size; mimic that here.
    widget.roi_manager.add_roi(
        RectangleROI(top_left=(10, 20), bot_right=(110, 100), fov_size=widget._fov)
    )
    assert widget._add_btn.isEnabled()
    assert not widget._apply_btn.isEnabled()

    # Add the matching second ROI; it must be the same size.
    widget._add_matching_roi()
    assert widget.roi_manager.roi_model.rowCount() == 2
    rois = widget._current_pixel_rois()
    assert rois[0][2:] == rois[1][2:] == (100, 80)

    # Resizing the second ROI snaps back to the locked size (move-only).
    roi2 = widget._rois()[1]
    roi2.translate_vertex(2, 40, 40)
    model = widget.roi_manager.roi_model
    idx = model.index_of(roi2)
    model.dataChanged.emit(idx, idx, [model.VERTEX_ROLE])
    left, top, right, bottom = roi2.bbox()
    assert abs((right - left) - 100) < 1.0
    assert abs((bottom - top) - 80) < 1.0

    # Apply: pretend the camera supports multi-ROI and capture the call.
    captured: dict[str, Any] = {}
    monkeypatch.setattr(mmcore, "isMultiROISupported", lambda: True)
    monkeypatch.setattr(
        mmcore,
        "setMultiROI",
        lambda xs, ys, ws, hs: captured.update(xs=xs, ys=ys, ws=ws, hs=hs),
    )
    widget._update_controls()
    assert widget._apply_btn.isEnabled()
    widget._apply()
    assert captured["ws"] == [100, 100]
    assert captured["hs"] == [80, 80]
