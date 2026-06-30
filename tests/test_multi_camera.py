from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import tifffile
import useq

from pymmcore_gui._multi_camera_handler import (
    MultiCameraHandler,
    per_camera_path,
    without_cam_index,
)
from pymmcore_gui.actions._action_info import ActionInfo
from pymmcore_gui.actions.widget_actions import WidgetAction


class _StubCore:
    """Minimal stand-in exposing only what MultiCameraHandler needs."""

    def __init__(self, cameras: list[str]) -> None:
        self._cameras = cameras

    def getNumberOfCameraChannels(self) -> int:
        return len(self._cameras)

    def getCameraDevice(self) -> str:
        return "MultiCam" if len(self._cameras) > 1 else self._cameras[0]

    def getPhysicalCameraDevice(self, i: int) -> str:
        return self._cameras[i]


def test_crisp_action_registered() -> None:
    """CRISPy is registered as a widget action so it appears in the GUI."""
    infos = ActionInfo.widget_actions()
    assert WidgetAction.CRISP.value in infos
    assert infos[WidgetAction.CRISP.value].text == "CRISPy Autofocus"


def test_per_camera_path() -> None:
    assert per_camera_path("d/exp.ome.tiff", "Camera-1") == "d/exp_Camera-1.ome.tiff"
    assert per_camera_path("d/exp.ome.zarr", "Cam 2") == "d/exp_Cam_2.ome.zarr"
    # no recognized suffix -> sibling directory (tiff sequence)
    assert per_camera_path("d/exp", "Camera-1") == "d/exp_Camera-1"


def test_multi_camera_handler_writes_one_file_per_camera(tmp_path: Path) -> None:
    """Per-camera frames sharing one event index land in separate, complete files.

    The stock engine yields one frame per physical camera for every event, all with
    the *same* ``event.index`` and differing only by ``meta["camera_device"]``.  This
    drives the handler with that exact pattern and asserts no frames are lost.
    """
    cameras = ["Camera", "Camera2"]
    handler = MultiCameraHandler(tmp_path / "acq.ome.tiff", mmcore=_StubCore(cameras))

    seq = useq.MDASequence(
        channels=["DAPI"],  # pyright: ignore[reportArgumentType]
        z_plan=useq.ZRangeAround(range=2, step=1),  # 3 z slices
    )

    handler.sequenceStarted(seq, {})
    # Give each camera a distinct constant value so we can verify routing.
    values = {"Camera": 100, "Camera2": 200}
    for event in seq:
        for cam in cameras:
            frame = np.full((8, 8), values[cam], dtype=np.uint16)
            handler.frameReady(frame, event, {"camera_device": cam})
    handler.sequenceFinished(seq)

    assert set(handler._writers) == set(cameras)
    for cam in cameras:
        out = Path(per_camera_path(tmp_path / "acq.ome.tiff", cam))
        assert out.exists(), f"missing output for {cam}: {out}"
        data = tifffile.imread(out)
        # 3 z-slices preserved (not overwritten by the other camera).
        assert data.shape[0] == 3
        # Every pixel belongs to the routed camera.
        assert int(data.min()) == int(data.max()) == values[cam]


def test_without_cam_index_strips_only_cam() -> None:
    event = useq.MDAEvent(index={"c": 0, "z": 2, "cam": 1})
    stripped = without_cam_index(event)
    assert "cam" not in stripped.index
    assert dict(stripped.index) == {"c": 0, "z": 2}

    # No-op (same object) when there is no cam axis, e.g. single-camera events.
    plain = useq.MDAEvent(index={"c": 0, "z": 2})
    assert without_cam_index(plain) is plain


def test_tensorstore_rejects_unstripped_cam_axis() -> None:
    """Regression: the engine's ``cam`` axis is absent from the per-camera store.

    The per-camera ``TensorStoreHandler`` builds its store dimensions from
    ``seq.sizes`` (here ``{c, z}``), so an event carrying the engine-injected
    ``cam`` index can't be written until that axis is stripped.
    """
    pytest.importorskip("tensorstore")
    from pymmcore_plus.mda.handlers import TensorStoreHandler

    seq = useq.MDASequence(
        channels=["DAPI"],  # pyright: ignore[reportArgumentType]
        z_plan=useq.ZRangeAround(range=2, step=1),  # 3 z slices
    )
    event = next(iter(seq))
    cam_event = event.model_copy(update={"index": {**event.index, "cam": 0}})
    frame = np.zeros((8, 8), dtype=np.uint16)

    # Unstripped event -> IndexError (the bug this fix targets).
    failing = TensorStoreHandler(driver="zarr", kvstore="memory://")
    failing.reset(seq)
    with pytest.raises(IndexError):
        failing.frameReady(frame, cam_event, {})

    # Stripped event -> writes cleanly into the cam-less store.
    handler = TensorStoreHandler(driver="zarr", kvstore="memory://")
    handler.reset(seq)
    for ev in seq:
        cam_ev = ev.model_copy(update={"index": {**ev.index, "cam": 0}})
        handler.frameReady(frame, without_cam_index(cam_ev), {})
    handler.sequenceFinished(seq)

    assert handler.store is not None
    assert "cam" not in set(handler.store.domain.labels)
