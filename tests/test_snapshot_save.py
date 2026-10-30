from __future__ import annotations

import json
from datetime import datetime
from typing import TYPE_CHECKING

import numpy as np
import tifffile

from pymmcore_gui._ndv_viewers import NDVViewersManager
from pymmcore_gui._qt.QtWidgets import QWidget
from pymmcore_gui._snapshot_save import LatestFrame, collect_scope_state, save_frames

if TYPE_CHECKING:
    from pathlib import Path

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot


def _frame(label: str, value: int) -> LatestFrame:
    data = np.full((4, 6), value, dtype=np.uint16)
    stamp = datetime(2026, 10, 21, 11, 30, 5)
    return LatestFrame(label, data, "live", stamp, {"Camera": label, "ROI": "0-0-6-4"})


def test_save_frames_writes_tiff_and_sidecar(tmp_path: Path) -> None:
    frames = [_frame("Camera-1", 7), _frame("Camera-2", 9)]
    state = {"exposure_ms": 10.0, "pixel_size_um": 0.115, "channel_preset": "405nm"}

    paths = save_frames(frames, tmp_path / "snaps", state)

    assert [p.name for p in paths] == [
        "20261021-113005_live_Camera-1.ome.tif",
        "20261021-113005_live_Camera-2.ome.tif",
    ]
    for path, item in zip(paths, frames, strict=True):
        np.testing.assert_array_equal(tifffile.imread(path), item.frame)
        sidecar = json.loads(path.with_name(path.name[:-8] + ".json").read_text())
        assert sidecar["camera"] == item.camera_label
        assert sidecar["source"] == "live"
        assert sidecar["exposure_ms"] == 10.0
        assert sidecar["channel_preset"] == "405nm"
        assert sidecar["camera_metadata"]["ROI"] == "0-0-6-4"
        assert sidecar["shape"] == [4, 6]


def test_save_frames_never_overwrites(tmp_path: Path) -> None:
    first = save_frames([_frame("Camera-1", 1)], tmp_path)
    second = save_frames([_frame("Camera-1", 2)], tmp_path)

    assert first[0] != second[0]
    assert second[0].name == "20261021-113005_live_Camera-1_1.ome.tif"
    assert tifffile.imread(first[0])[0, 0] == 1
    assert tifffile.imread(second[0])[0, 0] == 2


def test_collect_scope_state_tolerates_missing_devices(mmcore: CMMCorePlus) -> None:
    state = collect_scope_state(mmcore)

    # The demo config has no ASI galvo or CRISP: recorded as None, not raised.
    assert state["galvo_offset_deg"] is None
    assert state["crisp_state"] is None
    assert state["exposure_ms"] == mmcore.getExposure()
    assert state["focus_device"] == mmcore.getFocusDevice()


def test_latest_frames_keeps_newest_snap(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    dummy = QWidget()
    manager = NDVViewersManager(dummy, mmcore)
    assert manager.latest_frames() == []

    mmcore.snapImage()
    frames = manager.latest_frames()

    assert [f.camera_label for f in frames] == [mmcore.getCameraDevice()]
    assert frames[0].source == "snap"
    np.testing.assert_array_equal(frames[0].frame, mmcore.getImage())
    dummy.deleteLater()
