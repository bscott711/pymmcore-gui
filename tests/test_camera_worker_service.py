from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import patch

import pytest

from pymmcore_gui.asi_z_stack.camera_handoff import (
    CameraHandoffSnapshot,
    _CameraSnapshot,
)
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
    WorkerConfigGroup,
    _detach_unloadable_config_groups,
    has_camera,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot


class _FakePool:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[tuple[str, str], ...]]] = []

    def set_properties(
        self, camera_label: str, values: tuple[tuple[str, str], ...]
    ) -> None:
        self.calls.append((camera_label, values))


@pytest.fixture
def core(mmcore: CMMCorePlus) -> CMMCorePlus:
    # The global instance: the stock PresetsWidget ignores the table's core.
    mmcore.loadSystemConfiguration()
    return mmcore


def _service(core: CMMCorePlus, pool: _FakePool) -> CameraWorkerService:
    svc = CameraWorkerService()
    svc._mmc = core
    svc._pool = pool  # type: ignore[assignment]
    svc._state = CameraWorkerServiceState.IDLE
    svc._snapshot = CameraHandoffSnapshot(
        n_cameras=2,
        camera_labels=("Camera-1", "Camera-2"),
        core_camera_role="Multi Camera",
        image_width=16,
        image_height=16,
        bytes_per_pixel=2,
        n_components=1,
        per_camera={
            "Camera-1": _CameraSnapshot(None, None, {"Port": "Speed"}),
            "Camera-2": _CameraSnapshot(None, None, {"Port": "Speed"}),
        },
    )
    svc._worker_config_groups = {
        "Cameras": WorkerConfigGroup(
            presets={
                "Sensitivity": (
                    ("Camera-1", "Port", "Sensitivity"),
                    ("Camera-2", "Port", "Sensitivity"),
                    ("Multi Camera", "Physical Camera 1", "Camera-1"),
                    ("Dichroic", "Label", "400DCLP"),
                ),
                "Speed": (
                    ("Camera-1", "Port", "Speed"),
                    ("Camera-2", "Port", "Speed"),
                ),
            },
            current="Speed",
        )
    }
    return svc


def test_detach_captures_and_removes_groups_touching_released_devices(
    core: CMMCorePlus,
) -> None:
    current = core.getCurrentConfig("Camera")
    presets = set(core.getAvailableConfigs("Camera"))
    groups = _detach_unloadable_config_groups(core, {"Camera"})

    # "System" also references Camera in the demo config's Startup preset.
    assert set(groups) == {"Camera", "System"}
    assert groups["Camera"].current == current
    assert set(groups["Camera"].presets) == presets
    assert all(g not in core.getAvailableConfigGroups() for g in groups)

    # Every remaining group is still evaluable once the device is gone.
    core.unloadDevice("Camera")
    for group in core.getAvailableConfigGroups():
        core.getCurrentConfig(group)


def test_apply_worker_config_routes_settings(core: CMMCorePlus) -> None:
    pool = _FakePool()
    svc = _service(core, pool)
    seen: list[tuple[str, str]] = []
    svc.workerConfigChanged.connect(lambda g, p: seen.append((g, p)))

    svc.apply_worker_config("Cameras", "Sensitivity")

    assert pool.calls == [
        ("Camera-1", (("Port", "Sensitivity"),)),
        ("Camera-2", (("Port", "Sensitivity"),)),
    ]
    # Loaded main-process device set directly; unloaded composite skipped.
    assert core.getProperty("Dichroic", "Label") == "400DCLP"
    assert svc.worker_config_groups["Cameras"].current == "Sensitivity"
    assert seen == [("Cameras", "Sensitivity")]
    # Respawn snapshot follows the change.
    assert svc._snapshot is not None
    assert svc._snapshot.per_camera["Camera-1"].property_values["Port"] == (
        "Sensitivity"
    )


def test_apply_worker_config_refused_during_mda(core: CMMCorePlus) -> None:
    pool = _FakePool()
    svc = _service(core, pool)
    svc._state = CameraWorkerServiceState.MDA
    with pytest.raises(RuntimeError, match="MDA"):
        svc.apply_worker_config("Cameras", "Sensitivity")
    assert pool.calls == []
    assert svc.worker_config_groups["Cameras"].current == "Speed"


def test_apply_worker_config_restarts_live(
    core: CMMCorePlus, monkeypatch: pytest.MonkeyPatch
) -> None:
    pool = _FakePool()
    svc = _service(core, pool)
    svc._state = CameraWorkerServiceState.LIVE
    order: list[str] = []
    monkeypatch.setattr(svc, "stop_live", lambda: order.append("stop"))
    monkeypatch.setattr(svc, "start_live", lambda: order.append("start"))
    monkeypatch.setattr(
        pool, "set_properties", lambda label, values: order.append(label)
    )

    svc.apply_worker_config("Cameras", "Speed")

    assert order == ["stop", "Camera-1", "Camera-2", "start"]


def test_has_camera_true_when_service_active(core: CMMCorePlus) -> None:
    core.unloadDevice("Camera")
    assert core.getCameraDevice() == ""
    assert not has_camera(core)

    svc = _service(core, _FakePool())
    CameraWorkerService._active = svc
    try:
        assert has_camera(core)
    finally:
        CameraWorkerService._active = None


def test_worker_group_table_lists_and_applies_worker_groups(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    from qtpy.QtWidgets import QComboBox

    from pymmcore_gui.widgets._worker_group_preset_table import (
        WorkerGroupPresetTableWidget,
    )

    pool = _FakePool()
    svc = _service(core, pool)
    wdg = WorkerGroupPresetTableWidget(svc, mmcore=core)
    qtbot.addWidget(wdg)

    table = wdg.table_wdg
    rows = {table.item(r, 0).text(): r for r in range(table.rowCount())}
    assert "Cameras" in rows
    assert len(rows) == len(core.getAvailableConfigGroups()) + 1

    combo = table.cellWidget(rows["Cameras"], 1)
    assert isinstance(combo, QComboBox)
    assert combo.currentText() == "Speed"

    combo.textActivated.emit("Sensitivity")
    assert svc.worker_config_groups["Cameras"].current == "Sensitivity"
    assert pool.calls[0] == ("Camera-1", (("Port", "Sensitivity"),))

    # A failed apply reverts the combo to what's actually in effect.
    svc._state = CameraWorkerServiceState.MDA
    combo.setCurrentText("Speed")
    with patch(
        "pymmcore_gui.widgets._worker_group_preset_table.QMessageBox.warning"
    ) as warn:
        combo.textActivated.emit("Speed")
    warn.assert_called_once()
    assert combo.currentText() == "Sensitivity"


def test_snap_arms_internal_trigger_and_drains(core: CMMCorePlus) -> None:
    from unittest.mock import MagicMock

    import numpy as np

    pool = MagicMock()
    frame = np.zeros((2, 2), dtype="uint16")
    pool.iter_frames.return_value = iter(
        [("Camera-1", 0, frame, {}, 0), ("Camera-2", 0, frame, {}, 0)]
    )
    svc = _service(core, pool)

    frames = svc.snap()

    assert set(frames) == {"Camera-1", "Camera-2"}
    assert pool.arm_all.call_args.kwargs["external_trigger"] is False
    pool.stop_and_drain.assert_called_once()
    pool.stop_all.assert_not_called()
