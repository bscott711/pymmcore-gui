"""OPM pin tests: must stay green after every EPI step.

These pin today's OPM behaviour (written before any EPI mode code exists, so
nothing here imports ``pymmcore_gui.epi``). They use fakes only -- no hardware,
no window -- and cover the seams the EPI mode toggle must leave untouched:

* ``physical_camera_labels``
* ``MicroManagerGUI._register_mda_engine`` installing ``ASISPIMEngine``
* the Snap / Live laser path in ``actions/core_actions.py``
* the default (and ``exclude_defaults``) settings JSON
* the ``CameraWorkerService`` Live / Snap / MDA / ROI / exposure paths
* ``shutdown_mmcore`` closing lasers
"""

from __future__ import annotations

import threading
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from pymmcore_gui._main_window import MicroManagerGUI
from pymmcore_gui._mmcore_shutdown import shutdown_mmcore
from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.actions.core_actions import snap_image, toggle_live
from pymmcore_gui.asi_z_stack import asi_controller
from pymmcore_gui.asi_z_stack.camera_handoff import (
    CameraHandoffSnapshot,
    _CameraSnapshot,
)
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus


# ---------------------------------------------------------------------------
# physical_camera_labels
# ---------------------------------------------------------------------------


def _service(pool: object, mmc: object | None = None) -> CameraWorkerService:
    """A ``CameraWorkerService`` wired to *pool* in the IDLE state (no processes)."""
    svc = CameraWorkerService()
    svc._mmc = mmc if mmc is not None else MagicMock()  # type: ignore[assignment]
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
    return svc


def _state_of(svc: CameraWorkerService) -> CameraWorkerServiceState:
    return svc.state


class _FakePool:
    """Records ``set_properties`` / ``set_roi`` calls; reports no dead workers."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[tuple[str, str], ...]]] = []
        self.roi_calls: list[tuple[str, int, int, int, int, float]] = []

    def set_properties(
        self, camera_label: str, values: tuple[tuple[str, str], ...]
    ) -> None:
        self.calls.append((camera_label, values))

    def set_roi(
        self, camera_label: str, x: int, y: int, w: int, h: int, timeout: float = 5.0
    ) -> tuple[int, int, int, int]:
        self.roi_calls.append((camera_label, x, y, w, h, timeout))
        return x, y, w, h

    def dead_workers(self) -> list[tuple[str, int | None]]:
        return []


def test_physical_camera_labels_service_active_uses_service_labels() -> None:
    svc = _service(_FakePool())
    mmc = MagicMock()
    with patch.object(CameraWorkerService, "_active", svc):
        labels = physical_camera_labels(mmc)
    assert labels == ["Camera-1", "Camera-2"]
    assert isinstance(labels, list)
    # the service answers alone: the core is not consulted
    mmc.getNumberOfCameraChannels.assert_not_called()
    mmc.getCameraDevice.assert_not_called()


def test_physical_camera_labels_no_service_single_camera() -> None:
    mmc = MagicMock()
    mmc.getNumberOfCameraChannels.return_value = 1
    mmc.getCameraDevice.return_value = "Camera"
    with patch.object(CameraWorkerService, "_active", None):
        assert physical_camera_labels(mmc) == ["Camera"]


def test_physical_camera_labels_no_service_multi_camera() -> None:
    mmc = MagicMock()
    mmc.getNumberOfCameraChannels.return_value = 2
    mmc.getPhysicalCameraDevice.side_effect = lambda i: f"Cam{i}"
    with patch.object(CameraWorkerService, "_active", None):
        assert physical_camera_labels(mmc) == ["Cam0", "Cam1"]


# ---------------------------------------------------------------------------
# _register_mda_engine installs ASISPIMEngine
# ---------------------------------------------------------------------------


def test_register_mda_engine_installs_asi_spim_engine(mmcore: CMMCorePlus) -> None:
    from pymmcore_plus.mda import MDAEngine

    from pymmcore_gui.asi_z_stack.engine import ASISPIMEngine

    fake_window = SimpleNamespace(_mmc=mmcore)
    try:
        with patch(
            "pymmcore_gui.asi_z_stack.asi_controller.asi_zstack_hardware_available",
            lambda: True,
        ):
            MicroManagerGUI._register_mda_engine(fake_window)  # type: ignore[arg-type]
        engine = mmcore.mda.engine
        assert isinstance(engine, ASISPIMEngine)

        # idempotent while the hardware is still there
        with patch(
            "pymmcore_gui.asi_z_stack.asi_controller.asi_zstack_hardware_available",
            lambda: True,
        ):
            MicroManagerGUI._register_mda_engine(fake_window)  # type: ignore[arg-type]
        assert mmcore.mda.engine is engine

        # a non-ASI config restores the stock engine
        MicroManagerGUI._register_mda_engine(fake_window)  # type: ignore[arg-type]
        assert type(mmcore.mda.engine) is MDAEngine
    finally:
        mmcore.mda.set_engine(MDAEngine(mmcore))


def test_register_mda_engine_keeps_stock_engine_without_asi_hardware(
    mmcore: CMMCorePlus,
) -> None:
    from pymmcore_plus.mda import MDAEngine

    engine = mmcore.mda.engine
    MicroManagerGUI._register_mda_engine(SimpleNamespace(_mmc=mmcore))  # type: ignore[arg-type]
    assert mmcore.mda.engine is engine
    assert type(engine) is MDAEngine


# ---------------------------------------------------------------------------
# Snap / Live laser path
# ---------------------------------------------------------------------------


@pytest.fixture
def laser_calls(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """Record the laser helpers the Snap/Live actions import at call time."""
    calls: list[str] = []
    monkeypatch.setattr(
        asi_controller, "open_selected_lasers", lambda: calls.append("open")
    )
    monkeypatch.setattr(
        asi_controller, "close_selected_lasers", lambda: calls.append("close")
    )
    monkeypatch.setattr(
        asi_controller, "close_all_lasers", lambda: calls.append("close_all")
    )
    return calls


def _action(mmc: object) -> SimpleNamespace:
    return SimpleNamespace(mmc=mmc, setChecked=MagicMock())


def _fake_service(calls: list[str], state: CameraWorkerServiceState) -> SimpleNamespace:
    return SimpleNamespace(
        state=state,
        snap=lambda: calls.append("svc.snap"),
        start_live=lambda: calls.append("svc.start_live"),
        stop_live=lambda: calls.append("svc.stop_live"),
    )


def test_snap_without_service_brackets_snap_with_selected_lasers(
    laser_calls: list[str],
) -> None:
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = False
    mmc.snapImage.side_effect = lambda: laser_calls.append("snapImage")
    with patch.object(CameraWorkerService, "_active", None):
        snap_image(_action(mmc), False)  # type: ignore[arg-type]
    assert laser_calls == ["open", "snapImage", "close"]
    mmc.stopSequenceAcquisition.assert_not_called()


def test_snap_without_service_stops_running_sequence_first(
    laser_calls: list[str],
) -> None:
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = True
    mmc.stopSequenceAcquisition.side_effect = lambda: laser_calls.append("stop")
    mmc.snapImage.side_effect = lambda: laser_calls.append("snapImage")
    with patch.object(CameraWorkerService, "_active", None):
        snap_image(_action(mmc), False)  # type: ignore[arg-type]
    assert laser_calls == ["stop", "open", "snapImage", "close"]


def test_snap_without_service_closes_lasers_when_snap_fails(
    laser_calls: list[str],
) -> None:
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = False
    mmc.snapImage.side_effect = RuntimeError("camera")
    with patch.object(CameraWorkerService, "_active", None):
        with pytest.raises(RuntimeError, match="camera"):
            snap_image(_action(mmc), False)  # type: ignore[arg-type]
    assert laser_calls == ["open", "close"]


def test_snap_with_service_routes_through_pool_not_core(
    laser_calls: list[str],
) -> None:
    mmc = MagicMock()
    svc = _fake_service(laser_calls, CameraWorkerServiceState.IDLE)
    with patch.object(CameraWorkerService, "_active", svc):
        snap_image(_action(mmc), False)  # type: ignore[arg-type]
    assert laser_calls == ["open", "svc.snap", "close"]
    mmc.snapImage.assert_not_called()


def test_snap_with_service_closes_lasers_when_snap_fails(
    laser_calls: list[str],
) -> None:
    svc = _fake_service(laser_calls, CameraWorkerServiceState.IDLE)

    def _boom() -> None:
        raise RuntimeError("workers")

    svc.snap = _boom
    with patch.object(CameraWorkerService, "_active", svc):
        with pytest.raises(RuntimeError, match="workers"):
            snap_image(_action(MagicMock()), False)  # type: ignore[arg-type]
    assert laser_calls == ["open", "close"]


def test_live_without_service_start_then_stop(
    laser_calls: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(asi_controller, "circular_buffer_growing", False)
    mmc = MagicMock()
    mmc.startContinuousSequenceAcquisition.side_effect = lambda *_: laser_calls.append(
        "start"
    )
    mmc.stopSequenceAcquisition.side_effect = lambda: laser_calls.append("stop")
    action = _action(mmc)
    with patch.object(CameraWorkerService, "_active", None):
        mmc.isSequenceRunning.return_value = False
        toggle_live(action, True)  # type: ignore[arg-type]
        mmc.isSequenceRunning.return_value = True
        toggle_live(action, False)  # type: ignore[arg-type]
    assert laser_calls == ["open", "start", "stop", "close_all"]
    mmc.startContinuousSequenceAcquisition.assert_called_once_with(0)
    action.setChecked.assert_not_called()


def test_live_without_service_refused_while_circular_buffer_growing(
    laser_calls: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(asi_controller, "circular_buffer_growing", True)
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = False
    action = _action(mmc)
    with patch.object(CameraWorkerService, "_active", None):
        toggle_live(action, True)  # type: ignore[arg-type]
    assert laser_calls == []
    mmc.startContinuousSequenceAcquisition.assert_not_called()
    action.setChecked.assert_called_once_with(False)


def test_live_with_service_idle_opens_lasers_then_starts_live(
    laser_calls: list[str],
) -> None:
    mmc = MagicMock()
    svc = _fake_service(laser_calls, CameraWorkerServiceState.IDLE)
    with patch.object(CameraWorkerService, "_active", svc):
        toggle_live(_action(mmc), True)  # type: ignore[arg-type]
    assert laser_calls == ["open", "svc.start_live"]
    mmc.startContinuousSequenceAcquisition.assert_not_called()


def test_live_with_service_live_stops_then_closes_all_lasers(
    laser_calls: list[str],
) -> None:
    svc = _fake_service(laser_calls, CameraWorkerServiceState.LIVE)
    with patch.object(CameraWorkerService, "_active", svc):
        toggle_live(_action(MagicMock()), False)  # type: ignore[arg-type]
    assert laser_calls == ["svc.stop_live", "close_all"]


@pytest.mark.parametrize(
    "state", [CameraWorkerServiceState.SPAWNING, CameraWorkerServiceState.MDA]
)
def test_live_with_service_busy_unchecks_and_leaves_lasers_alone(
    laser_calls: list[str], state: CameraWorkerServiceState
) -> None:
    svc = _fake_service(laser_calls, state)
    action = _action(MagicMock())
    with patch.object(CameraWorkerService, "_active", svc):
        toggle_live(action, True)  # type: ignore[arg-type]
    assert laser_calls == []
    action.setChecked.assert_called_once_with(False)


# ---------------------------------------------------------------------------
# Settings JSON
# ---------------------------------------------------------------------------

_SETTINGS_TOP_LEVEL_KEYS = [
    "version",
    "window",
    "spectral",
    "argus_stream",
    "mda_writer",
    "focus",
    "spot_focus",
    "snapshot",
    "spim_timing",
    "galvo",
    "epi",
    "send_error_reports",
    "last_config",
    "auto_load_last_config",
    "fallback_to_demo_config",
]


def test_default_settings_exclude_defaults_json_is_empty() -> None:
    # Pytest never reads the user's settings file, so this is the pure default.
    assert SettingsV1().model_dump_json(exclude_defaults=True) == "{}"


def test_non_default_opm_settings_json_is_literal() -> None:
    settings = SettingsV1(
        spim_timing={"mode": "all_rows", "margin_ms": 0.0},
        galvo={"live_offset_deg": 0.0},
        send_error_reports=False,
        auto_load_last_config=True,
    )
    assert settings.model_dump_json(exclude_defaults=True) == (
        '{"spim_timing":{"mode":"all_rows","margin_ms":0.0},'
        '"galvo":{"live_offset_deg":0.0},'
        '"send_error_reports":false,"auto_load_last_config":true}'
    )


def test_default_settings_top_level_keys_are_pinned() -> None:
    assert list(SettingsV1().model_dump()) == _SETTINGS_TOP_LEVEL_KEYS


def test_default_settings_existing_section_keys_still_present() -> None:
    expected: dict[str, set[str]] = {
        "window": {
            "dock_manager_state",
            "geometry",
            "mda_viewer_keep_runs",
            "open_widgets",
            "window_state",
        },
        "mda_writer": {"backlog_budget_mb", "zarr_compression"},
        "snapshot": {"save_dir"},
        "spim_timing": {
            "fallback_readout_ms",
            "galvo_settle_ms",
            "level_trigger_overlap",
            "margin_ms",
            "mode",
            "park_at_start",
            "readout_ms_override",
            "volume_start_settle_ms",
        },
        "galvo": {"live_offset_deg"},
    }
    dumped = SettingsV1().model_dump()
    for section, keys in expected.items():
        assert keys <= set(dumped[section]), section
    assert dumped["spim_timing"]["mode"] == "legacy"
    assert dumped["galvo"]["live_offset_deg"] == -0.26
    assert dumped["mda_writer"]["zarr_compression"] is False
    assert dumped["fallback_to_demo_config"] is False


# ---------------------------------------------------------------------------
# CameraWorkerService methods drive the pool as today
# ---------------------------------------------------------------------------


def test_service_start_live_arms_pool_with_last_exposure_and_stop_live() -> None:
    release = threading.Event()

    def _frames(*_a: Any, **_k: Any) -> Any:
        release.wait(5.0)
        return iter(())

    pool = MagicMock()
    pool.dead_workers.return_value = []
    pool.iter_frames.side_effect = _frames
    pool.stop_all.side_effect = release.set
    svc = _service(pool)
    svc.last_known_exposure_ms = 12.5
    states: list[bool] = []
    svc.liveStateChanged.connect(states.append)

    svc.start_live()
    try:
        assert _state_of(svc) is CameraWorkerServiceState.LIVE
        assert states == [True]
        pool.arm_live_all.assert_called_once()
        kwargs = pool.arm_live_all.call_args.kwargs
        assert kwargs["exposure_ms"] == 12.5
        assert kwargs["armed_timeout"] == svc.hw.worker_arm_timeout_s
        svc.start_live()  # already live: no second arm
        pool.arm_live_all.assert_called_once()
    finally:
        svc.stop_live()
        release.set()

    pool.stop_all.assert_called_once()
    pool.stop_and_drain.assert_called_once()
    assert _state_of(svc) is CameraWorkerServiceState.IDLE


def test_service_snap_arms_internal_trigger_and_emits_each_frame() -> None:
    frame = np.zeros((2, 2), dtype="uint16")
    pool = MagicMock()
    pool.dead_workers.return_value = []
    pool.iter_frames.return_value = iter(
        [("Camera-1", 0, frame, {}, 0), ("Camera-2", 0, frame, {}, 0)]
    )
    svc = _service(pool)
    svc.last_known_exposure_ms = 7.0
    seen: list[str] = []
    svc.frameReady.connect(lambda label, *_: seen.append(label))

    frames = svc.snap()

    assert set(frames) == {"Camera-1", "Camera-2"}
    assert seen == ["Camera-1", "Camera-2"]
    pool.arm_all.assert_called_once()
    assert pool.arm_all.call_args.args == (1,)
    assert pool.arm_all.call_args.kwargs["external_trigger"] is False
    assert pool.arm_all.call_args.kwargs["exposure_ms"] == 7.0
    pool.stop_and_drain.assert_called_once()
    pool.stop_all.assert_not_called()


def test_service_acquire_for_mda_claims_pool_and_release_returns_to_idle() -> None:
    pool = MagicMock()
    pool.dead_workers.return_value = []
    svc = _service(pool)

    assert svc.acquire_for_mda() is pool
    assert _state_of(svc) is CameraWorkerServiceState.MDA
    with pytest.raises(RuntimeError, match="already acquired"):
        svc.acquire_for_mda()

    svc.release_from_mda()
    assert _state_of(svc) is CameraWorkerServiceState.IDLE
    svc.release_from_mda()  # no-op when not in MDA
    assert _state_of(svc) is CameraWorkerServiceState.IDLE
    pool.arm_all.assert_not_called()
    pool.arm_live_all.assert_not_called()


def test_service_acquire_for_mda_stops_live_first(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pool = MagicMock()
    pool.dead_workers.return_value = []
    svc = _service(pool)
    svc._state = CameraWorkerServiceState.LIVE
    order: list[str] = []

    def _stop() -> None:
        order.append("stop_live")
        svc._state = CameraWorkerServiceState.IDLE

    monkeypatch.setattr(svc, "stop_live", _stop)

    assert svc.acquire_for_mda() is pool
    assert order == ["stop_live"]
    assert _state_of(svc) is CameraWorkerServiceState.MDA


def test_service_set_roi_goes_to_pool_and_tracks_crop() -> None:
    pool = _FakePool()
    svc = _service(pool)
    seen: list[str] = []
    svc.roiChanged.connect(seen.append)
    assert svc.roi("Camera-1") == (0, 0, 16, 16)

    applied = svc.set_roi("Camera-1", 2, 3, 8, 5)

    assert pool.roi_calls == [("Camera-1", 2, 3, 8, 5, 5.0)]
    assert applied == (2, 3, 8, 5)
    assert svc.roi("Camera-1") == (2, 3, 8, 5)
    assert svc.roi("Camera-2") == (0, 0, 16, 16)
    assert seen == ["Camera-1"]


@pytest.mark.parametrize(
    "state", [CameraWorkerServiceState.LIVE, CameraWorkerServiceState.MDA]
)
def test_service_set_roi_refused_unless_idle(
    state: CameraWorkerServiceState,
) -> None:
    pool = _FakePool()
    svc = _service(pool)
    svc._state = state
    with pytest.raises(RuntimeError, match="stop Live/MDA"):
        svc.set_roi("Camera-1", 0, 0, 4, 4)
    assert pool.roi_calls == []


def test_service_exposure_and_property_path_drives_every_worker() -> None:
    pool = _FakePool()
    mmc = MagicMock()
    svc = _service(pool, mmc)
    seen: list[float] = []
    svc.exposureChanged.connect(seen.append)

    svc.set_exposure(25.5)

    assert pool.calls == [
        ("Camera-1", (("Exposure", "25.5"),)),
        ("Camera-2", (("Exposure", "25.5"),)),
    ]
    assert svc.last_known_exposure_ms == 25.5
    assert seen == [25.5]

    # the generic property path: worker cameras via the pool, others via the core
    pool.calls.clear()
    svc._set_camera_properties(
        {"Camera-1": [("Port", "Sensitivity")]},
        main_process=(("Dichroic", "Label", "400DCLP"),),
    )
    assert pool.calls == [("Camera-1", (("Port", "Sensitivity"),))]
    mmc.setProperty.assert_called_once_with("Dichroic", "Label", "400DCLP")


def test_service_property_change_stops_and_restarts_live(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pool = _FakePool()
    svc = _service(pool)
    svc._state = CameraWorkerServiceState.LIVE
    order: list[str] = []
    monkeypatch.setattr(svc, "stop_live", lambda: order.append("stop"))
    monkeypatch.setattr(svc, "start_live", lambda: order.append("start"))
    monkeypatch.setattr(
        pool, "set_properties", lambda label, values: order.append(label)
    )

    svc.set_exposure(10.0)

    assert order == ["stop", "Camera-1", "Camera-2", "start"]


def test_service_property_change_refused_during_mda() -> None:
    pool = _FakePool()
    svc = _service(pool)
    svc.last_known_exposure_ms = 10.0
    svc._state = CameraWorkerServiceState.MDA
    with pytest.raises(RuntimeError, match="MDA"):
        svc.set_exposure(50.0)
    assert pool.calls == []
    assert svc.last_known_exposure_ms == 10.0


# ---------------------------------------------------------------------------
# _mmcore_shutdown closes lasers
# ---------------------------------------------------------------------------


def test_shutdown_mmcore_closes_all_lasers_between_mda_cancel_and_unload() -> None:
    order: list[str] = []
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = False
    mmc.mda.is_running.return_value = False
    mmc.mda.cancel.side_effect = lambda: order.append("mda.cancel")
    mmc.unloadAllDevices.side_effect = lambda: order.append("unloadAllDevices")

    def _quiesce(_timeout: float) -> None:
        order.append("quiesce")

    with (
        patch(
            "pymmcore_gui.asi_z_stack.asi_controller.close_all_lasers",
            lambda: order.append("close_all_lasers"),
        ),
        patch("pymmcore_gui._mmcore_shutdown._quiesce_background_reads", _quiesce),
    ):
        shutdown_mmcore(mmc)

    assert order == ["mda.cancel", "close_all_lasers", "quiesce", "unloadAllDevices"]


def test_shutdown_mmcore_still_unloads_when_laser_close_fails() -> None:
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = False
    mmc.mda.is_running.return_value = False

    def _boom() -> None:
        raise RuntimeError("serial")

    with (
        patch("pymmcore_gui.asi_z_stack.asi_controller.close_all_lasers", _boom),
        patch("pymmcore_gui._mmcore_shutdown._quiesce_background_reads", MagicMock()),
    ):
        shutdown_mmcore(mmc)  # must not raise

    mmc.unloadAllDevices.assert_called_once_with()


def test_shutdown_mmcore_shuts_service_down_first() -> None:
    order: list[str] = []
    mmc = MagicMock()
    mmc.isSequenceRunning.side_effect = lambda: order.append("isSequenceRunning")
    mmc.mda.is_running.return_value = False
    svc = MagicMock()
    svc.shutdown.side_effect = lambda **_: order.append("service.shutdown")

    with (
        patch("pymmcore_gui.asi_z_stack.asi_controller.close_all_lasers", MagicMock()),
        patch("pymmcore_gui._mmcore_shutdown._quiesce_background_reads", MagicMock()),
    ):
        shutdown_mmcore(mmc, camera_worker_service=svc)

    assert order[:2] == ["service.shutdown", "isSequenceRunning"]
    svc.shutdown.assert_called_once()
