"""EPI MDA engine (v1): validation, LED/pool handling, metadata, wiring gates.

Hardware-free: the main core is a ``MagicMock`` wired to the ASITiger-shaped
``FakeLedCore``, the Orca service sits on a ``MagicMock`` pool (``spawn_async`` is
replaced, no process starts), and the stock summary metadata is stubbed.
"""

from __future__ import annotations

import json
import logging
import re
from inspect import signature
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock

import numpy as np
import pytest
import useq
from PyQt6.QtCore import QObject

from pymmcore_gui._argus_stream._session import StreamState
from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._qt.QtWidgets import QMessageBox
from pymmcore_gui._settings import EpiCameraSettingsV1, SettingsV1
from pymmcore_gui._vendored.mda_handlers import handler_for_path
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
)
from pymmcore_gui.asi_z_stack.engine import ASISPIMEngine
from pymmcore_gui.epi import _camera_service
from pymmcore_gui.epi._camera_service import (
    NOT_CONFIGURED,
    EpiCameraWorkerService,
    shutdown_epi_service,
)
from pymmcore_gui.epi._engine import (
    CRISP_LOCKED_STATES,
    EPI_CRISP_LABEL,
    PCT_UNITS,
    SEQUENCE_METADATA_KEY,
    EpiMDAEngine,
    EpiSequenceRefused,
    check_epi_sequence,
    with_epi_metadata,
)
from pymmcore_gui.epi._led import LED_INTENSITY_PROP
from pymmcore_gui.epi._mode import ScopeMode, set_active_controller
from pymmcore_gui.epi._tiger_guard import EPI_CHANNEL_GROUP, STAGE_Z_LABEL
from pymmcore_gui.widgets import _mda_widget
from pymmcore_gui.widgets._mda_widget import GuiMDAWidget, _epi_precheck
from test_argus_stream import (
    _feed_sequence,
    _full_settings,
    _NoopTunnel,
    _PlanCore,
    _prepare_session,
    _prepares,
    _save_seq,
    _StubCore,
    _wait,
    fake_receiver,  # noqa: F401
)
from test_asi_zstack_engine import _make_engine, _queue_frames
from test_epi_led import DECOY_X, TGLED, FakeLedCore
from test_opm_mode_pinning import _FakePool, _service

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

    from test_argus_stream import _FakeReceiver

ORCA = "Orca"
PRESET = "LED488"
IDLE = CameraWorkerServiceState.IDLE
MDA = CameraWorkerServiceState.MDA
FRAME_SHAPE = (16, 32)


class _EpiCtrl:
    mode = ScopeMode.EPI


class _OpmCtrl:
    mode = ScopeMode.OPM


class Rig:
    """A main core (``MagicMock``) with the TGLED, the EPI CRISP and stage Z."""

    def __init__(
        self,
        *,
        crisp: str | None = "Lock",
        labels: list[str] | None = None,
        stage_z: float = 123.4,
    ) -> None:
        self.led = FakeLedCore(labels=labels)
        self.crisp = crisp
        core = MagicMock()
        core.getLoadedDevices.return_value = [
            STAGE_Z_LABEL,
            *([EPI_CRISP_LABEL] if crisp is not None else []),
        ]
        core.getLoadedDevicesOfType.side_effect = self.led.getLoadedDevicesOfType
        core.getDeviceLibrary.side_effect = self.led.getDeviceLibrary
        core.setShutterOpen.side_effect = self.led.setShutterOpen
        core.setProperty.side_effect = self.led.setProperty
        core.getProperty.side_effect = self._get_property
        core.getPosition.return_value = stage_z
        core.getPixelSizeUm.return_value = 0.5
        core.getROI.return_value = (0, 0, 32, 16)
        core._last_xy_position = {}
        core._last_config = ("", "")
        self.core = core

    def _get_property(self, dev: str, prop: str) -> str:
        if prop == "CRISP State":
            assert dev == EPI_CRISP_LABEL
            assert self.crisp is not None
            return self.crisp
        return self.led.getProperty(dev, prop)

    def led_writes(self) -> list[tuple[Any, ...]]:
        return self.led.writes()


def _frames(*_a: object, **_k: object) -> Iterator[tuple[Any, ...]]:
    return iter([(ORCA, 0, np.zeros(FRAME_SHAPE, np.uint16), {"k": 1}, 0)])


def _camera() -> EpiCameraSettingsV1:
    return EpiCameraSettingsV1(
        label=ORCA,
        adapter_module="TestAdapter",
        device_name="TestCam",
        width=FRAME_SHAPE[1],
        height=FRAME_SHAPE[0],
        dtype="uint16",
    )


def _seq(loops: int = 2, **overrides: Any) -> useq.MDASequence:
    kwargs: dict[str, Any] = {
        "time_plan": {"interval": 0, "loops": loops},
        "channels": [{"config": PRESET, "group": EPI_CHANNEL_GROUP, "exposure": 20}],
    }
    kwargs.update(overrides)
    return useq.MDASequence(**kwargs)


@pytest.fixture(autouse=True)
def _reset(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    set_active_controller(None)
    monkeypatch.setattr(_camera_service, "_epi_service", None)
    monkeypatch.setattr(_camera_service, "_live_led", None)
    monkeypatch.setattr(EpiCameraWorkerService, "_active", None)
    monkeypatch.setattr(CameraWorkerService, "_active", None)
    yield
    set_active_controller(None)


@pytest.fixture(autouse=True)
def _stock_summary(monkeypatch: pytest.MonkeyPatch) -> None:
    """The stock engine reads a whole core for its summary; use a stub."""
    monkeypatch.setattr(
        "pymmcore_plus.mda._engine.summary_metadata",
        lambda *_a, **_k: {"format": "summary-dict", "version": "1.0"},
    )


@pytest.fixture
def epi_mode() -> None:
    set_active_controller(_EpiCtrl())


@pytest.fixture
def owner() -> Iterator[QObject]:
    parent = QObject()
    yield parent
    shutdown_epi_service(timeout=0.1)


@pytest.fixture
def pool() -> MagicMock:
    pool = MagicMock()
    pool.dead_workers.return_value = []
    pool.iter_frames.side_effect = _frames
    return pool


@pytest.fixture
def svc(
    owner: QObject, pool: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> EpiCameraWorkerService:
    """A registered, idle EPI service on *pool* (no process)."""
    monkeypatch.setattr(EpiCameraWorkerService, "spawn_async", lambda self, **_k: None)
    SettingsV1.instance().epi.camera = _camera()
    service = EpiCameraWorkerService(owner)
    service.begin(_camera())
    service._pool = pool
    service._state = IDLE
    _camera_service._epi_service = service
    return service


@pytest.fixture
def rig() -> Rig:
    return Rig()


@pytest.fixture
def engine(rig: Rig, svc: EpiCameraWorkerService, epi_mode: None) -> EpiMDAEngine:
    return EpiMDAEngine(rig.core)


def _run(
    engine: EpiMDAEngine, seq: useq.MDASequence
) -> tuple[Any, list[tuple[Any, Any, Any]]]:
    """setup_sequence, every event, teardown_sequence (the runner's order)."""
    summary = engine.setup_sequence(seq)
    out: list[tuple[Any, Any, Any]] = []
    try:
        for event in engine.event_iterator(seq):
            engine.setup_event(event)
            try:
                out.extend(engine.exec_event(event))
            finally:
                engine.teardown_event(event)
    finally:
        engine.teardown_sequence(seq)
    return summary, out


# ----------------------------------------------------------------- refusals


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"z_plan": useq.ZRangeAround(range=2, step=1)}, "single-plane"),
        (
            {
                "autofocus_plan": useq.AxesBasedAF(
                    autofocus_device_name="AF", axes=("t",)
                )
            },
            "autofocus",
        ),
        ({"grid_plan": useq.GridRowsColumns(rows=2, columns=2)}, "grid"),
        (
            {"stage_positions": [{"x": 0, "y": 0}, {"x": 1, "y": 1}]},
            "more than one XY position",
        ),
        ({"channels": [{"config": "488nm", "group": "Lasers"}]}, "'Lasers' group"),
        ({"channels": []}, "at least one channel"),
        (
            {"channels": [{"config": "LED999", "group": EPI_CHANNEL_GROUP}]},
            "not an LED channel",
        ),
        (  # channel 4 (405) is hidden in the default settings
            {"channels": [{"config": "LED405", "group": EPI_CHANNEL_GROUP}]},
            "hidden",
        ),
    ],
)
def test_setup_refuses_before_any_hardware_move(
    engine: EpiMDAEngine, rig: Rig, overrides: dict[str, Any], message: str
) -> None:
    seq = _seq(**overrides)
    assert message in (check_epi_sequence(rig.core, seq) or "")
    with pytest.raises(EpiSequenceRefused, match=message) as info:
        engine.setup_sequence(seq)
    assert info.value.persistent_notice is True
    assert rig.led_writes() == []
    rig.core.setConfig.assert_not_called()
    rig.core.setXYPosition.assert_not_called()
    rig.core.setZPosition.assert_not_called()


def test_setup_refuses_a_preset_whose_led_is_not_loaded(
    svc: EpiCameraWorkerService, epi_mode: None
) -> None:
    rig = Rig(labels=[*TGLED[1:], DECOY_X])  # channel 1 (488) is missing
    with pytest.raises(EpiSequenceRefused, match="not loaded"):
        EpiMDAEngine(rig.core).setup_sequence(_seq())
    assert rig.led_writes() == []


def test_the_decoy_x_led_never_counts_as_an_led(
    svc: EpiCameraWorkerService, epi_mode: None
) -> None:
    rig = Rig(labels=[DECOY_X])  # only LED:X:31 is loaded
    with pytest.raises(EpiSequenceRefused, match="not loaded"):
        EpiMDAEngine(rig.core).setup_sequence(_seq())
    assert rig.led_writes() == []


def test_setup_refuses_without_the_epi_service(rig: Rig, epi_mode: None) -> None:
    SettingsV1.instance().epi.camera = _camera()
    with pytest.raises(EpiSequenceRefused, match="not running"):
        EpiMDAEngine(rig.core).setup_sequence(_seq())


def test_setup_refuses_an_unconfigured_orca(rig: Rig, epi_mode: None) -> None:
    # the default is the Orca identity, so blank the camera explicitly
    SettingsV1.instance().epi.camera = EpiCameraSettingsV1(adapter_module="")
    with pytest.raises(EpiSequenceRefused, match=re.escape(NOT_CONFIGURED)):
        EpiMDAEngine(rig.core).setup_sequence(_seq())


def test_setup_refuses_while_the_orca_is_busy(
    engine: EpiMDAEngine, svc: EpiCameraWorkerService
) -> None:
    svc._state = CameraWorkerServiceState.SPAWNING
    with pytest.raises(EpiSequenceRefused, match="not ready"):
        engine.setup_sequence(_seq())


def test_setup_refuses_outside_epi_mode(rig: Rig, svc: EpiCameraWorkerService) -> None:
    set_active_controller(_OpmCtrl())
    with pytest.raises(EpiSequenceRefused, match="EPI mode"):
        EpiMDAEngine(rig.core).setup_sequence(_seq())
    assert rig.led_writes() == []


# ------------------------------------------------- D13: CRISP Z is never a refusal


def _crisp_warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    return [
        r.getMessage()
        for r in caplog.records
        if r.name == "pymmcore_gui"
        and r.levelno == logging.WARNING
        and "CRISP" in r.getMessage()
    ]


@pytest.mark.parametrize("state", ["Ready", "Idle", None])
def test_unlocked_crisp_z_warns_once_and_runs(
    svc: EpiCameraWorkerService,
    epi_mode: None,
    caplog: pytest.LogCaptureFixture,
    state: str | None,
) -> None:
    rig = Rig(crisp=state)  # None = CRISP Z not loaded
    engine = EpiMDAEngine(rig.core)
    with caplog.at_level(logging.WARNING, logger="pymmcore_gui"):
        summary, frames = _run(engine, _seq(loops=3))
    assert len(frames) == 3  # the run was not stopped
    warnings = _crisp_warnings(caplog)
    assert len(warnings) == 1  # one line at the start, none per time point
    assert "not loaded" in warnings[0] if state is None else state in warnings[0]
    assert summary["extra"]["epi"]["crisp_z_state"] == state
    assert state not in CRISP_LOCKED_STATES


@pytest.mark.parametrize("state", sorted(CRISP_LOCKED_STATES))
def test_locked_crisp_z_logs_nothing(
    svc: EpiCameraWorkerService,
    epi_mode: None,
    caplog: pytest.LogCaptureFixture,
    state: str,
) -> None:
    rig = Rig(crisp=state)
    with caplog.at_level(logging.WARNING, logger="pymmcore_gui"):
        _run(EpiMDAEngine(rig.core), _seq())
    assert _crisp_warnings(caplog) == []


def test_a_lost_lock_mid_run_is_logged_once_per_change_and_never_stops_the_run(
    engine: EpiMDAEngine, rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    seq = _seq(loops=4)
    states = iter(["Lock", "Ready", "Ready", "Lock"])
    engine.setup_sequence(seq)
    metas: list[Any] = []
    with caplog.at_level(logging.WARNING, logger="pymmcore_gui"):
        for event in engine.event_iterator(seq):
            rig.crisp = next(states)
            engine.setup_event(event)
            metas.extend(m for _i, _e, m in engine.exec_event(event))
            engine.teardown_event(event)
    engine.teardown_sequence(seq)
    assert [m["extra"]["epi"]["crisp_z_state"] for m in metas] == [
        "Lock",
        "Ready",
        "Ready",
        "Lock",
    ]
    # Lock -> Ready, then Ready -> Lock: two changes, four time points
    assert len(_crisp_warnings(caplog)) == 2
    assert len(metas) == 4


# ------------------------------------------------------------------ the run


def test_accepts_a_time_series_with_an_injected_position_and_never_sets_z(
    engine: EpiMDAEngine, rig: Rig
) -> None:
    # what MDAWidget.value() sends: the current position, with z, on every event
    seq = _seq(stage_positions=[{"x": 1.0, "y": 2.0, "z": 3.0}])
    assert all(e.z_pos == 3.0 for e in seq)
    assert check_epi_sequence(rig.core, seq) is None
    _summary, frames = _run(engine, seq)
    assert len(frames) == 2
    rig.core.setZPosition.assert_not_called()
    rig.core.setExposure.assert_not_called()
    assert not rig.core.setProperty.call_args_list or all(
        c.args[0] in TGLED for c in rig.core.setProperty.call_args_list
    )
    # the initial state is never restored (that would command z)
    assert engine.restore_initial_state is False


def test_the_engine_touches_nothing_but_the_led_and_the_channel_preset(
    engine: EpiMDAEngine, rig: Rig, svc: EpiCameraWorkerService
) -> None:
    _run(engine, _seq())
    touched = {c[0] for c in rig.core.method_calls}
    forbidden = {
        "setZPosition",
        "setPosition",
        "setRelativePosition",
        "setExposure",
        "setShutterDevice",
        "setFocusDevice",
        "setAutoFocusDevice",
        "enableContinuousFocus",
        "fullFocus",
    }
    assert not touched & forbidden
    # no write to any CRISP, stage, laser or PLogic property
    for call in rig.core.setProperty.call_args_list:
        assert call.args[0] in TGLED
    assert all(DECOY_X not in str(c) for c in rig.led.calls)


def test_the_led_is_lit_before_the_arm_and_dark_before_the_frame_is_yielded(
    engine: EpiMDAEngine, rig: Rig, pool: MagicMock, svc: EpiCameraWorkerService
) -> None:
    seen: dict[str, Any] = {}

    def arm(*_a: object, **kw: object) -> None:
        seen["lit_at_arm"] = rig.led.open[TGLED[0]]
        seen["state_at_arm"] = svc.state
        seen["kw"] = kw

    pool.arm_all.side_effect = arm
    seq = _seq(loops=1)
    engine.setup_sequence(seq)
    event = next(iter(seq))
    engine.setup_event(event)
    gen: Any = engine.exec_event(event)
    _img, _ev, _meta = next(iter(gen))
    assert seen["lit_at_arm"] is True
    assert seen["state_at_arm"] is MDA  # Snap/Live are refused during the run
    assert seen["kw"] == {
        "armed_timeout": svc.hw.worker_arm_timeout_s,
        "external_trigger": False,
        "exposure_ms": 20.0,
    }
    # LED already closed and the pool already stopped when the frame is handed over
    assert rig.led.open[TGLED[0]] is False
    pool.stop_and_drain.assert_called_once_with(
        timeout=svc.hw.worker_shutdown_timeout_s
    )
    gen.close()
    engine.teardown_sequence(seq)
    assert svc.state is IDLE


def test_uses_the_percent_snapshot_even_if_settings_change_mid_run(
    engine: EpiMDAEngine, rig: Rig
) -> None:
    leds = SettingsV1.instance().epi.leds
    leds[0].percent = 30
    seq = _seq(loops=2)
    engine.setup_sequence(seq)
    leds[0].percent = 80  # the user edits settings (or the panel) mid-run
    for event in engine.event_iterator(seq):
        engine.setup_event(event)
        list(engine.exec_event(event))
        engine.teardown_event(event)
    engine.teardown_sequence(seq)
    written = [
        c[3]
        for c in rig.led.calls
        if c[0] == "setProperty" and c[2] == LED_INTENSITY_PROP
    ]
    assert written == [30, 30]
    # percent is written before the shutter opens, every event
    opens = [c for c in rig.led_writes() if c[0] == "setShutterOpen" and c[2]]
    assert len(opens) == 2


@pytest.mark.parametrize("failure", ["arm", "frames"])
def test_led_closed_and_pool_drained_when_the_snap_raises(
    engine: EpiMDAEngine,
    rig: Rig,
    pool: MagicMock,
    svc: EpiCameraWorkerService,
    failure: str,
) -> None:
    if failure == "arm":
        pool.arm_all.side_effect = TimeoutError("armed ack")
    else:
        pool.iter_frames.side_effect = TimeoutError("stalled")
    seq = _seq(loops=1)
    engine.setup_sequence(seq)
    event = next(iter(seq))
    engine.setup_event(event)
    with pytest.raises(TimeoutError):
        list(engine.exec_event(event))
    assert rig.led.open[TGLED[0]] is False
    assert rig.led.calls[-1] == ("setShutterOpen", TGLED[0], False)
    pool.stop_and_drain.assert_called_once_with(
        timeout=svc.hw.worker_shutdown_timeout_s
    )
    engine.teardown_sequence(seq)
    assert svc.state is IDLE  # the claim is released even after a failure
    assert all(not v for v in rig.led.open.values())


def test_led_closed_when_opening_it_fails_halfway(
    engine: EpiMDAEngine, rig: Rig, pool: MagicMock
) -> None:
    seq = _seq(loops=1)
    engine.setup_sequence(seq)
    event = next(iter(seq))
    engine.setup_event(event)
    rig.led.fail_on = {TGLED[0]}  # setShutterOpen raises for this LED
    with pytest.raises(RuntimeError, match="serial error"):
        list(engine.exec_event(event))
    pool.arm_all.assert_not_called()
    pool.stop_and_drain.assert_called_once()
    assert ("setShutterOpen", TGLED[0], False) in rig.led.calls


def test_a_cancel_send_ends_the_event(engine: EpiMDAEngine) -> None:
    seq = _seq(loops=1)
    engine.setup_sequence(seq)
    event = next(iter(seq))
    engine.setup_event(event)
    gen: Any = iter(engine.exec_event(event))
    next(gen)
    with pytest.raises(StopIteration):
        gen.send("cancel")


def test_never_calls_acquire_for_mda(
    engine: EpiMDAEngine, monkeypatch: pytest.MonkeyPatch
) -> None:
    def forbidden(*_a: object, **_k: object) -> None:
        raise AssertionError("the EPI engine must not call acquire_for_mda")

    monkeypatch.setattr(CameraWorkerService, "acquire_for_mda", forbidden)
    _summary, frames = _run(engine, _seq())
    assert len(frames) == 2


def test_teardown_closes_every_led_and_releases_the_pool(
    engine: EpiMDAEngine, rig: Rig, svc: EpiCameraWorkerService
) -> None:
    seq = _seq(loops=1)
    engine.setup_sequence(seq)
    event = next(iter(seq))
    engine.setup_event(event)
    list(engine.exec_event(event))
    assert svc.state is MDA
    rig.led.calls.clear()
    engine.teardown_sequence(seq)
    closed = {c[1] for c in rig.led.calls if c[:1] == ("setShutterOpen",) and not c[2]}
    assert closed == set(TGLED)  # channel 4 too; never the X-axis LED
    assert svc.state is IDLE
    assert not any(c[0] == "setProperty" for c in rig.led.calls)  # no intensity


# ------------------------------------------------------------------ metadata


def test_frame_and_summary_meta_carry_extra_epi(engine: EpiMDAEngine, rig: Rig) -> None:
    SettingsV1.instance().epi.leds[0].percent = 42
    summary, frames = _run(engine, _seq())
    assert summary["extra"]["epi"] == {
        "leds": {
            PRESET: {
                "led": TGLED[0],
                "led_channel": 1,
                "led_name": "488",
                "led_pct": 42,
            }
        },
        "crisp_z_label": EPI_CRISP_LABEL,
        "crisp_z_state": "Lock",
        "stage_z_um": 123.4,
        "pct_units": PCT_UNITS,
    }
    assert PCT_UNITS == "card native, uncalibrated"
    for img, event, meta in frames:
        assert img.shape == FRAME_SHAPE
        assert meta["camera_device"] == ORCA
        assert meta["exposure_ms"] == 20.0
        assert meta["extra"]["epi"] == {
            "led": TGLED[0],
            "led_channel": 1,
            "led_name": "488",
            "led_pct": 42,
            "crisp_z_state": "Lock",
        }
        assert "cam" not in event.index  # single camera: no cam index
        assert event.channel is not None
        assert event.channel.config == PRESET


def test_summary_stage_z_is_none_when_the_stage_is_not_loaded(
    svc: EpiCameraWorkerService, epi_mode: None
) -> None:
    rig = Rig()
    rig.core.getLoadedDevices.return_value = [EPI_CRISP_LABEL]
    summary: Any = EpiMDAEngine(rig.core).setup_sequence(_seq())
    assert summary["extra"]["epi"]["stage_z_um"] is None
    rig.core.getPosition.assert_not_called()


def test_opm_engine_frame_meta_has_no_extra_epi() -> None:
    _core, opm, pool = _make_engine(n_cameras=1, n_slices=1)
    _queue_frames(pool, [("cam0", 0)])
    payloads = list(opm.exec_event(useq.MDAEvent(index={"t": 0})))
    assert len(payloads) == 1
    assert "extra" not in payloads[0][2]
    assert isinstance(opm, ASISPIMEngine)


# -------------------------------------------------- camera labels (saving gate)


def test_physical_camera_labels_in_epi_is_the_orca(
    svc: EpiCameraWorkerService, epi_mode: None
) -> None:
    mmc = MagicMock()
    assert physical_camera_labels(mmc) == [ORCA]
    mmc.getNumberOfCameraChannels.assert_not_called()


def test_physical_camera_labels_in_epi_without_a_service_is_empty_and_warns(
    epi_mode: None, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING, logger="pymmcore_gui"):
        assert physical_camera_labels(MagicMock()) == []
    assert any("no EPI camera service" in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize("controller", [None, _OpmCtrl()])
def test_physical_camera_labels_in_opm_is_unchanged(
    svc: EpiCameraWorkerService,
    monkeypatch: pytest.MonkeyPatch,
    controller: object | None,
) -> None:
    set_active_controller(controller)
    opm = _service(_FakePool())
    monkeypatch.setattr(CameraWorkerService, "_active", opm)
    assert physical_camera_labels(MagicMock()) == ["Camera-1", "Camera-2"]
    monkeypatch.setattr(CameraWorkerService, "_active", None)
    mmc = MagicMock()
    mmc.getNumberOfCameraChannels.return_value = 1
    mmc.getCameraDevice.return_value = "Camera"
    assert physical_camera_labels(mmc) == ["Camera"]  # the EPI service is ignored


# --------------------------------------------------------------------- Argus


def test_argus_streams_in_opm_and_skips_in_epi(
    fake_receiver: _FakeReceiver,  # noqa: F811
) -> None:
    settings = _full_settings(
        _argus(fake_receiver)  # enabled, pointing at the fake receiver
    )
    states: list[StreamState] = []

    def make() -> Any:
        from pymmcore_gui._argus_stream import ArgusStreamSession

        core: Any = _StubCore()
        tunnel: Any = _NoopTunnel()
        return ArgusStreamSession(
            core,
            tunnel,
            get_settings=lambda: settings,
            on_state_changed=lambda s, _d: states.append(s),
        )

    # EPI: skipped, nothing sent
    set_active_controller(_EpiCtrl())
    _feed_sequence(make(), _save_seq(z_plan=None))
    assert StreamState.SKIPPED in states
    assert fake_receiver.messages == []

    # OPM (no skip): the same run streams
    set_active_controller(_OpmCtrl())
    states.clear()
    _feed_sequence(make(), _save_seq(z_plan=useq.ZRangeAround(range=1, step=1)))
    assert _wait(lambda: len(fake_receiver.messages) > 0)
    assert StreamState.SKIPPED not in states


def _argus(receiver: _FakeReceiver) -> Any:
    from pymmcore_gui._settings import ArgusStreamSettingsV1

    return ArgusStreamSettingsV1(
        enabled=True, local_port=receiver.port, gpfs_scratch_root="/scratch"
    )


def test_argus_prepare_is_skipped_in_epi_only(
    fake_receiver: _FakeReceiver,  # noqa: F811
) -> None:
    session = _prepare_session(fake_receiver, _PlanCore())
    set_active_controller(_EpiCtrl())
    assert session.prepare(_save_seq()) is False
    assert _prepares(fake_receiver) == []
    set_active_controller(_OpmCtrl())
    assert session.prepare(_save_seq()) is True
    assert _wait(lambda: len(_prepares(fake_receiver)) == 1)


# ------------------------------------------------------------ MDA widget side


@pytest.fixture
def mda_widget(mmcore: CMMCorePlus, qtbot: QtBot) -> GuiMDAWidget:
    wdg = GuiMDAWidget(mmcore=mmcore)
    qtbot.addWidget(wdg)
    return wdg


@pytest.fixture
def leds_loaded(monkeypatch: pytest.MonkeyPatch) -> None:
    """The demo core has no TGLED: report channels 1-4 as loaded."""
    monkeypatch.setattr(
        "pymmcore_gui.epi._engine.resolve_led_channels",
        lambda _mmc, _addr="37": dict(enumerate(TGLED, start=1)),
    )


def _epi_widget_ctrl() -> Any:
    from types import SimpleNamespace

    return SimpleNamespace(mode=ScopeMode.EPI, epi_engine_factory=lambda m: None)


def test_epi_precheck_applies_the_v1_validation_in_epi_only(
    svc: EpiCameraWorkerService, leds_loaded: None, mmcore: CMMCorePlus
) -> None:
    set_active_controller(_epi_widget_ctrl())
    assert _epi_precheck(_seq(), mmcore) is None
    z = _seq(z_plan=useq.ZRangeAround(range=2, step=1))
    assert "single-plane" in (_epi_precheck(z, mmcore) or "")
    laser = useq.MDASequence(channels=[{"config": "488nm", "group": "Lasers"}])
    assert "'Lasers' group" in (_epi_precheck(laser, mmcore) or "")
    # OPM: the EPI validation is not consulted (a Z plan on lasers is fine)
    set_active_controller(_OpmCtrl())
    assert _epi_precheck(laser, mmcore) is None
    laser_z = useq.MDASequence(
        channels=[{"config": "488nm", "group": "Lasers"}],
        z_plan=useq.ZRangeAround(range=2, step=1),
    )
    assert _epi_precheck(laser_z, mmcore) is None


def test_execute_mda_in_epi_adds_the_led_map_to_the_sequence_metadata(
    mda_widget: GuiMDAWidget,
    mmcore: CMMCorePlus,
    svc: EpiCameraWorkerService,
    leds_loaded: None,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    SettingsV1.instance().epi.leds[0].percent = 65
    seq = _seq()
    monkeypatch.setattr(mda_widget, "value", lambda: seq)
    run = MagicMock()
    monkeypatch.setattr(mmcore, "run_mda", run)
    set_active_controller(_epi_widget_ctrl())

    mda_widget.execute_mda(None)

    (sent,) = run.call_args.args
    assert sent is not seq
    assert SEQUENCE_METADATA_KEY not in seq.metadata  # the original is untouched
    epi = sent.metadata[SEQUENCE_METADATA_KEY]["epi"]
    assert epi == {
        "leds": {
            PRESET: {
                "led": TGLED[0],
                "led_channel": 1,
                "led_name": "488",
                "led_pct": 65,
            }
        },
        "pct_units": PCT_UNITS,
    }
    # everything else about the sequence is unchanged
    assert [e.index for e in sent] == [e.index for e in seq]
    assert sent.channels == seq.channels
    assert sent.time_plan == seq.time_plan


def test_execute_mda_in_epi_refuses_a_bad_sequence_with_a_message(
    mda_widget: GuiMDAWidget,
    mmcore: CMMCorePlus,
    svc: EpiCameraWorkerService,
    leds_loaded: None,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    seq = _seq(z_plan=useq.ZRangeAround(range=2, step=1))
    monkeypatch.setattr(mda_widget, "value", lambda: seq)
    run = MagicMock()
    monkeypatch.setattr(mmcore, "run_mda", run)
    warning = MagicMock()
    monkeypatch.setattr(QMessageBox, "warning", warning)
    set_active_controller(_epi_widget_ctrl())
    mda_widget.execute_mda(None)
    run.assert_not_called()
    assert "single-plane" in warning.call_args.args[2]


@pytest.mark.parametrize("controller", [None, _OpmCtrl()])
def test_execute_mda_in_opm_leaves_the_sequence_untouched(
    mda_widget: GuiMDAWidget,
    mmcore: CMMCorePlus,
    monkeypatch: pytest.MonkeyPatch,
    controller: object | None,
) -> None:
    seq = useq.MDASequence(channels=[{"config": "488nm", "group": "Lasers"}])
    monkeypatch.setattr(mda_widget, "value", lambda: seq)
    run = MagicMock()
    monkeypatch.setattr(mmcore, "run_mda", run)
    set_active_controller(controller)
    mda_widget.execute_mda(None)
    run.assert_called_once_with(seq, output=None)
    assert run.call_args.args[0] is seq
    assert SEQUENCE_METADATA_KEY not in seq.metadata


def test_with_epi_metadata_keeps_other_metadata(
    svc: EpiCameraWorkerService, leds_loaded: None, epi_mode: None, rig: Rig
) -> None:
    seq = _seq(metadata={"pymmcore_widgets": {"format": "ome-zarr"}})
    out = with_epi_metadata(seq, rig.core)
    assert out.metadata["pymmcore_widgets"] == {"format": "ome-zarr"}
    assert out.metadata[SEQUENCE_METADATA_KEY]["epi"]["pct_units"] == PCT_UNITS


# ---------------------------------------------------------------- saving path


def test_epi_save_goes_through_the_single_camera_async_writer(
    mda_widget: GuiMDAWidget,
    mmcore: CMMCorePlus,
    svc: EpiCameraWorkerService,
    leds_loaded: None,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    seq = _seq()
    monkeypatch.setattr(mda_widget, "value", lambda: seq)
    run = MagicMock()
    monkeypatch.setattr(mmcore, "run_mda", run)
    set_active_controller(_epi_widget_ctrl())
    built: list[Any] = []

    class _RecordingWriter:
        def __init__(self, handler: object, **kwargs: object) -> None:
            built.append((handler, kwargs))

    class _NoMulti:
        def __init__(self, *_a: object, **_k: object) -> None:
            raise AssertionError("EPI is single-camera: no MultiCameraHandler")

    monkeypatch.setattr(_mda_widget, "AsyncWriter", _RecordingWriter)
    monkeypatch.setattr(_mda_widget, "MultiCameraHandler", _NoMulti)

    mda_widget.execute_mda(str(tmp_path / "epi.ome.zarr"))

    assert len(built) == 1
    assert isinstance(run.call_args.kwargs["output"], _RecordingWriter)
    # the labels the handler routes on are exactly [orca]
    assert physical_camera_labels(mmcore) == [ORCA]


def _write_through(
    handler: Any, seq: useq.MDASequence, frames: list[tuple[Any, Any, Any]]
) -> None:
    summary = {"format": "summary-dict", "version": "1.0"}
    if len(signature(handler.sequenceStarted).parameters) > 1:
        handler.sequenceStarted(seq, summary)
    else:
        handler.sequenceStarted(seq)
    for img, event, meta in frames:
        handler.frameReady(img, event, meta)
    handler.sequenceFinished(seq)


def test_single_camera_writers_receive_the_orca_frames_and_persist_extra(
    engine: EpiMDAEngine, tmp_path: Path
) -> None:
    """Which single-camera writers keep ``extra.epi`` (reported, not changed)."""
    seq = _seq(loops=2)
    _summary, frames = _run(engine, seq)

    # image sequence (a path with no known suffix): the full frame meta is
    # written to _frame_metadata.json, so extra.epi is persisted
    seq_dir = tmp_path / "seq"
    _write_through(handler_for_path(str(seq_dir)), seq, frames)
    meta_json = json.loads((seq_dir / "_frame_metadata.json").read_text())
    assert len(meta_json) == 2
    assert all(m["extra"]["epi"]["led"] == TGLED[0] for m in meta_json.values())
    assert len(list(seq_dir.glob("*.tif*"))) == 2

    # OME-TIFF: the writer stores no per-frame metadata, so extra.epi is NOT kept
    tiff = tmp_path / "run.ome.tiff"
    _write_through(handler_for_path(str(tiff)), seq, frames)
    assert tiff.exists()
    assert b"led_pct" not in tiff.read_bytes()


def test_the_zarr_writer_persists_extra_in_frame_meta(
    engine: EpiMDAEngine, tmp_path: Path
) -> None:
    import zarr

    seq = _seq(loops=2)
    _summary, frames = _run(engine, seq)
    path = tmp_path / "run.ome.zarr"
    _write_through(handler_for_path(str(path)), seq, frames)
    store = zarr.open(str(path), mode="r")
    arrays = (
        [store[k] for k in store.array_keys()] if hasattr(store, "array_keys") else []
    )
    metas = [a.attrs["frame_meta"] for a in arrays if "frame_meta" in a.attrs]
    assert metas
    assert metas[0][0]["extra"]["epi"]["led_pct"] == 50
