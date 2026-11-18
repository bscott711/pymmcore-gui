"""OPM <-> EPI mode switch: controller, safe transitions, guards, toolbar.

Hardware-free: a fake core (shaped like the one in ``test_epi_led``) records every
write, the camera worker service runs on a ``MagicMock`` pool, and the forbidden
Tiger helpers are replaced by functions that fail the test if called.
"""

from __future__ import annotations

import logging
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, cast
from unittest.mock import MagicMock, patch

import pytest
import useq
from pymmcore_plus import DeviceType
from pymmcore_plus.mda import MDAEngine
from PyQt6.QtWidgets import QToolBar

from pymmcore_gui._main_window import MicroManagerGUI
from pymmcore_gui._qt.QtWidgets import QMessageBox
from pymmcore_gui.actions.core_actions import (
    _prepare_for_reload,
    snap_image,
    toggle_live,
)
from pymmcore_gui.asi_z_stack import asi_controller, live_offset
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
)
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.engine import ASISPIMEngine
from pymmcore_gui.epi._led import LED_INTENSITY_PROP
from pymmcore_gui.epi._mode import (
    EpiModeRefusal,
    ModeController,
    ScopeMode,
    current_mode,
    set_active_controller,
)
from pymmcore_gui.epi._tiger_guard import EPI_CHANNEL_GROUP
from pymmcore_gui.widgets import _crisp_widget
from pymmcore_gui.widgets._galvo_control import OFFSET_PROP, GalvoControlWidget
from pymmcore_gui.widgets._led_control import LedControlWidget
from pymmcore_gui.widgets._mda_widget import GuiMDAWidget, _epi_precheck
from test_epi_led import DECOY_X, TGLED
from test_opm_mode_pinning import _FakePool, _service

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

    from pymmcore_gui._settings import Settings

GALVO = HardwareConstants().galvo_a_label
CRISP_Z = "CRISPAFocus:Z:32"
CRISP_P = "CRISPAFocus:P:34"
PIEZO = "PiezoStage:P:34"
ZSTAGE = "ZStage:Z:32"
NO_TGLED = "Load 20261118-OPM_EPI.cfg to use EPI"


class FakeTigerCore:
    """The slice of CMMCorePlus the mode switch uses, recording every write.

    Holds the four TGLED channels plus the unrelated ``LED:X:31`` decoy, the
    OPM Tiger devices the snapshot reads, and the Core roles. ``events`` and
    ``mda`` come from a real core so widgets and the engine swap behave normally.
    """

    def __init__(self, real: CMMCorePlus, labels: list[str] | None = None) -> None:
        self.events = real.events
        self.mda = real.mda
        self.labels = [*TGLED, DECOY_X] if labels is None else labels
        self.devices = [GALVO, CRISP_Z, CRISP_P, PIEZO, ZSTAGE, "PLogic:E:36"]
        self.props: dict[tuple[str, str], str] = {
            (CRISP_Z, "CRISP State"): "Ready",
            (CRISP_P, "CRISP State"): "Lock",
            (GALVO, "SPIMState"): "Idle",
            (GALVO, "BeamEnabled"): "Yes",
            (GALVO, OFFSET_PROP): "-0.260000",
        }
        self.roles: dict[str, Any] = {
            "focus": PIEZO,
            "shutter": "PLogic:E:36",
            "channel_group": "Channel",
            "autoshutter": True,
        }
        self.groups = ["Channel", "Lasers", EPI_CHANNEL_GROUP]
        self.open = dict.fromkeys(self.labels, False)
        self.writes: list[tuple[Any, ...]] = []

    # ---- discovery
    def getLoadedDevices(self) -> list[str]:
        return [*self.labels, *self.devices]

    def getLoadedDevicesOfType(self, dev_type: Any) -> list[str]:
        return [*self.labels, "Shutter"] if dev_type == DeviceType.ShutterDevice else []

    def getDeviceLibrary(self, label: str) -> str:
        return "ASITiger" if label in self.labels else "DemoCamera"

    # ---- reads
    def hasProperty(self, dev: str, prop: str) -> bool:
        return (dev, prop) in self.props

    def getProperty(self, dev: str, prop: str) -> str:
        return self.props[dev, prop]

    def getPosition(self, dev: str) -> float:
        return 0.0

    def getFocusDevice(self) -> str:
        return str(self.roles["focus"])

    def getShutterDevice(self) -> str:
        return str(self.roles["shutter"])

    def getChannelGroup(self) -> str:
        return str(self.roles["channel_group"])

    def getAutoShutter(self) -> bool:
        return bool(self.roles["autoshutter"])

    def getAvailableConfigGroups(self) -> list[str]:
        return list(self.groups)

    def getShutterOpen(self, label: str) -> bool:
        return self.open[label]

    # ---- writes (recorded)
    def setShutterOpen(self, label: str, state: bool) -> None:
        self.writes.append(("setShutterOpen", label, state))
        self.open[label] = state

    def setChannelGroup(self, group: str) -> None:
        self.writes.append(("setChannelGroup", group))
        self.roles["channel_group"] = group

    def setFocusDevice(self, label: str) -> None:
        self.writes.append(("setFocusDevice", label))
        self.roles["focus"] = label

    def setShutterDevice(self, label: str) -> None:
        self.writes.append(("setShutterDevice", label))
        self.roles["shutter"] = label

    def setAutoShutter(self, state: bool) -> None:
        self.writes.append(("setAutoShutter", state))
        self.roles["autoshutter"] = state

    def setProperty(self, dev: str, prop: str, value: Any) -> None:
        self.writes.append(("setProperty", dev, prop, value))
        self.props[dev, prop] = str(value)

    def setConfig(self, group: str, preset: str) -> None:
        self.writes.append(("setConfig", group, preset))


class _StubEpiEngine(MDAEngine):
    """Stands in for the Step 4 EPI engine."""


class _OpmCtrl:
    mode = ScopeMode.OPM


class _EpiCtrl:
    mode = ScopeMode.EPI


@pytest.fixture(autouse=True)
def _reset_mode() -> Iterator[None]:
    set_active_controller(None)
    yield
    set_active_controller(None)


@pytest.fixture(autouse=True)
def lasers(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """Record laser closes; make every forbidden Tiger helper fail the test."""
    calls: list[str] = []
    monkeypatch.setattr(
        asi_controller, "close_all_lasers", lambda: calls.append("close_all")
    )
    monkeypatch.setattr(
        asi_controller, "open_selected_lasers", lambda: calls.append("open")
    )
    monkeypatch.setattr(
        asi_controller, "close_selected_lasers", lambda: calls.append("close")
    )

    def _forbidden(name: str) -> Any:
        def _fail(*_a: Any, **_k: Any) -> None:
            raise AssertionError(f"a mode switch must never call {name}")

        return _fail

    for name in (
        "ensure_global_shutter_open",
        "configure_plogic_for_dual_nrt_pulses",
        "configure_plogic_for_delayed_laser",
        "set_plogic_evaluation_clock",
        "set_property",
        "open_global_shutter",
    ):
        monkeypatch.setattr(asi_controller, name, _forbidden(name))
    return calls


@pytest.fixture
def restored_offsets(monkeypatch: pytest.MonkeyPatch) -> list[Any]:
    calls: list[Any] = []
    monkeypatch.setattr(
        live_offset, "restore_live_offset", lambda mmc, *a, **k: calls.append(mmc)
    )
    return calls


@pytest.fixture
def opm_engine(mmcore: CMMCorePlus) -> Iterator[ASISPIMEngine]:
    """The OPM engine installed on the shared core, as on the rig."""
    engine = ASISPIMEngine(mmcore, HardwareConstants())
    mmcore.mda.set_engine(engine)
    try:
        yield engine
    finally:
        mmcore.mda.set_engine(MDAEngine(mmcore))


@pytest.fixture
def fake(mmcore: CMMCorePlus) -> FakeTigerCore:
    return FakeTigerCore(mmcore)


def _controller(
    mmcore: CMMCorePlus, fake: FakeTigerCore, *, factory: bool = True
) -> ModeController:
    """A controller on *fake*; reinstalling the OPM engine as the window does."""

    def register() -> None:
        with patch.object(
            asi_controller, "asi_zstack_hardware_available", lambda: True
        ):
            MicroManagerGUI._register_mda_engine(SimpleNamespace(_mmc=mmcore))  # type: ignore[arg-type]

    ctrl = ModeController(fake, register_opm_engine=register)  # type: ignore[arg-type]
    if factory:
        ctrl.epi_engine_factory = lambda m: _StubEpiEngine(mmcore)
    return ctrl


def _svc(state: CameraWorkerServiceState) -> tuple[CameraWorkerService, MagicMock]:
    pool = MagicMock()
    pool.dead_workers.return_value = []
    svc = _service(pool)
    svc._state = state
    return svc, pool


# ----------------------------------------------------------------- the round trip


def test_round_trip_makes_only_the_expected_core_writes(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    lasers: list[str],
    restored_offsets: list[Any],
) -> None:
    before = dict(fake.roles)
    ctrl = _controller(mmcore, fake)
    seen: list[ScopeMode] = []
    ctrl.modeChanged.connect(seen.append)
    assert ctrl.mode is ScopeMode.OPM

    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert current_mode() is ScopeMode.EPI
    assert isinstance(mmcore.mda.engine, _StubEpiEngine)
    assert fake.roles["channel_group"] == EPI_CHANNEL_GROUP

    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert ctrl.mode is ScopeMode.OPM
    assert seen == [ScopeMode.EPI, ScopeMode.OPM]

    leds_off = [("setShutterOpen", label, False) for label in TGLED]
    assert fake.writes == [
        ("setChannelGroup", EPI_CHANNEL_GROUP),
        *leds_off,  # enter: every TGLED channel dark, nothing else
        *leds_off,  # exit: LEDs first ...
        ("setChannelGroup", "Channel"),  # ... then the channel group back
    ]
    # the unrelated X-axis LED is never touched
    assert all(DECOY_X not in w for w in fake.writes)
    # lasers closed on entry and on exit
    assert lasers == ["close_all", "close_all"]
    # Core roles equal the snapshot; the unchanged galvo offset is not rewritten
    assert fake.roles == before
    assert restored_offsets == []
    # the OPM engine is back, as a fresh ASISPIMEngine
    assert isinstance(mmcore.mda.engine, ASISPIMEngine)


def test_switch_never_writes_crisp_joystick_or_properties(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert ctrl.request_mode(ScopeMode.OPM) is None
    kinds = {w[0] for w in fake.writes}
    assert kinds <= {"setShutterOpen", "setChannelGroup"}
    # nothing touched a CRISP, the stage, the piezo or any joystick property
    touched = {w[1] for w in fake.writes if len(w) > 2 and isinstance(w[1], str)}
    assert not touched & {CRISP_Z, CRISP_P, PIEZO, ZSTAGE, GALVO}
    # (the Tiger helpers that write CRISP/PLogic/joystick state fail the test if
    # called -- see the ``lasers`` fixture)


def test_pool_sees_no_calls_during_a_switch(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    svc, pool = _svc(CameraWorkerServiceState.IDLE)
    monkeypatch.setattr(CameraWorkerService, "_active", svc)
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert pool.mock_calls == []
    assert svc.state is CameraWorkerServiceState.IDLE


def test_without_an_engine_factory_no_engine_swap_happens(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake, factory=False)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert mmcore.mda.engine is opm_engine
    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert mmcore.mda.engine is opm_engine  # not even re-registered


def test_only_changes_to_a_new_mode_emit(
    mmcore: CMMCorePlus, fake: FakeTigerCore
) -> None:
    ctrl = _controller(mmcore, fake)
    seen: list[ScopeMode] = []
    ctrl.modeChanged.connect(seen.append)
    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert seen == []
    assert fake.writes == []


def test_exit_restores_drifted_core_roles_and_warns(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    caplog: pytest.LogCaptureFixture,
) -> None:
    before = dict(fake.roles)
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    # Property Browser / "Set as default" / Console changed the roles in EPI
    fake.roles.update(focus=ZSTAGE, shutter=TGLED[0], autoshutter=False)
    with caplog.at_level(logging.WARNING, logger="pymmcore_gui"):
        assert ctrl.request_mode(ScopeMode.OPM) is None
    assert fake.roles == before
    assert "Core Focus changed during EPI" in caplog.text
    assert "Core Shutter changed during EPI" in caplog.text
    assert "Core AutoShutter changed during EPI" in caplog.text


def test_exit_reapplies_the_live_offset_only_when_it_moved(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    restored_offsets: list[Any],
) -> None:
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    fake.props[GALVO, OFFSET_PROP] = "0.500000"
    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert restored_offsets == [fake]


def test_failed_entry_is_rolled_back_and_refused(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake)

    def _boom(_m: object) -> MDAEngine:
        raise RuntimeError("no engine")

    ctrl.epi_engine_factory = _boom
    reason = ctrl.request_mode(ScopeMode.EPI)
    assert reason is not None
    assert "no engine" in reason
    assert ctrl.mode is ScopeMode.OPM
    assert fake.roles["channel_group"] == "Channel"
    assert mmcore.mda.engine is opm_engine


# ------------------------------------------------------------------- refusals


def test_refused_while_an_mda_is_running(
    mmcore: CMMCorePlus, fake: FakeTigerCore
) -> None:
    ctrl = _controller(mmcore, fake)
    fake.mda = SimpleNamespace(is_running=lambda: True)  # type: ignore[assignment]
    reason = ctrl.request_mode(ScopeMode.EPI)
    assert reason is not None
    assert "running acquisition" in reason
    assert ctrl.mode is ScopeMode.OPM
    assert fake.writes == []


@pytest.mark.parametrize(
    "state", [CameraWorkerServiceState.SPAWNING, CameraWorkerServiceState.MDA]
)
def test_refused_while_the_opm_service_is_busy(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    monkeypatch: pytest.MonkeyPatch,
    state: CameraWorkerServiceState,
) -> None:
    svc, pool = _svc(state)
    monkeypatch.setattr(CameraWorkerService, "_active", svc)
    ctrl = _controller(mmcore, fake)
    reason = ctrl.request_mode(ScopeMode.EPI)
    assert reason is not None
    assert state.name in reason
    assert ctrl.mode is ScopeMode.OPM
    assert fake.writes == []
    assert pool.mock_calls == []


def test_opm_live_is_stopped_first(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    svc, _pool = _svc(CameraWorkerServiceState.LIVE)
    order: list[str] = []

    def _stop() -> None:
        order.append("stop_live")
        svc._state = CameraWorkerServiceState.IDLE

    monkeypatch.setattr(svc, "stop_live", _stop)
    monkeypatch.setattr(CameraWorkerService, "_active", svc)
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert order == ["stop_live"]


def test_leaving_epi_refused_while_the_epi_service_is_busy(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake)
    epi_svc = SimpleNamespace(state=CameraWorkerServiceState.MDA)
    ctrl.epi_service_provider = lambda: epi_svc
    assert ctrl.request_mode(ScopeMode.EPI) is None
    reason = ctrl.request_mode(ScopeMode.OPM)
    assert reason is not None
    assert "EPI" in reason
    assert ctrl.mode is ScopeMode.EPI


def test_extra_preconditions_can_refuse(
    mmcore: CMMCorePlus, fake: FakeTigerCore
) -> None:
    ctrl = _controller(mmcore, fake)
    ctrl.extra_preconditions.append(lambda: "Orca settings missing")
    assert ctrl.request_mode(ScopeMode.EPI) == "Orca settings missing"
    assert ctrl.mode is ScopeMode.OPM
    assert fake.writes == []


def test_refused_without_a_tgled(mmcore: CMMCorePlus) -> None:
    fake = FakeTigerCore(mmcore, labels=[])
    ctrl = _controller(mmcore, fake)
    reason = ctrl.request_mode(ScopeMode.EPI)
    assert reason is not None
    assert reason.startswith(NO_TGLED)
    assert "no TGLED at address 37" in reason
    assert fake.writes == []


def test_refused_with_only_the_x_axis_led_loaded(mmcore: CMMCorePlus) -> None:
    fake = FakeTigerCore(mmcore, labels=[DECOY_X])
    ctrl = _controller(mmcore, fake)
    reason = ctrl.request_mode(ScopeMode.EPI)
    assert reason is not None
    assert reason.startswith(NO_TGLED)
    assert ctrl.mode is ScopeMode.OPM
    assert fake.writes == []  # LED:X:31 never opened or closed


def test_refused_when_no_loaded_channel_is_visible(
    mmcore: CMMCorePlus, settings: Settings
) -> None:
    for led in settings.epi.leds:
        led.visible = led.channel != 1
    fake = FakeTigerCore(mmcore, labels=[TGLED[0]])  # only channel 1 is loaded
    ctrl = _controller(mmcore, fake)
    reason = ctrl.request_mode(ScopeMode.EPI)
    assert reason is not None
    assert reason.startswith(NO_TGLED)


def test_enter_without_an_epi_group_leaves_the_channel_group(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    fake.groups.remove(EPI_CHANNEL_GROUP)
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert ("setChannelGroup", EPI_CHANNEL_GROUP) not in fake.writes
    assert fake.roles["channel_group"] == "Channel"


# ------------------------------------------------------------- config-load reset


def test_prepare_for_reload_in_epi_ends_in_opm_with_the_opm_engine(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert isinstance(mmcore.mda.engine, _StubEpiEngine)
    seen: list[ScopeMode] = []
    ctrl.modeChanged.connect(seen.append)

    assert _prepare_for_reload(fake) is True  # type: ignore[arg-type]

    assert ctrl.mode is ScopeMode.OPM
    assert current_mode() is ScopeMode.OPM
    assert isinstance(mmcore.mda.engine, ASISPIMEngine)
    assert seen == [ScopeMode.OPM]


def test_prepare_for_reload_in_opm_changes_nothing(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    _controller(mmcore, fake)
    assert _prepare_for_reload(fake) is True  # type: ignore[arg-type]
    assert fake.writes == []
    assert mmcore.mda.engine is opm_engine


def test_force_opm_tolerates_unloaded_led_devices(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    caplog: pytest.LogCaptureFixture,
) -> None:
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    fake.labels.clear()  # the reload already unloaded every device
    fake.devices.clear()
    fake.writes.clear()
    ctrl.force_opm()  # must not raise
    assert ctrl.mode is ScopeMode.OPM
    assert isinstance(mmcore.mda.engine, ASISPIMEngine)
    # nothing from the old config is restored onto the new one
    assert fake.writes == []


def test_force_opm_never_raises_even_if_the_core_fails(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None

    def _boom(*_a: Any) -> Any:
        raise RuntimeError("core gone")

    fake.getLoadedDevicesOfType = _boom  # type: ignore[assignment,method-assign]
    ctrl.force_opm()
    assert ctrl.mode is ScopeMode.OPM


def test_force_opm_waits_for_a_running_mda(
    mmcore: CMMCorePlus, fake: FakeTigerCore, opm_engine: ASISPIMEngine
) -> None:
    ctrl = _controller(mmcore, fake)
    assert ctrl.request_mode(ScopeMode.EPI) is None
    fake.mda = SimpleNamespace(is_running=lambda: True)  # type: ignore[assignment]
    ctrl.force_opm()
    assert ctrl.mode is ScopeMode.EPI  # the engine cannot be swapped mid-run


@pytest.fixture
def gui(qtbot: QtBot, monkeypatch: pytest.MonkeyPatch) -> Iterator[MicroManagerGUI]:
    # The window's own config-load handler runs the session writers (no-ops
    # without PLogic); they are not part of a mode switch, so lift the guard the
    # ``lasers`` fixture put on them.
    monkeypatch.setattr(asi_controller, "ensure_global_shutter_open", lambda: None)
    window = MicroManagerGUI()
    qtbot.addWidget(window)
    yield window


def test_direct_config_load_in_epi_ends_in_opm(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    """The Config Wizard / Console path: loadSystemConfiguration, no prepare step."""
    mmc = gui.mmcore
    stub = _StubEpiEngine(mmc)
    previous = mmc.mda.engine
    ctrl = gui.mode_controller
    ctrl._mode = ScopeMode.EPI
    mmc.mda.set_engine(stub)
    from pymmcore_gui.epi._tiger_guard import TigerSnapshot

    ctrl._snapshot = TigerSnapshot(
        previous_engine=previous, epi_engine=stub, engine_swapped=True
    )
    try:
        with qtbot.waitSignal(mmc.events.systemConfigurationLoaded):
            mmc.loadSystemConfiguration()
        assert ctrl.mode is ScopeMode.OPM
        assert current_mode() is ScopeMode.OPM
        # the demo config has no ASI hardware, so the stock engine is back
        # (with ASI hardware _register_mda_engine installs ASISPIMEngine; see
        # test_prepare_for_reload_in_epi_ends_in_opm_with_the_opm_engine)
        assert not isinstance(mmc.mda.engine, _StubEpiEngine)
        assert type(mmc.mda.engine) is MDAEngine
    finally:
        mmc.mda.set_engine(MDAEngine(mmc))


# ------------------------------------------------------------------ service guards

_GUARDED: dict[str, Any] = {
    "start_live": lambda svc: svc.start_live(),
    "snap": lambda svc: svc.snap(),
    "acquire_for_mda": lambda svc: svc.acquire_for_mda(),
    "set_camera_properties": lambda svc: svc._set_camera_properties(
        {"Camera-1": [("Port", "Speed")]}
    ),
    "set_exposure": lambda svc: svc.set_exposure(5.0),
    "set_roi": lambda svc: svc.set_roi("Camera-1", 0, 0, 4, 4),
}


@pytest.mark.parametrize("name", list(_GUARDED))
def test_guarded_service_methods_refuse_in_epi_without_touching_the_pool(
    name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    svc, pool = _svc(CameraWorkerServiceState.IDLE)
    set_active_controller(_EpiCtrl())
    with pytest.raises(EpiModeRefusal, match="not available in EPI"):
        _GUARDED[name](svc)
    assert pool.mock_calls == []
    assert svc.state is CameraWorkerServiceState.IDLE
    # it is a RuntimeError, so existing refusal handlers cover it
    assert issubclass(EpiModeRefusal, RuntimeError)


@pytest.mark.parametrize("controller", [None, _OpmCtrl()])
def test_guarded_service_methods_are_unchanged_in_opm(
    controller: object | None,
) -> None:
    set_active_controller(controller)
    pool = _FakePool()
    svc = _service(pool)
    assert svc.set_roi("Camera-1", 1, 2, 4, 3) == (1, 2, 4, 3)
    svc.set_exposure(25.5)
    assert [c[0] for c in pool.calls] == ["Camera-1", "Camera-2"]
    assert pool.roi_calls == [("Camera-1", 1, 2, 4, 3, 5.0)]


def test_a_subclass_for_epi_is_not_refused() -> None:
    class _EpiService(CameraWorkerService):
        _ALLOW_IN_EPI = True

    pool = _FakePool()
    svc = _EpiService()
    svc._pool = pool  # type: ignore[assignment]
    svc._state = CameraWorkerServiceState.IDLE
    svc._mmc = MagicMock()
    set_active_controller(_EpiCtrl())
    assert svc.set_roi("Orca", 0, 0, 8, 8) == (0, 0, 8, 8)
    assert pool.roi_calls == [("Orca", 0, 0, 8, 8, 5.0)]
    # the base service stays refused
    base, _ = _svc(CameraWorkerServiceState.IDLE)
    with pytest.raises(EpiModeRefusal):
        base.set_roi("Camera-1", 0, 0, 8, 8)


def test_stop_live_and_shutdown_are_not_guarded_in_epi() -> None:
    svc, _pool = _svc(CameraWorkerServiceState.IDLE)
    set_active_controller(_EpiCtrl())
    svc.stop_live()  # needed to leave a mode; must not refuse
    svc.release_from_mda()


# ----------------------------------------------------------------- galvo guard


@pytest.fixture
def galvo(
    mmcore: CMMCorePlus, monkeypatch: pytest.MonkeyPatch
) -> tuple[CMMCorePlus, list[Any]]:
    writes: list[Any] = []
    real_devices = mmcore.getLoadedDevices
    monkeypatch.setattr(mmcore, "getLoadedDevices", lambda: (*real_devices(), GALVO))
    monkeypatch.setattr(mmcore, "getProperty", lambda d, p: "0.0")
    monkeypatch.setattr(mmcore, "hasPropertyLimits", lambda d, p: False)
    monkeypatch.setattr(mmcore, "hasProperty", lambda d, p: True)
    monkeypatch.setattr(mmcore, "setProperty", lambda d, p, v: writes.append(v))
    monkeypatch.setattr(mmcore, "setGalvoPosition", lambda d, x, y: None)
    monkeypatch.setattr(mmcore, "getGalvoPosition", lambda d: (0.0, 0.0))
    return mmcore, writes


def test_galvo_offset_refused_in_epi_and_nothing_saved(
    galvo: tuple[CMMCorePlus, list[Any]], qtbot: QtBot, settings: Settings
) -> None:
    core, writes = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    saved = settings.galvo.live_offset_deg
    set_active_controller(_EpiCtrl())
    w.set_offset_deg(1.0)
    w.zero.click()
    w.plus.click()
    assert writes == []
    assert settings.galvo.live_offset_deg == saved


def test_galvo_offset_unchanged_in_opm(
    galvo: tuple[CMMCorePlus, list[Any]], qtbot: QtBot, settings: Settings
) -> None:
    core, writes = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    set_active_controller(_OpmCtrl())
    w.set_offset_deg(1.0)
    assert writes == ["1.000000"]
    assert settings.galvo.live_offset_deg == 1.0


# --------------------------------------------------------- Snap / Live / CRISPy


def _action(window: object | None = None) -> SimpleNamespace:
    return SimpleNamespace(
        mmc=MagicMock(), setChecked=MagicMock(), parent=lambda: window
    )


def test_snap_refused_in_epi_without_lasers_or_cameras(
    lasers: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    svc = MagicMock()
    monkeypatch.setattr(CameraWorkerService, "_active", svc)
    window = SimpleNamespace(nm=MagicMock())
    action = _action(window)
    set_active_controller(_EpiCtrl())
    snap_image(action, False)  # type: ignore[arg-type]
    assert lasers == []
    svc.snap.assert_not_called()
    action.mmc.snapImage.assert_not_called()
    window.nm.show_warning_message.assert_called_once()
    assert "EPI" in window.nm.show_warning_message.call_args.args[0]


def test_live_refused_in_epi_and_the_button_is_unchecked(
    lasers: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    svc = MagicMock()
    monkeypatch.setattr(CameraWorkerService, "_active", svc)
    window = SimpleNamespace(nm=MagicMock())
    action = _action(window)
    set_active_controller(_EpiCtrl())
    toggle_live(action, True)  # type: ignore[arg-type]
    assert lasers == []
    svc.start_live.assert_not_called()
    action.mmc.startContinuousSequenceAcquisition.assert_not_called()
    action.setChecked.assert_called_once_with(False)
    window.nm.show_warning_message.assert_called_once()


def test_snap_and_live_still_use_the_lasers_in_opm_with_a_controller(
    lasers: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[str] = []
    svc = SimpleNamespace(
        state=CameraWorkerServiceState.IDLE,
        snap=lambda: calls.append("snap"),
        start_live=lambda: calls.append("live"),
    )
    monkeypatch.setattr(CameraWorkerService, "_active", svc)
    set_active_controller(_OpmCtrl())
    snap_image(_action(), False)  # type: ignore[arg-type]
    toggle_live(_action(), True)  # type: ignore[arg-type]
    assert lasers == ["open", "close", "open"]
    assert calls == ["snap", "live"]


def test_crispy_snap_refused_in_epi(
    lasers: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    svc = MagicMock()
    monkeypatch.setattr(CameraWorkerService, "get_active", lambda: svc)
    mmc = MagicMock()
    set_active_controller(_EpiCtrl())
    with pytest.raises(EpiModeRefusal):
        _crisp_widget.snap_for_crispy(mmc)
    assert lasers == []
    svc.snap.assert_not_called()
    mmc.snapImage.assert_not_called()


def test_crispy_snap_unchanged_in_opm(
    lasers: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    svc = MagicMock()
    svc.snap.return_value = {"Camera-1": object()}
    monkeypatch.setattr(CameraWorkerService, "get_active", lambda: svc)
    set_active_controller(_OpmCtrl())
    assert _crisp_widget.snap_for_crispy(MagicMock()) is svc.snap.return_value
    assert lasers == ["open", "close"]


# ------------------------------------------------------------------- execute_mda


def _led_sequence() -> useq.MDASequence:
    return useq.MDASequence(channels=[{"config": "LED488", "group": EPI_CHANNEL_GROUP}])


def _laser_sequence() -> useq.MDASequence:
    return useq.MDASequence(channels=[{"config": "488nm", "group": "Lasers"}])


def test_epi_precheck_values() -> None:
    # no controller, and an OPM controller: only EPI LEDs channels are refused
    assert _epi_precheck(_laser_sequence()) is None
    assert _epi_precheck(_led_sequence()) is not None
    set_active_controller(_OpmCtrl())
    assert _epi_precheck(_laser_sequence()) is None
    assert "EPI mode" in (_epi_precheck(_led_sequence()) or "")
    # EPI: refused until the engine factory exists (Step 4)
    set_active_controller(_EpiCtrl())
    assert _epi_precheck(_led_sequence()) == "EPI MDA is not available yet."
    ctrl = SimpleNamespace(mode=ScopeMode.EPI, epi_engine_factory=lambda m: None)
    set_active_controller(ctrl)
    assert _epi_precheck(_led_sequence()) is None


@pytest.fixture
def mda_widget(mmcore: CMMCorePlus, qtbot: QtBot) -> GuiMDAWidget:
    wdg = GuiMDAWidget(mmcore=mmcore)
    qtbot.addWidget(wdg)
    return wdg


@pytest.mark.parametrize(
    ("controller", "sequence", "runs"),
    [
        (None, _laser_sequence, True),
        (_OpmCtrl(), _laser_sequence, True),
        (_OpmCtrl(), _led_sequence, False),  # EPI LEDs channel in OPM
        (_EpiCtrl(), _laser_sequence, False),  # no EPI engine yet
        (_EpiCtrl(), _led_sequence, False),
        (
            SimpleNamespace(mode=ScopeMode.EPI, epi_engine_factory=lambda m: None),
            _led_sequence,
            True,  # Step 4 hooked the engine up
        ),
    ],
)
def test_execute_mda_both_sides(
    mda_widget: GuiMDAWidget,
    mmcore: CMMCorePlus,
    monkeypatch: pytest.MonkeyPatch,
    controller: object | None,
    sequence: Any,
    runs: bool,
) -> None:
    seq = sequence()
    monkeypatch.setattr(mda_widget, "value", lambda: seq)
    run = MagicMock()
    monkeypatch.setattr(mmcore, "run_mda", run)
    warning = MagicMock()
    monkeypatch.setattr(QMessageBox, "warning", warning)
    set_active_controller(controller)

    mda_widget.execute_mda(None)

    assert run.called is runs
    assert warning.called is (not runs)
    if runs:
        run.assert_called_once_with(seq, output=None)


# ------------------------------------------------------------- toolbar and panel


def test_toolbar_reverts_and_explains_a_refused_switch(
    gui: MicroManagerGUI,
) -> None:
    from pymmcore_gui._main_window import Toolbar

    bar = cast("QToolBar | None", gui.findChild(QToolBar, str(Toolbar.MODE)))
    assert bar is not None
    actions = {a.text(): a for a in bar.actions() if a.text() in {"OPM", "EPI"}}
    assert set(actions) == {"OPM", "EPI"}
    assert actions["OPM"].isChecked()
    assert not actions["EPI"].isChecked()

    # the demo config has no TGLED, so EPI is refused: the button reverts
    actions["EPI"].trigger()
    assert gui.mode_controller.mode is ScopeMode.OPM
    assert actions["OPM"].isChecked()
    assert not actions["EPI"].isChecked()
    shown = [n.message for n in gui.nm.notifications()]
    assert any(NO_TGLED in m for m in shown)


def test_toolbar_follows_the_controller(gui: MicroManagerGUI) -> None:
    from pymmcore_gui._main_window import Toolbar

    bar = cast("QToolBar | None", gui.findChild(QToolBar, str(Toolbar.MODE)))
    assert bar is not None
    actions = {a.text(): a for a in bar.actions() if a.text() in {"OPM", "EPI"}}
    gui.mode_controller._mode = ScopeMode.EPI
    gui.mode_controller.modeChanged.emit(ScopeMode.EPI)
    assert actions["EPI"].isChecked()
    assert not actions["OPM"].isChecked()
    gui.mode_controller._mode = ScopeMode.OPM
    gui.mode_controller.modeChanged.emit(ScopeMode.OPM)
    assert actions["OPM"].isChecked()


def test_toolbars_do_not_add_a_core_or_widget_action() -> None:
    from pymmcore_gui._main_window import Toolbar

    assert Toolbar.MODE.value == "Mode"
    assert list(MicroManagerGUI.TOOLBARS)[:3] == [
        Toolbar.CAMERA_ACTIONS,
        Toolbar.OPTICAL_CONFIGS,
        Toolbar.WIDGETS,
    ]


def test_led_panel_follows_the_mode(
    mmcore: CMMCorePlus,
    fake: FakeTigerCore,
    opm_engine: ASISPIMEngine,
    qtbot: QtBot,
) -> None:
    ctrl = _controller(mmcore, fake)
    panel = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(panel)
    assert not panel._controls.isEnabled()
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert panel._controls.isEnabled()
    # entering EPI wrote no LED intensity
    assert not any(w[0] == "setProperty" for w in fake.writes)
    assert LED_INTENSITY_PROP not in {w[2] for w in fake.writes if len(w) > 2}
    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert not panel._controls.isEnabled()
