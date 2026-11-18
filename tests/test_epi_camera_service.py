"""EPI (Orca) camera service: settings, lifecycle, Snap/Live routing, exposure.

Hardware-free: the worker pool is a fake, ``spawn_async`` is replaced so no process
starts, and the LED card is the ``FakeLedCore`` shaped like the ASITiger adapter.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock

import numpy as np
import pytest
from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QMessageBox, QWidget

from pymmcore_gui import _mmcore_shutdown
from pymmcore_gui._main_window import MicroManagerGUI
from pymmcore_gui._settings import EpiCameraSettingsV1, SettingsV1
from pymmcore_gui.actions import widget_actions
from pymmcore_gui.actions.core_actions import (
    _prepare_for_reload,
    snap_image,
    toggle_live,
)
from pymmcore_gui.asi_z_stack import asi_controller
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
)
from pymmcore_gui.epi import _camera_service
from pymmcore_gui.epi._camera_service import (
    NOT_CONFIGURED,
    EpiCameraWorkerService,
    active_camera_service,
    epi_camera_precondition,
    epi_service_provider,
    get_epi_service,
    shutdown_epi_service,
)
from pymmcore_gui.epi._led import LED_INTENSITY_PROP
from pymmcore_gui.epi._mode import (
    EpiModeRefusal,
    ModeController,
    ScopeMode,
    set_active_controller,
)
from pymmcore_gui.widgets import _crisp_widget
from pymmcore_gui.widgets._exposure_widget import ExposureWidget
from test_epi_led import DECOY_X, TGLED, FakeLedCore
from test_epi_mode import FakeTigerCore, _controller, opm_engine  # noqa: F401
from test_opm_mode_pinning import _FakePool, _service

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

ORCA = "Orca"
IDLE = CameraWorkerServiceState.IDLE
LIVE = CameraWorkerServiceState.LIVE


def _camera(**overrides: Any) -> EpiCameraSettingsV1:
    values: dict[str, Any] = {
        "label": ORCA,
        "adapter_module": "TestAdapter",
        "device_name": "TestCam",
        "width": 32,
        "height": 16,
        "dtype": "uint16",
        "pre_init_props": [("Mode", "Fast")],
    }
    values.update(overrides)
    return EpiCameraSettingsV1(**values)


def _configure(**overrides: Any) -> EpiCameraSettingsV1:
    """Fill ``settings.epi.camera`` (test values, not the real Orca defaults)."""
    camera = _camera(**overrides)
    SettingsV1.instance().epi.camera = camera
    return camera


class _EpiCtrl:
    mode = ScopeMode.EPI


class _OpmCtrl:
    mode = ScopeMode.OPM


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
def lasers(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """Record laser calls; EPI Snap/Live must never open one."""
    calls: list[str] = []
    for name in ("open_selected_lasers", "close_selected_lasers"):
        monkeypatch.setattr(asi_controller, name, lambda name=name: calls.append(name))
    monkeypatch.setattr(asi_controller, "close_all_lasers", lambda: None)
    return calls


@pytest.fixture(autouse=True)
def spawned(monkeypatch: pytest.MonkeyPatch) -> list[EpiCameraWorkerService]:
    """Replace ``spawn_async`` so no worker process starts; record the calls."""
    calls: list[EpiCameraWorkerService] = []
    monkeypatch.setattr(
        EpiCameraWorkerService,
        "spawn_async",
        lambda self, on_ready=None: calls.append(self),
    )
    return calls


@pytest.fixture
def owner() -> Iterator[QObject]:
    parent = QObject()
    yield parent
    shutdown_epi_service(timeout=0.1)


def _idle_service(owner: QObject, pool: object) -> EpiCameraWorkerService:
    """A registered EPI service in IDLE with *pool* (no process)."""
    svc = EpiCameraWorkerService(owner)
    svc.begin(_camera())
    svc._pool = pool  # type: ignore[assignment]
    svc._state = IDLE
    _camera_service._epi_service = svc
    return svc


# ------------------------------------------------------------------- settings


def test_camera_settings_default_to_the_orca_identity() -> None:
    cam = EpiCameraSettingsV1()
    assert cam.label == "Orca"
    assert cam.adapter_module == "HamamatsuHam"
    assert cam.device_name == "HamamatsuHam_DCAM"
    assert (cam.width, cam.height, cam.dtype) == (2048, 2048, "uint16")
    assert cam.pre_init_props == []
    assert cam.is_configured()
    assert SettingsV1.instance().epi.camera == cam


def test_a_blanked_default_field_is_not_configured() -> None:
    cam = EpiCameraSettingsV1(adapter_module="")
    assert not cam.is_configured()
    assert not EpiCameraSettingsV1(width=0).is_configured()


def test_default_settings_dump_is_still_empty() -> None:
    assert SettingsV1().model_dump_json(exclude_defaults=True) == "{}"


def test_camera_settings_round_trip() -> None:
    settings = SettingsV1()
    settings.epi.camera = _camera()
    restored = SettingsV1.model_validate_json(
        settings.model_dump_json(exclude_defaults=True)
    )
    assert restored.epi.camera == _camera()
    assert restored.epi.camera.pre_init_props == [("Mode", "Fast")]
    assert restored.epi.camera.is_configured()


@pytest.mark.parametrize(
    "empty",
    [
        {"label": ""},
        {"adapter_module": ""},
        {"device_name": ""},
        {"width": 0},
        {"height": 0},
        {"dtype": ""},
    ],
)
def test_is_configured_needs_every_field(empty: dict[str, Any]) -> None:
    assert _camera().is_configured()
    assert not _camera(**empty).is_configured()


# -------------------------------------------------------------- the precondition


def test_precondition_refuses_until_the_camera_is_configured() -> None:
    assert epi_camera_precondition() is None  # the default is the Orca identity
    assert "Set the Orca camera in settings (epi.camera) first" in NOT_CONFIGURED
    _configure(adapter_module="")
    assert epi_camera_precondition() == NOT_CONFIGURED
    _configure()
    assert epi_camera_precondition() is None


def test_precondition_refuses_an_unknown_dtype() -> None:
    _configure(dtype="not-a-dtype")
    reason = epi_camera_precondition()
    assert reason is not None
    assert "dtype" in reason


@pytest.mark.usefixtures("opm_engine")
def test_entry_refused_when_the_camera_is_not_configured(
    mmcore: CMMCorePlus,
    spawned: list[EpiCameraWorkerService],
    owner: QObject,
) -> None:
    _configure(adapter_module="")  # blank the (default-configured) camera
    fake = FakeTigerCore(mmcore)
    ctrl = _controller(mmcore, fake)
    ctrl.epi_service_provider = epi_service_provider(owner)
    ctrl.extra_preconditions.append(epi_camera_precondition)

    assert ctrl.request_mode(ScopeMode.EPI) == NOT_CONFIGURED
    assert ctrl.mode is ScopeMode.OPM
    assert get_epi_service() is None
    assert spawned == []
    assert fake.writes == []  # nothing was changed


# ----------------------------------------------------------------- the lifecycle


@pytest.mark.usefixtures("opm_engine")
def test_created_lazily_reused_and_kept_across_switches_and_reloads(
    mmcore: CMMCorePlus,
    spawned: list[EpiCameraWorkerService],
    owner: QObject,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _configure()
    fake = FakeTigerCore(mmcore)
    ctrl = _controller(mmcore, fake)
    created: list[EpiCameraWorkerService] = []
    ctrl.epi_service_provider = epi_service_provider(owner, created.append)
    ctrl.epi_service_peek = get_epi_service
    ctrl.extra_preconditions.append(epi_camera_precondition)
    shutdowns: list[str] = []
    monkeypatch.setattr(
        EpiCameraWorkerService,
        "shutdown",
        lambda self, timeout=None: shutdowns.append("x"),
    )

    # nothing exists before the first entry
    assert get_epi_service() is None
    assert ctrl.request_mode(ScopeMode.EPI) is None
    svc = get_epi_service()
    assert svc is not None
    assert created == [svc]
    assert spawned == [svc]
    assert svc.state is CameraWorkerServiceState.SPAWNING
    # the OPM registry is untouched
    assert CameraWorkerService.get_active() is None
    assert EpiCameraWorkerService.get_active() is svc

    svc._state = IDLE  # the worker came up
    assert ctrl.request_mode(ScopeMode.OPM) is None
    assert get_epi_service() is svc  # kept, not shut down
    assert shutdowns == []

    # second entry: same service, no second spawn
    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert get_epi_service() is svc
    assert created == [svc]
    assert spawned == [svc]

    # a menu config load returns to OPM and leaves the Orca alone
    assert _prepare_for_reload(fake) is True  # type: ignore[arg-type]
    assert ctrl.mode is ScopeMode.OPM
    assert get_epi_service() is svc
    assert svc.state is IDLE
    assert shutdowns == []

    # the class-level hooks are inert too
    EpiCameraWorkerService.prepare_for_reload(mmcore)
    assert svc.begin_release(mmcore, None) is False
    assert shutdowns == []
    assert EpiCameraWorkerService.get_active() is svc

    # and force_opm from EPI, as a direct config load does
    assert ctrl.request_mode(ScopeMode.EPI) is None
    ctrl.force_opm()
    assert ctrl.mode is ScopeMode.OPM
    assert get_epi_service() is svc
    assert shutdowns == []


@pytest.mark.usefixtures("opm_engine")
def test_a_reload_shuts_down_the_opm_service_but_not_the_orca(
    mmcore: CMMCorePlus, owner: QObject
) -> None:
    epi = _idle_service(owner, _FakePool())
    opm = _service(_FakePool())
    CameraWorkerService._active = opm
    assert CameraWorkerService.prepare_for_reload(mmcore) is None
    assert opm.state is CameraWorkerServiceState.INACTIVE
    assert epi.state is IDLE
    assert get_epi_service() is epi


@pytest.mark.usefixtures("opm_engine")
def test_a_failed_spawn_is_retried_on_the_next_entry_and_never_blocks_exit(
    mmcore: CMMCorePlus,
    spawned: list[EpiCameraWorkerService],
    owner: QObject,
) -> None:
    _configure()
    ctrl = _controller(mmcore, FakeTigerCore(mmcore))
    ctrl.epi_service_provider = epi_service_provider(owner)
    ctrl.epi_service_peek = get_epi_service
    assert ctrl.request_mode(ScopeMode.EPI) is None
    svc = get_epi_service()
    assert svc is not None

    # the spawn failed: INACTIVE, no pool. Leaving EPI must still work.
    svc._state = CameraWorkerServiceState.INACTIVE
    EpiCameraWorkerService._active = None
    assert ctrl.request_mode(ScopeMode.OPM) is None

    assert ctrl.request_mode(ScopeMode.EPI) is None
    assert get_epi_service() is svc
    assert spawned == [svc, svc]  # spawned again


def test_shut_down_once_at_app_exit_after_the_leds_are_closed(
    mmcore: CMMCorePlus, owner: QObject, monkeypatch: pytest.MonkeyPatch
) -> None:
    svc = _idle_service(owner, _FakePool())
    order: list[str] = []
    monkeypatch.setattr(
        _mmcore_shutdown, "_close_leds", lambda mmc: order.append("leds")
    )
    monkeypatch.setattr(
        EpiCameraWorkerService,
        "shutdown",
        lambda self, timeout=None: order.append("epi"),
    )
    _mmcore_shutdown.shutdown_mmcore(mmcore)
    assert order == ["leds", "epi"]
    assert get_epi_service() is None
    # a second call (closeEvent + aboutToQuit) is harmless
    _mmcore_shutdown.shutdown_mmcore(mmcore)
    assert order == ["leds", "epi", "leds"]  # the Orca is shut down only once
    assert svc is not None


def test_shutdown_without_an_epi_service_is_a_noop(mmcore: CMMCorePlus) -> None:
    _mmcore_shutdown.shutdown_mmcore(mmcore)
    assert get_epi_service() is None


def test_an_epi_service_shutdown_failure_is_logged_not_raised(
    owner: QObject, monkeypatch: pytest.MonkeyPatch
) -> None:
    _idle_service(owner, _FakePool())

    def _boom(self: object, timeout: float | None = None) -> None:
        raise RuntimeError("worker would not stop")

    monkeypatch.setattr(EpiCameraWorkerService, "shutdown", _boom)
    shutdown_epi_service(timeout=0.1)
    assert get_epi_service() is None


# ------------------------------------------------- configs carry the settings


def test_the_worker_config_carries_the_settings(
    spawned: list[EpiCameraWorkerService], owner: QObject
) -> None:
    svc = EpiCameraWorkerService(owner)
    svc.begin(_camera())
    assert spawned == [svc]
    assert svc.camera_labels == (ORCA,)
    assert svc.frame_size(ORCA) == (32, 16)
    assert svc.geometry is not None
    assert (svc.geometry.image_width, svc.geometry.image_height) == (32, 16)

    handle = svc._build_worker_handle(ORCA)
    cfg = handle.config
    assert cfg.camera_label == ORCA
    assert cfg.adapter_module == "TestAdapter"
    assert cfg.adapter_device_name == "TestCam"
    assert cfg.pre_init_props == (("Mode", "Fast"),)
    assert cfg.geometry == (16, 32, "uint16")
    assert (handle.height, handle.width, handle.dtype) == (16, 32, "uint16")
    assert cfg.circular_buffer_mb == svc.hw.worker_circular_buffer_mb
    assert cfg.live_max_fps == svc.hw.live_max_fps
    assert handle.n_slots == svc.hw.frame_ring_slots_per_camera


def test_begin_refuses_unconfigured_or_bad_settings(
    spawned: list[EpiCameraWorkerService], owner: QObject
) -> None:
    svc = EpiCameraWorkerService(owner)
    with pytest.raises(ValueError, match="Orca camera"):
        svc.begin(EpiCameraSettingsV1(adapter_module=""))
    with pytest.raises(ValueError, match="dtype"):
        svc.begin(_camera(dtype="nope"))
    assert svc.state is CameraWorkerServiceState.INACTIVE
    assert spawned == []


def test_the_service_does_not_touch_the_main_core_or_the_opm_pool(
    spawned: list[EpiCameraWorkerService], owner: QObject
) -> None:
    svc = EpiCameraWorkerService(owner)
    svc.begin(_camera())
    assert svc._mmc is None  # no main core: nothing to release or reload
    assert CameraWorkerService.get_active() is None
    assert svc.worker_config_groups == {}


# ------------------------------------------------------------ the EPI guard


def test_the_epi_service_is_not_refused_in_epi_but_the_opm_service_is(
    owner: QObject,
) -> None:
    pool = _FakePool()
    epi = _idle_service(owner, pool)
    opm = _service(_FakePool())
    set_active_controller(_EpiCtrl())

    assert epi.set_roi(ORCA, 0, 0, 8, 8) == (0, 0, 8, 8)
    assert pool.roi_calls == [(ORCA, 0, 0, 8, 8, 5.0)]
    epi.set_exposure(25.0)
    assert pool.calls == [(ORCA, (("Exposure", "25"),))]
    assert epi.last_known_exposure_ms == 25.0

    with pytest.raises(EpiModeRefusal):
        opm.set_roi("Camera-1", 0, 0, 8, 8)
    with pytest.raises(EpiModeRefusal):
        opm.set_exposure(5.0)


# ------------------------------------------------------- Snap / Live (D12)


class _Core(FakeLedCore):
    """``FakeLedCore`` plus the channel group's current preset."""

    def __init__(self, preset: str = "LED488", labels: list[str] | None = None) -> None:
        super().__init__(labels=labels)
        self.preset = preset

    def getCurrentConfig(self, group: str) -> str:
        self.calls.append(("getCurrentConfig", group))
        return self.preset


class _Svc:
    """A stand-in EPI service recording its calls into the core's call list."""

    def __init__(
        self,
        core: _Core,
        state: CameraWorkerServiceState = IDLE,
        *,
        snap_error: Exception | None = None,
        start_error: Exception | None = None,
    ) -> None:
        self.core = core
        self.state = state
        self.snap_error = snap_error
        self.start_error = start_error

    def snap(self) -> dict[str, np.ndarray]:
        self.core.calls.append(("snap",))
        if self.snap_error is not None:
            raise self.snap_error
        return {ORCA: np.zeros((2, 2), dtype=np.uint16)}

    def start_live(self) -> None:
        self.core.calls.append(("start_live",))
        if self.start_error is not None:
            raise self.start_error
        self.state = LIVE

    def stop_live(self) -> None:
        self.core.calls.append(("stop_live",))
        self.state = IDLE


def _writes(core: _Core) -> list[tuple[Any, ...]]:
    keep = {"setProperty", "setShutterOpen", "snap", "start_live", "stop_live"}
    return [c for c in core.calls if c[0] in keep]


def _epi_setup(
    monkeypatch: pytest.MonkeyPatch,
    *,
    preset: str = "LED488",
    labels: list[str] | None = None,
    state: CameraWorkerServiceState = IDLE,
    **svc_kwargs: Any,
) -> tuple[_Core, _Svc, SimpleNamespace, SimpleNamespace]:
    core = _Core(preset, labels)
    svc = _Svc(core, state, **svc_kwargs)
    monkeypatch.setattr(_camera_service, "_epi_service", svc)
    set_active_controller(_EpiCtrl())
    window = SimpleNamespace(nm=MagicMock())
    action = SimpleNamespace(mmc=core, setChecked=MagicMock(), parent=lambda: window)
    return core, svc, action, window


def test_snap_lights_the_selected_led_snaps_then_closes_it(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    SettingsV1.instance().epi.leds[0].percent = 70
    core, svc, action, _window = _epi_setup(monkeypatch)
    snap_image(action, False)  # type: ignore[arg-type]
    assert _writes(core) == [
        ("setProperty", "LED:L:37:1", LED_INTENSITY_PROP, 70),
        ("setShutterOpen", "LED:L:37:1", True),
        ("snap",),
        ("setShutterOpen", "LED:L:37:1", False),
    ]
    assert core.open["LED:L:37:1"] is False
    assert lasers == []  # never open_selected_lasers
    # no other LED, and never the unrelated X-axis LED
    assert all(c[1] != DECOY_X for c in core.calls if len(c) > 1)
    assert not any(
        c[1] in TGLED[1:]
        for c in core.calls
        if c[0] in ("setShutterOpen", "setProperty")
    )


def test_snap_picks_the_led_by_the_preset_channel(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, _window = _epi_setup(monkeypatch, preset="LED642")
    snap_image(action, False)  # type: ignore[arg-type]
    assert _writes(core)[0][:3] == ("setProperty", "LED:L:37:3", LED_INTENSITY_PROP)
    assert core.open["LED:L:37:3"] is False


def test_snap_closes_the_led_when_the_snap_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, _window = _epi_setup(
        monkeypatch, snap_error=RuntimeError("camera timed out")
    )
    with pytest.raises(RuntimeError, match="camera timed out"):
        snap_image(action, False)  # type: ignore[arg-type]
    assert _writes(core)[-1] == ("setShutterOpen", "LED:L:37:1", False)
    assert core.open["LED:L:37:1"] is False


def test_snap_closes_the_led_even_if_opening_it_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, svc, action, _window = _epi_setup(monkeypatch)
    core.fail_on = {"LED:L:37:1"}
    with pytest.raises(RuntimeError, match="serial error"):
        snap_image(action, False)  # type: ignore[arg-type]
    # the close was still attempted (and its failure only logged)
    assert ("setShutterOpen", "LED:L:37:1", False) in core.calls
    assert ("snap",) not in core.calls
    assert svc.state is IDLE


@pytest.mark.parametrize(
    ("preset", "labels"),
    [
        ("", None),  # no preset selected
        ("Mystery", None),  # a preset that is not one of the LED channels
        ("LED488", [DECOY_X, "LED:L:37:2"]),  # its LED is not loaded
    ],
)
def test_snap_without_a_resolvable_led_gives_no_light_and_no_snap(
    monkeypatch: pytest.MonkeyPatch,
    lasers: list[str],
    preset: str,
    labels: list[str] | None,
) -> None:
    core, _svc, action, window = _epi_setup(monkeypatch, preset=preset, labels=labels)
    snap_image(action, False)  # type: ignore[arg-type]
    assert _writes(core) == []
    assert lasers == []
    window.nm.show_warning_message.assert_called_once()


def test_snap_while_the_orca_is_not_ready_is_refused_without_light(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, window = _epi_setup(
        monkeypatch, state=CameraWorkerServiceState.SPAWNING
    )
    snap_image(action, False)  # type: ignore[arg-type]
    assert _writes(core) == []
    assert "not ready" in window.nm.show_warning_message.call_args.args[0]


def test_snap_while_live_stops_live_and_its_led_first(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, _window = _epi_setup(monkeypatch)
    toggle_live(action, True)  # type: ignore[arg-type]
    del core.calls[:]
    snap_image(action, False)  # type: ignore[arg-type]
    assert _writes(core) == [
        ("stop_live",),
        ("setShutterOpen", "LED:L:37:1", False),
        ("setProperty", "LED:L:37:1", LED_INTENSITY_PROP, 50),
        ("setShutterOpen", "LED:L:37:1", True),
        ("snap",),
        ("setShutterOpen", "LED:L:37:1", False),
    ]


def test_live_start_and_stop_order(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    core, svc, action, _window = _epi_setup(monkeypatch)
    toggle_live(action, True)  # type: ignore[arg-type]
    assert _writes(core) == [
        ("setProperty", "LED:L:37:1", LED_INTENSITY_PROP, 50),
        ("setShutterOpen", "LED:L:37:1", True),
        ("start_live",),
    ]
    assert core.open["LED:L:37:1"] is True  # lit while Live runs
    assert svc.state is LIVE

    del core.calls[:]
    toggle_live(action, False)  # type: ignore[arg-type]
    assert _writes(core) == [("stop_live",), ("setShutterOpen", "LED:L:37:1", False)]
    assert core.open["LED:L:37:1"] is False
    assert lasers == []
    action.setChecked.assert_not_called()


def test_live_closes_the_led_when_it_fails_to_start(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, _window = _epi_setup(
        monkeypatch, start_error=RuntimeError("arm failed")
    )
    with pytest.raises(RuntimeError, match="arm failed"):
        toggle_live(action, True)  # type: ignore[arg-type]
    assert _writes(core)[-1] == ("setShutterOpen", "LED:L:37:1", False)
    assert core.open["LED:L:37:1"] is False
    action.setChecked.assert_called_once_with(False)
    assert _camera_service._live_led is None


def test_live_without_a_preset_gives_no_light_and_unchecks(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, window = _epi_setup(monkeypatch, preset="")
    toggle_live(action, True)  # type: ignore[arg-type]
    assert _writes(core) == []
    action.setChecked.assert_called_once_with(False)
    window.nm.show_warning_message.assert_called_once()


def test_live_while_the_orca_is_spawning_is_refused_without_light(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, action, window = _epi_setup(
        monkeypatch, state=CameraWorkerServiceState.SPAWNING
    )
    toggle_live(action, True)  # type: ignore[arg-type]
    assert _writes(core) == []
    action.setChecked.assert_called_once_with(False)
    assert "not ready" in window.nm.show_warning_message.call_args.args[0]


def test_live_that_ends_on_its_own_closes_its_led(
    monkeypatch: pytest.MonkeyPatch, owner: QObject
) -> None:
    core, _svc, action, _window = _epi_setup(monkeypatch)
    toggle_live(action, True)  # type: ignore[arg-type]
    assert core.open["LED:L:37:1"] is True

    svc = _idle_service(owner, _FakePool())  # the real service's signal handler
    svc._on_live_state_changed(True)
    assert core.open["LED:L:37:1"] is True  # a "Live started" signal closes nothing
    svc._on_live_state_changed(False)
    assert core.open["LED:L:37:1"] is False
    svc._on_live_state_changed(False)  # idempotent


def test_crispy_snap_in_epi_uses_the_orca_and_the_led(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    core, _svc, _action, _window = _epi_setup(monkeypatch)
    frames = _crisp_widget.snap_for_crispy(core)  # type: ignore[arg-type]
    assert list(frames) == [ORCA]
    assert _writes(core) == [
        ("setProperty", "LED:L:37:1", LED_INTENSITY_PROP, 50),
        ("setShutterOpen", "LED:L:37:1", True),
        ("snap",),
        ("setShutterOpen", "LED:L:37:1", False),
    ]
    assert lasers == []


def test_crispy_snap_in_epi_closes_the_led_when_the_snap_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, _action, _window = _epi_setup(
        monkeypatch, snap_error=RuntimeError("boom")
    )
    with pytest.raises(RuntimeError, match="boom"):
        _crisp_widget.snap_for_crispy(core)  # type: ignore[arg-type]
    assert core.open["LED:L:37:1"] is False


def test_crispy_snap_in_epi_with_no_led_selected_is_refused(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _svc, _action, _window = _epi_setup(monkeypatch, preset="")
    with pytest.raises(EpiModeRefusal):
        _crisp_widget.snap_for_crispy(core)  # type: ignore[arg-type]
    assert _writes(core) == []


def test_snap_in_opm_still_uses_the_lasers_and_the_opm_service(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    calls: list[str] = []
    opm = SimpleNamespace(snap=lambda: calls.append("snap"))
    monkeypatch.setattr(CameraWorkerService, "_active", opm)
    set_active_controller(_OpmCtrl())
    window = SimpleNamespace(nm=MagicMock())
    action = SimpleNamespace(mmc=MagicMock(), parent=lambda: window)
    snap_image(action, False)  # type: ignore[arg-type]
    assert lasers == ["open_selected_lasers", "close_selected_lasers"]
    assert calls == ["snap"]


# ------------------------------------------------- the service the app targets


def test_active_camera_service_follows_the_mode(owner: QObject) -> None:
    epi = _idle_service(owner, _FakePool())
    opm = _service(_FakePool())
    CameraWorkerService._active = opm

    assert active_camera_service() is opm  # no controller: OPM
    set_active_controller(_OpmCtrl())
    assert active_camera_service() is opm
    set_active_controller(_EpiCtrl())
    assert active_camera_service() is epi
    # get_active() keeps meaning "the OPM service" in both modes
    assert CameraWorkerService.get_active() is opm


def test_active_camera_service_in_epi_before_the_orca_exists_is_the_opm_one() -> None:
    opm = _service(_FakePool())
    CameraWorkerService._active = opm
    set_active_controller(_EpiCtrl())
    assert active_camera_service() is opm  # its calls are refused in EPI


def test_exposure_widget_routes_to_the_active_service(
    mmcore: CMMCorePlus,
    qtbot: QtBot,
    owner: QObject,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        QMessageBox,
        "warning",
        lambda *a, **k: pytest.fail(f"exposure was refused: {a}"),
    )
    ctrl = ModeController(mmcore)
    opm_pool, epi_pool = _FakePool(), _FakePool()
    opm = _service(opm_pool)
    CameraWorkerService._active = opm
    _idle_service(owner, epi_pool)

    wdg = ExposureWidget(mmcore)
    qtbot.addWidget(wdg)
    assert wdg.isEnabled()

    wdg.spin.setValue(12.0)  # OPM: both PVCAM workers
    assert [c[0] for c in opm_pool.calls] == ["Camera-1", "Camera-2"]
    assert opm.last_known_exposure_ms == 12.0
    assert epi_pool.calls == []

    ctrl._mode = ScopeMode.EPI
    ctrl.modeChanged.emit(ScopeMode.EPI)  # the widget re-syncs
    assert wdg.isEnabled()
    assert wdg.spin.value() == 10.0  # the Orca's own exposure, not OPM's 12
    wdg.spin.setValue(30.0)  # EPI: the Orca worker only
    assert epi_pool.calls == [(ORCA, (("Exposure", "30"),))]
    assert [c[0] for c in opm_pool.calls] == ["Camera-1", "Camera-2"]  # unchanged
    assert opm.last_known_exposure_ms == 12.0

    ctrl._mode = ScopeMode.OPM
    ctrl.modeChanged.emit(ScopeMode.OPM)
    assert wdg.spin.value() == 12.0
    wdg.spin.setValue(14.0)
    assert [c[0] for c in opm_pool.calls][-2:] == ["Camera-1", "Camera-2"]
    assert len(epi_pool.calls) == 1


def test_camera_roi_widget_follows_the_active_service(
    mmcore: CMMCorePlus, qtbot: QtBot, owner: QObject
) -> None:
    ctrl = ModeController(mmcore)
    pool = MagicMock()
    pool.get_roi.return_value = (0, 0, 8, 8)
    pool.dead_workers.return_value = []
    opm = _service(pool)
    CameraWorkerService._active = opm
    epi = _idle_service(owner, pool)

    host = QWidget()
    qtbot.addWidget(host)
    wdg = widget_actions.create_camera_roi(host)
    qtbot.addWidget(wdg)
    assert isinstance(wdg, widget_actions._ModeFollowingRoi)
    assert wdg._child is not None
    assert wdg._child._service is opm

    ctrl._mode = ScopeMode.EPI
    ctrl.modeChanged.emit(ScopeMode.EPI)
    assert wdg._child._service is epi
    ctrl._mode = ScopeMode.OPM
    ctrl.modeChanged.emit(ScopeMode.OPM)
    assert wdg._child._service is opm


def test_camera_roi_widget_without_a_controller_is_the_plain_widget(
    qtbot: QtBot,
) -> None:
    opm = _service(MagicMock())
    CameraWorkerService._active = opm
    host = QWidget()
    qtbot.addWidget(host)
    wdg = widget_actions.create_camera_roi(host)
    qtbot.addWidget(wdg)
    assert not isinstance(wdg, widget_actions._ModeFollowingRoi)
    assert wdg._service is opm


# ------------------------------------------------------------ app wiring


@pytest.fixture
def gui(qtbot: QtBot, monkeypatch: pytest.MonkeyPatch) -> Iterator[MicroManagerGUI]:
    monkeypatch.setattr(asi_controller, "ensure_global_shutter_open", lambda: None)
    window = MicroManagerGUI()
    qtbot.addWidget(window)
    yield window


def test_the_window_wires_the_provider_and_precondition(
    gui: MicroManagerGUI, spawned: list[EpiCameraWorkerService]
) -> None:
    ctrl = gui.mode_controller
    assert epi_camera_precondition in ctrl.extra_preconditions
    assert ctrl.epi_service_provider is not None
    assert ctrl.epi_service_peek is get_epi_service
    assert get_epi_service() is None  # nothing spawned at startup

    _configure()
    provider = ctrl.epi_service_provider
    assert provider is not None
    svc = provider()
    assert isinstance(svc, EpiCameraWorkerService)
    assert spawned == [svc]
    assert svc.parent() is gui
    assert provider() is svc  # reused
    assert spawned == [svc]

    # the preview follows the service: a frame is shown in the Orca's dock
    preview = MagicMock()
    manager = gui._viewers_manager
    manager._get_or_create_camera_preview = MagicMock(  # type: ignore[method-assign]
        return_value=(preview, True)
    )
    frame = np.zeros((4, 4), dtype=np.uint16)
    svc.frameReady.emit(ORCA, 0, frame, {}, 0)
    manager._get_or_create_camera_preview.assert_called_once_with(ORCA)
    preview.append.assert_called_once_with(frame)
    assert ORCA in manager._latest_frames


def test_the_orca_service_is_not_shut_down_by_a_reload_but_is_at_close(
    gui: MicroManagerGUI,
    spawned: list[EpiCameraWorkerService],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _configure()
    provider = gui.mode_controller.epi_service_provider
    assert provider is not None
    svc = provider()
    assert svc is not None
    shutdowns: list[str] = []
    monkeypatch.setattr(
        EpiCameraWorkerService,
        "shutdown",
        lambda self, timeout=None: shutdowns.append("epi"),
    )
    assert _prepare_for_reload(gui.mmcore) is True
    assert shutdowns == []
    gui.close()
    assert shutdowns == ["epi"]
    assert get_epi_service() is None
