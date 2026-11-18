"""Safe OPM <-> EPI transitions for the Tiger hardware.

``enter_epi`` / ``exit_epi`` change only: the Core ``ChannelGroup``, lasers off
and LEDs off, and the MDA engine. They never lock, unlock or write CRISP, never
move stage Z or the piezo, never touch the joystick wheels, and never call
``ensure_global_shutter_open`` (its unguarded ``CCA X=0`` is the BNC1 regression
trap), ``configure_plogic_*``, ``set_plogic_evaluation_clock`` or
``setConfig("Lasers", ...)``. Everything else is a read, logged.
"""

from __future__ import annotations

import logging
from contextlib import suppress
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from pymmcore_gui.epi._led import all_leds_off, resolve_led_channels

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_plus import CMMCorePlus
    from pymmcore_plus.mda import MDAEngine

logger = logging.getLogger("pymmcore_gui")

#: Config group holding the ``LED488`` / ``LED561`` / ``LED642`` presets. Each sets
#: ``EPI LED Selector`` / ``Physical Shutter 1`` to an LED label (never an LED).
EPI_CHANNEL_GROUP = "EPI LEDs"
#: CRISP that servos the EPI objective's stage Z (logged only, never written).
CRISP_Z_LABEL = "CRISPAFocus:Z:32"
#: CRISP that servos the OPM piezo (logged only, never written).
CRISP_P_LABEL = "CRISPAFocus:P:34"
#: Stage Z (EPI objective) and the OPM piezo, for the position snapshot.
STAGE_Z_LABEL = "ZStage:Z:32"
PIEZO_LABEL = "PiezoStage:P:34"
#: Galvo properties checked on exit (read only).
_GALVO_SPIM_STATE = "SPIMState"
_GALVO_BEAM = "BeamEnabled"
#: Offset difference (deg) below which the galvo offset counts as unchanged.
_OFFSET_TOLERANCE_DEG = 1e-4


class SwitchRefused(RuntimeError):
    """A mode switch was refused; the message is shown to the user."""


@dataclass
class TigerSnapshot:
    """Read-only state captured on EPI entry, used for exit checks."""

    core_focus: str | None = None
    core_shutter: str | None = None
    channel_group: str | None = None
    auto_shutter: bool | None = None
    positions: dict[str, float | None] = field(default_factory=dict)
    crisp_states: dict[str, str | None] = field(default_factory=dict)
    galvo_state: dict[str, str | None] = field(default_factory=dict)
    galvo_offset_deg: float | None = None
    previous_engine: Any = None
    epi_engine: Any = None
    engine_swapped: bool = False


# ---------------------------------------------------------------- read helpers


def _loaded(mmc: CMMCorePlus) -> set[str]:
    try:
        return set(mmc.getLoadedDevices())
    except Exception:
        logger.exception("Could not list loaded devices")
        return set()


def _read_property(
    mmc: CMMCorePlus, loaded: set[str], dev: str, prop: str
) -> str | None:
    """Read ``dev.prop`` if the device is loaded and has it; otherwise ``None``."""
    if dev not in loaded:
        return None
    try:
        if not mmc.hasProperty(dev, prop):
            return None
        return str(mmc.getProperty(dev, prop))
    except Exception:
        logger.warning("Could not read %s.%s", dev, prop, exc_info=True)
        return None


def _read_position(mmc: CMMCorePlus, loaded: set[str], dev: str) -> float | None:
    if dev not in loaded:
        return None
    try:
        return float(mmc.getPosition(dev))
    except Exception:
        logger.warning("Could not read the position of %s", dev, exc_info=True)
        return None


def _galvo_label() -> str:
    from pymmcore_gui.asi_z_stack.common import HardwareConstants

    return HardwareConstants().galvo_a_label


def read_snapshot(mmc: CMMCorePlus) -> TigerSnapshot:
    """Read (never write) the state the exit checks compare against. Logs it."""
    from pymmcore_gui.asi_z_stack.live_offset import OFFSET_PROP

    loaded = _loaded(mmc)
    snap = TigerSnapshot()
    for attr, getter in (
        ("core_focus", mmc.getFocusDevice),
        ("core_shutter", mmc.getShutterDevice),
        ("channel_group", mmc.getChannelGroup),
        ("auto_shutter", mmc.getAutoShutter),
    ):
        try:
            setattr(snap, attr, getter())
        except Exception:
            logger.warning("Could not read Core %s", attr, exc_info=True)
    for dev in (PIEZO_LABEL, STAGE_Z_LABEL):
        snap.positions[dev] = _read_position(mmc, loaded, dev)
    for dev in (CRISP_Z_LABEL, CRISP_P_LABEL):
        snap.crisp_states[dev] = _read_property(mmc, loaded, dev, "CRISP State")
    galvo = _galvo_label()
    for prop in (_GALVO_SPIM_STATE, _GALVO_BEAM):
        snap.galvo_state[prop] = _read_property(mmc, loaded, galvo, prop)
    offset = _read_property(mmc, loaded, galvo, OFFSET_PROP)
    with suppress(TypeError, ValueError):
        snap.galvo_offset_deg = None if offset is None else float(offset)
    logger.info(
        "EPI snapshot: focus=%s shutter=%s channel_group=%r autoshutter=%s "
        "positions=%s crisp=%s galvo=%s offset=%s",
        snap.core_focus,
        snap.core_shutter,
        snap.channel_group,
        snap.auto_shutter,
        snap.positions,
        snap.crisp_states,
        snap.galvo_state,
        snap.galvo_offset_deg,
    )
    return snap


# ---------------------------------------------------------------- preconditions


def _service_state_name(svc: object) -> str | None:
    state = getattr(svc, "state", None)
    return getattr(state, "name", None)


def _stop_live_and_check_idle(svc: object, what: str) -> str | None:
    """Stop Live on *svc*, then require it to be IDLE; return a refusal or None."""
    if _service_state_name(svc) == "LIVE":
        stop = getattr(svc, "stop_live", None)
        if callable(stop):
            try:
                stop()
            except Exception as exc:
                logger.exception("Could not stop %s Live", what)
                return f"Could not stop {what} Live: {exc}"
    name = _service_state_name(svc)
    if name != "IDLE":
        return f"The {what} camera service is busy (state={name}); try again shortly."
    return None


def check_preconditions(
    mmc: CMMCorePlus, *, entering: bool, epi_service: object | None = None
) -> str | None:
    """Return why the switch is refused, or ``None`` when it may proceed.

    Stops Live on the OPM service (and the EPI service, if given) first; that is
    the only side effect.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    entering : bool
        ``True`` for OPM -> EPI, ``False`` for EPI -> OPM.
    epi_service : object | None
        The EPI camera service, if it exists (checked on exit).
    """
    from pymmcore_gui._settings import SettingsV1
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

    try:
        if mmc.mda.is_running():
            return "Stop the running acquisition before changing mode."
    except Exception as exc:
        return f"Could not check for a running acquisition: {exc}"

    if entering:
        epi = SettingsV1.instance().epi
        try:
            resolved = resolve_led_channels(mmc, epi.card_address)
        except Exception as exc:
            return f"Could not look for the TGLED: {exc}"
        visible = {led.channel for led in epi.leds if led.visible}
        if not visible & set(resolved):
            return (
                "Load 20261118-OPM_EPI.cfg to use EPI "
                f"(no TGLED at address {epi.card_address})"
            )

    if (svc := CameraWorkerService.get_active()) is not None:
        if reason := _stop_live_and_check_idle(svc, "OPM"):
            return reason
    if not entering and epi_service is not None:
        if reason := _stop_live_and_check_idle(epi_service, "EPI"):
            return reason
    return None


# ---------------------------------------------------------------------- enter


def _close_lasers() -> None:
    from pymmcore_gui.asi_z_stack import asi_controller

    asi_controller.close_all_lasers()


def enter_epi(
    mmc: CMMCorePlus,
    *,
    engine_factory: Callable[[CMMCorePlus], MDAEngine] | None = None,
    service_provider: Callable[[], object | None] | None = None,
) -> TigerSnapshot:
    """Switch the hardware-facing state to EPI. Preconditions are checked first.

    Returns the read-only snapshot for the exit checks. On any failure the
    partial change is undone best effort and `SwitchRefused` is raised.
    """
    from pymmcore_gui._settings import SettingsV1

    if reason := check_preconditions(mmc, entering=True):
        raise SwitchRefused(reason)
    card = SettingsV1.instance().epi.card_address

    try:
        _close_lasers()
    except Exception as exc:
        logger.exception("Could not close the lasers")
        raise SwitchRefused(f"Could not close the lasers: {exc}") from exc

    snap = read_snapshot(mmc)
    try:
        if EPI_CHANNEL_GROUP in mmc.getAvailableConfigGroups():
            if snap.channel_group != EPI_CHANNEL_GROUP:
                mmc.setChannelGroup(EPI_CHANNEL_GROUP)
        else:
            logger.info("No %r config group; leaving ChannelGroup", EPI_CHANNEL_GROUP)
        all_leds_off(mmc, card)
        if service_provider is not None:
            service_provider()
        if engine_factory is not None:
            engine = engine_factory(mmc)
            snap.previous_engine = mmc.mda.engine
            mmc.mda.set_engine(engine)
            snap.epi_engine = engine
            snap.engine_swapped = True
    except Exception as exc:
        logger.exception("Could not enter EPI mode")
        exit_epi(mmc, snap, register_opm_engine=None)
        raise SwitchRefused(f"Could not enter EPI mode: {exc}") from exc
    logger.info("Entered EPI mode")
    return snap


# ----------------------------------------------------------------------- exit


def _restore_core_state(mmc: CMMCorePlus, snap: TigerSnapshot) -> None:
    """Restore ChannelGroup, and Core roles that drifted (with a warning)."""
    if snap.channel_group is not None:
        try:
            if mmc.getChannelGroup() != snap.channel_group:
                mmc.setChannelGroup(snap.channel_group)
        except Exception:
            logger.exception("Could not restore ChannelGroup %r", snap.channel_group)
    roles: list[tuple[str, Any, Callable[[], Any], Callable[[Any], Any]]] = [
        ("Focus", snap.core_focus, mmc.getFocusDevice, mmc.setFocusDevice),
        ("Shutter", snap.core_shutter, mmc.getShutterDevice, mmc.setShutterDevice),
        ("AutoShutter", snap.auto_shutter, mmc.getAutoShutter, mmc.setAutoShutter),
    ]
    for name, want, get, put in roles:
        if want is None:
            continue
        try:
            have = get()
            if have != want:
                logger.warning(
                    "Core %s changed during EPI (%r -> %r); restoring it",
                    name,
                    want,
                    have,
                )
                put(want)
        except Exception:
            logger.exception("Could not restore Core %s", name)


def _reinstall_opm_engine(
    mmc: CMMCorePlus,
    snap: TigerSnapshot,
    register_opm_engine: Callable[[], None] | None,
) -> None:
    if not snap.engine_swapped:
        return
    try:
        if register_opm_engine is not None:
            register_opm_engine()
        # _register_mda_engine leaves a foreign engine alone when no ASI hardware
        # is loaded; never leave the EPI engine installed.
        if snap.epi_engine is not None and mmc.mda.engine is snap.epi_engine:
            mmc.mda.set_engine(snap.previous_engine)
    except Exception:
        logger.exception("Could not reinstall the OPM MDA engine")


def _check_galvo(mmc: CMMCorePlus, snap: TigerSnapshot) -> None:
    """Read-only galvo checks; re-apply the Live offset only if it moved."""
    from pymmcore_gui.asi_z_stack.live_offset import OFFSET_PROP, restore_live_offset

    loaded = _loaded(mmc)
    galvo = _galvo_label()
    if galvo not in loaded:
        return
    state = _read_property(mmc, loaded, galvo, _GALVO_SPIM_STATE)
    if state is not None and state != "Idle":
        logger.warning(
            "Galvo %s is %r after leaving EPI (expected Idle)", _GALVO_SPIM_STATE, state
        )
    beam = _read_property(mmc, loaded, galvo, _GALVO_BEAM)
    if beam is not None and beam != "Yes":
        logger.warning(
            "Galvo %s is %r after leaving EPI (expected Yes)", _GALVO_BEAM, beam
        )
    raw = _read_property(mmc, loaded, galvo, OFFSET_PROP)
    if raw is None or snap.galvo_offset_deg is None:
        return
    try:
        now = float(raw)
    except ValueError:
        return
    if abs(now - snap.galvo_offset_deg) > _OFFSET_TOLERANCE_DEG:
        logger.warning(
            "Galvo %s changed during EPI (%s -> %s); restoring the Live offset",
            OFFSET_PROP,
            snap.galvo_offset_deg,
            now,
        )
        restore_live_offset(mmc)


def exit_epi(
    mmc: CMMCorePlus,
    snap: TigerSnapshot,
    *,
    register_opm_engine: Callable[[], None] | None = None,
    restore_state: bool = True,
) -> None:
    """Switch the hardware-facing state back to OPM. Never raises.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    snap : TigerSnapshot
        The snapshot `enter_epi` returned.
    register_opm_engine : Callable[[], None] | None
        Reinstalls the OPM engine (``MicroManagerGUI._register_mda_engine``);
        only called when the engine was swapped.
    restore_state : bool
        ``False`` for a config reload: the snapshot belongs to the old config,
        so ChannelGroup, Core roles and the galvo are left alone.
    """
    from pymmcore_gui._settings import SettingsV1

    try:
        card = SettingsV1.instance().epi.card_address
        all_leds_off(mmc, card)
    except Exception:
        logger.exception("Could not turn the LEDs off")
    if restore_state:
        _restore_core_state(mmc, snap)
    _reinstall_opm_engine(mmc, snap, register_opm_engine)
    try:
        _close_lasers()
    except Exception:
        logger.exception("Could not close the lasers")
    if restore_state:
        try:
            loaded = _loaded(mmc)
            logger.info(
                "Leaving EPI: crisp=%s",
                {
                    dev: _read_property(mmc, loaded, dev, "CRISP State")
                    for dev in (CRISP_Z_LABEL, CRISP_P_LABEL)
                },
            )
            _check_galvo(mmc, snap)
        except Exception:
            logger.exception("Galvo checks after leaving EPI failed")
    logger.info("Left EPI mode")
