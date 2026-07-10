# src/microscope/asi_z_stack/asi_controller.py
import logging
import time
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from .common import HardwareConstants

# Direct import - this fixes the Pylance error

if TYPE_CHECKING:
    from .common import AcquisitionSettings

logger = logging.getLogger(__name__)

mmc = CMMCorePlus.instance()
_HW = HardwareConstants()


def set_property(device_label: str, property_name: str, value: str) -> None:
    """Sets a Micro-Manager device property if it has changed."""
    if device_label in mmc.getLoadedDevices() and mmc.hasProperty(
        device_label, property_name
    ):
        if mmc.getProperty(device_label, property_name) != str(value):
            mmc.setProperty(device_label, property_name, value)
    else:
        logger.warning(
            f"Cannot set '{property_name}' for device '{device_label}'. "
            "Device or property not found."
        )


def get_property(device_label: str, property_name: str) -> str | None:
    """Safely gets a Micro-Manager device property value."""
    if device_label in mmc.getLoadedDevices() and mmc.hasProperty(
        device_label, property_name
    ):
        return mmc.getProperty(device_label, property_name)
    logger.warning(
        f"Cannot get '{property_name}' for device '{device_label}'. "
        "Device or property not found."
    )
    return None


def _send_tiger_command(
    cmd: str, tiger_comm_hub_label: str, axis: str = ""
) -> str | None:
    """Internal helper to send a serial command to the Tiger controller.

    Prepends ``axis`` (a card address prefix, e.g. ``"33"``) to ``cmd`` per
    ASI's ``[axis][command][options]`` serial convention, so callers can
    address a specific card without building the prefix into ``cmd``
    themselves. Returns the ``SerialResponse`` property's value if the hub
    exposes one, so query-style commands (e.g. ``RA``) can be read back by
    callers; other commands can simply ignore the return value.
    """
    full_cmd = f"{axis}{cmd}"
    if tiger_comm_hub_label in mmc.getLoadedDevices():
        logger.debug(f"  -> {full_cmd}")
        mmc.setProperty(tiger_comm_hub_label, "SerialCommand", full_cmd)
        time.sleep(0.01)
        if mmc.hasProperty(tiger_comm_hub_label, "SerialResponse"):
            response = mmc.getProperty(tiger_comm_hub_label, "SerialResponse")
            logger.debug(f"  <- {response}")
            return response
    else:
        logger.warning(f"TigerCommHub not found. Cannot send command: {full_cmd}")
    return None


def set_plogic_evaluation_clock(tiger_comm_hub_label: str, running: bool) -> None:
    """Send the raw ``PM E=<0|1>`` command, completely unaddressed.

    Matches a captured DEBUG-level log of the microscope-control sibling
    repo's actual, successful 201-slice acquisition on this exact hardware
    byte-for-byte: ``Tiger command sent: PM E=1`` (no axis prefix at all)
    right before triggering the galvo, ``Tiger command sent: PM E=0`` during
    cleanup. Two earlier versions of this function guessed at an explicit
    prefix instead -- first the scanner (``33PM E=1``, which the controller
    rejects outright: ``:N-2``, unrecognized command), then PLogic itself
    (``36PM E=1``, which the controller accepts, ``:A``, but which did not
    reproduce the sibling repo's working behavior). Neither guess was
    needed: the real log sends this command with no prefix at all, so
    match that exactly rather than re-deriving the addressing model from
    ASI's general documentation.
    """
    mode = 1 if running else 0
    _send_tiger_command(f"PM E={mode}", tiger_comm_hub_label)


def open_global_shutter(
    plogic_label: str,
    tiger_comm_hub_label: str,
    plogic_always_on_cell: int,
    plogic_bnc3_addr: int,
) -> None:
    """Configures and opens a global shutter on PLogic BNC3."""
    logger.debug("Opening global shutter (BNC3 HIGH)...")
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)

    try:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        # Order and bundling (Y and Z together) both matter here -- they
        # match the sibling repo's oscilloscope-confirmed working sequence.
        _send_tiger_command(f"{plogic_addr_prefix}CCA X=0", tiger_comm_hub_label)
        _send_tiger_command(f"M E={plogic_always_on_cell}", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCA Y=0 Z=5", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCB X=1", tiger_comm_hub_label)
        _send_tiger_command(f"M E={plogic_bnc3_addr}", tiger_comm_hub_label)
        _send_tiger_command(
            f"{plogic_addr_prefix}CCA Z={plogic_always_on_cell}", tiger_comm_hub_label
        )
        _send_tiger_command(f"{plogic_addr_prefix}SS Z", tiger_comm_hub_label)
        logger.info("Global shutter is open (BNC3 is HIGH).")
    except Exception:
        logger.error("Error opening global shutter.", exc_info=True)
    finally:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "Yes")


def close_global_shutter(
    plogic_label: str, tiger_comm_hub_label: str, plogic_bnc3_addr: int
) -> None:
    """Closes the global shutter on PLogic BNC3."""
    logger.debug("Closing global shutter (BNC3 LOW)...")
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)

    try:
        if plogic_label not in mmc.getLoadedDevices():
            logger.warning("PLogic device not found, cannot close shutter.")
            return

        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        _send_tiger_command(f"M E={plogic_bnc3_addr}", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCA Z=0", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}SS Z", tiger_comm_hub_label)
        logger.info("Global shutter is closed (BNC3 is LOW).")
    except Exception:
        logger.warning("Could not close global shutter.", exc_info=True)
    finally:
        if (
            original_hub_setting == "Yes"
            and get_property(tiger_comm_hub_label, hub_prop) == "No"
        ):
            set_property(tiger_comm_hub_label, hub_prop, "Yes")


# External trigger modes, in order of preference. Shared between
# set_camera_trigger_mode (session/main-process cameras) and
# preferred_external_trigger_mode (camera worker subprocesses, each with
# their own CMMCorePlus instance -- see camera_worker.py).
EXTERNAL_TRIGGER_MODES = ("Level Trigger", "Edge Trigger")


def preferred_external_trigger_mode(
    core: CMMCorePlus,
    camera_label: str,
    desired_modes: tuple[str, ...] = EXTERNAL_TRIGGER_MODES,
) -> str | None:
    """Return the first of *desired_modes* that *camera_label* allows.

    Parameters
    ----------
    core : CMMCorePlus
        The core *camera_label* is loaded on (not necessarily the session
        singleton -- camera worker subprocesses each have their own).
    camera_label : str
        The device label of the camera.
    desired_modes : tuple[str, ...]
        Trigger mode names to search for, in order of preference.

    Returns
    -------
    str | None
        The first allowed mode from *desired_modes*, or ``None`` if the
        camera has no ``TriggerMode`` property or none of them are allowed.
    """
    if not core.hasProperty(camera_label, "TriggerMode"):
        return None
    try:
        allowed_modes = core.getAllowedPropertyValues(camera_label, "TriggerMode")
    except Exception:
        return None
    return next((mode for mode in desired_modes if mode in allowed_modes), None)


def set_camera_trigger_mode(camera_label: str) -> bool:
    """
    Finds and sets the appropriate external trigger mode on the specified camera.

    Args:
        camera_label (str): The device label of the camera.

    Returns
    -------
        bool: True if a valid trigger mode was set, False otherwise.
    """
    if camera_label not in mmc.getLoadedDevices():
        logger.warning(f"Camera '{camera_label}' not found.")
        return False

    if not mmc.hasProperty(camera_label, "TriggerMode"):
        logger.warning(f"Camera '{camera_label}' has no 'TriggerMode' property.")
        return False

    try:
        mode = preferred_external_trigger_mode(mmc, camera_label)
        if mode is None:
            logger.warning(
                f"Could not find a suitable trigger mode for '{camera_label}'"
            )
            return False
        logger.debug(f"Setting '{camera_label}' trigger mode to '{mode}'")
        mmc.setProperty(camera_label, "TriggerMode", mode)
        return True
    except Exception:
        logger.error(f"Error setting trigger mode for '{camera_label}'.", exc_info=True)
        return False


def configure_plogic_for_dual_nrt_pulses(
    settings: "AcquisitionSettings",
    plogic_label: str,
    tiger_comm_hub_label: str,
    plogic_camera_cell: int,
    pulses_per_ms: float,
    plogic_4khz_clock_addr: int,
    plogic_trigger_ttl_addr: int,
    plogic_laser_on_cell: int,
) -> None:
    """Configures PLogic for two independent, synchronized NRT one-shot pulses.

    Deliberately does not touch laser BNC routing (no preset load): which
    physical laser cell 10 drives is exclusively the ``"Lasers"`` ConfigGroup's
    ``OutputChannel`` property's job (set per-MDA-channel by the stock
    ``MDAEngine``/``_set_event_channel``). This function previously loaded
    PLogic preset 30 here first -- confirmed against ASI's own Tiger PLogic
    documentation to be "diSPIM: simultaneous 4-color" (wires BNC5-8 *all* to
    cell 10 at once), a different preset family entirely from the single-laser
    presets (5-8) ``OutputChannel`` uses. Loading it unconditionally on every
    sequence setup forced "all lasers on" as a transient baseline that a
    channel's own config switch could, in some cases, never correct (see
    ``_ASITriggerEngineBase._reset_channel_config_cache``) -- removed rather
    than fixed in place, since this function has no business selecting a
    laser preset at all.
    """
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)

    def _send(cmd: str) -> None:
        _send_tiger_command(cmd, tiger_comm_hub_label)

    try:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        # 1. Program Camera Pulse (NRT One-Shot #1)
        _send(f"M E={plogic_camera_cell}")
        camera_pulse_cycles = int(settings.camera_exposure_ms * pulses_per_ms)
        # Y (NRT one-shot mode) and Z (pulse length) must be bundled into a
        # single command -- sent as separate calls, the Tiger controller
        # drops the cell-edit context between them.
        _send(f"{plogic_addr_prefix}CCA Y=14 Z={camera_pulse_cycles}")
        _send(
            f"{plogic_addr_prefix}CCB X={plogic_trigger_ttl_addr} "
            f"Y={plogic_4khz_clock_addr} Z=0"
        )

        # 2. Program Laser Pulse (NRT One-Shot #2)
        _send(f"M E={plogic_laser_on_cell}")
        laser_pulse_cycles = int(settings.laser_trig_duration_ms * pulses_per_ms)
        _send(f"{plogic_addr_prefix}CCA Y=14 Z={laser_pulse_cycles}")
        _send(
            f"{plogic_addr_prefix}CCB X={plogic_trigger_ttl_addr} "
            f"Y={plogic_4khz_clock_addr} Z=0"
        )

        # 3. Route Camera Trigger Cell Output to BNC1 (Address 33)
        _send("M E=33")
        _send(f"{plogic_addr_prefix}CCA Z={plogic_camera_cell}")

        # 4. Save the configuration
        _send(f"{plogic_addr_prefix}SS Z")
        logger.info("PLogic configured for dual NRT pulses.")

    finally:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "Yes")


def read_plogic_trigger_chain_state(
    plogic_label: str, tiger_comm_hub_label: str
) -> dict[str, str | None]:
    """Read back PLogic's front-panel, backplane, and cell-output bitmasks.

    Wraps the ``RA`` query family so the trigger chain (galvo pulse in ->
    NRT cell output -> BNC out) can be confirmed directly against hardware
    state during a bench run, rather than only inferred from an oscilloscope
    trace. Each value is the raw bitmask string the firmware returns (bit
    N-1 corresponds to address/cell N); interpret it against the specific
    address of interest (e.g. the trigger address or camera/laser cell
    number) since bit width/ordering can vary by firmware revision.
    """
    plogic_addr_prefix = plogic_label.split(":")[-1]
    return {
        "front_panel (RA X?)": _send_tiger_command(
            f"{plogic_addr_prefix}RA X?", tiger_comm_hub_label
        ),
        "backplane (RA Y?)": _send_tiger_command(
            f"{plogic_addr_prefix}RA Y?", tiger_comm_hub_label
        ),
        "cell_outputs (RA Z?)": _send_tiger_command(
            f"{plogic_addr_prefix}RA Z?", tiger_comm_hub_label
        ),
    }


def log_plogic_trigger_chain_state(
    plogic_label: str, tiger_comm_hub_label: str
) -> None:
    """Print PLogic's trigger-chain bitmasks.

    See :func:`read_plogic_trigger_chain_state` for what each field means.
    """
    for label, value in read_plogic_trigger_chain_state(
        plogic_label, tiger_comm_hub_label
    ).items():
        logger.info(f"  [PLogic] {label} = {value}")


def set_laser_outputs(
    plogic_label: str,
    tiger_comm_hub_label: str,
    bnc_addrs: list[int],
    on: bool,
    always_on_cell: int,
) -> None:
    """Drive PLogic BNC outputs to gate individual lasers on or off.

    Each BNC's source cell is pointed at ``always_on_cell`` (a constant-high cell
    configured when the global shutter is opened) to turn its laser on, or at cell
    0 to turn it off. The CCA edits take effect immediately; a single ``SS Z`` at
    the end persists them (mirroring ``open_global_shutter``).

    Parameters
    ----------
    plogic_label : str
        Device label of the PLogic card (e.g. ``"PLogic:E:36"``).
    tiger_comm_hub_label : str
        Device label of the Tiger comm hub used to send serial commands.
    bnc_addrs : list[int]
        PLogic addresses of the BNC outputs to drive (front-panel BNC n = 32 + n).
    on : bool
        Whether to turn the lasers on (True) or off (False).
    always_on_cell : int
        PLogic cell number wired to a constant-high value.
    """
    if not bnc_addrs:
        return
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)
    source = always_on_cell if on else 0

    try:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        for addr in bnc_addrs:
            _send_tiger_command(f"M E={addr}", tiger_comm_hub_label)
            _send_tiger_command(
                f"{plogic_addr_prefix}CCA Z={source}", tiger_comm_hub_label
            )
        _send_tiger_command(f"{plogic_addr_prefix}SS Z", tiger_comm_hub_label)
    finally:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "Yes")


def _plogic_available() -> bool:
    """Return True if the PLogic card and Tiger comm hub are both loaded."""
    devices = mmc.getLoadedDevices()
    return _HW.plogic_label in devices and _HW.tiger_comm_hub_label in devices


def asi_zstack_hardware_available() -> bool:
    """Return True if the PLogic, Tiger hub, and SPIM galvo are all loaded.

    Used by the GUI to decide whether to register the PLogic-triggered
    :class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` for MDA
    z-stacks. Requires the galvo (which that engine drives as trigger
    master) in addition to the PLogic hardware checked by
    :func:`_plogic_available`.
    """
    devices = mmc.getLoadedDevices()
    return (
        _HW.plogic_label in devices
        and _HW.tiger_comm_hub_label in devices
        and _HW.galvo_a_label in devices
    )


def ensure_global_shutter_open() -> None:
    """Raise the fiber-optic global shutter and configure the always-on cell.

    No-op when the ASI PLogic hardware is not loaded. This must run once per
    session (e.g. on system-configuration load) before software snap/live can
    gate individual lasers: the laser BNCs are routed to
    ``plogic_always_on_cell``, which is only set up as a constant-high cell
    (and BNC3 raised) inside :func:`open_global_shutter`.
    """
    if not _plogic_available():
        return
    open_global_shutter(
        _HW.plogic_label,
        _HW.tiger_comm_hub_label,
        _HW.plogic_always_on_cell,
        _HW.plogic_bnc3_addr,
    )


def ensure_beam_enabled() -> None:
    """Enable the galvo's beam once per session.

    No-op if the galvo isn't loaded. A captured debug log of the
    microscope-control sibling repo's actual working ``CustomPLogicMDAEngine``
    run showed ``BeamEnabled`` set to ``Yes`` once, as a session-level
    initialization step ("Enabling SPIM beam for the session"), not
    toggled on/off around every individual MDA run the way an earlier
    version of :class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` did.
    """
    if _HW.galvo_a_label not in mmc.getLoadedDevices():
        return
    set_property(_HW.galvo_a_label, "BeamEnabled", "Yes")


def ensure_circular_buffer_capacity() -> None:
    """Pre-allocate a large circular buffer once, at session startup.

    No-op if the buffer is already at least ``HardwareConstants.
    circular_buffer_target_mb``. Deliberately session-level, not per-MDA:
    resizing the circular buffer right after arming cameras for external
    triggering (an earlier version of
    :meth:`~pymmcore_gui.asi_z_stack.engine._ASITriggerEngineBase.
    _ensure_circular_buffer_capacity` did this inside ``setup_sequence``)
    crashed PVCAM's driver even more reliably than the wraparound it was
    meant to prevent -- almost certainly because resizing while a camera is
    already armed leaves the device adapter's buffer pointers stale
    relative to the newly-reallocated core buffer. Doing this once at
    launch, before any camera has been armed for anything, avoids that
    entirely. The one-time allocation cost (a few seconds for tens of GB)
    happens during app startup instead of in the middle of triggering an
    acquisition; snap/live afterward read/write the same pre-sized buffer,
    so they aren't slowed down by it. No-op if PLogic/the Tiger hub aren't
    loaded (demo or non-ASI configs don't need this large a buffer).
    """
    if not _plogic_available():
        return
    target_mb = _HW.circular_buffer_target_mb
    current_mb = mmc.getCircularBufferMemoryFootprint()
    if current_mb >= target_mb:
        return
    logger.info(f"Growing circular buffer footprint {current_mb} -> {target_mb} MB.")
    try:
        mmc.setCircularBufferMemoryFootprint(target_mb)
    except Exception:
        logger.error(
            f"Failed to grow circular buffer to {target_mb} MB; keeping "
            f"{current_mb} MB.",
            exc_info=True,
        )


def _selected_laser_bncs() -> list[int]:
    """BNC addresses for the currently selected laser preset (empty if none)."""
    group = _HW.laser_config_group
    if group not in mmc.getAvailableConfigGroups():
        return []
    preset = mmc.getCurrentConfig(group)
    if preset == _HW.all_lasers_preset:
        return list(_HW.laser_bnc_addr.values())
    addr = _HW.laser_bnc_addr.get(preset)
    return [addr] if addr is not None else []


def open_selected_lasers() -> None:
    """Turn on the laser(s) for the current selection (no-op without PLogic)."""
    if not _plogic_available():
        return
    set_laser_outputs(
        _HW.plogic_label,
        _HW.tiger_comm_hub_label,
        _selected_laser_bncs(),
        True,
        _HW.plogic_always_on_cell,
    )


def close_selected_lasers() -> None:
    """Turn off the laser(s) for the current selection (no-op without PLogic)."""
    if not _plogic_available():
        return
    set_laser_outputs(
        _HW.plogic_label,
        _HW.tiger_comm_hub_label,
        _selected_laser_bncs(),
        False,
        _HW.plogic_always_on_cell,
    )


def close_all_lasers() -> None:
    """Turn off every mapped laser BNC (no-op without PLogic)."""
    if not _plogic_available():
        return
    set_laser_outputs(
        _HW.plogic_label,
        _HW.tiger_comm_hub_label,
        list(_HW.laser_bnc_addr.values()),
        False,
        _HW.plogic_always_on_cell,
    )
