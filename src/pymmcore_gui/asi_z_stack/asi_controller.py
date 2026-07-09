# src/microscope/asi_z_stack/asi_controller.py
import time
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from .common import HardwareConstants

# Direct import - this fixes the Pylance error

if TYPE_CHECKING:
    from .common import AcquisitionSettings

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
        print(
            f"Warning: Cannot set '{property_name}' for device '{device_label}'. "
            "Device or property not found."
        )


def get_property(device_label: str, property_name: str) -> str | None:
    """Safely gets a Micro-Manager device property value."""
    if device_label in mmc.getLoadedDevices() and mmc.hasProperty(
        device_label, property_name
    ):
        return mmc.getProperty(device_label, property_name)
    print(
        f"Warning: Cannot get '{property_name}' for device '{device_label}'. "
        "Device or property not found."
    )
    return None


def _send_tiger_command(cmd: str, tiger_comm_hub_label: str) -> str | None:
    """Internal helper to send a serial command to the Tiger controller.

    Returns the ``SerialResponse`` property's value if the hub exposes one,
    so query-style commands (e.g. ``RA``) can be read back by callers; other
    commands can simply ignore the return value.
    """
    if tiger_comm_hub_label in mmc.getLoadedDevices():
        print(f"  -> {cmd}")
        mmc.setProperty(tiger_comm_hub_label, "SerialCommand", cmd)
        time.sleep(0.01)
        if mmc.hasProperty(tiger_comm_hub_label, "SerialResponse"):
            response = mmc.getProperty(tiger_comm_hub_label, "SerialResponse")
            print(f"  <- {response}")
            return response
    else:
        print(f"Warning: TigerCommHub not found. Cannot send command: {cmd}")
    return None


def set_plogic_evaluation_clock(
    plogic_label: str, tiger_comm_hub_label: str, running: bool
) -> None:
    """Send the raw ``PM E=<0|1>`` command around a triggered acquisition.

    Matches the ``microscope-control`` sibling repo's oscilloscope-confirmed
    working ``PLogicMDAEngine`` (``PM E=1`` sent once right before triggering
    the galvo, ``PM E=0`` during cleanup) exactly, including sending it at
    all: ASI's PLogic docs describe ``PM``'s ``E`` parameter as selecting the
    card's cell-evaluation clock source rather than an arm/disarm toggle, and
    an earlier version of this codebase removed this call on that basis --
    but the sibling repo sends it unconditionally and its acquisitions do
    trigger the camera and laser, so that removal was an unconfirmed,
    documentation-only guess. What ``PM E`` actually does on this hardware is
    still not settled; this function exists to match known-working behavior,
    not because its effect is understood.
    """
    plogic_addr_prefix = plogic_label.split(":")[-1]
    mode = 1 if running else 0
    _send_tiger_command(f"{plogic_addr_prefix}PM E={mode}", tiger_comm_hub_label)


def enable_axis_spim_ttl_output(axis_label: str, tiger_comm_hub_label: str) -> None:
    """Send ``TTL X=0 Y=20`` on an axis card to arm its per-slice trigger.

    ASI's serial command docs define ``TTL`` output mode ``Y=20`` as "TTL
    OUT0 set during SPIM state machine operation" (requires the ``MM_SPIM``
    firmware module) -- i.e. the card pulses its own ``TTL OUT0`` line each
    time the SPIM state machine settles at a new slice. Mode ``0`` (the
    card's default) is "TTL OUT0 unconditionally set LOW", so without this
    call the card never emits a trigger pulse of any kind, regardless of how
    PLogic is wired downstream. Currently only used by the galvo-driven
    :class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` (not currently
    registered by the GUI -- see its docstring). ASI's own reference plugin
    (see :class:`~pymmcore_gui.asi_z_stack.engine.ASIStationaryTriggerEngine`'s
    docstring) never sends this command at all, so the currently-registered
    engine doesn't call it either.

    This is also the documented mechanism behind ASI's diSPIM reference
    architecture, where the master card's settle pulse feeds PLogic's own
    cell-evaluation clock (selected via ``PM E=1``, PLC clock-source code 1,
    "Backplane C7" -- see :func:`set_plogic_evaluation_clock`), so the whole
    16-cell array, including the dual-NRT camera/laser trigger cells, only
    re-evaluates at the instant the master axis has settled. Without this
    call, ``PM E=1`` selects an evaluation clock line nothing is driving --
    the array never re-evaluates again, which matched the exact symptom
    observed with the galvo as master: it stepped through its own SPIM
    state machine as usual (that motion doesn't depend on PLogic), but no
    PLogic cell, including the camera/laser NRT ones, ever fired.
    """
    axis_addr_prefix = axis_label.split(":")[-1]
    _send_tiger_command(f"{axis_addr_prefix}TTL X=0 Y=20", tiger_comm_hub_label)
    _send_tiger_command(f"{axis_addr_prefix}SS Z", tiger_comm_hub_label)


def reset_axis_ttl_output(axis_label: str, tiger_comm_hub_label: str) -> None:
    """Send ``TTL X=0 Y=0`` on an axis card, restoring its documented default.

    ``Y=0`` is "TTL OUT0 unconditionally set LOW" -- ASI's documented
    default, and the state :class:`~pymmcore_gui.asi_z_stack.engine.
    ASISPIMEngine` (and the confirmed-working microscope-control sibling
    repo, which never touches ``TTL X=/Y=`` at all) both implicitly assume
    the galvo card is already in.

    This exists to undo, not merely avoid: earlier debugging this session
    called :func:`enable_axis_spim_ttl_output` (``TTL X=0 Y=20``) on the
    galvo several times, each followed by ``SS Z`` -- saving mode 20 to the
    card's non-volatile memory. Simply no longer calling that function does
    *not* revert the card's persisted state; a live bench test showed the
    galvo still failed to trigger the camera after that call was removed
    from ``ASISPIMEngine``, with the timeout/symptom otherwise identical to
    every earlier galvo-driven attempt this session -- consistent with the
    card still silently carrying the mode-20 setting from an earlier,
    now-removed code path. Call this once during setup to guarantee a known
    state instead of relying on the card never having been touched.
    """
    axis_addr_prefix = axis_label.split(":")[-1]
    _send_tiger_command(f"{axis_addr_prefix}TTL X=0 Y=0", tiger_comm_hub_label)
    _send_tiger_command(f"{axis_addr_prefix}SS Z", tiger_comm_hub_label)


def open_global_shutter(
    plogic_label: str,
    tiger_comm_hub_label: str,
    plogic_always_on_cell: int,
    plogic_bnc3_addr: int,
) -> None:
    """Configures and opens a global shutter on PLogic BNC3."""
    print("Opening global shutter (BNC3 HIGH)...")
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)

    try:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        _send_tiger_command(f"M E={plogic_always_on_cell}", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCA X=0", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCA Y=0", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCA Z=5", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCB X=1", tiger_comm_hub_label)
        _send_tiger_command(f"M E={plogic_bnc3_addr}", tiger_comm_hub_label)
        _send_tiger_command(
            f"{plogic_addr_prefix}CCA Z={plogic_always_on_cell}", tiger_comm_hub_label
        )
        _send_tiger_command(f"{plogic_addr_prefix}SS Z", tiger_comm_hub_label)
        print("Global shutter is open (BNC3 is HIGH).")
    except Exception as e:
        print(f"Error opening global shutter: {e}")
    finally:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "Yes")


def close_global_shutter(
    plogic_label: str, tiger_comm_hub_label: str, plogic_bnc3_addr: int
) -> None:
    """Closes the global shutter on PLogic BNC3."""
    print("Closing global shutter (BNC3 LOW)...")
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)

    try:
        if plogic_label not in mmc.getLoadedDevices():
            print("PLogic device not found, cannot close shutter.")
            return

        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        _send_tiger_command(f"M E={plogic_bnc3_addr}", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}CCA Z=0", tiger_comm_hub_label)
        _send_tiger_command(f"{plogic_addr_prefix}SS Z", tiger_comm_hub_label)
        print("Global shutter is closed (BNC3 is LOW).")
    except Exception as e:
        print(f"Warning: Could not close global shutter. Error: {e}")
    finally:
        if (
            original_hub_setting == "Yes"
            and get_property(tiger_comm_hub_label, hub_prop) == "No"
        ):
            set_property(tiger_comm_hub_label, hub_prop, "Yes")


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
        print(f"Warning: Camera '{camera_label}' not found.")
        return False

    trigger_prop = "TriggerMode"
    if not mmc.hasProperty(camera_label, trigger_prop):
        print(f"Warning: Camera '{camera_label}' has no 'TriggerMode' property.")
        return False

    # List of desired trigger modes, in order of preference
    desired_modes = ["Level Trigger", "Edge Trigger"]
    try:
        allowed_modes = mmc.getAllowedPropertyValues(camera_label, trigger_prop)
        for mode in desired_modes:
            if mode in allowed_modes:
                print(f"Setting '{camera_label}' trigger mode to '{mode}'")
                mmc.setProperty(camera_label, trigger_prop, mode)
                return True
        print(f"Warning: Could not find a suitable trigger mode for '{camera_label}'")
        return False
    except Exception as e:
        print(f"Error setting trigger mode for '{camera_label}': {e}")
        return False


def configure_plogic_for_dual_nrt_pulses(
    settings: "AcquisitionSettings",
    plogic_label: str,
    tiger_comm_hub_label: str,
    plogic_laser_preset_num: int,
    plogic_camera_cell: int,
    pulses_per_ms: float,
    plogic_4khz_clock_addr: int,
    plogic_trigger_ttl_addr: int,
    plogic_laser_on_cell: int,
) -> None:
    """Configures PLogic for two independent, synchronized NRT one-shot pulses."""
    plogic_addr_prefix = plogic_label.split(":")[-1]
    hub_prop = "OnlySendSerialCommandOnChange"
    original_hub_setting = get_property(tiger_comm_hub_label, hub_prop)

    def _send(cmd: str) -> None:
        _send_tiger_command(cmd, tiger_comm_hub_label)

    try:
        if original_hub_setting == "Yes":
            set_property(tiger_comm_hub_label, hub_prop, "No")

        # 1. Program Laser Preset
        _send(f"{plogic_addr_prefix}CCA X={plogic_laser_preset_num}")
        print(f"Laser preset number: {plogic_laser_preset_num}")

        # 2. Program Camera Pulse (NRT One-Shot #1)
        _send(f"M E={plogic_camera_cell}")
        _send(f"{plogic_addr_prefix}CCA Y=14")  # NRT one-shot mode
        camera_pulse_cycles = int(settings.camera_exposure_ms * pulses_per_ms)
        _send(f"{plogic_addr_prefix}CCA Z={camera_pulse_cycles}")
        _send(
            f"{plogic_addr_prefix}CCB X={plogic_trigger_ttl_addr} "
            f"Y={plogic_4khz_clock_addr} Z=0"
        )

        # 3. Program Laser Pulse (NRT One-Shot #2)
        _send(f"M E={plogic_laser_on_cell}")
        _send(f"{plogic_addr_prefix}CCA Y=14")  # NRT one-shot mode
        laser_pulse_cycles = int(settings.laser_trig_duration_ms * pulses_per_ms)
        _send(f"{plogic_addr_prefix}CCA Z={laser_pulse_cycles}")
        _send(
            f"{plogic_addr_prefix}CCB X={plogic_trigger_ttl_addr} "
            f"Y={plogic_4khz_clock_addr} Z=0"
        )

        # 4. Route Camera Trigger Cell Output to BNC1 (Address 33)
        _send("M E=33")
        _send(f"{plogic_addr_prefix}CCA Z={plogic_camera_cell}")

        # 5. Save the configuration
        _send(f"{plogic_addr_prefix}SS Z")
        print("PLogic configured for dual NRT pulses.")

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
        print(f"  [PLogic] {label} = {value}")


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
