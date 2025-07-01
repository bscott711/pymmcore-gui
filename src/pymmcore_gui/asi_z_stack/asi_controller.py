# src/microscope/asi_z_stack/asi_controller.py
import time
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

# Direct import - this fixes the Pylance error

if TYPE_CHECKING:
    from .common import AcquisitionSettings

mmc = CMMCorePlus.instance()


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


def _send_tiger_command(cmd: str, tiger_comm_hub_label: str) -> None:
    """Internal helper to send a serial command to the Tiger controller."""
    if tiger_comm_hub_label in mmc.getLoadedDevices():
        mmc.setProperty(tiger_comm_hub_label, "SerialCommand", cmd)
        time.sleep(0.01)
    else:
        print(f"Warning: TigerCommHub not found. Cannot send command: {cmd}")


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

        _send_tiger_command(f"{plogic_addr_prefix}CCA X=0", tiger_comm_hub_label)
        _send_tiger_command(f"M E={plogic_always_on_cell}", tiger_comm_hub_label)
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
