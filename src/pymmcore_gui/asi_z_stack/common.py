# src/pymmcore_gui/asi_z_stack/common.py
from dataclasses import dataclass, field


@dataclass
class AcquisitionSettings:
    """Stores all user-configurable acquisition parameters."""

    num_slices: int = 3
    step_size_um: float = 1.0
    laser_trig_duration_ms: float = 10.0
    camera_exposure_ms: float = 10.0


@dataclass
class HardwareConstants:
    """Stores fixed hardware configuration and constants."""

    cfg_path: str = "hardware_profiles/20250701-SingleChannelOPM.cfg"
    galvo_a_label: str = "Scanner:AB:33"
    piezo_a_label: str = "PiezoStage:P:34"
    camera_a_label: str = "Camera-1"
    plogic_label: str = "PLogic:E:36"
    tiger_comm_hub_label: str = "TigerCommHub"
    plogic_trigger_ttl_addr: int = 41
    plogic_4khz_clock_addr: int = 192
    plogic_laser_on_cell: int = 10
    plogic_camera_cell: int = 11
    plogic_always_on_cell: int = 12
    plogic_bnc3_addr: int = 35
    pulses_per_ms: float = 4.0
    plogic_laser_preset_num: int = 30

    # Per-laser PLogic BNC outputs used for software snap/live gating.
    # Front-panel BNC n maps to PLogic address 32 + n (so BNC3 = 35, as used by
    # the global shutter). Each wavelength has its own BNC: 638->5, 488->6,
    # 405->7, 561->8. A laser is turned on by pointing its BNC source at the
    # always-on cell (``plogic_always_on_cell``) and off by pointing it at 0.
    laser_config_group: str = "Lasers"
    all_lasers_preset: str = "AllLasers"
    laser_bnc_addr: dict[str, int] = field(
        default_factory=lambda: {
            "638nm": 37,
            "488nm": 38,
            "405nm": 39,
            "561nm": 40,
        }
    )

    # Extended parameters for SPIM Z-stack calculations
    slice_calibration_slope_um_per_deg: float = 100.0
    line_scans_per_slice: int = 1
    delay_before_scan_ms: float = 0.0
    line_scan_duration_ms: float = 1.0
    delay_before_side_ms: float = 0.0
