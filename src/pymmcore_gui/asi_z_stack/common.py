# src/microscope/asi_z_stack/common.py
from dataclasses import dataclass


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
