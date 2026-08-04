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

    # Hold shutter-gated lasers open for the whole per-volume burst instead of
    # blanking them per slice. A mechanical shutter (e.g. the 561 line, an
    # Oxxius L4C CW laser behind a physical shutter) cannot follow per-frame
    # TTL toggling reliably -- opening once per stack actuates it twice total
    # instead of once per slice. Diode lasers not listed here keep per-frame
    # blanking (less photobleaching).
    laser_open_full_stack: bool = True
    shutter_gated_wavelengths: tuple[str, ...] = ("561nm",)
    # Delay after opening a shutter-gated laser before the burst starts, so
    # the mechanical shutter is fully open before the first exposure. Tune on
    # the bench/oscilloscope for the actual shutter in use.
    shutter_open_settle_ms: float = 10.0

    # Extended parameters for SPIM Z-stack calculations
    slice_calibration_slope_um_per_deg: float = 100.0

    # Galvo slice-timing defaults. ``delay_before_side_ms``/``delay_before_repeat_ms``
    # are confirmed against the ``microscope-control`` sibling repo's
    # validated-working ``CustomPLogicMDAEngine`` (HEAD commit 5f5f573,
    # ``hardware_profiles/default_config.yml``). These are unrelated to ASI's
    # native per-slice camera/laser trigger properties (``SPIMDelayBeforeCamera(ms)``
    # etc.) -- this design triggers the camera and laser entirely through the
    # PLogic dual-NRT cells (see ``configure_plogic_for_dual_nrt_pulses``), so
    # those native properties are never set (zeroed in engine.py instead).
    #
    # ``line_scan_duration_ms`` was previously kept at 10.5 based on an
    # early, unverified claim that 1.0 produced zero observable galvo
    # motion. That claim is now directly contradicted: a full run of the
    # microscope-control sibling repo's actual CustomPLogicMDAEngine on this
    # exact hardware, captured in its debug log, set
    # "Scanner:AB:33.SPIMScanDuration(ms) = 1.0" and went on to collect all
    # 201 real frames of a z-stack. Reverted to match that confirmed run.
    line_scan_duration_ms: float = 1.0
    delay_before_side_ms: float = 0.0
    delay_before_repeat_ms: float = 0.0

    # MMCore's circular buffer footprint defaults to 100-250 MB, far too
    # small for a full hardware-triggered z-stack on this rig's Kinetix/PVCAM
    # cameras (~11 MB/frame at 2400x2400x16-bit) -- e.g. 4 cameras at 201
    # slices needs over 13 GB with headroom. Provisioned once at session
    # startup (see ensure_circular_buffer_capacity_async in asi_controller.py)
    # so it's never resized mid-acquisition -- resizing right after arming
    # cameras for external triggering crashed PVCAM's driver (see
    # engine.py's _warn_if_circular_buffer_too_small).
    circular_buffer_target_mb: int = 30_000

    # Camera worker-process tuning (see asi_z_stack/camera_worker.py,
    # worker_pool.py, camera_handoff.py). Each physical camera runs in its own
    # OS process for the duration of a hardware-triggered MDA, so a
    # pvcam64.dll crash during concurrent dual-camera acquisition can, at
    # worst, take down one disposable worker instead of the whole app.
    #
    # Bench-measured on the real rig (2026-07-09): a fresh worker process
    # loads pymmcore-plus, initializes a Kinetix/PVCAM camera, and reports
    # ready in ~3-4s -- worker_ready_timeout_s has generous headroom above
    # that. worker_circular_buffer_mb is per-*worker* (one camera each), so
    # it doesn't need circular_buffer_target_mb's full session-wide headroom.
    worker_ready_timeout_s: float = 30.0
    worker_arm_timeout_s: float = 10.0
    worker_shutdown_timeout_s: float = 10.0
    worker_circular_buffer_mb: int = 4096
    frame_ring_slots_per_camera: int = 8
