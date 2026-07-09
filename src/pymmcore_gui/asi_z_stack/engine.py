# src/pymmcore_gui/asi_z_stack/engine.py
import logging
import time
from collections.abc import Iterable, Iterator

import numpy as np
from pymmcore_plus import CMMCorePlus
from pymmcore_plus.mda import MDAEngine
from pymmcore_plus.metadata import (
    FrameMetaV1,
    SummaryMetaV1,
    summary_metadata,
)
from useq import MDAEvent, MDASequence

from pymmcore_gui._multi_camera_handler import physical_camera_labels

from .asi_controller import (
    configure_plogic_for_dual_nrt_pulses,
    log_plogic_trigger_chain_state,
    set_camera_trigger_mode,
    set_plogic_evaluation_clock,
)
from .common import AcquisitionSettings, HardwareConstants

logger = logging.getLogger(__name__)


class _ASITriggerEngineBase(MDAEngine):
    """Shared plumbing for ASI PLogic-triggered SPIM MDA engines.

    Subclasses drive the galvo as the SPIM state machine's "master" -- the
    device whose ``SPIMState`` property is toggled to trigger a stack, and
    whose settle pulses feed PLogic once :func:`set_plogic_evaluation_clock`
    (``PM E=1``) selects them as its cell-evaluation clock -- see
    :class:`ASISPIMEngine` (real z-stack, galvo amplitude derived from the
    z-plan) and :class:`ASIStationaryTriggerEngine` (galvo held stationary,
    piezo armed alongside it, both amplitude ~0 -- see its docstring for
    why). Everything below is genuinely identical between them: draining
    the camera's circular buffer, tagging frames with camera/slice indices,
    and saving/restoring each camera's ``TriggerMode`` around the MDA.
    """

    #: Device label whose SPIMState property triggers/idles a stack. Set by
    #: each subclass's __init__ before setup_sequence runs.
    _master_axis_label: str

    #: Value to write to SPIMState to start a stack. Not necessarily the
    #: same across axis card types -- the galvo's SPIMState accepts
    #: "Running" directly, but a live bench test showed the piezo's allowed
    #: values are only ('Armed', 'Idle') (mmc.getAllowedPropertyValues),
    #: with no "Running" value at all; setting "Running" on it fails
    #: immediately in the ASI adapter's own validation, before any serial
    #: command is even sent. Matches ASI's own reference plugin (see
    #: ASIStationaryTriggerEngine's docstring): the piezo is only ever
    #: armed, never told to run -- the galvo is always what receives
    #: "Running". No current subclass overrides this default, since none of
    #: them use the piezo as _master_axis_label; kept as an extension point.
    _trigger_spim_state_value: str = "Running"

    def __init__(self, mmc: CMMCorePlus, hw: HardwareConstants):
        # We disable pymmcore-plus hardware sequencing because the ASI SPIM
        # state machine handles the Z-stack sequencing internally via TTL
        # triggers.
        super().__init__(mmc, use_hardware_sequencing=False)
        self.hw = hw
        self._num_slices = 0
        self._exposure_ms = 10.0
        self._original_autoshutter = True
        self._original_trigger_modes: dict[str, str] = {}

    def _arm_cameras(self) -> None:
        """Switch every physical camera to external triggering.

        A "Multi Camera" utility device has no TriggerMode property of its
        own -- each physical camera behind it must be switched individually.
        Saves each camera's pre-MDA TriggerMode so :meth:`_restore_cameras`
        can put it back -- without this, a camera left in "Level Trigger"
        after the MDA ends (whether it succeeded or not) hangs the next
        Live/Snap waiting for an external trigger that never comes.
        """
        self._original_trigger_modes = {}
        for cam_label in physical_camera_labels(self.mmcore):
            if self.mmcore.hasProperty(cam_label, "TriggerMode"):
                self._original_trigger_modes[cam_label] = self.mmcore.getProperty(
                    cam_label, "TriggerMode"
                )
            set_camera_trigger_mode(cam_label)

    def _warn_if_circular_buffer_too_small(self, total_images: int) -> None:
        """Log a warning if the circular buffer can't fit a full stack.

        MMCore's default circular buffer footprint (100-250 MB) is far
        smaller than a full hardware-triggered z-stack on this rig's
        Kinetix/PVCAM cameras (~11 MB/frame at 2400x2400x16-bit) -- e.g. a
        201-slice, 2-camera acquisition needs ~4.4 GB. An earlier version of
        this method resized the buffer here, mid-``setup_sequence`` (right
        after :meth:`_arm_cameras`), and that crashed PVCAM's driver
        (``pvcam64.dll``, exception ``0xc0000409`` /
        STATUS_STACK_BUFFER_OVERRUN) even *more* reliably than the
        wraparound it was meant to prevent -- almost certainly because
        resizing while a camera is already armed for external triggering
        leaves the device adapter's buffer pointers stale relative to the
        newly-reallocated core buffer. The buffer is now sized once, at
        session startup, before any camera is ever armed -- see
        :func:`~pymmcore_gui.asi_z_stack.asi_controller.
        ensure_circular_buffer_capacity`. This just warns if that session-
        level allocation somehow isn't enough for the current sequence,
        rather than trying to fix it here.
        """
        mmc = self.mmcore
        bytes_per_frame = (
            mmc.getImageWidth() * mmc.getImageHeight() * mmc.getBytesPerPixel()
        )
        if bytes_per_frame <= 0:
            return
        required_mb = (bytes_per_frame * total_images * 1.5) / (1024 * 1024)
        current_mb = mmc.getCircularBufferMemoryFootprint()
        if required_mb > current_mb:
            logger.warning(
                f"Circular buffer ({current_mb} MB) may be too small for "
                f"{total_images} frames (~{required_mb:.0f} MB needed) -- "
                "consider raising HardwareConstants.circular_buffer_target_mb."
            )

    def _restore_cameras(self) -> None:
        """Undo :meth:`_arm_cameras`."""
        for cam_label, original_mode in self._original_trigger_modes.items():
            if cam_label in self.mmcore.getLoadedDevices():
                self.mmcore.setProperty(cam_label, "TriggerMode", original_mode)

    def event_iterator(self, events: Iterable[MDAEvent]) -> Iterator[MDAEvent]:
        """Collapse each hardware z-stack down to a single event.

        The ASI SPIM state machine acquires an entire z-stack per trigger,
        so :meth:`exec_event` yields every slice itself. useq emits one
        event per z-slice, so forward only the first slice of each stack
        (``z`` index 0, or events with no ``z`` axis) and drop the rest --
        otherwise the stack would be re-triggered once per slice.
        """
        for event in events:
            if event.index.get("z", 0) == 0:
                yield event

    def exec_event(
        self, event: MDAEvent
    ) -> Iterable[tuple[np.ndarray, MDAEvent, FrameMetaV1]]:
        """Trigger the PLogic/SPIM stack and yield one payload per slice/camera.

        A single event (see :meth:`event_iterator`) drives the whole z-stack.
        Frames land interleaved in the circular buffer; each is tagged with its
        physical camera (``camera_device`` + ``cam`` index) and its slice (``z``
        index) so per-camera writers receive a full, distinct z-stack.
        """
        mmc = self.mmcore
        active_cam = mmc.getCameraDevice()
        n_cameras = mmc.getNumberOfCameraChannels()
        total_images = self._num_slices * n_cameras

        # Arm the buffer and trigger the hardware z-stack.
        mmc.startSequenceAcquisition(active_cam, total_images, 0, True)
        logger.info(
            f"Camera armed for {total_images} images "
            f"(sequence running: {mmc.isSequenceRunning()})."
        )
        # See _trigger_spim_state_value's docstring: currently always
        # "Running" on the galvo (its NO_SCAN/SLICE_SCAN_ONLY trigger path)
        # -- the piezo's SPIMState property has no "Running" value at all,
        # so no engine here uses it as _master_axis_label.
        mmc.setProperty(
            self._master_axis_label, "SPIMState", self._trigger_spim_state_value
        )
        spim_state = mmc.getProperty(self._master_axis_label, "SPIMState")
        logger.info(f"{self._master_axis_label} SPIMState readback: '{spim_state}'.")

        runner_t0 = event.metadata.get("runner_t0")
        # Per-physical-camera counter -> the z index of that camera's next slice.
        slice_counts: dict[int, int] = {}
        images_collected = 0
        # A fixed total-duration budget (total_images * exposure_ms + grace) is
        # too tight: measured on the bench at ~10ms exposure, sustained
        # throughput is ~36ms/image once galvo settle, PLogic cycle overhead,
        # and readout are included, not the nominal 10ms -- so a long sequence
        # can still be steadily progressing when a total-duration budget runs
        # out. Time out only on a genuine stall (no new image for
        # stall_timeout_s) instead, so slower-than-nominal but healthy
        # throughput never trips it.
        stall_timeout_s = 5.0
        start_time = time.time()
        last_progress_log = start_time
        last_image_time = start_time

        while images_collected < total_images:
            now = time.time()
            if now - last_image_time > stall_timeout_s:
                raise TimeoutError(
                    f"Acquisition stalled: no new image for "
                    f"{stall_timeout_s:.1f}s ({images_collected}/{total_images} "
                    "collected)."
                )
            if now - last_progress_log > 1.0:
                # Camera-buffer-side only -- no Tiger serial traffic here.
                # Once triggered, this acquisition is fully hardware-timed;
                # the wait loop must not compete with the SPIM state
                # machine's own timing for the serial link.
                logger.debug(
                    f"...waiting: {images_collected}/{total_images} collected, "
                    f"{mmc.getRemainingImageCount()} buffered, "
                    f"sequence running: {mmc.isSequenceRunning()}."
                )
                last_progress_log = now

            remaining = mmc.getRemainingImageCount()
            if remaining > 0:
                img, mm_meta = mmc.popNextImageAndMD()

                # Physical-camera channel: the Multi Camera adapter tags each
                # frame with a ``*CameraChannelIndex``; single-camera frames
                # default to channel 0.
                ch_index = int(
                    next(
                        (
                            v
                            for k, v in mm_meta.items()
                            if k.endswith("CameraChannelIndex")
                        ),
                        0,
                    )
                )
                try:
                    # In circular-buffer metadata this tag is literally "Camera"
                    # (the physical camera label), not Keyword.CoreCamera.
                    camera_device = mm_meta.GetSingleTag("Camera").GetValue()
                except Exception:
                    camera_device = mmc.getPhysicalCameraDevice(ch_index)

                slice_idx = slice_counts.get(ch_index, 0)
                slice_counts[ch_index] = slice_idx + 1

                new_index = {**event.index, "z": slice_idx}
                if n_cameras > 1:
                    new_index["cam"] = ch_index
                sub_event = event.model_copy(update={"index": new_index})

                runner_time_ms = (
                    (time.perf_counter() - runner_t0) * 1000.0 if runner_t0 else 0.0
                )
                meta = self.get_frame_metadata(
                    sub_event,
                    prop_values=(),
                    runner_time_ms=runner_time_ms,
                    camera_device=camera_device,
                    include_position=self._include_frame_position_metadata is True,
                )
                meta["hardware_triggered"] = True
                meta["images_remaining_in_buffer"] = remaining - 1
                yield img, sub_event, meta
                images_collected += 1
                last_image_time = now
            elif not mmc.isSequenceRunning():
                raise RuntimeError(
                    f"Sequence stopped unexpectedly after {images_collected} images."
                )
            else:
                time.sleep(0.005)


class ASISPIMEngine(_ASITriggerEngineBase):
    """Custom MDA Engine for ASI SPIM Z-stacks, galvo-driven TTL triggering.

    ``setup_sequence`` deliberately does NOT call
    :func:`~pymmcore_gui.asi_z_stack.asi_controller.open_global_shutter` --
    that's session-level (opened once via ``ensure_global_shutter_open``),
    not per-MDA. Confirmed on the bench: that function's first command
    (``CCA X=0``) has no ``M E=`` guard, so it lands on whatever PLogic cell
    was last selected. Called right after
    :func:`~pymmcore_gui.asi_z_stack.asi_controller.
    configure_plogic_for_dual_nrt_pulses` (as it used to be here), that's
    address 33 (BNC1) -- clobbering the camera-cell routing that call just
    set up, silently, every single MDA run. Likewise ``BeamEnabled`` is set
    once per session (``ensure_beam_enabled``), not toggled per MDA.
    """

    def __init__(self, mmc: CMMCorePlus, hw: HardwareConstants):
        super().__init__(mmc, hw)
        self._master_axis_label = hw.galvo_a_label

    def setup_sequence(self, sequence: MDASequence) -> SummaryMetaV1 | None:
        """Prepare hardware and calculate Z-stack parameters."""
        # 1. Calculate Z-stack parameters
        if sequence.z_plan:
            z_positions = list(sequence.z_plan)
            self._num_slices = len(z_positions)
            step_size_um = (
                abs(z_positions[1] - z_positions[0]) if self._num_slices > 1 else 0.0
            )
            amplitude_um = (self._num_slices - 1) * step_size_um
            galvo_amplitude_deg = (
                amplitude_um / self.hw.slice_calibration_slope_um_per_deg
            )
        else:
            self._num_slices = 1
            galvo_amplitude_deg = 0.0

        # 2. Determine exposure
        if (
            sequence.channels
            and len(sequence.channels) > 0
            and sequence.channels[0].exposure
        ):
            self._exposure_ms = sequence.channels[0].exposure
        else:
            self._exposure_ms = self.mmcore.getExposure()

        # 3. Prepare Hardware
        logger.info("--- SEQUENCE STARTED: Preparing PLogic and Camera ---")
        self._original_autoshutter = self.mmcore.getAutoShutter()
        self.mmcore.setAutoShutter(False)
        self._arm_cameras()
        self._warn_if_circular_buffer_too_small(
            self._num_slices * self.mmcore.getNumberOfCameraChannels()
        )

        settings = AcquisitionSettings(
            camera_exposure_ms=self._exposure_ms,
            laser_trig_duration_ms=self._exposure_ms,
        )
        configure_plogic_for_dual_nrt_pulses(
            settings,
            self.hw.plogic_label,
            self.hw.tiger_comm_hub_label,
            self.hw.plogic_laser_preset_num,
            self.hw.plogic_camera_cell,
            self.hw.pulses_per_ms,
            self.hw.plogic_4khz_clock_addr,
            self.hw.plogic_trigger_ttl_addr,
            self.hw.plogic_laser_on_cell,
        )

        # Deliberately not calling open_global_shutter() here -- it's
        # session-level (opened once via ensure_global_shutter_open), not
        # per-MDA. Confirmed on the bench: its first command (`CCA X=0`) has
        # no M E= guard, so it lands on whatever cell was last selected --
        # which, called right after configure_plogic_for_dual_nrt_pulses, is
        # address 33 (BNC1), clobbering the camera-cell routing that call
        # just set up. The sibling repo never has this problem because it
        # only calls open_global_shutter once at startup, before any PLogic
        # cell has ever been selected.

        # 4. Configure Galvo. BeamEnabled is deliberately not touched here --
        # see ensure_beam_enabled in asi_controller.py.
        logger.debug("Configuring ASI galvo for SPIM scan...")
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SPIMNumSlices", str(self._num_slices)
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SingleAxisYAmplitude(deg)",
            f"{galvo_amplitude_deg:.4f}",
        )
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMNumRepeats", "1")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMNumSides", "1")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMFirstSide", "A")
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SPIMAlternateDirectionsEnable", "No"
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SPIMScanDuration(ms)",
            str(self.hw.line_scan_duration_ms),
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SPIMInterleaveSidesEnable", "No"
        )
        # Reverted: an earlier version of this line set "Yes" (disable piezo
        # homing), on the theory that a piezo-home wait was stalling the
        # state machine. That was unconfirmed and turned out to contradict
        # the microscope-control sibling repo's reference config
        # (hardware_profiles/default_config.yml), which explicitly uses "No"
        # (piezo home enabled, the SCANR default) for this exact galvo card
        # -- match that known-good value instead of guessing.
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMPiezoHomeDisable", "No")
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SingleAxisXAmplitude(deg)", "0.0"
        )
        self.mmcore.setProperty(self.hw.galvo_a_label, "SingleAxisXOffset(deg)", "0.0")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SingleAxisYOffset(deg)", "0.0")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMNumSlicesPerPiezo", "1")
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SPIMDelayBeforeRepeat(ms)",
            str(self.hw.delay_before_repeat_ms),
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SPIMDelayBeforeSide(ms)",
            str(self.hw.delay_before_side_ms),
        )
        # Deliberately not touching ASI's native per-slice camera/laser
        # trigger properties (SPIMDelayBeforeScan(ms), SPIMDelayBeforeCamera(ms),
        # SPIMCameraDuration(ms), SPIMDelayBeforeLaser(ms), SPIMLaserDuration(ms))
        # -- the microscope-control sibling repo's confirmed-working engine
        # never touches them either, driving the camera and laser entirely
        # through PLogic's dual-NRT cells. Whatever is currently persisted
        # on the card for those SPIM properties is left alone.
        #
        # No raw "TTL X=/Y=" command is sent to the scanner anywhere in
        # this engine -- confirmed against a captured log of the sibling
        # repo's real, successful run, which never sends one either. The
        # SPIM scan is driven entirely through the SPIMState/SingleAxis*
        # device-adapter properties set above.
        set_plogic_evaluation_clock(self.hw.tiger_comm_hub_label, running=True)

        logger.info(
            f"--- PLogic and Camera ready --- "
            f"(slices={self._num_slices}, amplitude={galvo_amplitude_deg:.4f}deg, "
            f"exposure={self._exposure_ms:.1f}ms)"
        )

        # Return summary metadata. This automatically queries
        # getNumberOfCameraChannels() and informs the OmeWritersSink
        # that it needs to expect interleaved multi-camera data.
        return summary_metadata(self.mmcore, mda_sequence=sequence)

    def teardown_sequence(self, sequence: MDASequence) -> None:
        """Clean up hardware state after the sequence finishes."""
        logger.info("--- SEQUENCE FINISHED: Cleaning up hardware ---")
        set_plogic_evaluation_clock(self.hw.tiger_comm_hub_label, running=False)
        if self.hw.galvo_a_label in self.mmcore.getLoadedDevices():
            self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMState", "Idle")

        # Deliberately not closing the global shutter here -- it's
        # session-level infrastructure (opened once via
        # ensure_global_shutter_open so software snap/live can gate
        # individual lasers), not something to toggle per MDA run. The
        # sibling repo's own cleanup never closes it either.
        time.sleep(0.1)
        self.mmcore.setAutoShutter(self._original_autoshutter)
        self._restore_cameras()
        logger.info("--- Hardware cleanup complete ---")


class ASIStationaryTriggerEngine(_ASITriggerEngineBase):
    """PLogic-triggered MDA engine with the galvo and piezo both held still.

    Rebuilt from ASI's own reference implementation (the ``ASIdiSPIM``
    Micro-Manager plugin's ``ControllerUtils.java``,
    ``triggerControllerStartAcquisition``/``prepareControllerForAquisition_Side``
    methods -- see https://github.com/mdcurtis/micromanager-upstream/blob/
    master/plugins/ASIdiSPIM/src/org/micromanager/asidispim/Utils/
    ControllerUtils.java), not a guess: in every non-stage-scan acquisition
    mode, including piezo-driven ones, that plugin *never* sets the piezo's
    ``SPIMState`` to ``"Running"`` -- it only ever arms it (``"Armed"``,
    set during ``prepareControllerForAquisition_Side``, alongside its
    ``SingleAxisAmplitude``/``SingleAxisOffset``). The device that actually
    receives ``SPIMState="Running"`` to kick off the whole synchronized
    sequence is always the galvo/scanner. This matches what a live bench
    test found independently: the piezo's ``SPIMState`` property has no
    ``"Running"`` value at all (``mmc.getAllowedPropertyValues`` returned
    only ``('Armed', 'Idle')``); trying to set it failed instantly in the
    ASI adapter's own validation, before any serial command was even sent.

    So: the galvo is the trigger master (:class:`ASISPIMEngine`'s mechanism,
    ``SPIMState="Running"``), but its own ``SingleAxisYAmplitude(deg)`` is
    forced near zero here instead of being derived from the z-plan, so it
    does not optically scan either -- matching the user's requirement that
    *nothing* physically moves, since this engine exists purely to validate
    the trigger chain (and later, CRISP coexistence) in isolation from any
    real Z motion. The piezo is armed in parallel (``SPIMState="Armed"``,
    also near-zero amplitude) since ASI's reference always arms it even in
    modes where it isn't the one stepping, suggesting the Tiger controller's
    internal multi-axis SPIM coordination may expect it.

    ASI's reference plugin also never sends ``TTL X=/Y=`` or ``PM E=``
    anywhere in this file -- both dropped here, on the theory that whatever
    physical/firmware path carries the galvo's settle pulse to PLogic is
    either fixed/hardwired or handled internally by mechanisms this engine
    doesn't need to touch directly. This engine keeps the existing dual-NRT
    PLogic cell programming (:func:`configure_plogic_for_dual_nrt_pulses`)
    rather than replicating ASI's actual camera/laser mechanism
    (``setupHardwareChannelSwitching`` in the same file: a counter cell
    clocked by the falling edge of backplane TTL1/address 42, cycling
    through laser-channel BNCs via a mod-N counter) -- that's a
    substantially more complex, multi-channel-cycling design that's out of
    scope unless dual-NRT still doesn't work with this corrected trigger
    architecture.
    """

    def __init__(self, mmc: CMMCorePlus, hw: HardwareConstants):
        super().__init__(mmc, hw)
        self._master_axis_label = hw.galvo_a_label
        # _trigger_spim_state_value stays at the base class default
        # ("Running") -- matches ASI's reference, which always sends
        # SPIMState="Running" to the galvo/scanner, never the piezo.

    def setup_sequence(self, sequence: MDASequence) -> SummaryMetaV1 | None:
        """Prepare hardware; both galvo and piezo stay stationary."""
        # 1. Number of trigger pulses/frames wanted. Neither axis moves, so
        # there's no amplitude/step-size to compute from the z_plan
        # positions -- only their count matters.
        if sequence.z_plan:
            self._num_slices = len(list(sequence.z_plan))
        else:
            self._num_slices = 1

        # 2. Determine exposure
        if (
            sequence.channels
            and len(sequence.channels) > 0
            and sequence.channels[0].exposure
        ):
            self._exposure_ms = sequence.channels[0].exposure
        else:
            self._exposure_ms = self.mmcore.getExposure()

        # 3. Prepare hardware
        logger.info(
            "--- SEQUENCE STARTED: Preparing PLogic and Camera (stationary) ---"
        )
        self._original_autoshutter = self.mmcore.getAutoShutter()
        self.mmcore.setAutoShutter(False)
        self._arm_cameras()
        self._warn_if_circular_buffer_too_small(
            self._num_slices * self.mmcore.getNumberOfCameraChannels()
        )

        settings = AcquisitionSettings(
            camera_exposure_ms=self._exposure_ms,
            laser_trig_duration_ms=self._exposure_ms,
        )
        configure_plogic_for_dual_nrt_pulses(
            settings,
            self.hw.plogic_label,
            self.hw.tiger_comm_hub_label,
            self.hw.plogic_laser_preset_num,
            self.hw.plogic_camera_cell,
            self.hw.pulses_per_ms,
            self.hw.plogic_4khz_clock_addr,
            self.hw.plogic_trigger_ttl_addr,
            self.hw.plogic_laser_on_cell,
        )

        # Deliberately not calling open_global_shutter() here -- see
        # ASISPIMEngine.setup_sequence's comment on the same point.

        # 4. Configure the galvo as trigger master, held stationary.
        # SingleAxisYAmplitude(deg) is set just below the piezo's own step
        # resolution equivalent for the galvo (a small nonzero value, not
        # exactly 0) -- the piezo's identical-shaped property hard-rejected
        # SPIMState=Running at amplitude 0.0 on the bench, so the same
        # nonzero-amplitude validation is assumed possible here too.
        logger.debug("Configuring ASI galvo as a stationary SPIM trigger master...")
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SPIMNumSlices", str(self._num_slices)
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SingleAxisYAmplitude(deg)", "0.0001"
        )
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMNumRepeats", "1")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMNumSides", "1")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMFirstSide", "A")
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SPIMAlternateDirectionsEnable", "No"
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SPIMScanDuration(ms)",
            str(self.hw.line_scan_duration_ms),
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SPIMInterleaveSidesEnable", "No"
        )
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMPiezoHomeDisable", "No")
        self.mmcore.setProperty(
            self.hw.galvo_a_label, "SingleAxisXAmplitude(deg)", "0.0"
        )
        self.mmcore.setProperty(self.hw.galvo_a_label, "SingleAxisXOffset(deg)", "0.0")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SingleAxisYOffset(deg)", "0.0")
        self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMNumSlicesPerPiezo", "1")
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SPIMDelayBeforeRepeat(ms)",
            str(self.hw.delay_before_repeat_ms),
        )
        self.mmcore.setProperty(
            self.hw.galvo_a_label,
            "SPIMDelayBeforeSide(ms)",
            str(self.hw.delay_before_side_ms),
        )

        # 5. Arm the piezo in parallel, also held stationary. Matches ASI's
        # reference (prepareControllerForAquisition_Side): the piezo is
        # always armed even when it isn't the axis actually stepping.
        logger.debug("Arming ASI piezo (stationary, SPIMState=Armed)...")
        self.mmcore.setProperty(
            self.hw.piezo_a_label, "SPIMNumSlices", str(self._num_slices)
        )
        self.mmcore.setProperty(
            self.hw.piezo_a_label, "SingleAxisAmplitude(um)", "0.0001"
        )
        self.mmcore.setProperty(self.hw.piezo_a_label, "SPIMState", "Armed")

        # Diagnostic: dump both axes' live SPIM/scan state so the run log
        # shows exactly what each card accepted.
        for label, tag in (
            (self.hw.galvo_a_label, "galvo"),
            (self.hw.piezo_a_label, "piezo"),
        ):
            for prop in self.mmcore.getDevicePropertyNames(label):
                if "SPIM" in prop or "SingleAxis" in prop:
                    value = self.mmcore.getProperty(label, prop)
                    logger.info(f"  [{tag}] {prop} = {value}")

        log_plogic_trigger_chain_state(
            self.hw.plogic_label, self.hw.tiger_comm_hub_label
        )

        logger.info(
            f"--- PLogic and Camera ready (stationary) --- "
            f"(slices={self._num_slices}, exposure={self._exposure_ms:.1f}ms)"
        )

        return summary_metadata(self.mmcore, mda_sequence=sequence)

    def teardown_sequence(self, sequence: MDASequence) -> None:
        """Clean up hardware state after the sequence finishes."""
        logger.info("--- SEQUENCE FINISHED: Cleaning up hardware (stationary) ---")
        if self.hw.galvo_a_label in self.mmcore.getLoadedDevices():
            self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMState", "Idle")
        if self.hw.piezo_a_label in self.mmcore.getLoadedDevices():
            self.mmcore.setProperty(self.hw.piezo_a_label, "SPIMState", "Idle")

        # Deliberately not closing the global shutter here -- see
        # ASISPIMEngine.teardown_sequence's comment on the same point.
        time.sleep(0.1)
        self.mmcore.setAutoShutter(self._original_autoshutter)
        self._restore_cameras()
        logger.info("--- Hardware cleanup complete ---")
