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

from .asi_controller import (
    configure_plogic_for_dual_nrt_pulses,
    log_plogic_trigger_chain_state,
    set_plogic_evaluation_clock,
)
from .camera_handoff import (
    CameraHandoffSnapshot,
    release_cameras_for_workers,
    reload_cameras_after_handoff,
)
from .camera_worker import CameraWorkerConfig
from .common import AcquisitionSettings, HardwareConstants
from .worker_pool import CameraWorkerHandle, CameraWorkerPool, WorkerDiedError

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
        self._pixel_size_um = 0.0
        self._original_autoshutter = True
        self._snapshot: CameraHandoffSnapshot | None = None
        self._worker_pool: CameraWorkerPool | None = None
        self._piezo_pos_at_setup: float | None = None

    def _set_event_z(self, event: MDAEvent) -> None:
        """No-op: Z-stepping here is done entirely by the galvo hardware trigger.

        ``event_iterator`` below forwards only the z-index-0 sub-event of
        each collapsed stack, so the inherited ``MDAEngine._set_event_z``
        would call ``mmcore.setZPosition()`` with that sub-event's resolved
        position -- the *bottom* of the intended range for the default
        ``go_up=True`` direction -- physically moving the piezo (this rig's
        Core-Focus device) by ``-range/2`` before the galvo stack even
        triggers. Since the galvo's own sweep is always centered on wherever
        the focus device physically sits at trigger time
        (``SingleAxisYOffset(deg)`` is hardcoded to ``"0.0"`` in
        ``setup_sequence``), that stray pre-move shifts the whole optical
        stack by another ``range/2`` in the same direction -- landing the
        pre-acquisition focus at the very last slice instead of the middle,
        and leaving the piezo parked away from where the user left it
        (nothing restores it afterward). Confirmed bench symptom: a
        symmetric ``ZRangeAround`` stack put the focused plane at the last
        slice instead of the middle, and the piezo's position read
        differently after a run than before it.
        """

    def setup_single_event(self, event: MDAEvent) -> None:
        """Set up hardware for one event, replicating the base method minus exposure.

        Copied from ``MDAEngine.setup_single_event`` verbatim, with one
        deliberate omission: the trailing ``mmcore.setExposure(event.exposure)``
        block. This engine bakes exposure into the PLogic pulse width at
        ``setup_sequence`` time instead (see
        :func:`~pymmcore_gui.asi_z_stack.asi_controller.
        configure_plogic_for_dual_nrt_pulses`), and each camera worker
        subprocess owns its own independent ``CMMCorePlus`` (see
        :meth:`_handoff_to_workers`) -- so ``core.setExposure()`` against the
        main process's core is never meaningful here, not even for the very
        first event, before the handoff has run. Left in place, the
        inherited version's own try/except silently logs "Failed to set
        exposure. %s" on every event from the 2nd one onward, once every
        physical camera has been released to a worker and the main-process
        core has no camera device left for ``setExposure`` to target -- this
        is the exact spurious warning seen in production logs. Everything
        else here (XY position, channel switching, properties/ROI/SLM
        no-ops, the trailing keep-shutter-open block) is kept verbatim for
        fidelity with the base dispatch, even where it's a no-op for this
        engine (``_set_event_z`` is already overridden separately -- see its
        docstring).

        Parameters
        ----------
        event : MDAEvent
            The event to use for the hardware config.
        """
        if event.keep_shutter_open:
            ...

        self._set_event_xy_position(event)

        if event.z_pos is not None:
            self._set_event_z(event)
        if event.slm_image is not None:
            self._set_event_slm_image(event)

        self._set_event_channel(event)

        mmcore = self.mmcore
        if event.properties is not None:
            self._set_event_properties(event.properties)
        if event.roi is not None:
            self._set_event_roi(event)
        if (
            # (if autoshutter wasn't set at the beginning of the sequence
            # then it never matters...)
            self._autoshutter_was_set
            # if we want to leave the shutter open after this event, and
            # autoshutter is currently enabled...
            and event.keep_shutter_open
            and mmcore.getAutoShutter()
        ):
            # we have to disable autoshutter and open the shutter
            mmcore.setAutoShutter(False)
            mmcore.setShutterOpen(True)

    def _snapshot_piezo_position(self) -> None:
        """Record the piezo's position so :meth:`_warn_if_piezo_moved` can compare.

        Called at the top of each subclass's ``setup_sequence``, before any
        hardware is touched.
        """
        mmc = self.mmcore
        if self.hw.piezo_a_label in mmc.getLoadedDevices():
            self._piezo_pos_at_setup = mmc.getPosition(self.hw.piezo_a_label)
        else:
            self._piezo_pos_at_setup = None

    def _warn_if_piezo_moved(self) -> None:
        """Log a warning if the piezo's position changed since ``setup_sequence``.

        Defensive check for this class of bug: with :meth:`_set_event_z`
        now a no-op, nothing in this engine should ever move the piezo. If
        it moves anyway (e.g. an ASI-firmware auto-home side effect of
        setting the galvo's ``SPIMState`` to ``"Running"``, which is
        plausible but unconfirmed), this turns a silent focus drift into a
        visible warning in the run log instead of requiring another round
        of bench detective work.
        """
        mmc = self.mmcore
        if (
            self._piezo_pos_at_setup is None
            or self.hw.piezo_a_label not in mmc.getLoadedDevices()
        ):
            return
        current = mmc.getPosition(self.hw.piezo_a_label)
        delta = current - self._piezo_pos_at_setup
        if abs(delta) > 0.05:  # um -- above readback noise, well below a real move
            logger.warning(
                f"Piezo ({self.hw.piezo_a_label}) moved {delta:+.3f} um during "
                f"acquisition: {self._piezo_pos_at_setup:.3f} -> {current:.3f}. "
                "This engine never commands the piezo -- investigate whether "
                "the ASI SPIM state machine is auto-homing it."
            )

    def _handoff_to_workers(self) -> None:
        """Release every physical camera and spawn one worker process per camera.

        Replaces the old ``_arm_cameras`` -- instead of switching each
        physical camera's ``TriggerMode`` while it stays loaded in the main
        process (behind the ``Multi Camera`` composite), the main process
        lets go of every physical camera entirely and hands it to its own
        worker subprocess (see :mod:`~pymmcore_gui.asi_z_stack.worker_pool`).
        Each worker owns its camera's ``pvcam64.dll`` in its own address
        space, so a driver-level crash during concurrent dual-camera
        acquisition can, at worst, take down one disposable worker instead of
        the whole app. :meth:`_reclaim_from_workers` undoes this.

        Called lazily from :meth:`exec_event` on its first invocation, not
        from ``setup_sequence`` -- ``MDARunner`` emits ``sequenceStarted``
        immediately after ``setup_sequence`` returns, and both
        ``MultiCameraHandler.sequenceStarted`` and
        ``NDVViewersManager._on_sequence_started`` independently call
        ``physical_camera_labels(mmc)`` right then to eagerly create one
        writer/viewer per physical camera, which needs the cameras to still
        be loaded at that moment. By the time the first ``exec_event`` call
        happens, ``sequenceStarted`` has already fired, so releasing the
        cameras here is safe.

        Caches ``pixel_size_um`` here (while the cameras are still loaded)
        because it's needed for per-frame metadata built later in
        :meth:`exec_event`, once the cameras -- and the Camera-role-dependent
        core methods that would otherwise supply it -- are gone.
        """
        mmc = self.mmcore
        self._pixel_size_um = mmc.getPixelSizeUm(True)
        self._snapshot = release_cameras_for_workers(mmc, self.hw)

        def _worker_for(label: str) -> CameraWorkerHandle:
            snap = self._snapshot
            assert snap is not None
            cam = snap.per_camera.get(label)
            return CameraWorkerHandle(
                camera_label=label,
                config=CameraWorkerConfig(
                    camera_label=label,
                    adapter_device_name=label,
                    property_snapshot=cam.property_values if cam else {},
                    roi=cam.roi if cam else None,
                    circular_buffer_mb=self.hw.worker_circular_buffer_mb,
                ),
                height=snap.image_height,
                width=snap.image_width,
                dtype=snap.dtype_str,
                n_slots=self.hw.frame_ring_slots_per_camera,
            )

        self._worker_pool = CameraWorkerPool(
            [_worker_for(label) for label in self._snapshot.camera_labels]
        )
        self._worker_pool.spawn_all(ready_timeout=self.hw.worker_ready_timeout_s)
        logger.info(
            f"Camera worker pool ready: {self._snapshot.camera_labels} "
            f"({self._snapshot.image_width}x{self._snapshot.image_height} "
            f"{self._snapshot.dtype_str})."
        )

    def _reclaim_from_workers(self) -> None:
        """Undo :meth:`_handoff_to_workers`.

        Written defensively (``None`` guards, catch-and-log) since
        :class:`~pymmcore_plus.mda.MDARunner` calls ``teardown_sequence``
        unconditionally on completion, cancellation, *or* any exception out
        of ``setup_sequence``/``exec_event`` -- this may run against a
        partial handoff (e.g. the pool spawned but ``setup_sequence`` raised
        before finishing).
        """
        if self._worker_pool is not None:
            try:
                self._worker_pool.shutdown_all(
                    timeout=self.hw.worker_shutdown_timeout_s
                )
            except Exception:
                logger.error("Error shutting down camera worker pool.", exc_info=True)
            self._worker_pool = None
        if self._snapshot is not None:
            try:
                reload_cameras_after_handoff(self.mmcore, self.hw, self._snapshot)
            except Exception:
                logger.error("Error reloading cameras after handoff.", exc_info=True)
            self._snapshot = None

    def _warn_if_circular_buffer_too_small(self, per_camera_images: int) -> None:
        """Log a warning if a worker's circular buffer can't fit one z-stack.

        Each camera now lives in its own worker process with its own
        circular buffer (``HardwareConstants.worker_circular_buffer_mb``) --
        unlike the old single shared 30 GB main-process buffer this replaces,
        each worker's buffer only ever needs to hold *one* camera's frames,
        not ``n_cameras`` worth. Must run against the main process's core
        *before* :meth:`_handoff_to_workers` releases the cameras -- it needs
        their live image geometry, which isn't available once they're gone.
        Mirrors an earlier, hard-won lesson from the old single-buffer
        design: never resize a circular buffer after a camera is armed for
        external triggering (that crashed PVCAM's driver,
        ``pvcam64.dll``, exception ``0xc0000409`` / STATUS_STACK_BUFFER_OVERRUN,
        even more reliably than the wraparound it was meant to prevent) --
        so this only warns, it never resizes anything itself. Each worker
        sizes its own buffer once at startup, before arming -- see
        :func:`~pymmcore_gui.asi_z_stack.camera_worker.run_camera_worker`.

        Parameters
        ----------
        per_camera_images : int
            The number of frames one camera's z-stack will produce (not
            multiplied by camera count -- each worker only buffers its own).
        """
        mmc = self.mmcore
        bytes_per_frame = (
            mmc.getImageWidth() * mmc.getImageHeight() * mmc.getBytesPerPixel()
        )
        if bytes_per_frame <= 0:
            return
        required_mb = (bytes_per_frame * per_camera_images * 1.5) / (1024 * 1024)
        if required_mb > self.hw.worker_circular_buffer_mb:
            logger.warning(
                f"Worker circular buffer ({self.hw.worker_circular_buffer_mb} MB) "
                f"may be too small for {per_camera_images} frames "
                f"(~{required_mb:.0f} MB needed) -- consider raising "
                "HardwareConstants.worker_circular_buffer_mb."
            )

    def _reset_channel_config_cache(self) -> None:
        """Force the next per-channel config switch to actually happen.

        Mirrors the stock ``MDAEngine.setup_sequence``'s own
        ``core._last_config = ("", "")`` reset (added for
        https://github.com/pymmcore-plus/pymmcore-plus/issues/503) -- which
        neither ``ASISPIMEngine`` nor ``ASIStationaryTriggerEngine`` ever ran,
        since both override ``setup_sequence`` completely rather than calling
        ``super()``. Without it, ``_set_event_channel`` compares the
        sequence's first channel against whatever ``"Lasers"`` config was
        last actually applied (e.g. a Live/Snap selection, or the previous
        MDA's final channel); if they match, it treats the channel as already
        correct and skips calling ``mmc.setConfig(...)`` for it entirely --
        silently leaving that channel's whole z-stack running under whatever
        raw PLogic BNC wiring ``setup_sequence`` happened to leave behind,
        rather than its own selected laser. Calling this at the top of every
        ``setup_sequence`` guarantees the first channel always gets a real,
        unconditional ``setConfig`` call, same as every later channel change.
        """
        self.mmcore._last_config = ("", "")

    def event_iterator(self, events: Iterable[MDAEvent]) -> Iterator[MDAEvent]:
        """Collapse each hardware z-stack down to a single event.

        The ASI SPIM state machine acquires an entire z-stack per trigger,
        so :meth:`exec_event` yields every slice itself. useq emits one
        event per z-slice, so forward only the first slice of each stack
        (``z`` index 0, or events with no ``z`` axis) and drop the rest --
        otherwise the stack would be re-triggered once per slice.

        Only **per-volume** channel switching is possible today: a whole
        z-stack is triggered by a single, uninterrupted hardware burst (see
        :meth:`exec_event`), with no software checkpoint between slices where
        a different laser could be selected. This holds regardless of which
        ``axis_order`` ("...cz" vs "...zc") the MDA sequence uses -- that
        setting has no effect on this engine's actual trigger timing, only on
        event bookkeeping order. True per-slice (interleaved) laser switching
        would need a different PLogic wiring scheme entirely (a hardware
        mod-N BNC counter clocked per-slice, as ASI's own diSPIM plugin
        implements) and is not currently implemented.
        """
        for event in events:
            if event.index.get("z", 0) == 0:
                yield event

    def _build_frame_meta(
        self,
        sub_event: MDAEvent,
        camera_label: str,
        camera_metadata: dict[str, object],
        images_remaining: int,
        runner_time_ms: float,
    ) -> FrameMetaV1:
        """Build ``FrameMetaV1`` by hand, avoiding the stock ``get_frame_metadata``.

        ``self.get_frame_metadata`` calls Camera-role-dependent core methods
        (``getExposure()`` with no camera argument, in particular) that
        misbehave once every physical camera has been released to a worker
        process -- so this reconstructs the same fields from values already
        cached while the cameras were still loaded (see
        :meth:`_handoff_to_workers`), plus the per-frame ``camera_metadata``
        each worker reports.

        Parameters
        ----------
        sub_event : MDAEvent
            This frame's per-slice/per-camera event.
        camera_label : str
            Which physical camera produced this frame.
        camera_metadata : dict[str, object]
            The raw MMCore image-tag dict the worker read off its own
            circular buffer.
        images_remaining : int
            Images still buffered in the worker's circular buffer.
        runner_time_ms : float
            Elapsed time since the MDA sequence started.
        """
        meta: FrameMetaV1 = {
            "format": "frame-dict",
            "version": "1.0",
            "pixel_size_um": self._pixel_size_um,
            "camera_device": camera_label,
            "exposure_ms": self._exposure_ms,
            "property_values": (),
            "runner_time_ms": runner_time_ms,
            "mda_event": sub_event,
            "hardware_triggered": True,
            "images_remaining_in_buffer": images_remaining,
            "camera_metadata": camera_metadata,
        }
        if self._include_frame_position_metadata is True:
            from pymmcore_plus.metadata.functions import position

            meta["position"] = position(self.mmcore)
        return meta

    def exec_event(
        self, event: MDAEvent
    ) -> Iterable[tuple[np.ndarray, MDAEvent, FrameMetaV1]]:
        """Trigger the PLogic/SPIM stack and yield one payload per slice/camera.

        A single event (see :meth:`event_iterator`) drives the whole z-stack.
        Each physical camera's worker process streams its own frames back
        independently (see :mod:`~pymmcore_gui.asi_z_stack.worker_pool`); each
        is tagged with its physical camera (``camera_device`` + ``cam`` index,
        the latter now from the worker pool's own ordering, not a Multi
        Camera-specific metadata tag) and its slice (``z`` index, the
        worker's own arrival-order counter) so per-camera writers receive a
        full, distinct z-stack.

        A generator ``send()`` of ``"cancel"`` (how
        :class:`~pymmcore_plus.mda.MDARunner` delivers MDA cancellation into a
        running ``exec_event``) stops both workers and returns early instead
        of being silently ignored, as it was before this method streamed
        frames straight from a shared circular buffer.

        The camera handoff itself happens lazily, here, on the first call --
        see :meth:`_handoff_to_workers`'s docstring for why it can't happen
        in ``setup_sequence``. Subsequent calls within the same MDA run (one
        per timepoint/collapsed z-stack) reuse the same worker pool.
        """
        if self._worker_pool is None:
            self._handoff_to_workers()
        assert self._worker_pool is not None and self._snapshot is not None
        n_cameras = self._snapshot.n_cameras

        self._worker_pool.arm_all(
            self._num_slices, armed_timeout=self.hw.worker_arm_timeout_s
        )
        logger.info(f"{n_cameras} camera(s) armed for {self._num_slices} images each.")
        # See _trigger_spim_state_value's docstring: currently always
        # "Running" on the galvo (its NO_SCAN/SLICE_SCAN_ONLY trigger path)
        # -- the piezo's SPIMState property has no "Running" value at all,
        # so no engine here uses it as _master_axis_label. Both workers are
        # already armed and waiting for the shared PLogic/TTL trigger below --
        # that hardware signal, not software call order, is what actually
        # synchronizes the two cameras' exposures.
        mmc = self.mmcore
        mmc.setProperty(
            self._master_axis_label, "SPIMState", self._trigger_spim_state_value
        )
        spim_state = mmc.getProperty(self._master_axis_label, "SPIMState")
        logger.info(f"{self._master_axis_label} SPIMState readback: '{spim_state}'.")

        runner_t0 = event.metadata.get("runner_t0")
        try:
            for (
                camera_label,
                slice_idx,
                img,
                camera_metadata,
                images_remaining,
            ) in self._worker_pool.iter_frames(stall_timeout_s=5.0):
                cam_index = self._snapshot.camera_labels.index(camera_label)
                new_index = {**event.index, "z": slice_idx}
                if n_cameras > 1:
                    new_index["cam"] = cam_index
                sub_event = event.model_copy(update={"index": new_index})

                runner_time_ms = (
                    (time.perf_counter() - runner_t0) * 1000.0 if runner_t0 else 0.0
                )
                meta = self._build_frame_meta(
                    sub_event,
                    camera_label,
                    camera_metadata,
                    images_remaining,
                    runner_time_ms,
                )
                received = yield img, sub_event, meta
                if received == "cancel":
                    logger.info("MDA cancelled -- stopping camera workers.")
                    return
        except WorkerDiedError:
            logger.error(
                "A camera worker process died during acquisition.", exc_info=True
            )
            raise
        finally:
            # Guarantees every worker gets a StopCmd on every exit path from
            # this generator -- normal completion, "cancel", WorkerDiedError,
            # a TimeoutError from iter_frames's stall guard, a plain
            # RuntimeError wrapping a worker's ErrorMsg, or this generator
            # being abandoned mid-iteration by its caller and closed by the
            # interpreter (GeneratorExit thrown at the yield above) -- the
            # mechanism behind a production deadlock where an abandoned
            # generator left a worker blocked forever in
            # camera_worker._wait_for_free_slot, waiting for a SlotFreeCmd/
            # StopCmd that would never come. stop_all() is best-effort/
            # idempotent (worker_pool.py's own conn-closed/OSError guards),
            # so the extra call this now makes on ordinary per-event
            # completion (never called there before) is harmless: a StopCmd
            # against an already-idle worker.
            self._worker_pool.stop_all()


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
        # 0. Force the sequence's first channel to get a real hardware
        # config switch -- see _reset_channel_config_cache's docstring.
        self._reset_channel_config_cache()
        self._snapshot_piezo_position()

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
        # Circular-buffer sizing needs live image geometry, so it must run
        # against the main process's core before the cameras are released.
        # The handoff itself is deliberately NOT done here: MDARunner emits
        # sequenceStarted right after setup_sequence returns, and both
        # MultiCameraHandler.sequenceStarted and
        # NDVViewersManager._on_sequence_started independently call
        # physical_camera_labels(mmc) at that point to eagerly create one
        # writer/viewer per physical camera -- which needs the cameras still
        # loaded. _handoff_to_workers() runs lazily on the first exec_event
        # call instead, which always happens after sequenceStarted has
        # already fired.
        self._warn_if_circular_buffer_too_small(self._num_slices)

        settings = AcquisitionSettings(
            camera_exposure_ms=self._exposure_ms,
            laser_trig_duration_ms=self._exposure_ms,
        )
        configure_plogic_for_dual_nrt_pulses(
            settings,
            self.hw.plogic_label,
            self.hw.tiger_comm_hub_label,
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
        self._warn_if_piezo_moved()
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
        self._reclaim_from_workers()
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
        # 0. Force the sequence's first channel to get a real hardware
        # config switch -- see _reset_channel_config_cache's docstring.
        self._reset_channel_config_cache()
        self._snapshot_piezo_position()

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
        # Circular-buffer sizing needs live image geometry, so it must run
        # against the main process's core before the cameras are released.
        # The handoff itself is deliberately NOT done here: MDARunner emits
        # sequenceStarted right after setup_sequence returns, and both
        # MultiCameraHandler.sequenceStarted and
        # NDVViewersManager._on_sequence_started independently call
        # physical_camera_labels(mmc) at that point to eagerly create one
        # writer/viewer per physical camera -- which needs the cameras still
        # loaded. _handoff_to_workers() runs lazily on the first exec_event
        # call instead, which always happens after sequenceStarted has
        # already fired.
        self._warn_if_circular_buffer_too_small(self._num_slices)

        settings = AcquisitionSettings(
            camera_exposure_ms=self._exposure_ms,
            laser_trig_duration_ms=self._exposure_ms,
        )
        configure_plogic_for_dual_nrt_pulses(
            settings,
            self.hw.plogic_label,
            self.hw.tiger_comm_hub_label,
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
        self._warn_if_piezo_moved()
        if self.hw.galvo_a_label in self.mmcore.getLoadedDevices():
            self.mmcore.setProperty(self.hw.galvo_a_label, "SPIMState", "Idle")
        if self.hw.piezo_a_label in self.mmcore.getLoadedDevices():
            self.mmcore.setProperty(self.hw.piezo_a_label, "SPIMState", "Idle")

        # Deliberately not closing the global shutter here -- see
        # ASISPIMEngine.teardown_sequence's comment on the same point.
        time.sleep(0.1)
        self.mmcore.setAutoShutter(self._original_autoshutter)
        self._reclaim_from_workers()
        logger.info("--- Hardware cleanup complete ---")
