"""A session-lifetime owner of the dual-PVCAM camera worker pool.

Two physical Kinetix/PVCAM cameras crash the whole process (native
``pvcam64.dll`` exception ``0xc0000409`` / ``STATUS_STACK_BUFFER_OVERRUN``) when
both cameras' driver calls happen concurrently in one OS process -- a
thread-safety bug inside the vendor's compiled driver. The existing fix (see
:mod:`~pymmcore_gui.asi_z_stack.worker_pool`/:mod:`~pymmcore_gui.asi_z_stack.
camera_worker`) gives each physical camera its own OS subprocess, so a driver
crash can't corrupt the other camera / the main GUI process.

That mechanism used to be spawned fresh for every hardware-triggered MDA run
and torn down afterward. Worker spawn + PVCAM device init is bench-measured at
~3-4s *per worker* (see :class:`~pymmcore_gui.asi_z_stack.common.
HardwareConstants`'s docstring comment), which makes spawn-per-use unusable
for Live mode (toggled on/off far more often than an MDA run is started).
:class:`CameraWorkerService` instead spawns the two worker processes **once**,
keeps them alive for the whole program lifetime, and lets Live, Snap, and MDA
all share the one persistent pool as mutually-exclusive clients.

Because a physical camera can only be open in one OS process at a time
(driver exclusivity -- the same reason the old per-MDA handoff unloaded the
cameras from the main process first), once this service is active Camera-1/
Camera-2 are essentially never loaded in the main process's ``CMMCorePlus``
again. :func:`~pymmcore_gui._multi_camera_handler.physical_camera_labels`
falls back to this service's static camera-label list so the rest of the app
doesn't need to know the difference.

Threading invariant, load-bearing, read this before touching any method
below: ``self._lock`` is only ever held for short, non-blocking reads/writes
of ``self._state``/``self._pool``/``self._live_thread`` -- **never** held
across a blocking pool call (``arm_all``, ``iter_frames``, ``stop_all``) or a
``Thread.join()``. Holding it across a join is a real deadlock: the Live
drain thread's own ``finally`` block needs the same lock to record that it
has stopped, and if the joining thread is still holding the lock while
blocked in ``.join()``, neither thread can ever proceed.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING, ClassVar

from PyQt6.QtCore import QObject, pyqtSignal
from superqt.utils import create_worker

from .camera_handoff import (
    CameraHandoffSnapshot,
    release_cameras_for_workers,
    reload_cameras_after_handoff,
)
from .camera_worker import CameraWorkerConfig
from .common import HardwareConstants
from .worker_pool import CameraWorkerHandle, CameraWorkerPool

if TYPE_CHECKING:
    from collections.abc import Callable

    import numpy as np
    from pymmcore_plus import CMMCorePlus

logger = logging.getLogger(__name__)


def has_camera(mmc: CMMCorePlus) -> bool:
    """Whether Snap/Live have a camera to act on.

    ``mmc.getCameraDevice()`` is ``""`` once the service has released the
    cameras (unloading the current camera clears the Core camera role), so a
    plain truthiness check on it would leave Snap/Live disabled for the whole
    session on the worker-owned rig.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    """
    return bool(mmc.getCameraDevice()) or CameraWorkerService.get_active() is not None


@dataclass
class WorkerConfigGroup:
    """A config group that touches a device the main process no longer has.

    Captured by :meth:`CameraWorkerService.begin_release` while every device
    is still loaded, then deleted from the main-process core: once a
    referenced device is unloaded, ``getCurrentConfig``/``setConfig`` on the
    group raise ``No device with label ...``, which takes down every
    consumer that walks all groups (the Config Groups widget, MDA summary
    metadata). Presets are applied through
    :meth:`CameraWorkerService.apply_worker_config` instead.
    """

    #: preset name -> ordered (device, property, value) settings.
    presets: dict[str, tuple[tuple[str, str, str], ...]]
    #: The preset in effect at release time ("" if none matched), then
    #: whatever apply_worker_config last applied.
    current: str


def _detach_unloadable_config_groups(
    mmc: CMMCorePlus, released: set[str]
) -> dict[str, WorkerConfigGroup]:
    """Capture and delete every config group that references a *released* device.

    Must run while the *released* devices are still loaded (for
    ``getCurrentConfig``).

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    released : set[str]
        Labels of every device about to be unloaded from *mmc*.
    """
    groups: dict[str, WorkerConfigGroup] = {}
    for group in mmc.getAvailableConfigGroups():
        presets = {
            str(preset): tuple(
                (dev, prop, val) for dev, prop, val in mmc.getConfigData(group, preset)
            )
            for preset in mmc.getAvailableConfigs(group)
        }
        if not any(dev in released for s in presets.values() for dev, _, _ in s):
            continue
        try:
            current = str(mmc.getCurrentConfig(group))
        except Exception:
            current = ""
        groups[str(group)] = WorkerConfigGroup(presets=presets, current=current)
    for name in groups:
        mmc.deleteConfigGroup(name)
    return groups


class CameraWorkerServiceState(Enum):
    """See :class:`CameraWorkerService`'s state-machine summary."""

    #: No service is registered for this session (demo/single-camera/non-ASI
    #: config, or the active service was shut down).
    INACTIVE = auto()
    #: Camera labels/geometry are known (cameras released from the main
    #: process) but the worker processes haven't finished spawning yet.
    SPAWNING = auto()
    #: Pool is up and idle -- Live/Snap/MDA/ROI may all be used.
    IDLE = auto()
    LIVE = auto()
    MDA = auto()


class CameraWorkerService(QObject):
    """Owns one persistent :class:`CameraWorkerPool` for the whole session.

    Registered via a class-level slot (:meth:`get_active`) so free functions
    without a direct reference (e.g. :func:`~pymmcore_gui._multi_camera_handler.
    physical_camera_labels`) can still reach the active instance. Returns
    ``None`` for demo/single-camera/non-ASI configs -- every call site's
    "inactive" branch is exactly today's unchanged, pre-worker-service code
    path.

    Snap is deliberately not a persisted state: it's a short, synchronous
    operation (``IDLE -> busy -> IDLE`` inline within :meth:`snap`) rather
    than something other code needs to observe or react to mid-flight.
    """

    _active: ClassVar[CameraWorkerService | None] = None

    #: (camera_label, slice_idx, frame, camera_metadata, images_remaining).
    #: Emitted for both Live streaming (from the background drain thread,
    #: relying on Qt's auto-queued cross-thread connection to marshal onto
    #: the GUI thread) and Snap (emitted directly from the GUI thread, inside
    #: :meth:`snap`) -- one signal, one consumer
    #: (``NDVViewersManager._on_worker_frame``) for both, since both just
    #: mean "display this frame for this camera." frame/camera_metadata are
    #: typed ``object`` (not ``np.ndarray``/``dict`` directly) for
    #: guaranteed-safe PyQt signal marshaling.
    frameReady = pyqtSignal(str, int, object, object, int)
    #: (group, preset) after apply_worker_config succeeds.
    workerConfigChanged = pyqtSignal(str, str)
    liveStateChanged = pyqtSignal(bool)
    liveErrored = pyqtSignal(str)
    #: Internal: "camera_label has a new latest Live frame waiting." Carries
    #: no pixel data -- see _run_live_drain for why.
    _liveFrameAvailable = pyqtSignal(str)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._lock = threading.Lock()
        self._state = CameraWorkerServiceState.INACTIVE
        self._pool: CameraWorkerPool | None = None
        self._snapshot: CameraHandoffSnapshot | None = None
        self._worker_config_groups: dict[str, WorkerConfigGroup] = {}
        self._live_thread: threading.Thread | None = None
        #: Latest undelivered Live frame per camera, written by the drain
        #: thread and consumed on the GUI thread. Guarded by self._lock.
        self._latest_live: dict[str, tuple[int, object, object, int]] = {}
        self._liveFrameAvailable.connect(self._deliver_live_frame)
        self.hw = HardwareConstants()
        self._mmc: CMMCorePlus | None = None
        #: Exposure read from the main-process core once, in begin_release,
        #: while a camera was still loaded there -- a best-effort fallback
        #: for code that would otherwise call the now-cameraless main-process
        #: core's getExposure() (e.g. engine.py's per-sequence exposure
        #: default when a sequence omits it). Not kept live-updated.
        self.last_known_exposure_ms: float = 10.0

    # ------------------------------------------------------------------
    # Registry
    # ------------------------------------------------------------------

    @classmethod
    def get_active(cls) -> CameraWorkerService | None:
        """Return the session's service iff it has ever completed ``begin_release``.

        Returns non-``None`` from the moment camera labels/geometry are known
        (``SPAWNING``), not only once the pool is fully ``IDLE`` -- callers
        that only need :attr:`camera_labels`/:attr:`geometry` (e.g. eager
        preview-dock creation at startup) work correctly even while workers
        are still spawning; callers that need the pool itself must check
        :attr:`state`.
        """
        return cls._active

    # ------------------------------------------------------------------
    # Read-only info
    # ------------------------------------------------------------------

    @property
    def state(self) -> CameraWorkerServiceState:
        """Current state, one of :class:`CameraWorkerServiceState`'s members."""
        return self._state

    @property
    def camera_labels(self) -> tuple[str, ...]:
        """The physical camera labels this service owns, or ``()`` if inactive."""
        return self._snapshot.camera_labels if self._snapshot is not None else ()

    @property
    def geometry(self) -> CameraHandoffSnapshot | None:
        """Cached image/camera geometry captured once at :meth:`begin_release`."""
        return self._snapshot

    @property
    def worker_config_groups(self) -> dict[str, WorkerConfigGroup]:
        """Config groups moved off the main-process core at :meth:`begin_release`."""
        return self._worker_config_groups

    # ------------------------------------------------------------------
    # Startup (see _main_window.py::_on_system_config_loaded)
    # ------------------------------------------------------------------

    def begin_release(self, mmc: CMMCorePlus, hw: HardwareConstants) -> bool:
        """Gate-check and release Camera-1/2 from *mmc*, synchronously.

        Must be called at the very top of ``_on_system_config_loaded``,
        before any other ``systemConfigurationLoaded`` listener runs
        (guaranteed by connection order -- ``MicroManagerGUI`` connects that
        handler first, in ``__init__``). This ordering is what lets every
        other listener that needs camera labels (``MultiCameraHandler.
        sequenceStarted``, ``NDVViewersManager.create_default_camera_previews``,
        etc.) source them from this service instead of needing the cameras
        still loaded in-process at their own call time.

        Returns ``False`` (a no-op) for demo/single-camera/non-ASI configs.
        Defensively shuts down any still-active previous state first, in case
        a reload reached here without going through :meth:`prepare_for_reload`
        (e.g. a config loaded from the embedded console or Config Wizard).
        """
        if self._state is not CameraWorkerServiceState.INACTIVE:
            self.shutdown()

        from pymmcore_gui._multi_camera_handler import physical_camera_labels

        from .asi_controller import asi_zstack_hardware_available

        # Gated purely on the same condition MicroManagerGUI._register_mda_engine
        # uses to install ASISPIMEngine -- NOT on camera count. The old
        # per-MDA handoff this replaces was also never camera-count-gated
        # (one worker per camera, for however many are configured -- see
        # engine.py's _handoff_to_workers history); keeping the two gates in
        # lockstep here avoids a split-brain state where ASISPIMEngine is
        # installed but no worker service exists to hand its cameras to
        # (which would happen for a hypothetical single-camera ASI rig if
        # this required exactly 2 cameras).
        labels = physical_camera_labels(mmc)
        if not asi_zstack_hardware_available() or not labels or not all(labels):
            return False

        self.hw = hw
        self._mmc = mmc
        try:
            self.last_known_exposure_ms = mmc.getExposure()
        except Exception:
            pass
        # Everything release_cameras_for_workers is about to unload: the
        # physical cameras plus the Multi Camera composite (when it's the
        # Core camera). Groups referencing any of them must come off the core
        # first -- see WorkerConfigGroup.
        released = set(labels)
        if len(labels) > 1 and (role := mmc.getCameraDevice()):
            released.add(role)
        self._worker_config_groups = _detach_unloadable_config_groups(mmc, released)
        self._snapshot = release_cameras_for_workers(mmc, hw)
        with self._lock:
            self._state = CameraWorkerServiceState.SPAWNING
        type(self)._active = self
        return True

    def spawn_async(self, on_ready: Callable[[], None] | None = None) -> None:
        """Spawn the worker processes on a background thread (the slow half).

        No-op (calls *on_ready* immediately) if :meth:`begin_release` wasn't
        satisfied for the current config -- keeps the caller's chaining
        simple regardless of which rig is loaded. *on_ready* runs on the GUI
        thread either way (``superqt.utils.create_worker``'s signals are
        auto-queued back to the thread that created the worker).
        """
        snapshot = self._snapshot
        if snapshot is None:
            if on_ready is not None:
                on_ready()
            return

        hw = self.hw

        def _spawn() -> CameraWorkerPool:
            workers = [
                self._build_worker_handle(label) for label in snapshot.camera_labels
            ]
            pool = CameraWorkerPool(workers)
            pool.spawn_all(ready_timeout=hw.worker_ready_timeout_s)
            return pool

        def _on_finished(pool: object) -> None:
            with self._lock:
                self._pool = pool  # type: ignore[assignment]
                self._state = CameraWorkerServiceState.IDLE
            logger.info(
                f"Persistent camera worker pool ready: {snapshot.camera_labels}."
            )
            if on_ready is not None:
                on_ready()

        def _on_errored(exc: BaseException) -> None:
            logger.error("Failed to start persistent camera worker pool.", exc_info=exc)
            type(self)._active = None
            with self._lock:
                self._state = CameraWorkerServiceState.INACTIVE
            self._snapshot = None
            if self._mmc is not None:
                try:
                    reload_cameras_after_handoff(self._mmc, hw, snapshot)
                except Exception:
                    logger.exception(
                        "Also failed to reload cameras after a failed worker spawn."
                    )
            if on_ready is not None:
                on_ready()

        worker = create_worker(_spawn, _start_thread=True)
        worker.returned.connect(_on_finished)
        worker.errored.connect(_on_errored)

    def _build_worker_handle(self, label: str) -> CameraWorkerHandle:
        snapshot = self._snapshot
        assert snapshot is not None
        cam = snapshot.per_camera.get(label)
        return CameraWorkerHandle(
            camera_label=label,
            config=CameraWorkerConfig(
                camera_label=label,
                adapter_device_name=label,
                property_snapshot=cam.property_values if cam else {},
                roi=cam.roi if cam else None,
                circular_buffer_mb=self.hw.worker_circular_buffer_mb,
            ),
            height=snapshot.image_height,
            width=snapshot.image_width,
            dtype=snapshot.dtype_str,
            n_slots=self.hw.frame_ring_slots_per_camera,
        )

    # ------------------------------------------------------------------
    # Config-reload hazard
    # ------------------------------------------------------------------

    @classmethod
    def prepare_for_reload(cls, mmc: CMMCorePlus) -> None:
        """Tear down the active service, if any, before a system-config reload.

        Must be called before ``mmc.loadSystemConfiguration(...)`` whenever a
        persistent service might be active -- pymmcore-plus has no pre-load
        event, and a new config's ``initializeDevice`` for the same
        Camera-1/2 labels will fail if a previous session's workers still
        hold those PVCAM handles open (driver exclusivity). Raises if an MDA
        is currently running (mirrors ``MicroManagerGUI.
        _confirm_close_with_running_mda``'s own refusal) rather than
        silently tearing down mid-acquisition -- the caller should surface
        this to the user instead of proceeding with the reload.
        """
        svc = cls._active
        if svc is None:
            return
        if mmc.mda.is_running():
            raise RuntimeError(
                "Cannot reload the system configuration while an MDA is "
                "running. Stop or wait for the current acquisition to "
                "finish first."
            )
        svc.shutdown()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self, timeout: float | None = None) -> None:
        """Stop Live (if running) and tear down every worker process.

        Safe to call more than once, and when nothing is active. Stops Live
        and joins its drain thread *before* tearing down the pool -- see the
        module docstring's threading invariant for why that order matters.
        """
        self.stop_live(timeout=timeout)
        with self._lock:
            pool = self._pool
            self._pool = None
            was_active = self._state is not CameraWorkerServiceState.INACTIVE
            self._state = CameraWorkerServiceState.INACTIVE
        self._snapshot = None
        self._worker_config_groups = {}
        if type(self)._active is self:
            type(self)._active = None
        if pool is not None:
            try:
                pool.shutdown_all(
                    timeout=timeout
                    if timeout is not None
                    else self.hw.worker_shutdown_timeout_s
                )
            except Exception:
                logger.error(
                    "Error shutting down persistent camera worker pool.",
                    exc_info=True,
                )
        if was_active:
            logger.info("Persistent camera worker service shut down.")

    # ------------------------------------------------------------------
    # Live
    # ------------------------------------------------------------------

    def start_live(self) -> None:
        """Arm both workers for free-running acquisition and start the drain thread."""
        with self._lock:
            state = self._state
            pool = self._pool
        if state is CameraWorkerServiceState.LIVE:
            return
        if state is not CameraWorkerServiceState.IDLE or pool is None:
            raise RuntimeError(
                f"Camera worker service is not ready for Live (state={state.name})."
            )
        pool.arm_live_all(armed_timeout=self.hw.worker_arm_timeout_s)
        with self._lock:
            self._state = CameraWorkerServiceState.LIVE
            self._live_thread = threading.Thread(
                target=self._run_live_drain,
                args=(pool,),
                name="camera-worker-live-drain",
                daemon=True,
            )
            self._live_thread.start()
        self.liveStateChanged.emit(True)

    def stop_live(self, timeout: float | None = None) -> None:
        """Stop Live streaming and join the drain thread. No-op if not Live."""
        with self._lock:
            if self._state is not CameraWorkerServiceState.LIVE:
                return
            pool = self._pool
            thread = self._live_thread
        assert pool is not None
        pool.stop_all()
        if thread is not None:
            thread.join(
                timeout if timeout is not None else self.hw.worker_shutdown_timeout_s
            )
        if thread is None or not thread.is_alive():
            # The drain thread normally consumes each worker's StoppedMsg, but
            # if it died early (stall/error) the rest of the stream is still
            # queued. Only safe once it's gone: two readers can't share a pipe.
            pool.stop_and_drain(timeout=self.hw.worker_shutdown_timeout_s)
        else:
            logger.warning("Live drain thread did not exit; pipes left undrained.")
        with self._lock:
            self._live_thread = None

    def _run_live_drain(self, pool: CameraWorkerPool) -> None:
        """Background-thread body: drain frames until stopped, then flip to IDLE.

        Never touches Qt widgets directly -- only ever calls ``.emit()`` on
        this ``QObject``'s own signals, exactly the pattern
        ``_main_window.py``'s ``_ArgusStatusRelay`` already establishes for
        marshaling a plain sender thread's callbacks onto the GUI thread.

        Frames are coalesced to the latest one per camera: the cameras can
        free-run far faster than the GUI repaints (two Kinetix at ~11 MB/frame),
        and queueing every frame as its own cross-thread event grows the Qt
        event queue without bound whenever the GUI falls behind. Only the
        first frame after a delivery queues an event; later frames just
        replace the pending one, so at most one event per camera is ever in
        flight.
        """
        try:
            for (
                camera_label,
                slice_idx,
                img,
                camera_metadata,
                images_remaining,
            ) in pool.iter_frames(stall_timeout_s=5.0):
                with self._lock:
                    already_pending = camera_label in self._latest_live
                    self._latest_live[camera_label] = (
                        slice_idx,
                        img,
                        camera_metadata,
                        images_remaining,
                    )
                if not already_pending:
                    self._liveFrameAvailable.emit(camera_label)
        except Exception as exc:
            logger.error("Live streaming stopped unexpectedly.", exc_info=True)
            self.liveErrored.emit(str(exc))
        finally:
            with self._lock:
                if self._state is CameraWorkerServiceState.LIVE:
                    self._state = CameraWorkerServiceState.IDLE
            self.liveStateChanged.emit(False)

    def _deliver_live_frame(self, camera_label: str) -> None:
        """GUI-thread half of the Live coalescing in :meth:`_run_live_drain`."""
        with self._lock:
            entry = self._latest_live.pop(camera_label, None)
        if entry is None:
            return
        slice_idx, img, camera_metadata, images_remaining = entry
        self.frameReady.emit(
            camera_label, slice_idx, img, camera_metadata, images_remaining
        )

    # ------------------------------------------------------------------
    # Snap
    # ------------------------------------------------------------------

    def snap(self, timeout: float = 5.0) -> dict[str, np.ndarray]:
        """Grab one frame per camera, synchronously (matches today's blocking Snap UX).

        Called on the GUI thread, so :attr:`frameReady` is emitted directly
        (a plain, same-thread Qt connection) as each frame arrives -- no
        cross-thread marshaling needed here, unlike the Live drain thread.
        """
        with self._lock:
            if self._state is CameraWorkerServiceState.MDA:
                raise RuntimeError("Cannot Snap while an MDA is running.")
            was_live = self._state is CameraWorkerServiceState.LIVE
            pool = self._pool
        if pool is None:
            raise RuntimeError("Camera worker service has no active pool.")
        if was_live:
            self.stop_live()

        frames: dict[str, np.ndarray] = {}
        needed = set(self.camera_labels)
        # Internal trigger: outside an MDA nothing fires the external one, so
        # an externally-triggered Snap just times out with zero frames.
        pool.arm_all(
            1, armed_timeout=self.hw.worker_arm_timeout_s, external_trigger=False
        )
        try:
            for camera_label, slice_idx, img, meta, remaining in pool.iter_frames(
                stall_timeout_s=timeout
            ):
                frames[camera_label] = img
                self.frameReady.emit(camera_label, slice_idx, img, meta, remaining)
                needed.discard(camera_label)
                if not needed:
                    break
        finally:
            # Snap breaks out of an over-armed sequence after one frame per
            # camera: drain the rest so the next arm doesn't read it.
            pool.stop_and_drain(timeout=self.hw.worker_shutdown_timeout_s)
        return frames

    # ------------------------------------------------------------------
    # MDA
    # ------------------------------------------------------------------

    def acquire_for_mda(self) -> CameraWorkerPool:
        """Return the persistent pool for exclusive MDA use, stopping Live first."""
        with self._lock:
            if self._state is CameraWorkerServiceState.MDA:
                raise RuntimeError(
                    "Camera worker pool already acquired for an MDA run."
                )
            was_live = self._state is CameraWorkerServiceState.LIVE
            pool = self._pool
        if was_live:
            self.stop_live()
        with self._lock:
            if self._state is not CameraWorkerServiceState.IDLE or pool is None:
                raise RuntimeError(
                    "Camera worker service is not ready for MDA "
                    f"(state={self._state.name})."
                )
            self._state = CameraWorkerServiceState.MDA
        return pool

    def release_from_mda(self) -> None:
        """End this engine's exclusive MDA claim on the pool. No-op if not MDA."""
        with self._lock:
            if self._state is CameraWorkerServiceState.MDA:
                self._state = CameraWorkerServiceState.IDLE

    # ------------------------------------------------------------------
    # Config groups touching worker-owned cameras
    # ------------------------------------------------------------------

    def apply_worker_config(self, group: str, preset: str) -> None:
        """Apply *preset* of a :attr:`worker_config_groups` group.

        Settings on a worker-owned camera go to that camera's worker; settings
        on devices still loaded in the main process go to ``mmc.setProperty``;
        settings on other released devices (the ``Multi Camera`` composite,
        which exists in neither place anymore) are skipped. Live is stopped
        around the change and restarted, since PVCAM rejects settings like
        ``Port`` mid-acquisition. Refused while an MDA holds the pool.

        Parameters
        ----------
        group : str
            A key of :attr:`worker_config_groups`.
        preset : str
            One of that group's presets.
        """
        cfg = self._worker_config_groups[group]
        settings = cfg.presets[preset]
        with self._lock:
            state = self._state
            pool = self._pool
        if state is CameraWorkerServiceState.MDA:
            raise RuntimeError("Cannot change camera settings while an MDA is running.")
        if pool is None:
            raise RuntimeError(
                f"Camera workers are not ready yet (state={state.name})."
            )

        per_camera: dict[str, list[tuple[str, str]]] = {}
        main_process: list[tuple[str, str, str]] = []
        for dev, prop, val in settings:
            if dev in self.camera_labels:
                per_camera.setdefault(dev, []).append((prop, val))
            elif self._mmc is not None and dev in self._mmc.getLoadedDevices():
                main_process.append((dev, prop, val))
            else:
                logger.debug(f"Skipping {group}/{preset}: {dev} is not loaded.")

        was_live = state is CameraWorkerServiceState.LIVE
        if was_live:
            self.stop_live()
        try:
            for label, values in per_camera.items():
                pool.set_properties(label, tuple(values))
                # Keep the respawn snapshot current, so a worker restarted by
                # spawn_async's recovery path comes back with these settings.
                if self._snapshot is not None and (
                    cam := self._snapshot.per_camera.get(label)
                ):
                    cam.property_values.update(values)
            if self._mmc is not None:
                for dev, prop, val in main_process:
                    self._mmc.setProperty(dev, prop, val)
        finally:
            if was_live:
                self.start_live()
        cfg.current = preset
        self.workerConfigChanged.emit(group, preset)

    # ------------------------------------------------------------------
    # ROI (Camera ROI widget)
    # ------------------------------------------------------------------

    def set_roi(
        self, camera_label: str, x: int, y: int, w: int, h: int, timeout: float = 5.0
    ) -> tuple[int, int, int, int]:
        """Set *camera_label*'s hardware ROI. Only valid while idle."""
        with self._lock:
            if self._state is not CameraWorkerServiceState.IDLE:
                raise RuntimeError(
                    f"Camera busy (state={self._state.name}); stop Live/MDA "
                    "before changing ROI."
                )
            pool = self._pool
        if pool is None:
            raise RuntimeError("Camera worker service has no active pool.")
        return pool.set_roi(camera_label, x, y, w, h, timeout)

    def get_roi(
        self, camera_label: str, timeout: float = 5.0
    ) -> tuple[int, int, int, int]:
        """Read *camera_label*'s current hardware ROI. Only valid while idle."""
        with self._lock:
            if self._state is not CameraWorkerServiceState.IDLE:
                raise RuntimeError(
                    f"Camera busy (state={self._state.name}); cannot read ROI "
                    "right now."
                )
            pool = self._pool
        if pool is None:
            raise RuntimeError("Camera worker service has no active pool.")
        return pool.get_roi(camera_label, timeout)
