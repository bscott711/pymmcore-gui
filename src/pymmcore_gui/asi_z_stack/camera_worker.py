"""Subprocess entry point: owns exactly one physical camera for one MDA run.

:func:`run_camera_worker` is the sole ``multiprocessing.Process(target=...)`` this
package spawns. It constructs its own, independent ``CMMCorePlus`` instance (never
:meth:`~pymmcore_plus.CMMCorePlus.instance`), loads exactly one PVCAM camera device,
and drives it through repeated arm/collect/stop cycles under the control of the main
process, communicating over a ``multiprocessing.Pipe`` (see
:mod:`~pymmcore_gui.asi_z_stack.worker_messages`) and a shared-memory frame ring
buffer (see :mod:`~pymmcore_gui.asi_z_stack.worker_pool`).

Running each physical camera in its own OS process -- not just its own device-adapter
thread -- is the point: it gives each camera its own loaded copy of ``pvcam64.dll``,
so a driver-level crash during concurrent dual-camera acquisition can, at worst, take
down one disposable worker process instead of the whole GUI application.
"""

from __future__ import annotations

import sys
import time
import traceback
from dataclasses import dataclass, field
from multiprocessing.shared_memory import SharedMemory
from typing import TYPE_CHECKING

import numpy as np
from pymmcore_plus import CMMCorePlus

from .asi_controller import (
    EXTERNAL_TRIGGER_MODES,
    preferred_external_trigger_mode,
)
from .worker_messages import (
    ArmCmd,
    ArmedMsg,
    ArmLiveCmd,
    ErrorMsg,
    FrameMsg,
    GetROICmd,
    PropertiesSetMsg,
    ReadyMsg,
    RoiMsg,
    SetPropertiesCmd,
    SetROICmd,
    ShutdownCmd,
    SlotFreeCmd,
    StalledMsg,
    StopCmd,
    StoppedMsg,
)

if TYPE_CHECKING:
    from multiprocessing.connection import Connection

_ADAPTER_MODULE = "PVCAM"
_STALL_TIMEOUT_S = 5.0
_SLOT_WAIT_WARNING_S = 30.0

# Real hardware-triggered runs occasionally see a camera's own PVCAM
# sequence auto-stop (isSequenceRunning() -> False) one image short of the
# true expected count, confirmed on the bench 2026-07-09: arming for exactly
# n_images and relying on stopOnOverflow to end the sequence races against
# real trigger-count/timing jitter, and whichever camera loses the race
# drops the *last* slice. Arming for more than we'll ever really collect and
# stopping the sequence explicitly once our own software count reaches the
# true target (see _drain_sequence) sidesteps the race entirely -- the same
# fix the old single-process engine already relied on (it over-armed by a
# factor of n_cameras and always decided "done" in software, never via
# stopOnOverflow).
_ARM_COUNT_PADDING = 50

# Bounds how many Live FrameMsgs may be outstanding (sent but not yet
# acknowledged via SlotFreeCmd) at once. The point is to bound pipe-side
# staleness even if the *main* process stalls (a slow Qt paint event, GIL
# contention, a modal dialog) -- with a round-trip ack per frame, a stalled
# main process otherwise lets frames queue up on the worker side exactly
# like the FIFO circular-buffer backlog _drain_live is meant to avoid.
# Always kept <= config.n_slots (see _drain_live), so free_slots can never
# run dry here the way it sometimes does in _drain_sequence.
_LIVE_MAX_IN_FLIGHT = 2

# How often (seconds of worker wall-clock time) _drain_live logs a one-line
# sent/skipped throughput summary while Live is running.
_LIVE_STATUS_REPORT_INTERVAL_S = 10.0


@dataclass(frozen=True)
class CameraWorkerConfig:
    """Everything :func:`run_camera_worker` needs, picklable across the spawn."""

    camera_label: str
    adapter_device_name: str
    property_snapshot: dict[str, str] = field(default_factory=dict)
    roi: tuple[int, int, int, int] | None = None
    circular_buffer_mb: int = 4096
    shm_name: str = ""
    slot_nbytes: int = 0
    n_slots: int = 8
    live_max_fps: float = 30.0
    """Rate cap for :func:`_drain_live` (see
    :attr:`~pymmcore_gui.asi_z_stack.common.HardwareConstants.live_max_fps`).
    ``0`` means uncapped."""
    stderr_log_file: str = ""
    """Sibling file this worker's stderr/stdout is redirected to (see
    :func:`~pymmcore_gui.asi_z_stack.worker_pool._worker_stderr_file`).
    Empty means leave stderr as inherited from the parent process. Exists
    because a spawned multiprocessing child on Windows, under a GUI app
    with no attached console, has _log()'s stderr diagnostics go nowhere
    visible -- confirmed on the rig 2026-09-22: a worker hung during
    _apply_snapshot (most likely the TriggerMode set -- see diagnostics.py's
    "Level Trigger hangs" note) with zero trace of which call it was stuck
    on, anywhere."""


def _log(camera_label: str, message: str) -> None:
    """Print a worker-prefixed diagnostic line to stderr, flushed immediately.

    Parameters
    ----------
    camera_label : str
        The physical camera this worker owns, used as a log-line prefix.
    message : str
        The line to print.
    """
    print(f"[worker:{camera_label}] {message}", file=sys.stderr, flush=True)


def _apply_snapshot(mmc: CMMCorePlus, config: CameraWorkerConfig) -> None:
    """Best-effort reapply *config*'s property/ROI snapshot, then set external trigger.

    Restoring the snapshot first and searching for the camera's external
    trigger mode second (rather than receiving a pre-computed value from the
    main process) means this worker's own live ``getAllowedPropertyValues``
    is authoritative -- and correctly overrides any stale/idle-state
    ``TriggerMode`` the generic property sweep in ``config.property_snapshot``
    may have captured.

    Parameters
    ----------
    mmc : CMMCorePlus
        The worker's own core instance.
    config : CameraWorkerConfig
        Carries the property/ROI values to reapply.
    """
    label = config.camera_label
    for prop, value in config.property_snapshot.items():
        try:
            mmc.setProperty(label, prop, value)
        except Exception as exc:
            _log(label, f"could not restore property {prop!r}={value!r}: {exc}")
    if config.roi is not None:
        try:
            mmc.setROI(label, *config.roi)
        except Exception as exc:
            _log(label, f"could not restore ROI {config.roi}: {exc}")

    trigger_mode = preferred_external_trigger_mode(mmc, label)
    if trigger_mode is None:
        _log(label, "no external TriggerMode found; leaving current TriggerMode as-is")
        return
    try:
        mmc.setProperty(label, "TriggerMode", trigger_mode)
    except Exception as exc:
        _log(label, f"could not set TriggerMode={trigger_mode!r}: {exc}")


def _drain_incoming(
    conn: Connection, free_slots: list[int]
) -> StopCmd | ShutdownCmd | None:
    """Drain any pending ``SlotFreeCmd``/``StopCmd``/``ShutdownCmd`` without blocking.

    Parameters
    ----------
    conn : Connection
        The pipe to the main process.
    free_slots : list[int]
        Mutated in place: newly-freed slot indices are appended.

    Returns
    -------
    StopCmd | ShutdownCmd | None
        The ``ShutdownCmd`` if one was seen anywhere in this drain pass -- it
        always wins over a ``StopCmd`` seen in the same pass (see
        ``ShutdownCmd``'s docstring: "exit the worker process" is a
        stronger signal than "stop the current sequence"). Otherwise
        ``StopCmd`` if one was seen, otherwise ``None``.
    """
    stop_signal: StopCmd | ShutdownCmd | None = None
    while conn.poll(0):
        msg = conn.recv()
        if isinstance(msg, SlotFreeCmd):
            free_slots.append(msg.slot_index)
        elif isinstance(msg, ShutdownCmd):
            stop_signal = msg
        elif isinstance(msg, StopCmd) and not isinstance(stop_signal, ShutdownCmd):
            # Keep the latest StopCmd: its token is the one being waited on.
            stop_signal = msg
    return stop_signal


def _wait_for_free_slot(
    conn: Connection, free_slots: list[int], label: str
) -> StopCmd | ShutdownCmd | None:
    """Block until a slot frees up or a stop/shutdown is requested.

    Parameters
    ----------
    conn : Connection
        The pipe to the main process.
    free_slots : list[int]
        Mutated in place: a newly-freed slot index is appended.
    label : str
        The owning camera's label, for the slow-drain warning log line.

    Returns
    -------
    StopCmd | ShutdownCmd | None
        The ``StopCmd`` or ``ShutdownCmd`` -- whichever arrives first -- if one
        is seen while waiting, else ``None`` once a slot has freed up.
    """
    waited_s = 0.0
    while not free_slots:
        if conn.poll(1.0):
            msg = conn.recv()
            if isinstance(msg, SlotFreeCmd):
                free_slots.append(msg.slot_index)
            elif isinstance(msg, ShutdownCmd | StopCmd):
                return msg
        else:
            waited_s += 1.0
            if waited_s >= _SLOT_WAIT_WARNING_S:
                _log(label, f"no free frame slot for {waited_s:.0f}s -- main stalled?")
    return None


def _token(stop_signal: StopCmd | ShutdownCmd) -> int:
    """The token a StoppedMsg acknowledging *stop_signal* must echo."""
    return stop_signal.token if isinstance(stop_signal, StopCmd) else 0


def _set_trigger_mode(mmc: CMMCorePlus, label: str, mode: str | None) -> None:
    """Switch *label*'s ``TriggerMode`` to *mode*, only if it differs.

    Snap/Live need the camera's internal trigger and a hardware-triggered MDA
    needs the external one; the persistent worker serves all three, so the
    mode is chosen per arm instead of once at startup. Skipping no-op sets
    keeps back-to-back arms of the same kind free of PVCAM reconfiguration.
    """
    if mode is None or mmc.getProperty(label, "TriggerMode") == mode:
        return
    t0 = time.perf_counter()
    mmc.setProperty(label, "TriggerMode", mode)
    _log(label, f"TriggerMode -> {mode!r} ({(time.perf_counter() - t0) * 1e3:.0f} ms)")


def _internal_trigger_mode(
    mmc: CMMCorePlus, label: str, snapshot_mode: str | None
) -> str | None:
    """Pick the free-running (non-external) ``TriggerMode`` for Snap/Live.

    Prefers the mode the main process had the camera in before handing it
    off (that's what Snap/Live used before the camera moved to a worker),
    unless that was itself an external mode; then PVCAM's
    ``"Internal Trigger"``.
    """
    if not mmc.hasProperty(label, "TriggerMode"):
        return None
    allowed = set(mmc.getAllowedPropertyValues(label, "TriggerMode"))
    if (
        snapshot_mode
        and snapshot_mode in allowed
        and snapshot_mode not in EXTERNAL_TRIGGER_MODES
    ):
        return snapshot_mode
    if "Internal Trigger" in allowed:
        return "Internal Trigger"
    return None


def _stop_and_clear(mmc: CMMCorePlus, label: str) -> None:
    """Stop the camera's sequence (if running) and drop any buffered frames.

    Called at the end of every arm/drain cycle (bounded or live). New
    requirement now that a worker is armed/drained/stopped repeatedly across
    a whole session (Live toggles, Snaps, MDA runs) rather than exactly once
    per process lifetime, as it was when this module was MDA-only -- without
    this, frames left over in the camera's own circular buffer from one
    cycle could bleed into the next arm cycle's first frames.
    """
    if mmc.isSequenceRunning(label):
        mmc.stopSequenceAcquisition(label)
    mmc.clearCircularBuffer()


def _drain_sequence(
    mmc: CMMCorePlus,
    conn: Connection,
    shm: SharedMemory,
    config: CameraWorkerConfig,
    n_images: int,
) -> bool:
    """Pop frames off the camera's circular buffer until *n_images* or a stop.

    Parameters
    ----------
    mmc : CMMCorePlus
        The worker's own core instance, already armed via
        ``startSequenceAcquisition``.
    conn : Connection
        The pipe to the main process.
    shm : SharedMemory
        The attached frame ring buffer to write pixel data into.
    config : CameraWorkerConfig
        Supplies ``camera_label`` and ``slot_nbytes``.
    n_images : int
        The number of frames this arm cycle expects.

    Returns
    -------
    bool
        ``True`` if a ``ShutdownCmd`` ended this drain -- the caller
        (``run_camera_worker``) must break its outer command loop and exit
        the process. ``False`` for an ordinary ``StopCmd``, an unexpected
        sequence stop, or normal completion -- the caller returns to
        waiting for the next ``ArmCmd``.
    """
    label = config.camera_label
    free_slots = list(range(config.n_slots))
    slice_idx = 0
    images_collected = 0
    last_image_time = time.monotonic()
    last_stall_report = 0.0

    while images_collected < n_images:
        stop_signal = _drain_incoming(conn, free_slots)
        if stop_signal is not None:
            _stop_and_clear(mmc, label)
            conn.send(StoppedMsg(label, images_collected, _token(stop_signal)))
            return isinstance(stop_signal, ShutdownCmd)

        remaining = mmc.getRemainingImageCount()
        if remaining > 0:
            if not free_slots:
                stop_signal = _wait_for_free_slot(conn, free_slots, label)
                if stop_signal is not None:
                    _stop_and_clear(mmc, label)
                    conn.send(StoppedMsg(label, images_collected, _token(stop_signal)))
                    return isinstance(stop_signal, ShutdownCmd)

            slot = free_slots.pop(0)
            img, mm_meta = mmc.popNextImageAndMD()
            try:
                camera_metadata = dict(mm_meta.items())
            except Exception:
                camera_metadata = {}

            data = img.tobytes()
            offset = slot * config.slot_nbytes
            shm.buf[offset : offset + len(data)] = data
            conn.send(
                FrameMsg(
                    camera_label=label,
                    slot_index=slot,
                    slice_idx=slice_idx,
                    nbytes=len(data),
                    camera_metadata=camera_metadata,
                    images_remaining=remaining - 1,
                    worker_perf_counter=time.perf_counter(),
                )
            )
            slice_idx += 1
            images_collected += 1
            last_image_time = time.monotonic()
        elif not mmc.isSequenceRunning():
            _stop_and_clear(mmc, label)
            conn.send(
                ErrorMsg(
                    camera_label=label,
                    exc_type="RuntimeError",
                    message=(
                        f"Sequence stopped unexpectedly after "
                        f"{images_collected}/{n_images} images."
                    ),
                    traceback_text="",
                )
            )
            return False
        else:
            now = time.monotonic()
            if (
                now - last_image_time > _STALL_TIMEOUT_S
                and now - last_stall_report >= 1.0
            ):
                last_stall_report = now
                conn.send(
                    StalledMsg(
                        camera_label=label,
                        images_collected=images_collected,
                        seconds_since_last_image=now - last_image_time,
                    )
                )
            time.sleep(0.005)

    # We deliberately armed for more than n_images (see _ARM_COUNT_PADDING),
    # so the camera's own stopOnOverflow won't have ended the sequence yet --
    # our software count reaching the true target is what decides "done".
    _stop_and_clear(mmc, label)
    conn.send(StoppedMsg(label, images_collected))
    return False


def _drain_live(
    mmc: CMMCorePlus,
    conn: Connection,
    shm: SharedMemory,
    config: CameraWorkerConfig,
) -> bool:
    """Ship the newest camera frame to the main process, dropping the rest.

    Free-running counterpart to :func:`_drain_sequence` for Live streaming:
    no target frame count, no ``_ARM_COUNT_PADDING`` over-arm/software-stop
    dance (there's no hardware trigger jitter to race here -- the camera is
    simply told to run and told to stop).

    Unlike ``_drain_sequence``, this does **not** drain the circular buffer
    oldest-frame-first with ``popNextImageAndMD``. On this rig's real frame
    rate (~100 fps at ~11 MB/frame on a Kinetix/PVCAM camera) that FIFO drain
    could not keep up, so the 4096 MB buffer backed up and the Live preview
    lagged reality by up to ~4 s (observed on the rig 2026-10-05; matches the
    buffer's ~370-frame capacity at that rate). A live preview only ever
    wants to *see* the most recent frame, not work through a backlog -- so
    this instead grabs the newest frame with ``getLastImageAndMD`` and throws
    away everything else in the buffer with ``clearCircularBuffer``,
    mirroring the non-worker preview's own newest-frame logic (see
    ``_pop_latest_frame_group`` in ``widgets/image_preview/_preview_base.py``).

    Two independent caps bound how fast frames are actually shipped, no
    matter how fast the camera itself is running:

    - **Rate cap** (``config.live_max_fps``): a frame is fetched and sent
      only once at least ``1 / live_max_fps`` seconds have elapsed since
      the last send (``<= 0`` means uncapped). Between sends, the camera's
      circular buffer is left alone to keep filling -- the next send drops
      it wholesale, so pixel data is never read unless it's about to be
      used.
    - **In-flight cap** (``_LIVE_MAX_IN_FLIGHT``): at most this many
      ``FrameMsg`` messages may be outstanding (sent but not yet freed via
      ``SlotFreeCmd``) at once, so a stalled *main* process can't let
      frames pile up on the pipe/shm side either. This is always kept
      ``<= config.n_slots``, so (unlike ``_drain_sequence``) a free slot is
      always available whenever a send is actually due, and the
      ``_wait_for_free_slot`` blocking-wait dance is never needed here.

    Parameters
    ----------
    mmc : CMMCorePlus
        The worker's own core instance, already armed via
        ``startContinuousSequenceAcquisition``.
    conn : Connection
        The pipe to the main process.
    shm : SharedMemory
        The attached frame ring buffer to write pixel data into.
    config : CameraWorkerConfig
        Supplies ``camera_label``, ``slot_nbytes``, ``n_slots``, and
        ``live_max_fps``.

    Returns
    -------
    bool
        ``True`` if a ``ShutdownCmd`` ended this drain (caller must exit the
        process), ``False`` for an ordinary ``StopCmd`` or an unexpected
        sequence stop (caller returns to waiting for the next arm command).
    """
    label = config.camera_label
    max_in_flight = min(_LIVE_MAX_IN_FLIGHT, config.n_slots)
    free_slots = list(range(config.n_slots))
    slice_idx = 0
    images_collected = 0
    last_image_time = time.monotonic()
    last_stall_report = 0.0
    # -inf so the first iteration is always "due", whatever time.monotonic()
    # happens to return.
    last_send_time = -float("inf")
    min_send_interval = 1.0 / config.live_max_fps if config.live_max_fps > 0 else 0.0
    sent_since_report = 0
    skipped_since_report = 0
    last_report_time = time.monotonic()

    while True:
        stop_signal = _drain_incoming(conn, free_slots)
        if stop_signal is not None:
            _stop_and_clear(mmc, label)
            conn.send(StoppedMsg(label, images_collected, _token(stop_signal)))
            return isinstance(stop_signal, ShutdownCmd)

        now = time.monotonic()
        if now - last_report_time >= _LIVE_STATUS_REPORT_INTERVAL_S:
            _log(
                label,
                f"live: sent {sent_since_report} frames, skipped "
                f"{skipped_since_report} in {now - last_report_time:.1f}s",
            )
            sent_since_report = 0
            skipped_since_report = 0
            last_report_time = now

        remaining = mmc.getRemainingImageCount()
        if remaining > 0:
            # A frame is actually available -- reset the stall clock here,
            # independent of whether the caps below let us send it. If this
            # only reset on a *send*, the rate cap alone could make a
            # healthy camera look stalled.
            last_image_time = now

        in_flight = config.n_slots - len(free_slots)
        due = min_send_interval <= 0.0 or (now - last_send_time) >= min_send_interval
        if remaining > 0 and due and in_flight < max_in_flight:
            slot = free_slots.pop(0)
            img, mm_meta = mmc.getLastImageAndMD()
            try:
                camera_metadata = dict(mm_meta.items())
            except Exception:
                camera_metadata = {}

            if img.nbytes > config.slot_nbytes:
                raise ValueError(
                    f"{label}: frame is {img.nbytes} bytes, exceeds this "
                    f"worker's {config.slot_nbytes}-byte frame slots"
                )
            # Copy pixels straight into the slot (no tobytes() intermediate),
            # and do it *before* dropping the rest of the buffer below.
            dst: np.ndarray = np.ndarray(
                img.shape,
                dtype=img.dtype,
                buffer=shm.buf,
                offset=slot * config.slot_nbytes,
            )
            dst[...] = img
            mmc.clearCircularBuffer()

            skipped = remaining - 1
            conn.send(
                FrameMsg(
                    camera_label=label,
                    slot_index=slot,
                    slice_idx=slice_idx,
                    nbytes=img.nbytes,
                    camera_metadata=camera_metadata,
                    images_remaining=skipped,
                    worker_perf_counter=time.perf_counter(),
                )
            )
            slice_idx += 1
            images_collected += 1
            last_send_time = now
            sent_since_report += 1
            skipped_since_report += skipped
        elif remaining == 0 and not mmc.isSequenceRunning():
            _stop_and_clear(mmc, label)
            conn.send(
                ErrorMsg(
                    camera_label=label,
                    exc_type="RuntimeError",
                    message=(
                        f"Live sequence stopped unexpectedly after "
                        f"{images_collected} images."
                    ),
                    traceback_text="",
                )
            )
            return False
        else:
            # Either nothing is available yet, or a frame is waiting but
            # isn't due to be sent (rate cap) or there's no in-flight
            # headroom -- either way, don't touch the camera; just keep
            # servicing the pipe and check back shortly.
            if (
                now - last_image_time > _STALL_TIMEOUT_S
                and now - last_stall_report >= 1.0
            ):
                last_stall_report = now
                conn.send(
                    StalledMsg(
                        camera_label=label,
                        images_collected=images_collected,
                        seconds_since_last_image=now - last_image_time,
                    )
                )
            time.sleep(0.005)


def run_camera_worker(config: CameraWorkerConfig, conn: Connection) -> None:
    """``Process`` target: own *config.camera_label* for the life of the process.

    Parameters
    ----------
    config : CameraWorkerConfig
        Which camera to load and how to configure it.
    conn : Connection
        The pipe half connected to this worker's
        :class:`~pymmcore_gui.asi_z_stack.worker_pool.CameraWorkerHandle` in
        the main process.
    """
    label = config.camera_label
    if config.stderr_log_file:
        # Redirect BEFORE anything else runs, so even a startup failure
        # (the except block just below) or a hang with zero further output
        # still leaves whatever _log()/traceback lines did fire somewhere
        # recoverable -- see CameraWorkerConfig.stderr_log_file's docstring.
        # Best-effort: falling back to inherited stderr beats crashing the
        # worker over a logging setup failure.
        try:
            log_file = open(config.stderr_log_file, "a", buffering=1)
            sys.stderr = log_file
            sys.stdout = log_file
        except OSError:
            pass
    try:
        mmc = CMMCorePlus()
        mmc.setCircularBufferMemoryFootprint(config.circular_buffer_mb)
        mmc.loadDevice(label, _ADAPTER_MODULE, config.adapter_device_name)
        mmc.initializeDevice(label)
        _apply_snapshot(mmc, config)
        mmc.setCameraDevice(label)
        external_mode = preferred_external_trigger_mode(mmc, label)
        internal_mode = _internal_trigger_mode(
            mmc, label, config.property_snapshot.get("TriggerMode")
        )
        _log(
            label,
            f"trigger modes: internal={internal_mode!r} external={external_mode!r}",
        )
        shm = SharedMemory(name=config.shm_name, create=False)
    except Exception:
        _log(label, f"startup failed:\n{traceback.format_exc()}")
        conn.send(
            ErrorMsg(
                camera_label=label,
                exc_type="StartupError",
                message="worker failed to load/initialize its camera",
                traceback_text=traceback.format_exc(),
            )
        )
        return

    conn.send(ReadyMsg(label))

    try:
        while True:
            try:
                cmd = conn.recv()
            except EOFError:
                break

            if isinstance(cmd, ArmCmd):
                shutdown_requested = False
                try:
                    _set_trigger_mode(
                        mmc,
                        label,
                        external_mode if cmd.external_trigger else internal_mode,
                    )
                    armed_count = cmd.n_images + _ARM_COUNT_PADDING
                    mmc.startSequenceAcquisition(label, armed_count, 0, True)
                    conn.send(ArmedMsg(label))
                    shutdown_requested = _drain_sequence(
                        mmc, conn, shm, config, cmd.n_images
                    )
                except Exception:
                    _log(label, f"arm/drain failed:\n{traceback.format_exc()}")
                    conn.send(
                        ErrorMsg(
                            camera_label=label,
                            exc_type="AcquisitionError",
                            message="worker failed during arm/drain",
                            traceback_text=traceback.format_exc(),
                        )
                    )
                if shutdown_requested:
                    break
            elif isinstance(cmd, ArmLiveCmd):
                shutdown_requested = False
                try:
                    _set_trigger_mode(mmc, label, internal_mode)
                    mmc.startContinuousSequenceAcquisition(0)
                    conn.send(ArmedMsg(label))
                    shutdown_requested = _drain_live(mmc, conn, shm, config)
                except Exception:
                    _log(label, f"live arm/drain failed:\n{traceback.format_exc()}")
                    conn.send(
                        ErrorMsg(
                            camera_label=label,
                            exc_type="AcquisitionError",
                            message="worker failed during live arm/drain",
                            traceback_text=traceback.format_exc(),
                        )
                    )
                if shutdown_requested:
                    break
            elif isinstance(cmd, StopCmd):
                if mmc.isSequenceRunning(label):
                    mmc.stopSequenceAcquisition(label)
                if cmd.token:
                    conn.send(StoppedMsg(label, 0, cmd.token))
            elif isinstance(cmd, ShutdownCmd):
                break
            elif isinstance(cmd, SetROICmd):
                # Only reachable here (the idle command loop), never mid-drain
                # -- ROI changes only make sense while this camera isn't
                # streaming, matching the real hardware constraint.
                try:
                    mmc.setROI(label, cmd.x, cmd.y, cmd.w, cmd.h)
                    x, y, w, h = mmc.getROI(label)
                    conn.send(RoiMsg(label, x, y, w, h))
                except Exception as exc:
                    conn.send(RoiMsg(label, 0, 0, 0, 0, error=str(exc)))
            elif isinstance(cmd, SetPropertiesCmd):
                # Idle-loop only, like SetROICmd: e.g. PVCAM rejects a Port
                # change while a sequence is running.
                try:
                    for prop, value in cmd.values:
                        mmc.setProperty(label, prop, value)
                    conn.send(PropertiesSetMsg(label))
                except Exception as exc:
                    conn.send(PropertiesSetMsg(label, error=str(exc)))
            elif isinstance(cmd, GetROICmd):
                try:
                    x, y, w, h = mmc.getROI(label)
                    conn.send(RoiMsg(label, x, y, w, h))
                except Exception as exc:
                    conn.send(RoiMsg(label, 0, 0, 0, 0, error=str(exc)))
    finally:
        try:
            if mmc.isSequenceRunning(label):
                mmc.stopSequenceAcquisition(label)
        except Exception:
            pass
        try:
            mmc.unloadDevice(label)
        except Exception:
            pass
        shm.close()
