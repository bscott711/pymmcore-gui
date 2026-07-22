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

from pymmcore_plus import CMMCorePlus

from .asi_controller import preferred_external_trigger_mode
from .worker_messages import (
    ArmCmd,
    ArmedMsg,
    ErrorMsg,
    FrameMsg,
    ReadyMsg,
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
) -> type[StopCmd] | type[ShutdownCmd] | None:
    """Drain any pending ``SlotFreeCmd``/``StopCmd``/``ShutdownCmd`` without blocking.

    Parameters
    ----------
    conn : Connection
        The pipe to the main process.
    free_slots : list[int]
        Mutated in place: newly-freed slot indices are appended.

    Returns
    -------
    type[StopCmd] | type[ShutdownCmd] | None
        ``ShutdownCmd`` if one was seen anywhere in this drain pass -- it
        always wins over a ``StopCmd`` seen in the same pass (see
        ``ShutdownCmd``'s docstring: "exit the worker process" is a
        stronger signal than "stop the current sequence"). Otherwise
        ``StopCmd`` if one was seen, otherwise ``None``.
    """
    stop_signal: type[StopCmd] | type[ShutdownCmd] | None = None
    while conn.poll(0):
        msg = conn.recv()
        if isinstance(msg, SlotFreeCmd):
            free_slots.append(msg.slot_index)
        elif isinstance(msg, ShutdownCmd):
            stop_signal = ShutdownCmd
        elif isinstance(msg, StopCmd) and stop_signal is None:
            stop_signal = StopCmd
    return stop_signal


def _wait_for_free_slot(
    conn: Connection, free_slots: list[int], label: str
) -> type[StopCmd] | type[ShutdownCmd] | None:
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
    type[StopCmd] | type[ShutdownCmd] | None
        ``StopCmd`` or ``ShutdownCmd`` -- whichever arrives first -- if one
        is seen while waiting, else ``None`` once a slot has freed up.
    """
    waited_s = 0.0
    while not free_slots:
        if conn.poll(1.0):
            msg = conn.recv()
            if isinstance(msg, SlotFreeCmd):
                free_slots.append(msg.slot_index)
            elif isinstance(msg, ShutdownCmd):
                return ShutdownCmd
            elif isinstance(msg, StopCmd):
                return StopCmd
        else:
            waited_s += 1.0
            if waited_s >= _SLOT_WAIT_WARNING_S:
                _log(label, f"no free frame slot for {waited_s:.0f}s -- main stalled?")
    return None


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

    while images_collected < n_images:
        stop_signal = _drain_incoming(conn, free_slots)
        if stop_signal is not None:
            mmc.stopSequenceAcquisition(label)
            conn.send(StoppedMsg(label, images_collected))
            return stop_signal is ShutdownCmd

        remaining = mmc.getRemainingImageCount()
        if remaining > 0:
            if not free_slots:
                stop_signal = _wait_for_free_slot(conn, free_slots, label)
                if stop_signal is not None:
                    mmc.stopSequenceAcquisition(label)
                    conn.send(StoppedMsg(label, images_collected))
                    return stop_signal is ShutdownCmd

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
            if now - last_image_time > _STALL_TIMEOUT_S:
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
    if mmc.isSequenceRunning(label):
        mmc.stopSequenceAcquisition(label)
    conn.send(StoppedMsg(label, images_collected))
    return False


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
    try:
        mmc = CMMCorePlus()
        mmc.setCircularBufferMemoryFootprint(config.circular_buffer_mb)
        mmc.loadDevice(label, _ADAPTER_MODULE, config.adapter_device_name)
        mmc.initializeDevice(label)
        _apply_snapshot(mmc, config)
        mmc.setCameraDevice(label)
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
            elif isinstance(cmd, StopCmd):
                if mmc.isSequenceRunning(label):
                    mmc.stopSequenceAcquisition(label)
            elif isinstance(cmd, ShutdownCmd):
                break
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
