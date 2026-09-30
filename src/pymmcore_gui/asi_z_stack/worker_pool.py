"""Main-process orchestration of one or more camera worker subprocesses.

:class:`CameraWorkerPool` owns the whole lifecycle of a set of
:mod:`~pymmcore_gui.asi_z_stack.camera_worker` subprocesses for one hardware-
triggered MDA run: spawning them, arming them together, draining their
interleaved frame streams, and shutting them down. It never blocks on a single
worker in isolation -- every wait uses ``multiprocessing.connection.wait()``
across every worker's control pipe *and* its process sentinel together, so a
worker that dies outright (e.g. a native ``pvcam64.dll`` crash) is detected as
promptly as one that simply sends a message, and is reported as a distinct,
recognizable :class:`WorkerDiedError` rather than a hang.

Frame pixel data travels through a per-worker
``multiprocessing.shared_memory.SharedMemory`` ring buffer that this module
creates and owns (workers only attach to it); control messages travel over a
``multiprocessing.Pipe`` (see :mod:`~pymmcore_gui.asi_z_stack.worker_messages`).
"""

from __future__ import annotations

import contextlib
import os
import re
import time
from dataclasses import replace
from multiprocessing import get_context
from multiprocessing.connection import wait as mp_wait
from multiprocessing.shared_memory import SharedMemory
from typing import TYPE_CHECKING, Any

import numpy as np
from pymmcore_plus._logger import current_logfile
from pymmcore_plus._logger import logger as _pymmcore_plus_logger

from .camera_worker import CameraWorkerConfig, run_camera_worker
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
    from collections.abc import Iterator
    from multiprocessing.context import SpawnContext
    from multiprocessing.process import BaseProcess

    # multiprocessing.Pipe() returns platform-specific connection types
    # (Connection on POSIX, PipeConnection on Windows) with no common public
    # base type in typeshed -- Any is the pragmatic cross-platform choice.
    Connection = Any


def _worker_log_file(camera_label: str) -> str:
    """Compute a per-worker sibling of the main process's pymmcore-plus logfile.

    Every worker process re-imports ``pymmcore_plus`` on spawn, which
    re-runs its import-time ``configure_logging()`` and would otherwise
    attach a second, independent ``RotatingFileHandler`` to the exact same
    shared logfile as the main process. On Windows that second open handle
    blocks the main process's log rotation (``os.rename`` -> ``WinError
    32``). Pointing each worker at its own file via ``PYMM_LOG_FILE``
    sidesteps this without touching pymmcore-plus internals.

    Parameters
    ----------
    camera_label : str
        The camera this worker owns, used to make its logfile name unique.

    Returns
    -------
    str
        A sibling logfile path, or ``"0"`` (pymmcore-plus's own syntax for
        "disable file logging") if the main process has no logfile
        configured.
    """
    current = current_logfile(_pymmcore_plus_logger)
    if current is None:
        return "0"
    safe_label = re.sub(r"[^\w-]", "_", camera_label)
    return str(current.with_name(f"{current.stem}-worker-{safe_label}{current.suffix}"))


def _worker_stderr_file(camera_label: str) -> str:
    """Compute a per-worker stderr/stdout capture file, sibling to its log.

    ``camera_worker.py``'s own diagnostic ``_log()`` calls print to stderr,
    which a spawned ``multiprocessing`` child has no visible destination
    for under a GUI app launched with no attached console -- confirmed on
    the rig 2026-09-22: a worker hung during startup with zero trace of
    which call it was stuck on anywhere, once the process was gone. Reuses
    the same naming convention as :func:`_worker_log_file` so both files
    sit next to each other.

    Parameters
    ----------
    camera_label : str
        The camera this worker owns, used to make its file name unique.

    Returns
    -------
    str
        A sibling file path, or ``""`` (meaning "leave stderr as inherited,
        don't redirect") if the main process has no logfile configured --
        there is no sensible sibling location to pick in that case.
    """
    current = current_logfile(_pymmcore_plus_logger)
    if current is None:
        return ""
    safe_label = re.sub(r"[^\w-]", "_", camera_label)
    return str(
        current.with_name(f"{current.stem}-worker-{safe_label}-stderr{current.suffix}")
    )


class WorkerDiedError(RuntimeError):
    """A camera worker process exited unexpectedly (e.g. a native crash).

    Distinguishable from an ordinary :class:`RuntimeError` raised from a
    worker's own :class:`~pymmcore_gui.asi_z_stack.worker_messages.ErrorMsg`:
    this means the *process itself* is gone, not that it reported a soft
    error and returned to idle. Contains the OS exit code where available --
    a native crash typically shows up as a large negative/unsigned value
    (e.g. ``STATUS_STACK_BUFFER_OVERRUN``'s signed 32-bit form).
    """

    def __init__(self, camera_label: str, exitcode: int | None) -> None:
        self.camera_label = camera_label
        self.exitcode = exitcode
        super().__init__(
            f"camera worker for {camera_label!r} died (exit code {exitcode})"
        )


class CameraWorkerHandle:
    """One camera worker subprocess: its pipe, shared-memory ring, and state."""

    def __init__(
        self,
        camera_label: str,
        config: CameraWorkerConfig,
        height: int,
        width: int,
        dtype: str,
        n_slots: int,
    ) -> None:
        self.camera_label = camera_label
        self.config = config
        self.height = height
        self.width = width
        self.dtype = dtype
        self.n_slots = n_slots
        self.slot_nbytes = height * width * np.dtype(dtype).itemsize
        self.process: BaseProcess | None = None
        self.conn: Connection | None = None
        self.shm: SharedMemory | None = None

    def spawn(self, ctx: SpawnContext) -> None:
        """Create this worker's shared memory and pipe, then start its process.

        Parameters
        ----------
        ctx : SpawnContext
            The ``multiprocessing`` spawn context to launch the process with.
        """
        shm = SharedMemory(create=True, size=self.slot_nbytes * self.n_slots)
        self.shm = shm
        self.config = replace(
            self.config,
            shm_name=shm.name,
            slot_nbytes=self.slot_nbytes,
            n_slots=self.n_slots,
            stderr_log_file=_worker_stderr_file(self.camera_label),
        )
        parent_conn, child_conn = ctx.Pipe(duplex=True)
        self.conn = parent_conn
        process = ctx.Process(
            target=run_camera_worker,
            args=(self.config, child_conn),
            daemon=False,
        )
        # See _worker_log_file: give this worker its own pymmcore-plus
        # logfile so it doesn't fight the main process over rotation of
        # the shared one. spawn_all() spawns workers one at a time, so
        # this set/restore around start() (which snapshots the env for
        # the new process) can't race with another worker's spawn().
        prev_log_file = os.environ.get("PYMM_LOG_FILE")
        os.environ["PYMM_LOG_FILE"] = _worker_log_file(self.camera_label)
        try:
            process.start()
        finally:
            if prev_log_file is None:
                os.environ.pop("PYMM_LOG_FILE", None)
            else:
                os.environ["PYMM_LOG_FILE"] = prev_log_file
        self.process = process
        child_conn.close()

    def read_frame(self, slot_index: int, nbytes: int) -> np.ndarray:
        """Copy one frame out of shared memory slot *slot_index*.

        Parameters
        ----------
        slot_index : int
            Which ring-buffer slot the worker wrote the frame into.
        nbytes : int
            The exact byte length the worker wrote (for a truncated/short
            final row this can be less than ``slot_nbytes``, though normally
            they match).

        Returns
        -------
        np.ndarray
            An owned copy (safe to use after the slot is recycled).
        """
        assert self.shm is not None
        offset = slot_index * self.slot_nbytes
        expected = self.height * self.width * np.dtype(self.dtype).itemsize
        if nbytes != expected:
            raise ValueError(
                f"{self.camera_label} frame is {nbytes} bytes, expected {expected}"
            )
        arr: np.ndarray = np.ndarray(
            (self.height, self.width),
            dtype=self.dtype,
            buffer=self.shm.buf,
            offset=offset,
        )
        return arr.copy()

    def set_roi(
        self, x: int, y: int, w: int, h: int, timeout: float = 5.0
    ) -> tuple[int, int, int, int]:
        """Set this worker's camera ROI and return the ROI now in effect.

        Only valid while the worker is idle (not mid-arm/drain) -- callers
        (:class:`~pymmcore_gui.asi_z_stack.camera_worker_service.
        CameraWorkerService`) are responsible for enforcing that; this is a
        plain synchronous request/response over the same pipe ``iter_frames``
        streams frames over, safe here only because no concurrent
        ``FrameMsg`` traffic is possible while idle.
        """
        assert self.conn is not None
        self.conn.send(SetROICmd(self.camera_label, x, y, w, h))
        return self._recv_roi(timeout)

    def get_roi(self, timeout: float = 5.0) -> tuple[int, int, int, int]:
        """Read this worker's camera ROI. Only valid while the worker is idle."""
        assert self.conn is not None
        self.conn.send(GetROICmd(self.camera_label))
        return self._recv_roi(timeout)

    def set_properties(
        self, values: tuple[tuple[str, str], ...], timeout: float = 5.0
    ) -> None:
        """Set this worker's camera properties, in order. Only valid while idle.

        Same idle-only request/response contract as :meth:`set_roi`.
        """
        assert self.conn is not None
        self.conn.send(SetPropertiesCmd(self.camera_label, values))
        if not self.conn.poll(timeout):
            raise TimeoutError(
                f"{self.camera_label} did not respond to a property request "
                f"within {timeout:.1f}s"
            )
        msg = self.conn.recv()
        if not isinstance(msg, PropertiesSetMsg):
            raise RuntimeError(
                f"{self.camera_label}: unexpected reply to property request: {msg!r}"
            )
        if msg.error is not None:
            raise RuntimeError(f"{self.camera_label}: {msg.error}")

    def _recv_roi(self, timeout: float) -> tuple[int, int, int, int]:
        assert self.conn is not None
        if not self.conn.poll(timeout):
            raise TimeoutError(
                f"{self.camera_label} did not respond to an ROI request "
                f"within {timeout:.1f}s"
            )
        msg = self.conn.recv()
        if not isinstance(msg, RoiMsg):
            raise RuntimeError(
                f"{self.camera_label}: unexpected reply to ROI request: {msg!r}"
            )
        if msg.error is not None:
            raise RuntimeError(f"{self.camera_label}: {msg.error}")
        return msg.x, msg.y, msg.w, msg.h


class CameraWorkerPool:
    """Orchestrates a set of :class:`CameraWorkerHandle` for one MDA run."""

    def __init__(self, workers: list[CameraWorkerHandle]) -> None:
        self.workers = workers
        self._ctx = get_context("spawn")
        self._stop_token = 0

    def spawn_all(self, ready_timeout: float = 30.0) -> None:
        """Start every worker process and wait for all of them to report ready.

        Parameters
        ----------
        ready_timeout : float
            Seconds to wait for every worker's
            :class:`~pymmcore_gui.asi_z_stack.worker_messages.ReadyMsg`.
        """
        for worker in self.workers:
            worker.spawn(self._ctx)
        self._wait_for_all(ReadyMsg, ready_timeout)

    def arm_all(
        self,
        n_images: int,
        armed_timeout: float = 10.0,
        *,
        external_trigger: bool = True,
    ) -> None:
        """Arm every worker for *n_images* frames and wait for all acks.

        Parameters
        ----------
        n_images : int
            The true per-camera frame count this arm cycle expects (not
            multiplied by camera count -- each worker owns exactly one
            physical camera).
        armed_timeout : float
            Seconds to wait for every worker's
            :class:`~pymmcore_gui.asi_z_stack.worker_messages.ArmedMsg`.
        external_trigger : bool
            ``True`` (MDA) to wait for the hardware trigger; ``False`` (Snap)
            to free-run on the camera's internal trigger.
        """
        for worker in self.workers:
            assert worker.conn is not None
            worker.conn.send(ArmCmd(n_images, external_trigger=external_trigger))
        self._wait_for_all(ArmedMsg, armed_timeout)

    def arm_live_all(self, armed_timeout: float = 10.0) -> None:
        """Arm every worker for unbounded, free-running acquisition.

        Live counterpart to :meth:`arm_all` -- no target frame count, since
        there's nothing to over-arm/under-arm against (no hardware trigger).
        Frames are then drained the same way, via :meth:`iter_frames`, until
        :meth:`stop_all` is called.

        Parameters
        ----------
        armed_timeout : float
            Seconds to wait for every worker's
            :class:`~pymmcore_gui.asi_z_stack.worker_messages.ArmedMsg`.
        """
        for worker in self.workers:
            assert worker.conn is not None
            worker.conn.send(ArmLiveCmd())
        self._wait_for_all(ArmedMsg, armed_timeout)

    def _worker_for(self, camera_label: str) -> CameraWorkerHandle:
        for worker in self.workers:
            if worker.camera_label == camera_label:
                return worker
        raise KeyError(f"No camera worker for {camera_label!r}")

    def set_roi(
        self, camera_label: str, x: int, y: int, w: int, h: int, timeout: float = 5.0
    ) -> tuple[int, int, int, int]:
        """Set *camera_label*'s ROI via its worker. Only valid while idle.

        The pool itself does not enforce "must be idle" -- that policy lives
        on :class:`~pymmcore_gui.asi_z_stack.camera_worker_service.
        CameraWorkerService`, which is the only intended caller.
        """
        return self._worker_for(camera_label).set_roi(x, y, w, h, timeout)

    def get_roi(
        self, camera_label: str, timeout: float = 5.0
    ) -> tuple[int, int, int, int]:
        """Read *camera_label*'s current ROI via its worker. Only valid while idle."""
        return self._worker_for(camera_label).get_roi(timeout)

    def set_properties(
        self,
        camera_label: str,
        values: tuple[tuple[str, str], ...],
        timeout: float = 5.0,
    ) -> None:
        """Set *camera_label*'s properties via its worker. Only valid while idle."""
        self._worker_for(camera_label).set_properties(values, timeout)

    def iter_frames(
        self, stall_timeout_s: float = 5.0
    ) -> Iterator[tuple[str, int, np.ndarray, dict[str, Any], int]]:
        """Yield frames from every armed worker as they arrive, interleaved.

        Parameters
        ----------
        stall_timeout_s : float
            Raise :class:`TimeoutError` if *no* worker sends anything for
            this many seconds (a safety net behind each worker's own
            per-camera stall detection).

        Yields
        ------
        tuple[str, int, np.ndarray, dict[str, Any], int]
            ``(camera_label, slice_idx, frame, camera_metadata,
            images_remaining)`` for each frame, in arrival order across all
            workers.
        """
        active = {w.camera_label: w for w in self.workers}
        frames_seen = dict.fromkeys(active, 0)
        # Measured from the last *frame*, not the last message: a stalled
        # worker keeps sending StalledMsg, which used to reset this guard
        # forever -- the MDA hung with no error and could not be cancelled.
        last_frame_time = time.monotonic()
        while active:
            conn_map: dict[Any, CameraWorkerHandle] = {}
            sentinel_map: dict[Any, CameraWorkerHandle] = {}
            for worker in active.values():
                assert worker.conn is not None and worker.process is not None
                conn_map[worker.conn] = worker
                sentinel_map[worker.process.sentinel] = worker

            remaining_s = stall_timeout_s - (time.monotonic() - last_frame_time)
            ready = (
                mp_wait([*conn_map, *sentinel_map], timeout=remaining_s)
                if remaining_s > 0
                else []
            )
            if not ready:
                raise TimeoutError(
                    f"no frame from any camera worker for {stall_timeout_s:.1f}s "
                    f"(frames received so far: {frames_seen}; still waiting on "
                    f"{sorted(active)}) -- the cameras stopped being triggered"
                )

            for obj in ready:
                if obj in sentinel_map:
                    dead = sentinel_map[obj]
                    raise WorkerDiedError(dead.camera_label, dead.process.exitcode)  # type: ignore[union-attr]
                worker = conn_map[obj]
                msg = worker.conn.recv()  # type: ignore[union-attr]
                if isinstance(msg, FrameMsg):
                    img = worker.read_frame(msg.slot_index, msg.nbytes)
                    worker.conn.send(SlotFreeCmd(msg.slot_index))  # type: ignore[union-attr]
                    frames_seen[worker.camera_label] += 1
                    last_frame_time = time.monotonic()
                    yield (
                        worker.camera_label,
                        msg.slice_idx,
                        img,
                        msg.camera_metadata,
                        msg.images_remaining,
                    )
                elif isinstance(msg, StoppedMsg):
                    active.pop(worker.camera_label, None)
                elif isinstance(msg, StalledMsg):
                    pass
                elif isinstance(msg, ErrorMsg):
                    active.pop(worker.camera_label, None)
                    raise RuntimeError(
                        f"{msg.camera_label} worker error: {msg.message}\n"
                        f"{msg.traceback_text}"
                    )

    def stop_all(self) -> None:
        """Send every worker a ``StopCmd`` (best-effort, non-blocking)."""
        for worker in self.workers:
            if worker.conn is not None and not worker.conn.closed:
                try:
                    worker.conn.send(StopCmd())
                except OSError:
                    pass

    def stop_and_drain(self, timeout: float = 10.0) -> None:
        """Stop every worker and discard everything it sent before acknowledging.

        Leaves each pipe empty and each worker idle, so the next command's
        reply isn't preceded by leftovers from this cycle. A plain
        :meth:`stop_all` doesn't: breaking out of :meth:`iter_frames` early
        (Snap takes one frame of an over-armed sequence; an MDA cancel) left
        the rest of that cycle's ``FrameMsg``/``StoppedMsg`` queued, and the
        persistent pool's *next* :meth:`iter_frames` read them as its own --
        stale frames first, then a stale ``StoppedMsg`` ending it early.

        Best-effort: a dead worker is skipped and a timeout is logged, not
        raised, since this runs in ``finally`` blocks.

        Parameters
        ----------
        timeout : float
            Total seconds to wait for every worker's acknowledgement.
        """
        self._stop_token += 1
        token = self._stop_token
        pending: dict[str, CameraWorkerHandle] = {}
        for worker in self.workers:
            if worker.conn is None or worker.conn.closed:
                continue
            try:
                worker.conn.send(StopCmd(token))
            except OSError:
                continue
            pending[worker.camera_label] = worker

        deadline = time.monotonic() + timeout
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                _pymmcore_plus_logger.warning(
                    f"Camera workers {sorted(pending)} did not acknowledge stop "
                    f"within {timeout:.1f}s."
                )
                return
            conn_map: dict[Any, CameraWorkerHandle] = {
                w.conn: w for w in pending.values()
            }
            sentinel_map: dict[Any, CameraWorkerHandle] = {
                w.process.sentinel: w for w in pending.values() if w.process
            }
            for obj in mp_wait([*conn_map, *sentinel_map], timeout=remaining):
                if obj in sentinel_map:
                    pending.pop(sentinel_map[obj].camera_label, None)
                    continue
                worker = conn_map[obj]
                try:
                    msg = worker.conn.recv()  # type: ignore[union-attr]
                except (EOFError, OSError):
                    pending.pop(worker.camera_label, None)
                    continue
                if isinstance(msg, FrameMsg):
                    # Free the slot so a worker blocked waiting for one can
                    # still see the StopCmd.
                    with contextlib.suppress(OSError):
                        worker.conn.send(SlotFreeCmd(msg.slot_index))  # type: ignore[union-attr]
                elif isinstance(msg, StoppedMsg) and msg.token == token:
                    pending.pop(worker.camera_label, None)

    def shutdown_all(self, timeout: float = 10.0) -> None:
        """Send every worker a ``ShutdownCmd``, join, and release shared memory.

        Parameters
        ----------
        timeout : float
            Total seconds to wait, across all workers, for clean process exit
            before escalating to ``terminate()``.
        """
        for worker in self.workers:
            if worker.conn is not None and not worker.conn.closed:
                try:
                    worker.conn.send(ShutdownCmd())
                except OSError:
                    pass

        deadline = time.monotonic() + timeout
        for worker in self.workers:
            if worker.process is not None:
                remaining = max(0.0, deadline - time.monotonic())
                worker.process.join(timeout=remaining)
                if worker.process.is_alive():
                    worker.process.terminate()
                    worker.process.join(timeout=5.0)
            if worker.shm is not None:
                try:
                    worker.shm.close()
                except FileNotFoundError:
                    pass
                try:
                    worker.shm.unlink()
                except FileNotFoundError:
                    pass

    def _wait_for_all(self, expected: type, timeout: float) -> None:
        """Block until every worker has sent an *expected*-type message.

        Parameters
        ----------
        expected : type
            The worker-to-main message type each worker must send.
        timeout : float
            Total seconds to wait across all workers.
        """
        pending = {w.camera_label: w for w in self.workers}
        deadline = time.monotonic() + timeout
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"timed out waiting for {expected.__name__} from {sorted(pending)}"
                )
            conn_map: dict[Any, CameraWorkerHandle] = {}
            sentinel_map: dict[Any, CameraWorkerHandle] = {}
            for worker in pending.values():
                assert worker.conn is not None and worker.process is not None
                conn_map[worker.conn] = worker
                sentinel_map[worker.process.sentinel] = worker

            ready = mp_wait([*conn_map, *sentinel_map], timeout=remaining)
            for obj in ready:
                if obj in sentinel_map:
                    dead = sentinel_map[obj]
                    raise WorkerDiedError(dead.camera_label, dead.process.exitcode)  # type: ignore[union-attr]
                worker = conn_map[obj]
                msg = worker.conn.recv()  # type: ignore[union-attr]
                if isinstance(msg, ErrorMsg):
                    raise RuntimeError(
                        f"{msg.camera_label} worker error: {msg.message}\n"
                        f"{msg.traceback_text}"
                    )
                if isinstance(msg, expected):
                    pending.pop(worker.camera_label, None)
