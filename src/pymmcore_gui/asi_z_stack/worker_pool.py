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

import time
from dataclasses import replace
from multiprocessing import get_context
from multiprocessing.connection import wait as mp_wait
from multiprocessing.shared_memory import SharedMemory
from typing import TYPE_CHECKING, Any

import numpy as np

from .camera_worker import CameraWorkerConfig, run_camera_worker
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
    from collections.abc import Iterator
    from multiprocessing.context import SpawnContext
    from multiprocessing.process import BaseProcess

    # multiprocessing.Pipe() returns platform-specific connection types
    # (Connection on POSIX, PipeConnection on Windows) with no common public
    # base type in typeshed -- Any is the pragmatic cross-platform choice.
    Connection = Any


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
        )
        parent_conn, child_conn = ctx.Pipe(duplex=True)
        self.conn = parent_conn
        process = ctx.Process(
            target=run_camera_worker,
            args=(self.config, child_conn),
            daemon=False,
        )
        process.start()
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


class CameraWorkerPool:
    """Orchestrates a set of :class:`CameraWorkerHandle` for one MDA run."""

    def __init__(self, workers: list[CameraWorkerHandle]) -> None:
        self.workers = workers
        self._ctx = get_context("spawn")

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

    def arm_all(self, n_images: int, armed_timeout: float = 10.0) -> None:
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
        """
        for worker in self.workers:
            assert worker.conn is not None
            worker.conn.send(ArmCmd(n_images))
        self._wait_for_all(ArmedMsg, armed_timeout)

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
        while active:
            conn_map: dict[Any, CameraWorkerHandle] = {}
            sentinel_map: dict[Any, CameraWorkerHandle] = {}
            for worker in active.values():
                assert worker.conn is not None and worker.process is not None
                conn_map[worker.conn] = worker
                sentinel_map[worker.process.sentinel] = worker

            ready = mp_wait([*conn_map, *sentinel_map], timeout=stall_timeout_s)
            if not ready:
                raise TimeoutError(
                    f"no message from any camera worker for {stall_timeout_s:.1f}s "
                    f"(still waiting on {sorted(active)})"
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
