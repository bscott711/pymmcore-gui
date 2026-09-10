"""Run one MDA save writer's disk I/O on its own background thread.

The stock pymmcore-plus save path funnels *every* ``output=`` handler's
``frameReady`` through a single :class:`~pymmcore_plus.mda._thread_relay.MDARelayThread`
that also does the compression and disk writes -- serially, for every camera. For
a dual-camera SPIM z-stack that one thread can't keep up: its queue is unbounded
and its ``run()`` loop has no exception handling, so the backlog grows until the
thread dies (typically ``MemoryError`` inside ``numcodecs``) and every subsequent
frame is silently dropped -- the pre-allocated OME-Zarr just keeps its fill-value
zeros. The acquisition and the (separately-connected, memory-bounded) live viewer
never notice.

:class:`AsyncWriter` wraps a single underlying writer so that:

* each writer drains on its *own* thread -- the cameras write in parallel, and a
  slow write for one never stalls another's frame delivery;
* the in-RAM backlog is **bounded** by a byte budget -- exceeding it drops frames
  with a loud one-time alarm instead of growing until the process is killed;
* an exception in the writer is caught, reported to the user, and the run is
  marked incomplete -- never a silent truncation.

It presents the same ``sequenceStarted`` / ``frameReady`` / ``sequenceFinished``
interface as the writer it wraps, so it drops into the existing handler
composition (:class:`~pymmcore_gui._multi_camera_handler.MultiCameraHandler`,
:class:`~pymmcore_gui._spectral_channel_handler.SpectralChannelHandler`, or a bare
single-camera writer) transparently.

Frames are queued by reference, not copied: callers must not hand
:class:`AsyncWriter` a view into a buffer they will overwrite (every engine in
this app yields owned per-frame arrays, so this holds today).
"""

from __future__ import annotations

import queue
import threading
from inspect import signature
from typing import TYPE_CHECKING, Any

from pymmcore_gui._exceptions import report_background_exception

if TYPE_CHECKING:
    from collections.abc import Callable

    import numpy as np
    import useq
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1

# Default per-writer RAM budget for the not-yet-written backlog. Generous: a
# healthy run keeps the backlog near zero, so this only ever bites when the
# disk genuinely can't keep up -- at which point dropping (with an alarm) beats
# an eventual out-of-memory kill.
DEFAULT_BACKLOG_BUDGET_BYTES = 32 * 1024**3

_STOP = object()


class AsyncWriter:
    """Wrap a save writer so its ``frameReady`` runs on a dedicated thread."""

    def __init__(
        self,
        writer: Any,
        *,
        name: str,
        backlog_budget_bytes: int = DEFAULT_BACKLOG_BUDGET_BYTES,
        reporter: Callable[[BaseException, str], None] | None = None,
    ) -> None:
        self._writer = writer
        self._name = name
        self._budget = max(1, backlog_budget_bytes)
        self._reporter = reporter or report_background_exception

        self._queue: queue.SimpleQueue[Any] = queue.SimpleQueue()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._pending_bytes = 0
        self._backlog_alarmed = False
        self._dropped = 0
        self._failed: BaseException | None = None
        self._finished = False

    # ------------------------------------------------------------------
    # incomplete-save status
    # ------------------------------------------------------------------

    @property
    def incomplete(self) -> bool:
        """True if any frame failed to write or was dropped for backlog."""
        return self._failed is not None or self._dropped > 0

    @property
    def status_detail(self) -> str:
        """Human-readable description of what went wrong, or ``""``."""
        parts = []
        if self._failed is not None:
            parts.append(f"write error ({self._failed!r})")
        if self._dropped:
            parts.append(f"{self._dropped} frame(s) dropped (disk too slow)")
        return "; ".join(parts)

    # ------------------------------------------------------------------
    # MDA signal interface (matches the wrapped writer)
    # ------------------------------------------------------------------

    def sequenceStarted(
        self, seq: useq.MDASequence, meta: SummaryMetaV1 | dict | None = None
    ) -> None:
        # Run synchronously: this is what creates the on-disk array, and it must
        # exist before any frame is enqueued. Cheap (metadata only).
        method = getattr(self._writer, "sequenceStarted", None)
        if method is not None:
            try:
                n_params = len(signature(method).parameters)
            except (TypeError, ValueError):  # pragma: no cover - builtins
                n_params = 2
            if n_params >= 2:
                method(seq, meta)
            else:
                method(seq)
        self._ensure_thread()

    def frameReady(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        self._ensure_thread()
        if self._finished:  # pragma: no cover - engines don't emit after finish
            return
        nbytes = int(getattr(frame, "nbytes", 0))
        with self._lock:
            if self._pending_bytes + nbytes > self._budget and self._pending_bytes > 0:
                self._dropped += 1
                alarm = not self._backlog_alarmed
                self._backlog_alarmed = True
            else:
                self._pending_bytes += nbytes
                alarm = False
                self._queue.put((frame, event, meta, nbytes))
        if alarm:
            self._reporter(
                RuntimeError(
                    f"MDA save {self._name!r}: disk can't keep up, "
                    f"backlog exceeded {self._budget // 1024**2} MiB -- "
                    "frames are being dropped and the saved file will be incomplete."
                ),
                "MDA save",
            )

    def sequenceFinished(self, seq: useq.MDASequence) -> None:
        self._finished = True
        if self._thread is not None:
            self._queue.put(_STOP)
            self._thread.join()
        method = getattr(self._writer, "sequenceFinished", None)
        if method is not None:
            try:
                method(seq)
            except Exception as exc:
                self._note_failure(exc)
        if self.incomplete:
            self._reporter(
                RuntimeError(
                    f"MDA save {self._name!r} finished incomplete: {self.status_detail}"
                ),
                "MDA save",
            )

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------

    def _ensure_thread(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._drain, name=f"AsyncWriter[{self._name}]", daemon=True
        )
        self._thread.start()

    def _drain(self) -> None:
        while True:
            item = self._queue.get()
            if item is _STOP:
                return
            frame, event, meta, nbytes = item
            try:
                self._writer.frameReady(frame, event, meta)
            except Exception as exc:
                # Report only the first failure (a persistent fault -- disk
                # full, an index mismatch -- would otherwise spam); later
                # frames are still attempted in case the fault was transient.
                self._note_failure(exc)
            finally:
                with self._lock:
                    self._pending_bytes -= nbytes

    def _note_failure(self, exc: BaseException) -> None:
        with self._lock:
            first = self._failed is None
            if first:
                self._failed = exc
        if first:
            self._reporter(exc, f"MDA save {self._name!r}")
