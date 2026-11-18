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

import enum
import json
import logging
import os
import queue
import threading
from datetime import datetime
from inspect import signature
from pathlib import Path
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

logger = logging.getLogger(__name__)

# Sidecar left in a store that received no frames (see ``_write_zero_frame_marker``).
RUN_STATUS_NAME = ".run_status.json"


class ZeroFrameSaveError(RuntimeError):
    """An MDA run ended without a single frame reaching a channel's store.

    ``persistent_notice`` asks the GUI to keep the notification on screen until the
    user dismisses it (the store looks like data but holds none).
    """

    persistent_notice = True


class AsyncWriter:
    """Wrap a save writer so its ``frameReady`` runs on a dedicated thread."""

    def __init__(
        self,
        writer: Any,
        *,
        name: str,
        backlog_budget_bytes: int = DEFAULT_BACKLOG_BUDGET_BYTES,
        reporter: Callable[[BaseException, str], None] | None = None,
        finish_reason: Callable[[], object] | None = None,
    ) -> None:
        self._writer = writer
        self._name = name
        self._budget = max(1, backlog_budget_bytes)
        self._reporter = reporter or report_background_exception
        self._finish_reason = finish_reason

        self._queue: queue.SimpleQueue[Any] = queue.SimpleQueue()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._pending_bytes = 0
        self._backlog_alarmed = False
        self._dropped = 0
        self._received = 0
        self._written = 0
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
    def frames_written(self) -> int:
        """Number of frames the wrapped writer accepted without error."""
        return self._written

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
        self._received += 1
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
        # Snapshot first: a new run could reset the runner's status while we join.
        finish_reason = self._read_finish_reason()
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
        try:
            self._report_final_state(finish_reason)
        except Exception:
            logger.exception("MDA save %r: final-state report failed", self._name)

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
                self._written += 1
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

    def _read_finish_reason(self) -> str:
        """Return the run's finish reason as a string, ``"unknown"`` if unavailable."""
        if self._finish_reason is None:
            return "unknown"
        try:
            value = self._finish_reason()
        except Exception:
            return "unknown"
        if value is None:
            return "unknown"
        return str(value.value if isinstance(value, enum.Enum) else value)

    def _store_dir(self) -> Path | None:
        """Directory of the wrapped writer's OME-Zarr store, if it has one."""
        try:
            path = getattr(self._writer.group.store, "path", None)
        except Exception:
            return None
        if not path:
            return None
        store = Path(path)
        if store.is_dir() and store.name.endswith(".zarr"):
            return store
        return None

    def _report_final_state(self, finish_reason: str) -> None:
        """Log the per-store tally; flag and mark a store that got no frames."""
        store = self._store_dir()
        label = store.name if store is not None else self._name
        logger.info(
            "MDA save %r: received %d, written %d, dropped %d frame(s); "
            "run finished: %s",
            label,
            self._received,
            self._written,
            self._dropped,
            finish_reason,
        )
        if self._written:
            return
        err = ZeroFrameSaveError(
            f"MDA save {label!r} received 0 frames (run finished: "
            f"{finish_reason}); the store holds no image data"
        )
        if finish_reason == "canceled":
            # a cancelled run is expected to have no frames: report, but no sticky toast
            err.persistent_notice = False
        self._reporter(err, "MDA save")
        if store is not None:
            self._write_zero_frame_marker(store, finish_reason)

    def _write_zero_frame_marker(self, store: Path, finish_reason: str) -> None:
        """Write ``<store>/.run_status.json`` once, atomically; never raise."""
        part = store / (RUN_STATUS_NAME + ".part")
        try:
            status = {
                "format": 1,
                "run_status": "zero_frames",
                "frames_written": self._written,
                "frames_received": self._received,
                "frames_dropped": self._dropped,
                "finish_reason": finish_reason,
                "store": store.name,
                "finished_at": datetime.now()
                .astimezone()
                .isoformat(timespec="seconds"),
                "writer_error": None if self._failed is None else repr(self._failed),
            }
            part.write_text(json.dumps(status, indent=2) + "\n")
            os.replace(part, store / RUN_STATUS_NAME)
        except Exception:
            logger.warning(
                "could not write %s in %s", RUN_STATUS_NAME, store, exc_info=True
            )
            try:
                part.unlink(missing_ok=True)
            except OSError:
                pass
