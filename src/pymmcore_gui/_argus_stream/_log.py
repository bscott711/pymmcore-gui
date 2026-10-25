"""The Argus stream's own log: a file on this PC, and lines for Argus.

Until 2026-10-24 the stream loggers had no handler: only WARNING and above
reached stderr (``info.log`` beside ``launch_gui.ps1``), and every "link up",
per-run byte count and ssh message was lost. A post-mortem of ten runs that
streamed nothing then needed the rig itself, which Argus can't reach.

:func:`install` gives the ``pymmcore_gui._argus_stream`` loggers (the sender
and the tunnels) an INFO-level rotating file under the app's data dir, and
:data:`BUFFER` keeps the newest lines in memory so each run's sender can ship
them to Argus as ``CLIENT_LOG`` (see :class:`~._session._RunWorker`). Lines
logged while Argus was unreachable go with the next run that gets through.
"""

from __future__ import annotations

import logging
import threading
from collections import deque
from logging.handlers import RotatingFileHandler
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path

LOGGER_NAME = "pymmcore_gui._argus_stream"
LOG_FILE_NAME = "argus-stream.log"
_FORMAT = "%(asctime)s %(levelname)s %(name)s: %(message)s"
_FILE_BYTES = 5 * 1024 * 1024
_FILE_BACKUPS = 5
_BUFFER_LINES = 5000
# Per CLIENT_LOG message: keeps one message small next to the volumes.
MAX_LINES_PER_MESSAGE = 1000


class StreamLogBuffer(logging.Handler):
    """The newest formatted stream log lines, numbered, for ``CLIENT_LOG``.

    Thread-safe: records arrive from the sender, tunnel and GUI threads.
    """

    def __init__(self, maxlen: int = _BUFFER_LINES) -> None:
        super().__init__(level=logging.INFO)
        self.setFormatter(logging.Formatter(_FORMAT))
        self._lines: deque[tuple[int, str]] = deque(maxlen=maxlen)
        self._seq = 0
        self._sent_through = 0
        self._buf_lock = threading.Lock()
        # Held by a sender from unsent() through mark_sent(), so two runs'
        # senders don't ship the same lines.
        self.sending = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            line = self.format(record)
        except Exception:  # pragma: no cover - logging must never raise
            return
        with self._buf_lock:
            self._seq += 1
            self._lines.append((self._seq, line))

    def unsent(self, limit: int = MAX_LINES_PER_MESSAGE) -> tuple[int, list[str]]:
        """Return the oldest unsent lines (at most ``limit``) and the last one's number.

        ``(0, [])`` if there are none.
        """
        with self._buf_lock:
            new = [(s, ln) for s, ln in self._lines if s > self._sent_through]
        new = new[:limit]
        return (new[-1][0], [ln for _, ln in new]) if new else (0, [])

    def mark_sent(self, through: int) -> None:
        with self._buf_lock:
            self._sent_through = max(self._sent_through, through)


BUFFER = StreamLogBuffer()
_install_lock = threading.Lock()
_file_handler: RotatingFileHandler | None = None


def install(log_dir: Path | None = None) -> None:
    """Route the stream loggers at INFO into :data:`BUFFER` (and a log file).

    With ``log_dir``, also into a rotating ``argus-stream.log`` there.
    Idempotent.
    """
    global _file_handler
    logger = logging.getLogger(LOGGER_NAME)
    with _install_lock:
        if logger.level == logging.NOTSET or logger.level > logging.INFO:
            logger.setLevel(logging.INFO)
        if BUFFER not in logger.handlers:
            logger.addHandler(BUFFER)
        if log_dir is None or _file_handler is not None:
            return
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(
                log_dir / LOG_FILE_NAME,
                maxBytes=_FILE_BYTES,
                backupCount=_FILE_BACKUPS,
                encoding="utf-8",
                delay=True,
            )
        except OSError:
            logger.exception("Could not open the Argus stream log in %s", log_dir)
            return
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter(_FORMAT))
        logger.addHandler(handler)
        _file_handler = handler
