"""A persistent, size-rotated log file for the GUI's own loggers.

``--debug`` only attaches a stderr handler to ``pymmcore_gui.asi_z_stack`` and the
launcher's ``info.log`` used to be truncated at each launch, so most of what the app
logged was lost. :func:`install_gui_file_log` attaches one rotating file handler to
the ``pymmcore_gui`` logger regardless of ``--debug``.
"""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

LOGGER_NAME = "pymmcore_gui"
LOG_FILE_NAME = "pymmcore-gui.log"
_FORMAT = "%(asctime)s pid%(process)d [%(levelname)s] %(name)s: %(message)s"
_FILE_BYTES = 10 * 1024 * 1024
_FILE_BACKUPS = 10

_installed: tuple[RotatingFileHandler, Path] | None = None


def _default_directory() -> Path:
    """Beside the pymmcore-plus log file, else ``USER_DATA_DIR / "logs"``."""
    try:
        from pymmcore_plus._logger import current_logfile

        logfile = current_logfile(logging.getLogger("pymmcore-plus"))
        if logfile:
            return Path(logfile).parent
    except Exception:  # pragma: no cover - best effort
        pass
    from pymmcore_gui._settings import USER_DATA_DIR

    return USER_DATA_DIR / "logs"


def install_gui_file_log(directory: Path | None = None) -> Path | None:
    """Log the ``pymmcore_gui`` hierarchy at INFO into ``pymmcore-gui.log``.

    Idempotent: a second call returns the same path without adding a handler.
    Returns the log path, or None if the file could not be set up. Never raises.
    """
    global _installed
    logger = logging.getLogger(LOGGER_NAME)
    if _installed is not None and _installed[0] in logger.handlers:
        return _installed[1]
    try:
        directory = Path(directory) if directory is not None else _default_directory()
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / LOG_FILE_NAME
        handler = RotatingFileHandler(
            path,
            maxBytes=_FILE_BYTES,
            backupCount=_FILE_BACKUPS,
            encoding="utf-8",
            delay=True,
        )
    except OSError:
        return None
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter(_FORMAT))
    logger.addHandler(handler)
    # Only raise the threshold; never silence a more verbose (e.g. --debug) level.
    if logger.getEffectiveLevel() > logging.INFO:
        logger.setLevel(logging.INFO)
    _installed = (handler, path)
    return path
