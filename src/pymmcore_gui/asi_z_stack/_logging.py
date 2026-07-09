# src/pymmcore_gui/asi_z_stack/_logging.py
from __future__ import annotations

import logging
import sys

_LOGGER_NAME = "pymmcore_gui.asi_z_stack"


def configure_asi_logging(level: str = "INFO") -> None:
    """Attach a stderr handler to the ``asi_z_stack`` logger hierarchy.

    Every module in this package logs via ``logging.getLogger(__name__)``,
    so child loggers (e.g. ``pymmcore_gui.asi_z_stack.engine``) propagate up
    into this one configured parent -- no per-module handler wiring needed.
    Idempotent: calling this more than once (e.g. once per CLI invocation)
    won't attach duplicate handlers.
    """
    logger = logging.getLogger(_LOGGER_NAME)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    if not logger.hasHandlers():
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
        )
        logger.addHandler(handler)
