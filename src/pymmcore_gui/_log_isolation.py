from __future__ import annotations

import multiprocessing
import os


def isolate_process_logfile() -> None:
    """Point this process's pymmcore-plus logger at its own logfile.

    ``pymmcore_plus`` configures a single shared ``RotatingFileHandler`` at
    import time (see ``pymmcore_plus._logger``). Any *other* process that
    also imports pymmcore_plus without overriding ``PYMM_LOG_FILE`` --
    another pymmcore-gui instance, a leftover process, a Jupyter kernel --
    opens its own handle on that exact file. On Windows, when this
    process's handler tries to rotate on rollover, the rename fails with a
    ``PermissionError`` if any of those other handles are still open.
    Giving this process its own PID-suffixed logfile removes it from that
    contention entirely (mirrors the same fix applied to camera worker
    subprocesses in ``asi_z_stack.worker_pool``).

    This must run before anything else in the package logs: several modules
    (e.g. ``asi_z_stack.asi_controller``) create the global ``CMMCorePlus``
    at import time, and once the shared file is over its size limit every
    one of those log calls attempts (and fails) a rotation. It is therefore
    called from ``pymmcore_gui/__init__.py`` ahead of the other imports.

    Child processes are left alone: the parent already gives each camera
    worker its own file via ``PYMM_LOG_FILE``.
    """
    if multiprocessing.parent_process() is not None:
        return

    from pymmcore_plus import configure_logging
    from pymmcore_plus._logger import current_logfile
    from pymmcore_plus._logger import logger as _pymmcore_plus_logger

    current = current_logfile(_pymmcore_plus_logger)
    if current is None:
        return  # file logging disabled (e.g. PYMM_LOG_FILE=0, or under pytest)
    # Detach *and* close every existing handler ourselves first: configure_logging()
    # removes old handlers by iterating `logger.handlers` while calling
    # `removeHandler()` on that same live list, which skips every other handler
    # once there's more than one -- leaving a stale, already-closed handler
    # attached that would error (or silently reopen the shared file) on the
    # next log call. Clearing the list ourselves first makes that loop a no-op.
    for handler in list(_pymmcore_plus_logger.handlers):
        _pymmcore_plus_logger.removeHandler(handler)
        handler.close()
    unique = current.with_name(f"{current.stem}-pid{os.getpid()}{current.suffix}")
    configure_logging(file=unique)
