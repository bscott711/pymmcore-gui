"""Surface exceptions caught on background threads to the user.

The app installs a custom ``sys.excepthook`` (see :mod:`pymmcore_gui._app`) that
turns unhandled exceptions into an error notification, but that only covers the
main thread. Worker threads that catch their own exceptions (to stay alive) --
or, with :func:`install_threading_excepthook`, ones that don't -- route here so a
background failure is never silent.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def report_background_exception(exc: BaseException, context: str = "") -> None:
    """Log *exc* and raise it to the GUI as an error notification.

    Safe to call from any thread: the notification is delivered via the
    ``MMQApplication.exceptionRaised`` Qt signal, which Qt auto-queues onto the
    GUI thread. Never raises.

    Parameters
    ----------
    exc : BaseException
        The exception that was caught.
    context : str
        Optional human-readable prefix (e.g. ``"MDA save"``) shown before the
        exception message in the notification.
    """
    message = f"{context}: {exc}" if context else (str(exc) or repr(exc))
    logger.error("background exception (%s)", context or "?", exc_info=exc)
    try:
        from pymmcore_gui import _app
        from pymmcore_gui._qt.QtWidgets import QApplication

        # Wrap so the notification carries a meaningful message even for
        # exception types with an empty ``str()`` (e.g. MemoryError), while
        # keeping the original as ``__cause__`` for the traceback view.
        reported: BaseException = exc
        if message != str(exc):
            reported = RuntimeError(message)
            reported.__cause__ = exc

        _app.EXCEPTION_LOG.append((type(reported), reported, reported.__traceback__))
        app = QApplication.instance()
        if app is not None and (sig := getattr(app, "exceptionRaised", None)):
            sig.emit(reported)
    except Exception:  # pragma: no cover - reporting must never itself raise
        logger.exception("failed to surface background exception")


def install_threading_excepthook() -> None:
    """Route otherwise-unhandled worker-thread exceptions through the app hook.

    Mirrors :func:`pymmcore_gui._app._install_excepthook` for ``threading`` --
    without this, a thread that dies on an unhandled exception only prints to
    stderr (invisible when the app is launched from a shortcut).
    """
    import threading

    if getattr(threading, "_pmm_excepthook_installed", False):
        return

    def _hook(args: threading.ExceptHookArgs) -> None:
        exc = args.exc_value
        if exc is None or isinstance(exc, SystemExit):
            return
        name = args.thread.name if args.thread is not None else "?"
        report_background_exception(exc, context=f"thread {name}")

    threading.excepthook = _hook
    threading._pmm_excepthook_installed = True  # type: ignore[attr-defined]
