"""Discovery and loading of third-party plugins.

A plugin is any installed package that declares an entry point in the
``pymmcore_gui.plugins`` group. The entry point must resolve to a callable that
accepts the main window (a `MicroManagerGUI`)::

    [project.entry-points."pymmcore_gui.plugins"]
    my-plugin = "my_package.plugin:install"
"""

from __future__ import annotations

import importlib.metadata
import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pymmcore_gui._main_window import MicroManagerGUI

logger = logging.getLogger(__name__)

PLUGIN_GROUP = "pymmcore_gui.plugins"


def load_plugins(win: MicroManagerGUI) -> None:
    """Load every plugin registered under the ``pymmcore_gui.plugins`` group.

    Each entry point is loaded and called as ``fn(win)``. Plugins are isolated
    from each other and from the GUI: an exception while loading or calling one
    plugin is logged, shown as a warning notification, and does not stop the
    remaining plugins or the application.

    Parameters
    ----------
    win : MicroManagerGUI
        The main window, passed to each plugin callable.
    """
    try:
        eps = importlib.metadata.entry_points(group=PLUGIN_GROUP)
    except Exception:  # pragma: no cover
        logger.exception("Could not enumerate %s entry points", PLUGIN_GROUP)
        return

    for ep in eps:
        try:
            fn = ep.load()
            fn(win)
        except Exception as exc:
            logger.exception("Plugin %r failed to load", ep.name)
            _warn(win, f"Plugin {ep.name!r} failed to load: {exc}")
        else:
            logger.info("Loaded plugin %r", ep.name)


def _warn(win: MicroManagerGUI, message: str) -> None:
    try:
        win.nm.show_warning_message(message)
    except Exception:
        logger.debug("Could not show plugin warning notification", exc_info=True)
