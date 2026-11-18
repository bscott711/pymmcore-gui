"""Global look-and-feel standards for the GUI.

Colours here are expressed as Qt palette references (``palette(...)``) rather than
hard-coded hex values, so they follow the active (light or dark) palette.  Anything
that needs to restyle a third-party widget tree should add a snippet here and apply
it once from the main window, instead of scattering ``setStyleSheet`` calls.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_gui._qt.QtCore import QTimer
from pymmcore_gui._qt.QtWidgets import QWidget

if TYPE_CHECKING:
    from pymmcore_gui._qt.QtAds import CDockManager, CDockWidget

#: Text colour for all standard text (white on the dark palette, black on light).
TEXT = "palette(window-text)"

# The docking library's built-in stylesheet colours *unselected* tab labels with
# ``palette(dark)``, which is near-black on a dark palette and so unreadable.
# Every tab label, selected or not, uses the standard text colour instead.
DOCK_TAB_STYLE = f"""
ads--CDockWidgetTab QLabel,
ads--CDockWidgetTab[activeTab="true"] QLabel {{
    color: {TEXT};
}}
"""


def _repolish(widget: QWidget) -> None:
    """Re-evaluate the stylesheet for a widget and all of its children."""
    for w in (widget, *widget.findChildren(QWidget)):
        w.style().unpolish(w)
        w.style().polish(w)


def apply_dock_style(dock_manager: CDockManager) -> None:
    """Append the GUI's dock-tab overrides to a ``CDockManager``'s stylesheet.

    The docking library's own stylesheet is preserved; ours is appended so it takes
    precedence.  Tabs are first styled before they reach the dock manager, so Qt
    would keep applying the library's rules to them until their first state change;
    each newly added dock's tab is therefore re-polished once.

    Parameters
    ----------
    dock_manager : CDockManager
        The dock manager whose stylesheet is extended.
    """
    dock_manager.setStyleSheet(dock_manager.styleSheet() + DOCK_TAB_STYLE)

    def _on_dock_added(dock: CDockWidget) -> None:
        QTimer.singleShot(0, lambda: _repolish(dock.tabWidget()))

    dock_manager.dockWidgetAdded.connect(_on_dock_added)
