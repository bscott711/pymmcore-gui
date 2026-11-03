from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_gui._design import apply_dock_style
from pymmcore_gui._qt.QtAds import CDockManager, CDockWidget, DockWidgetArea
from pymmcore_gui._qt.QtGui import QColor, QPalette
from pymmcore_gui._qt.QtWidgets import QApplication, QLabel, QMainWindow

if TYPE_CHECKING:
    from pytestqt.qtbot import QtBot


def test_unselected_dock_tab_text_uses_window_text(qtbot: QtBot) -> None:
    """Unselected tab labels must not use the library's near-black palette(dark)."""
    qapp = QApplication.instance()
    assert isinstance(qapp, QApplication)
    old_palette = qapp.palette()
    try:
        dark = qapp.palette()
        dark.setColor(QPalette.ColorRole.WindowText, QColor("#ffffff"))
        dark.setColor(QPalette.ColorRole.Dark, QColor("#141414"))
        qapp.setPalette(dark)

        win = QMainWindow()
        qtbot.addWidget(win)
        dm = CDockManager(win)
        apply_dock_style(dm)
        first = CDockWidget("First", win)
        first.setWidget(QLabel("a"))
        second = CDockWidget("Second", win)
        second.setWidget(QLabel("b"))
        area = dm.addDockWidget(DockWidgetArea.CenterDockWidgetArea, first)
        dm.addDockWidgetTabToArea(second, area)
        first.setAsCurrentTab()
        win.show()
        qtbot.wait(50)

        assert not second.tabWidget().isActiveTab()
        for dock in (first, second):
            (label,) = dock.tabWidget().findChildren(QLabel, "dockWidgetTabLabel")
            assert label.palette().color(QPalette.ColorRole.WindowText).name() == (
                "#ffffff"
            )
    finally:
        qapp.setPalette(old_palette)
