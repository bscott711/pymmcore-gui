from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from PyQt6.QtGui import QAction, QActionGroup
from PyQt6.QtWidgets import QLabel, QToolBar

from pymmcore_gui.epi._mode import ScopeMode

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._main_window import MicroManagerGUI
    from pymmcore_gui.epi._mode import ModeController

logger = logging.getLogger("pymmcore_gui")


class ModeToolBar(QToolBar):
    """Exclusive OPM / EPI toggle backed by the window's `ModeController`.

    A refused switch reverts the checked button and shows the reason.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core (unused; kept for the toolbar-factory signature).
    parent : MicroManagerGUI
        The main window, which owns the mode controller and notifications.
    """

    def __init__(self, mmc: CMMCorePlus, parent: MicroManagerGUI) -> None:
        super().__init__("Mode", parent)
        self._controller: ModeController = parent.mode_controller
        self._window = parent

        self._actions: dict[ScopeMode, QAction] = {}
        self._group = QActionGroup(self)
        self._group.setExclusive(True)
        for mode in ScopeMode:
            action = QAction(mode.value, self)
            action.setCheckable(True)
            action.setToolTip(f"Switch to {mode.value} mode")
            action.setData(mode)
            self._group.addAction(action)
            self.addAction(action)
            self._actions[mode] = action
        self._label = QLabel()
        self.addWidget(self._label)

        self._group.triggered.connect(self._on_triggered)
        self._controller.modeChanged.connect(self._sync)
        self._sync(self._controller.mode)

    def _sync(self, mode: object = None) -> None:
        """Show the controller's current mode."""
        current = self._controller.mode
        self._actions[current].setChecked(True)
        self._label.setText(f" Mode: {current.value} ")

    def _on_triggered(self, action: QAction) -> None:
        mode = action.data()
        if not isinstance(mode, ScopeMode):
            return
        reason = self._controller.request_mode(mode)
        self._sync()  # revert the checked button on a refusal
        if reason is not None:
            logger.warning("Mode switch to %s refused: %s", mode.value, reason)
            self._window.nm.show_warning_message(reason)
