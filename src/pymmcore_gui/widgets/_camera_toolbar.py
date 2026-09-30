from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt6.QtWidgets import QToolBar

from pymmcore_gui.actions import CoreAction
from pymmcore_gui.widgets._exposure_widget import ExposureWidget

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._main_window import MicroManagerGUI


class CameraToolBar(QToolBar):
    """Snap, Live, and the Snap/Live exposure.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    parent : MicroManagerGUI
        The main window, which owns the Snap/Live actions.
    """

    def __init__(self, mmc: CMMCorePlus, parent: MicroManagerGUI) -> None:
        super().__init__("Camera Actions", parent)
        self.addAction(parent.get_action(CoreAction.SNAP))
        self.addAction(parent.get_action(CoreAction.TOGGLE_LIVE))
        self.exposure = ExposureWidget(mmc, self)
        self.addWidget(self.exposure)
