from __future__ import annotations

from contextlib import suppress
from itertools import chain

from pymmcore_plus import CMMCorePlus, DeviceType
from pymmcore_widgets import StageWidget

from pymmcore_gui._qt.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QSizePolicy,
    QWidget,
)
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.widgets._galvo_control import GalvoControlWidget

STAGE_DEVICES = {DeviceType.Stage, DeviceType.XYStage}


def is_piezo(mmc: CMMCorePlus, label: str) -> bool:
    """Whether *label* is a piezo stage (by its device name or description).

    A piezo keeps its 0 at the center of its travel, so its stage widget gets no
    "Zero" (set origin) control. ASI Tiger piezos are ``PiezoStage:<axis>:<card>``,
    "ASI Piezo Stage".
    """
    with suppress(Exception):
        text = f"{mmc.getDeviceName(label)} {mmc.getDeviceDescription(label)}"
        return "piezo" in text.lower()
    return False


class _Group(QGroupBox):
    def __init__(self, name: str, parent: QWidget | None = None) -> None:
        super().__init__(name, parent)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)


class StagesControlWidget(QWidget):
    """A widget to control all the XY and Z loaded stages."""

    def __init__(
        self, *, parent: QWidget | None = None, mmcore: CMMCorePlus | None = None
    ) -> None:
        super().__init__(parent=parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        self._mmc = mmcore or CMMCorePlus.instance()
        self._mmc.events.systemConfigurationLoaded.connect(self._on_cfg_loaded)

        self._layout = QGridLayout(self)
        self._layout.setContentsMargins(5, 5, 5, 5)
        self._layout.setSpacing(5)

        self._on_cfg_loaded()

    def _on_cfg_loaded(self) -> None:
        self._clear()

        stages = chain(
            self._mmc.getLoadedDevicesOfType(DeviceType.XYStage),  # pyright: ignore [reportArgumentType]
            self._mmc.getLoadedDevicesOfType(DeviceType.Stage),  # pyright: ignore [reportArgumentType]
        )
        idx = 0
        for stage_dev in stages:
            bx = _Group(stage_dev, self)
            stage = StageWidget(device=stage_dev, parent=bx, mmcore=self._mmc)
            # Older pymmcore-widgets have no origin controls to hide.
            if (enable_origin := getattr(stage, "enable_set_origin", None)) is not None:
                enable_origin(not is_piezo(self._mmc, stage_dev))
            if (lay := bx.layout()) is not None:
                lay.addWidget(stage)
            self._layout.addWidget(bx, idx // 2, idx % 2)
            idx += 1

        # The SPIM galvo sweeps the Z stack; its Live offset is set here.
        loaded = self._mmc.getLoadedDevices()
        galvos = list(self._mmc.getLoadedDevicesOfType(DeviceType.Galvo))  # pyright: ignore [reportArgumentType]
        fallback = HardwareConstants().galvo_a_label
        if fallback in loaded and fallback not in galvos:
            galvos.append(fallback)
        for galvo_dev in galvos:
            bx = _Group(galvo_dev, self)
            galvo = GalvoControlWidget(galvo_dev, self._mmc, parent=bx)
            if (lay := bx.layout()) is not None:
                lay.addWidget(galvo)
            self._layout.addWidget(bx, idx // 2, idx % 2)
            idx += 1
        self.resize(self.sizeHint())

    def _clear(self) -> None:
        while self._layout.count():
            if (item := self._layout.takeAt(0)) and (widget := item.widget()):
                widget.setParent(self)
                widget.deleteLater()
