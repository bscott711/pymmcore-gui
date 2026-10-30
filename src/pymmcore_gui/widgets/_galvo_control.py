"""Stage Control section for the SPIM galvo's Live offset.

The Z stack is swept by the galvo, centred on its ``SingleAxisYOffset(deg)``
(see :mod:`~pymmcore_gui.asi_z_stack.z_scan`). This widget reads and writes that
same property, so the offset set here is the sweep reference the MDA uses.
The offset is shown in um-equivalent (degrees times the slice calibration
slope), with the degree value beside it.
"""

from __future__ import annotations

import logging
from contextlib import suppress
from typing import Any

from pymmcore_plus import CMMCorePlus

from pymmcore_gui._qt.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QLabel,
    QPushButton,
    QWidget,
)
from pymmcore_gui.asi_z_stack.asi_controller import set_property
from pymmcore_gui.asi_z_stack.common import HardwareConstants

logger = logging.getLogger(__name__)

OFFSET_PROP = "SingleAxisYOffset(deg)"
#: Used when the card reports no property limits.
_FALLBACK_RANGE_UM = 1_000_000.0


class GalvoControlWidget(QWidget):
    """Set the galvo's Live offset (``SingleAxisYOffset(deg)``).

    Parameters
    ----------
    device_label : str
        Label of the galvo device (e.g. ``Scanner:AB:33``).
    mmcore : CMMCorePlus | None
        Core instance.
    """

    def __init__(
        self,
        device_label: str,
        mmcore: CMMCorePlus | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._device = device_label
        self._mmc = mmcore or CMMCorePlus.instance()
        self._slope = HardwareConstants().slice_calibration_slope_um_per_deg or 100.0

        self.offset = QDoubleSpinBox()
        self.offset.setDecimals(2)
        self.offset.setSuffix(" um")
        self.offset.setKeyboardTracking(False)
        self.offset.setRange(-_FALLBACK_RANGE_UM, _FALLBACK_RANGE_UM)
        self.offset.setToolTip(
            "Galvo Live offset in um-equivalent; the Z stack sweeps around it"
        )
        self.degrees = QLabel()

        self.step = QDoubleSpinBox()
        self.step.setDecimals(2)
        self.step.setRange(0.01, 10_000)
        self.step.setValue(1.0)
        self.step.setSuffix(" um")
        self.step.setToolTip("Step for the - and + buttons")
        self.minus = QPushButton("-")
        self.plus = QPushButton("+")
        self.zero = QPushButton("Go to 0")

        layout = QGridLayout(self)
        layout.addWidget(QLabel("Offset"), 0, 0)
        layout.addWidget(self.offset, 0, 1, 1, 2)
        layout.addWidget(self.degrees, 0, 3)
        layout.addWidget(QLabel("Step"), 1, 0)
        layout.addWidget(self.minus, 1, 1)
        layout.addWidget(self.step, 1, 2)
        layout.addWidget(self.plus, 1, 3)
        layout.addWidget(self.zero, 2, 0, 1, 4)

        self.offset.valueChanged.connect(self._on_spin_changed)
        self.minus.clicked.connect(lambda: self.nudge(-1))
        self.plus.clicked.connect(lambda: self.nudge(+1))
        self.zero.clicked.connect(lambda: self.set_offset_deg(0.0))

        self._mmc.events.propertyChanged.connect(self._on_property_changed)
        mda = self._mmc.mda.events
        mda.sequenceStarted.connect(lambda *_: self.setEnabled(False))
        mda.sequenceFinished.connect(lambda *_: self.setEnabled(True))

        self.refresh()

    # ------------------------------------------------------------------

    def _limits_deg(self) -> tuple[float, float] | None:
        mmc = self._mmc
        with suppress(Exception):
            if mmc.hasPropertyLimits(self._device, OFFSET_PROP):
                return (
                    float(mmc.getPropertyLowerLimit(self._device, OFFSET_PROP)),
                    float(mmc.getPropertyUpperLimit(self._device, OFFSET_PROP)),
                )
        return None

    def _read_deg(self) -> float | None:
        try:
            return float(self._mmc.getProperty(self._device, OFFSET_PROP))
        except Exception:
            logger.warning("Cannot read %s on %s", OFFSET_PROP, self._device)
            return None

    def refresh(self) -> None:
        """Show the card's current offset (no write)."""
        limits = self._limits_deg()
        deg = self._read_deg()
        self.offset.blockSignals(True)
        try:
            if limits is not None:
                lo, hi = sorted((limits[0] * self._slope, limits[1] * self._slope))
                self.offset.setRange(lo, hi)
            if deg is not None:
                self.offset.setValue(deg * self._slope)
        finally:
            self.offset.blockSignals(False)
        self.degrees.setText("" if deg is None else f"{deg:.4f} deg")

    def _on_property_changed(self, device: str, prop: str, value: Any = None) -> None:
        if device == self._device and prop == OFFSET_PROP:
            self.refresh()

    # ------------------------------------------------------------------

    def clamp_deg(self, deg: float) -> float:
        """*deg* limited to the card's property limits (if it reports any)."""
        if (limits := self._limits_deg()) is not None:
            lo, hi = sorted(limits)
            return min(max(deg, lo), hi)
        return deg

    def set_offset_deg(self, deg: float) -> None:
        """Write the offset (clamped to the limits); skip if unchanged."""
        deg = self.clamp_deg(deg)
        current = self._read_deg()
        if current is not None and abs(current - deg) < 1e-9:
            self.refresh()
            return
        set_property(self._device, OFFSET_PROP, f"{deg:.6f}")
        self.refresh()

    def nudge(self, direction: int) -> None:
        """Move the offset by one step (in um-equivalent), up or down."""
        current = self._read_deg()
        if current is None:
            return
        self.set_offset_deg(current + direction * self.step.value() / self._slope)

    def _on_spin_changed(self, um: float) -> None:
        self.set_offset_deg(um / self._slope)
