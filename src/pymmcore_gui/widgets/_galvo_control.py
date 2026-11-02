"""Stage Control section for the SPIM galvo's Live offset.

The Z stack is swept by the galvo, centred on its ``SingleAxisYOffset(deg)``
(see :mod:`~pymmcore_gui.asi_z_stack.z_scan`). This widget reads and writes that
same property, so the offset set here is the sweep centre the MDA snapshots as
the Live plane. The property alone does not move an idle mirror (it only sets
the sweep centre), so every write is followed by a static move of the slice axis
(:func:`~pymmcore_gui.asi_z_stack.galvo_position.move_galvo_slice`), which is
what positions the mirror in Live. A read-only label shows the mirror's
read-back position. The offset is shown in um-equivalent (degrees times the
slice calibration slope), with the degree value beside it. Each value set here
is saved and re-applied to the card on every config load
(:mod:`~pymmcore_gui.asi_z_stack.live_offset`).
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
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.galvo_position import (
    POSITION_TOLERANCE_DEG,
    move_galvo_slice,
    read_galvo_position,
)
from pymmcore_gui.asi_z_stack.live_offset import OFFSET_PROP, save_live_offset_deg

logger = logging.getLogger(__name__)

__all__ = ["OFFSET_PROP", "GalvoControlWidget"]

#: Used when the card reports no property limits.
_FALLBACK_RANGE_UM = 1_000_000.0


class GalvoControlWidget(QWidget):
    """Set the galvo's Live offset (``SingleAxisYOffset(deg)``).

    The property is the sweep centre the MDA reads; each write is also a static
    move of the slice axis (card axis B), which is what places the mirror in
    Live. Each user change is saved to the settings and re-applied on config
    load. The widget never writes ``SPIMState``, ``SingleAxis*Mode`` or
    ``BeamEnabled``.

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
        self._writing = False

        self.offset = QDoubleSpinBox()
        self.offset.setDecimals(2)
        self.offset.setSuffix(" um")
        self.offset.setKeyboardTracking(False)
        self.offset.setRange(-_FALLBACK_RANGE_UM, _FALLBACK_RANGE_UM)
        self.offset.setToolTip(
            "Galvo Live offset in um-equivalent; the Z stack sweeps around it"
        )
        self.degrees = QLabel()
        self.mirror = QLabel("mirror ?")

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
        layout.addWidget(self.mirror, 3, 0, 1, 4)

        self.offset.valueChanged.connect(self._on_spin_changed)
        self.minus.clicked.connect(lambda: self.nudge(-1))
        self.plus.clicked.connect(lambda: self.nudge(+1))
        self.zero.clicked.connect(lambda: self.set_offset_deg(0.0))

        self._mmc.events.propertyChanged.connect(self._on_property_changed)
        mda = self._mmc.mda.events
        mda.sequenceStarted.connect(lambda *_: self.setEnabled(False))
        mda.sequenceFinished.connect(self._on_sequence_finished)

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

    def _refresh_offset(self) -> float | None:
        """Show the card's offset property (no write, no mirror readback)."""
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
        return deg

    def _show_mirror(
        self, position: tuple[float, float] | None, offset_deg: float | None
    ) -> None:
        """Show the mirror read-back; flag it if it is not at the offset."""
        if position is None:
            self.mirror.setText("mirror ?")
            self.mirror.setToolTip("The mirror position could not be read")
            return
        self.mirror.setText(f"mirror {position[1]:.4f} deg")
        if offset_deg is not None and (
            abs(position[1] - offset_deg) > POSITION_TOLERANCE_DEG
        ):
            self.mirror.setToolTip(
                "The mirror is not at the offset: the beam is off or "
                "single-axis mode is on, so the static move was ignored"
            )
        else:
            self.mirror.setToolTip("Mirror position read back from the card")

    def refresh(self) -> None:
        """Show the card's current offset and mirror position (no write).

        The mirror is only read back when no MDA is running, so the GUI thread
        never issues serial readbacks mid-acquisition.
        """
        deg = self._refresh_offset()
        if not self._mmc.mda.is_running():
            self._show_mirror(read_galvo_position(self._mmc, self._device), deg)

    def _on_property_changed(self, device: str, prop: str, value: Any = None) -> None:
        if device == self._device and prop == OFFSET_PROP and not self._writing:
            self.refresh()

    def _on_sequence_finished(self, *_: Any) -> None:
        self.setEnabled(True)
        # The engine restores the offset and re-moves the mirror at teardown.
        self.refresh()

    # ------------------------------------------------------------------

    def clamp_deg(self, deg: float) -> float:
        """*deg* limited to the card's property limits (if it reports any)."""
        if (limits := self._limits_deg()) is not None:
            lo, hi = sorted(limits)
            return min(max(deg, lo), hi)
        return deg

    def set_offset_deg(self, deg: float) -> None:
        """Write the offset (clamped) and move the mirror there.

        The property write is skipped when the value is unchanged, but the
        static move is not: the mirror may not be at the offset yet.
        """
        deg = self.clamp_deg(deg)
        mmc = self._mmc
        if self._device not in mmc.getLoadedDevices():
            logger.warning("Cannot set %s on %s", OFFSET_PROP, self._device)
            self.refresh()
            return
        self._writing = True
        try:
            if mmc.hasProperty(self._device, OFFSET_PROP):
                current = self._read_deg()
                if current is None or abs(current - deg) >= 1e-9:
                    mmc.setProperty(self._device, OFFSET_PROP, f"{deg:.6f}")
                # Only user changes are saved (not refresh / engine writes).
                save_live_offset_deg(deg)
            else:
                logger.warning("Cannot set %s on %s", OFFSET_PROP, self._device)
            position = move_galvo_slice(mmc, self._device, deg)
        finally:
            self._writing = False
        offset = self._refresh_offset()
        self._show_mirror(position, offset)

    def nudge(self, direction: int) -> None:
        """Move the offset by one step (in um-equivalent), up or down."""
        current = self._read_deg()
        if current is None:
            return
        self.set_offset_deg(current + direction * self.step.value() / self._slope)

    def _on_spin_changed(self, um: float) -> None:
        self.set_offset_deg(um / self._slope)
