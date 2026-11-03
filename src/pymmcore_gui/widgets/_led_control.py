"""Dock widget for ASI TGLED epifluorescence LEDs.

A TGLED card shows up in Micro-Manager as one or more ``ASITiger`` shutter
devices named ``LED:...``. This panel discovers them at runtime (so it needs no
knowledge of the card address) and gives each a power button and, when the
device exposes one, an intensity control. It is independent of the OPM/SPIM
acquisition path: it only issues ``setShutterOpen`` / ``setProperty`` calls on
the main-process core, and locks itself while an MDA is running so it cannot
interleave serial commands with the engine.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus, DeviceType

from pymmcore_gui._qt.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

if TYPE_CHECKING:
    from collections.abc import Sequence

#: Device library that provides the TGLED shutter devices.
LED_LIBRARY = "ASITiger"
#: Device-label prefix the ASITiger adapter gives a TGLED shutter.
LED_PREFIX = "LED"


def find_led_devices(mmc: CMMCorePlus) -> list[str]:
    """Return the labels of all loaded ASITiger LED shutter devices."""
    found: list[str] = []
    for label in mmc.getLoadedDevicesOfType(DeviceType.ShutterDevice):
        if not label.startswith(LED_PREFIX):
            continue
        with suppress(RuntimeError):
            if mmc.getDeviceLibrary(label) == LED_LIBRARY:
                found.append(label)
    return found


def find_intensity_property(mmc: CMMCorePlus, label: str) -> str | None:
    """Return the name of the device's intensity property, if it has one."""
    for name in mmc.getDevicePropertyNames(label):
        if "intensity" in name.lower():
            return name
    return None


class _LedRow:
    """The controls for one LED device."""

    def __init__(self, label: str, intensity_prop: str | None, mmc: CMMCorePlus):
        self.label = label
        self.intensity_prop = intensity_prop
        self.name = QLabel(label)
        self.power = QPushButton("Off")
        self.power.setCheckable(True)
        self.power.setToolTip(f"Turn {label} on/off")
        self.intensity: QDoubleSpinBox | None = None
        if intensity_prop is not None:
            self.intensity = QDoubleSpinBox()
            self.intensity.setSuffix(" %")
            self.intensity.setDecimals(1)
            self.intensity.setSingleStep(1.0)
            lo, hi = 0.0, 100.0
            if mmc.hasPropertyLimits(label, intensity_prop):
                lo = mmc.getPropertyLowerLimit(label, intensity_prop)
                hi = mmc.getPropertyUpperLimit(label, intensity_prop)
            self.intensity.setRange(lo, hi)
            # apply on Enter/focus-out/arrow steps, not on every keystroke
            self.intensity.setKeyboardTracking(False)


class LedControlWidget(QWidget):
    """On/off and intensity control for every TGLED LED in the loaded config."""

    def __init__(self, *, mmcore: CMMCorePlus, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._mmc = mmcore
        self._rows: dict[str, _LedRow] = {}
        self._syncing = False

        self._grid = QGridLayout()
        self._status = QLabel()
        self._status.setWordWrap(True)
        self.all_off = QPushButton("All off")
        self.all_off.clicked.connect(self.turn_all_off)
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.setToolTip("Re-read LED state from the controller")
        self.refresh_button.clicked.connect(self.refresh)

        buttons = QHBoxLayout()
        buttons.addWidget(self.all_off)
        buttons.addWidget(self.refresh_button)
        buttons.addStretch()

        self._controls = QWidget()
        controls_layout = QVBoxLayout(self._controls)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.addLayout(self._grid)
        controls_layout.addLayout(buttons)

        layout = QVBoxLayout(self)
        layout.addWidget(self._controls)
        layout.addWidget(self._status)
        layout.addStretch()

        mmc = self._mmc
        mmc.events.systemConfigurationLoaded.connect(self.rebuild)
        mmc.mda.events.sequenceStarted.connect(self._on_mda_started)
        mmc.mda.events.sequenceFinished.connect(self._on_mda_finished)
        self.destroyed.connect(self._disconnect)
        self.rebuild()

    # ------------------------------------------------------------------ build

    def led_labels(self) -> Sequence[str]:
        """Labels of the LED devices currently shown."""
        return list(self._rows)

    def rebuild(self) -> None:
        """Re-discover LED devices and rebuild the control rows."""
        for row in self._rows.values():
            for w in (row.name, row.power, row.intensity):
                if w is not None:
                    self._grid.removeWidget(w)
                    w.deleteLater()
        self._rows.clear()

        for i, label in enumerate(find_led_devices(self._mmc)):
            row = _LedRow(label, find_intensity_property(self._mmc, label), self._mmc)
            self._rows[label] = row
            self._grid.addWidget(row.name, i, 0)
            self._grid.addWidget(row.power, i, 1)
            if row.intensity is not None:
                self._grid.addWidget(row.intensity, i, 2)
                row.intensity.valueChanged.connect(
                    lambda v, lbl=label: self._set_intensity(lbl, v)
                )
            row.power.toggled.connect(lambda on, lbl=label: self._set_power(lbl, on))

        self._controls.setVisible(bool(self._rows))
        if not self._rows:
            self._status.setText(
                "No ASITiger LED devices in the loaded configuration. Add the "
                "TGLED card's LED device(s) to the hardware config and reload."
            )
        else:
            self._status.clear()
            self.refresh()

    # ----------------------------------------------------------------- state

    def refresh(self) -> None:
        """Show the controller's current state for every LED."""
        self._syncing = True
        try:
            for label, row in self._rows.items():
                try:
                    is_open = bool(self._mmc.getShutterOpen(label))
                    value = (
                        float(self._mmc.getProperty(label, row.intensity_prop))
                        if row.intensity_prop is not None
                        else None
                    )
                except Exception as e:
                    self._status.setText(f"Could not read {label}: {e}")
                    continue
                self._show_power(row, is_open)
                if row.intensity is not None and value is not None:
                    row.intensity.setValue(value)
        finally:
            self._syncing = False

    @staticmethod
    def _show_power(row: _LedRow, is_open: bool) -> None:
        row.power.setChecked(is_open)
        row.power.setText("On" if is_open else "Off")

    def _set_power(self, label: str, on: bool) -> None:
        row = self._rows[label]
        row.power.setText("On" if on else "Off")
        if self._syncing:
            return
        try:
            self._mmc.setShutterOpen(label, on)
        except Exception as e:
            self._status.setText(f"Could not switch {label}: {e}")
            self.refresh()
        else:
            self._status.clear()

    def _set_intensity(self, label: str, value: float) -> None:
        if self._syncing:
            return
        row = self._rows[label]
        assert row.intensity_prop is not None
        try:
            self._mmc.setProperty(label, row.intensity_prop, value)
        except Exception as e:
            self._status.setText(f"Could not set {label} intensity: {e}")
            self.refresh()
        else:
            self._status.clear()

    def turn_all_off(self) -> None:
        """Switch every LED off."""
        for label, row in self._rows.items():
            if row.power.isChecked():
                row.power.setChecked(False)  # fires _set_power -> setShutterOpen
            else:
                with suppress(Exception):
                    self._mmc.setShutterOpen(label, False)

    # ------------------------------------------------------------------- MDA

    def _on_mda_started(self, *_: object) -> None:
        self._controls.setEnabled(False)

    def _on_mda_finished(self, *_: object) -> None:
        self._controls.setEnabled(True)
        self.refresh()

    def _disconnect(self) -> None:
        mmc = self._mmc
        with suppress(RuntimeError, TypeError):
            mmc.events.systemConfigurationLoaded.disconnect(self.rebuild)
        with suppress(RuntimeError, TypeError):
            mmc.mda.events.sequenceStarted.disconnect(self._on_mda_started)
        with suppress(RuntimeError, TypeError):
            mmc.mda.events.sequenceFinished.disconnect(self._on_mda_finished)
