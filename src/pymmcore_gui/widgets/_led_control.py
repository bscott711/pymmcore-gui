"""Dock widget for the ASI TGLED epifluorescence LEDs (EPI mode only).

The TGLED card (Tiger address ``37``) shows up in Micro-Manager as one
``ASITiger`` shutter device per channel, ``LED:<axis>:37:<n>``. This panel shows
one row per channel in ``settings.epi.leds`` (hidden channels are skipped): an
on/off toggle plus an integer percent slider/spinbox (1-100, the card's native
unit). Discovery goes only through
:func:`~pymmcore_gui.epi._led.resolve_led_channels`, so an unrelated LED such as
``LED:X:31`` never gets a row.

Every control is disabled unless the app is in EPI mode. Percent edits are saved
to settings; the property is written to the card only while that channel is on
(in :func:`~pymmcore_gui.epi._led.apply_led`, right before the shutter opens).
Opening the panel never writes to the hardware. The panel locks itself while an
MDA is running so it cannot interleave serial commands with the engine.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_gui._qt.QtCore import Qt, QTimer
from pymmcore_gui._qt.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.epi._led import (
    DEFAULT_CARD_ADDRESS,
    all_leds_off,
    apply_led,
    resolve_led_channels,
)
from pymmcore_gui.epi._mode import ScopeMode, current_mode

if TYPE_CHECKING:
    from collections.abc import Sequence

    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._settings import EpiLedChannel

_FLUSH_DELAY_MS = 500


def find_led_devices(
    mmc: CMMCorePlus, card_address: str = DEFAULT_CARD_ADDRESS
) -> list[str]:
    """Return the labels of the TGLED channels (thin wrapper, address-checked)."""
    return list(resolve_led_channels(mmc, card_address).values())


class _LedRow:
    """The controls for one settings channel."""

    def __init__(self, led: EpiLedChannel, label: str | None) -> None:
        self.led = led
        self.label = label  # None => not loaded
        text = f"{led.name}  ({label})" if label else f"{led.name}  (not loaded)"
        self.name = QLabel(text)
        self.power = QPushButton("Off")
        self.power.setCheckable(True)
        self.power.setToolTip(f"Turn the {led.name} LED on/off")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(1, 100)
        self.slider.setTracking(False)  # one hardware write per drag, on release
        self.spin = QSpinBox()
        self.spin.setRange(1, 100)
        self.spin.setSuffix(" %")
        self.spin.setKeyboardTracking(False)
        self.slider.setValue(led.percent)
        self.spin.setValue(led.percent)
        if label is None:
            for w in self.widgets():
                w.setEnabled(False)

    def widgets(self) -> tuple[QWidget, ...]:
        return (self.name, self.power, self.slider, self.spin)


class LedControlWidget(QWidget):
    """On/off and percent control for the TGLED LED channels (EPI mode)."""

    def __init__(self, *, mmcore: CMMCorePlus, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._mmc = mmcore
        self._rows: dict[int, _LedRow] = {}
        self._syncing = False
        self._epi_active = current_mode() is ScopeMode.EPI
        self._mda_running = False
        self._no_leds_hint = ""

        self._grid = QGridLayout()
        self._status = QLabel()
        self._status.setWordWrap(True)
        self.all_off = QPushButton("All off")
        self.all_off.clicked.connect(self.turn_all_off)
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.setToolTip("Re-read LED on/off state from the controller")
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

        # Deliberately unparented: a parented QTimer that is still pending when the
        # widget is torn down crashed PyQt at garbage collection in the test suite.
        self._flush_timer = QTimer()
        self._flush_timer.setSingleShot(True)
        self._flush_timer.setInterval(_FLUSH_DELAY_MS)
        self._flush_timer.timeout.connect(lambda: SettingsV1.instance().flush())

        mmc = self._mmc
        mmc.events.systemConfigurationLoaded.connect(self.rebuild)
        mmc.mda.events.sequenceStarted.connect(self._on_mda_started)
        mmc.mda.events.sequenceFinished.connect(self._on_mda_finished)
        self.destroyed.connect(self._disconnect)
        self.rebuild()

    # ------------------------------------------------------------------ build

    def led_labels(self) -> Sequence[str]:
        """Labels of the loaded LED devices currently shown."""
        return [r.label for r in self._rows.values() if r.label is not None]

    def rebuild(self) -> None:
        """Re-resolve the TGLED channels and rebuild the control rows."""
        for row in self._rows.values():
            for w in row.widgets():
                self._grid.removeWidget(w)
                w.deleteLater()
        self._rows.clear()

        epi = SettingsV1.instance().epi
        resolved = resolve_led_channels(self._mmc, epi.card_address)
        for i, led in enumerate(epi.leds):
            if not led.visible:
                continue
            row = _LedRow(led, resolved.get(led.channel))
            self._rows[led.channel] = row
            self._grid.addWidget(row.name, i, 0)
            self._grid.addWidget(row.power, i, 1)
            self._grid.addWidget(row.slider, i, 2)
            self._grid.addWidget(row.spin, i, 3)
            ch = led.channel
            row.power.toggled.connect(lambda on, c=ch: self._set_power(c, on))
            row.slider.sliderMoved.connect(lambda v, c=ch: self._show_slider(c, v))
            row.slider.valueChanged.connect(lambda v, c=ch: self._slider_changed(c, v))
            row.spin.valueChanged.connect(lambda v, c=ch: self._set_percent(c, v))

        self._no_leds_hint = (
            ""
            if resolved
            else (
                f"No TGLED channels at Tiger address {epi.card_address} are loaded. "
                "Load the EPI hardware config (20261118-OPM_EPI.cfg)."
            )
        )
        self._controls.setVisible(bool(resolved) and bool(self._rows))
        self._status.clear()
        self._update_enabled()
        if self._epi_active_now():
            self.refresh()

    # ------------------------------------------------------------------ mode

    def _epi_active_now(self) -> bool:
        return self._epi_active and current_mode() is ScopeMode.EPI

    def set_epi_active(self, active: bool) -> None:
        """Enable the controls in EPI, disable them in OPM. Never writes hardware."""
        self._epi_active = bool(active)
        self._update_enabled()
        if self._epi_active_now():
            self.refresh()

    def _update_enabled(self) -> None:
        epi = self._epi_active_now()
        self._controls.setEnabled(epi and not self._mda_running)
        if self._no_leds_hint:
            self._status.setText(self._no_leds_hint)
        elif not epi:
            self._status.setText("The LED panel is only active in EPI mode.")
        elif self._status.text().startswith("The LED panel"):
            self._status.clear()

    # ----------------------------------------------------------------- state

    def refresh(self) -> None:
        """Show the controller's current on/off state for every loaded LED."""
        if not self._epi_active_now():
            return
        self._syncing = True
        try:
            for row in self._rows.values():
                if row.label is None:
                    continue
                try:
                    is_open = bool(self._mmc.getShutterOpen(row.label))
                except Exception as e:
                    self._status.setText(f"Could not read {row.label}: {e}")
                    continue
                self._show_power(row, is_open)
        finally:
            self._syncing = False

    @staticmethod
    def _show_power(row: _LedRow, is_open: bool) -> None:
        row.power.setChecked(is_open)
        row.power.setText("On" if is_open else "Off")

    def _set_power(self, channel: int, on: bool) -> None:
        row = self._rows[channel]
        row.power.setText("On" if on else "Off")
        if self._syncing or row.label is None:
            return
        try:
            if on:
                apply_led(self._mmc, row.label, row.led.percent)
            else:
                self._mmc.setShutterOpen(row.label, False)
        except Exception as e:
            self._status.setText(f"Could not switch {row.label}: {e}")
            self._resync(row)
        else:
            self._status.clear()

    def _resync(self, row: _LedRow) -> None:
        """Snap the toggle back to what the device reports (after an error)."""
        if row.label is None:
            return
        self._syncing = True
        try:
            try:
                is_open = bool(self._mmc.getShutterOpen(row.label))
            except Exception:
                is_open = False
            self._show_power(row, is_open)
        finally:
            self._syncing = False

    def _show_slider(self, channel: int, value: int) -> None:
        """Mirror a slider drag in the spinbox without any side effects."""
        spin = self._rows[channel].spin
        spin.blockSignals(True)
        try:
            spin.setValue(value)
        finally:
            spin.blockSignals(False)

    def _slider_changed(self, channel: int, value: int) -> None:
        # fires on release / key steps; the spinbox may already show the value
        # (see _show_slider), so save and write explicitly.
        self._show_slider(channel, value)
        self._set_percent(channel, value)

    def _set_percent(self, channel: int, value: int) -> None:
        row = self._rows[channel]
        slider = row.slider
        if slider.value() != value:
            slider.blockSignals(True)
            try:
                slider.setValue(value)
            finally:
                slider.blockSignals(False)
        if self._syncing:
            return
        row.led.percent = value
        self._flush_timer.start()
        if row.label is None or not row.power.isChecked():
            return  # remembered only; written when the channel is switched on
        try:
            apply_led(self._mmc, row.label, value)
        except Exception as e:
            self._status.setText(f"Could not set {row.label} intensity: {e}")
            self._resync(row)
        else:
            self._status.clear()

    def turn_all_off(self) -> None:
        """Close every TGLED channel (also allowed outside EPI)."""
        all_leds_off(self._mmc, SettingsV1.instance().epi.card_address)
        self._syncing = True
        try:
            for row in self._rows.values():
                self._show_power(row, False)
        finally:
            self._syncing = False

    # ------------------------------------------------------------------- MDA

    def _on_mda_started(self, *_: object) -> None:
        self._mda_running = True
        self._update_enabled()

    def _on_mda_finished(self, *_: object) -> None:
        self._mda_running = False
        self._update_enabled()
        self.refresh()  # no-op outside EPI

    def _disconnect(self) -> None:
        mmc = self._mmc
        with suppress(RuntimeError, TypeError):
            mmc.events.systemConfigurationLoaded.disconnect(self.rebuild)
        with suppress(RuntimeError, TypeError):
            mmc.mda.events.sequenceStarted.disconnect(self._on_mda_started)
        with suppress(RuntimeError, TypeError):
            mmc.mda.events.sequenceFinished.disconnect(self._on_mda_finished)
