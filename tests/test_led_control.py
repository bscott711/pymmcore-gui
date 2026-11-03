from __future__ import annotations

from typing import TYPE_CHECKING, Any

import pytest

from pymmcore_gui.widgets._led_control import (
    LedControlWidget,
    find_intensity_property,
    find_led_devices,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus, DeviceType
    from pytestqt.qtbot import QtBot


class FakeLedCore:
    """The slice of CMMCorePlus the LED panel uses, backed by in-memory LEDs.

    Events and MDA signals come from the real (demo) core so the widget's
    connections behave normally.
    """

    def __init__(self, real: CMMCorePlus, leds: dict[str, bool]) -> None:
        self.events = real.events
        self.mda = real.mda
        self.open = dict(leds)
        self.intensity = dict.fromkeys(leds, 10.0)
        self.fail_on_open = False

    def getLoadedDevicesOfType(self, _dev_type: DeviceType) -> list[str]:
        return [*self.open, "Shutter"]  # "Shutter" = a non-LED shutter

    def getDeviceLibrary(self, label: str) -> str:
        return "DemoCamera" if label == "Shutter" else "ASITiger"

    def getDevicePropertyNames(self, label: str) -> list[str]:
        return ["Name", "LED Intensity(%)"] if label in self.intensity else ["Name"]

    def hasPropertyLimits(self, label: str, prop: str) -> bool:
        return True

    def getPropertyLowerLimit(self, label: str, prop: str) -> float:
        return 0.0

    def getPropertyUpperLimit(self, label: str, prop: str) -> float:
        return 100.0

    def getShutterOpen(self, label: str) -> bool:
        return self.open[label]

    def setShutterOpen(self, label: str, state: bool) -> None:
        if self.fail_on_open:
            raise RuntimeError("serial error")
        self.open[label] = state

    def getProperty(self, label: str, prop: str) -> str:
        return str(self.intensity[label])

    def setProperty(self, label: str, prop: str, value: Any) -> None:
        self.intensity[label] = float(value)


@pytest.fixture
def fake(mmcore: CMMCorePlus) -> FakeLedCore:
    return FakeLedCore(mmcore, {"LED:X:37": False, "LED:Y:37": True, "LED:Z:37": False})


def test_discovery_finds_only_asitiger_leds(fake: FakeLedCore) -> None:
    assert find_led_devices(fake) == ["LED:X:37", "LED:Y:37", "LED:Z:37"]  # type: ignore[arg-type]
    assert find_intensity_property(fake, "LED:X:37") == "LED Intensity(%)"  # type: ignore[arg-type]


def test_no_leds_in_demo_config_shows_hint(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    widget = LedControlWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    assert widget.led_labels() == []
    assert "No ASITiger LED devices" in widget._status.text()


def test_rows_reflect_hardware_state(fake: FakeLedCore, qtbot: QtBot) -> None:
    widget = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(widget)
    assert list(widget.led_labels()) == ["LED:X:37", "LED:Y:37", "LED:Z:37"]
    rows = widget._rows
    assert not rows["LED:X:37"].power.isChecked()
    assert rows["LED:Y:37"].power.isChecked()
    assert rows["LED:Y:37"].power.text() == "On"
    assert rows["LED:X:37"].intensity is not None
    assert rows["LED:X:37"].intensity.value() == 10.0


def test_toggle_and_intensity_drive_the_device(fake: FakeLedCore, qtbot: QtBot) -> None:
    widget = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(widget)
    row = widget._rows["LED:X:37"]

    row.power.setChecked(True)
    assert fake.open["LED:X:37"] is True

    assert row.intensity is not None
    row.intensity.setValue(42.5)
    assert fake.intensity["LED:X:37"] == 42.5

    # the initial refresh must not have written anything back to the hardware
    assert fake.open["LED:Z:37"] is False
    assert fake.intensity["LED:Z:37"] == 10.0


def test_all_off(fake: FakeLedCore, qtbot: QtBot) -> None:
    widget = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(widget)
    widget._rows["LED:X:37"].power.setChecked(True)
    widget.turn_all_off()
    assert not any(fake.open.values())
    assert not any(r.power.isChecked() for r in widget._rows.values())


def test_device_error_is_reported_and_state_resynced(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    widget = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(widget)
    fake.fail_on_open = True
    row = widget._rows["LED:X:37"]
    row.power.setChecked(True)
    assert "serial error" in widget._status.text()
    assert not row.power.isChecked()  # snapped back to what the device reports


def test_locked_during_mda(fake: FakeLedCore, qtbot: QtBot) -> None:
    widget = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(widget)
    fake.mda.events.sequenceStarted.emit(None, {})
    assert not widget._controls.isEnabled()
    fake.mda.events.sequenceFinished.emit(None)
    assert widget._controls.isEnabled()
