from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from pymmcore_gui.epi._led import LED_INTENSITY_PROP
from pymmcore_gui.epi._mode import set_active_controller
from pymmcore_gui.widgets._led_control import LedControlWidget, find_led_devices
from test_epi_led import (
    DECOY_OTHER_CARD,
    DECOY_X,
    TGLED,
    FakeLedCore,
    _EpiController,
)

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

    from pymmcore_gui._settings import SettingsV1


@pytest.fixture(autouse=True)
def _reset_mode() -> Iterator[None]:
    set_active_controller(None)
    yield
    set_active_controller(None)


@pytest.fixture
def fake(mmcore: CMMCorePlus) -> FakeLedCore:
    return FakeLedCore(mmcore)


def _epi() -> None:
    set_active_controller(_EpiController())


def _widget(fake: FakeLedCore, qtbot: QtBot) -> LedControlWidget:
    w = LedControlWidget(mmcore=fake)  # type: ignore[arg-type]
    qtbot.addWidget(w)
    return w


def test_discovery_wrapper_is_address_checked(fake: FakeLedCore) -> None:
    assert find_led_devices(fake) == TGLED  # type: ignore[arg-type]


def test_no_leds_in_demo_config_shows_hint(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    widget = LedControlWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    assert widget.led_labels() == []
    assert "No TGLED channels" in widget._status.text()


def test_rows_are_the_visible_tgled_channels_only(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    # channel 4 (405) is hidden by default; decoys never get a row
    assert list(widget.led_labels()) == TGLED[:3]
    assert set(widget._rows) == {1, 2, 3}
    for decoy in (DECOY_X, DECOY_OTHER_CARD):
        assert decoy not in widget.led_labels()
        assert fake.touched(decoy) == []


def test_visible_channel_4_gets_a_row(
    fake: FakeLedCore, settings: SettingsV1, qtbot: QtBot
) -> None:
    _epi()
    settings.epi.leds[3].visible = True
    widget = _widget(fake, qtbot)
    assert list(widget.led_labels()) == TGLED


def test_only_the_x_led_loaded_shows_the_hint(
    mmcore: CMMCorePlus, qtbot: QtBot
) -> None:
    _epi()
    fake = FakeLedCore(mmcore, labels=[DECOY_X])
    widget = _widget(fake, qtbot)
    assert widget.led_labels() == []
    assert "No TGLED channels" in widget._status.text()
    widget.turn_all_off()
    assert fake.touched(DECOY_X) == []


def test_unloaded_channel_is_shown_disabled(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    _epi()
    fake = FakeLedCore(mmcore, labels=[TGLED[0]])
    widget = _widget(fake, qtbot)
    assert list(widget.led_labels()) == [TGLED[0]]
    row2 = widget._rows[2]
    assert "not loaded" in row2.name.text()
    assert not row2.power.isEnabled()
    assert not row2.slider.isEnabled()
    assert not row2.spin.isEnabled()
    assert widget._rows[1].power.isEnabled()


def test_opening_the_panel_never_writes_hardware(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    _epi()
    fake.open[TGLED[1]] = True
    widget = _widget(fake, qtbot)
    assert fake.writes() == []
    assert widget._rows[2].power.isChecked()  # reflects the device
    assert widget._rows[1].spin.value() == 50  # settings value, not read from card
    assert all(c[0] != "getProperty" for c in fake.calls)


def test_controls_disabled_in_opm_and_no_hardware_reads(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    widget = _widget(fake, qtbot)
    assert not widget._controls.isEnabled()
    widget.refresh()
    assert fake.calls == []  # not even reads


def test_set_epi_active_enables_and_disables(fake: FakeLedCore, qtbot: QtBot) -> None:
    widget = _widget(fake, qtbot)
    widget.set_epi_active(True)  # flag alone is not enough: the mode is OPM
    assert not widget._controls.isEnabled()
    _epi()
    widget.set_epi_active(True)
    assert widget._controls.isEnabled()
    assert fake.writes() == []
    widget.set_epi_active(False)
    assert not widget._controls.isEnabled()


def test_toggle_on_writes_percent_then_opens(fake: FakeLedCore, qtbot: QtBot) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    widget._rows[2].spin.setValue(70)
    assert fake.writes() == []  # remembered only: the channel is off
    widget._rows[2].power.setChecked(True)
    assert fake.writes() == [
        ("setProperty", TGLED[1], LED_INTENSITY_PROP, 70),
        ("setShutterOpen", TGLED[1], True),
    ]
    assert fake.on_card[TGLED[1]] == 70
    widget._rows[2].power.setChecked(False)
    assert fake.writes()[-1] == ("setShutterOpen", TGLED[1], False)
    assert fake.touched(DECOY_X) == []


def test_percent_edit_while_on_is_written_and_saved(
    fake: FakeLedCore, settings: SettingsV1, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    row = widget._rows[1]
    row.power.setChecked(True)
    fake.calls.clear()
    row.spin.setValue(33)
    assert ("setProperty", TGLED[0], LED_INTENSITY_PROP, 33) in fake.calls
    assert fake.on_card[TGLED[0]] == 33
    assert settings.epi.leds[0].percent == 33
    assert row.slider.value() == 33


def test_percent_edit_while_off_is_saved_without_hardware_write(
    fake: FakeLedCore, settings: SettingsV1, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    widget._rows[3].spin.setValue(12)
    assert settings.epi.leds[2].percent == 12
    assert fake.writes() == []


def test_slider_and_spinbox_stay_in_sync(
    fake: FakeLedCore, settings: SettingsV1, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    row = widget._rows[1]
    assert (row.slider.minimum(), row.slider.maximum()) == (1, 100)
    assert (row.spin.minimum(), row.spin.maximum()) == (1, 100)
    assert row.spin.suffix() == " %"
    row.slider.setValue(81)  # as after a drag release
    assert row.spin.value() == 81
    assert settings.epi.leds[0].percent == 81
    row.spin.setValue(9)
    assert row.slider.value() == 9


def test_stored_percent_survives_a_rebuild(
    fake: FakeLedCore, settings: SettingsV1, qtbot: QtBot
) -> None:
    _epi()
    settings.epi.leds[0].percent = 77
    widget = _widget(fake, qtbot)
    assert widget._rows[1].spin.value() == 77
    assert fake.writes() == []


def test_all_off_closes_everything_including_hidden_channel(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    widget._rows[1].power.setChecked(True)
    fake.open[TGLED[3]] = True  # hidden channel 4 left on
    widget.turn_all_off()
    assert not any(fake.open[lbl] for lbl in TGLED)
    assert not any(r.power.isChecked() for r in widget._rows.values())
    assert fake.touched(DECOY_X) == []


def test_device_error_is_reported_and_state_resynced(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    fake.fail_on.add(TGLED[0])
    row = widget._rows[1]
    row.power.setChecked(True)
    assert "serial error" in widget._status.text()
    assert not row.power.isChecked()  # snapped back to what the device reports


def test_locked_during_mda_and_reenabled_only_in_epi(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    _epi()
    widget = _widget(fake, qtbot)
    fake.mda.events.sequenceStarted.emit(None, {})
    assert not widget._controls.isEnabled()
    fake.mda.events.sequenceFinished.emit(None)
    assert widget._controls.isEnabled()


def test_after_an_opm_mda_the_panel_stays_disabled_and_reads_nothing(
    fake: FakeLedCore, qtbot: QtBot
) -> None:
    widget = _widget(fake, qtbot)
    fake.mda.events.sequenceStarted.emit(None, {})
    fake.mda.events.sequenceFinished.emit(None)
    assert not widget._controls.isEnabled()
    assert fake.calls == []
