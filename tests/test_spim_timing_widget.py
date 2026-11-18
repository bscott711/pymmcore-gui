from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.widgets._spim_timing_widget import SpimTimingWidget

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot


def test_widget_previews_and_applies_timing(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    widget = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(widget)

    # defaults reproduce the original timing
    assert widget.timing().slice_period_ms == 35.75
    assert "Next MDA uses legacy" in widget.summary.text()

    widget.mode.setCurrentIndex(widget.mode.findData("all_rows"))
    widget.readout_auto.setChecked(False)
    widget.readout.setValue(6.0)
    widget.settle.setValue(2.0)
    widget.preview_exposure.setValue(10.0)

    # S=6, E=10, s=2, margin 0.5: camera pulse 17, hold max(17, 21) -> 23 ms
    assert widget.timing().slice_period_ms == 23.0
    assert "delays the laser" in widget.summary.text()

    widget.overlap.setChecked(True)
    assert SettingsV1.instance().spim_timing.level_trigger_overlap
    assert widget.timing().slice_period_ms == 19.0  # hold = camera pulse
    assert "Level Trigger Overlap" in widget.summary.text()

    # no Save step: every edit is already what the next MDA reads
    cfg = SettingsV1.instance().spim_timing
    assert cfg.mode == "all_rows"
    assert cfg.galvo_settle_ms == 2.0
    assert cfg.readout_ms_override == 6.0


def test_switching_to_legacy_applies_immediately(
    mmcore: CMMCorePlus, qtbot: QtBot
) -> None:
    cfg = SettingsV1.instance().spim_timing
    cfg.mode = "all_rows"
    cfg.galvo_settle_ms = 0.25
    widget = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    assert not widget.legacy_note.isVisibleTo(widget)

    widget.mode.setCurrentIndex(widget.mode.findData("legacy"))

    assert cfg.mode == "legacy"
    # legacy ignores the All Rows settle: the original 35.75 ms slice
    assert widget.timing().slice_period_ms == 35.75
    assert widget.legacy_note.isVisibleTo(widget)
    assert not widget.settle.isEnabled()


def test_widget_locks_edits_during_mda(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    widget = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    mmcore.mda.events.sequenceStarted.emit(None, {})
    assert not widget.settings_box.isEnabled()
    mmcore.mda.events.sequenceFinished.emit(None)
    assert widget.settings_box.isEnabled()


def test_park_checkbox_persists_and_locks_during_mda(
    mmcore: CMMCorePlus, qtbot: QtBot
) -> None:
    cfg = SettingsV1.instance().spim_timing
    assert cfg.park_at_start is False
    widget = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    assert not widget.park.isChecked()

    widget.park.setChecked(True)
    assert cfg.park_at_start is True
    widget.park.setChecked(False)
    assert cfg.park_at_start is False

    # loads the saved value
    cfg.park_at_start = True
    other = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(other)
    assert other.park.isChecked()

    # locked during an MDA (it sits in the settings box), and in legacy mode
    # it stays usable: parking is independent of the All Rows timing
    assert widget.park.isEnabled()
    mmcore.mda.events.sequenceStarted.emit(None, {})
    assert not widget.park.isEnabled()
    mmcore.mda.events.sequenceFinished.emit(None)
    assert widget.park.isEnabled()
