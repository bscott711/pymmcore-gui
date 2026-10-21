from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.widgets._spim_timing_widget import SpimTimingWidget

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot


def test_widget_previews_and_saves_timing(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    widget = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(widget)

    # defaults reproduce the original timing
    assert widget.timing().slice_period_ms == 35.75
    assert "legacy" in widget.summary.text()

    widget.mode.setCurrentIndex(widget.mode.findData("all_rows"))
    widget.readout_auto.setChecked(False)
    widget.readout.setValue(6.0)
    widget.settle.setValue(2.0)
    widget.preview_exposure.setValue(10.0)

    assert widget.timing().slice_period_ms == 22.5
    assert "Expose Out cabled" in widget.summary.text()

    widget._save()
    cfg = SettingsV1.instance().spim_timing
    assert cfg.mode == "all_rows"
    assert cfg.galvo_settle_ms == 2.0
    assert cfg.readout_ms_override == 6.0


def test_widget_locks_save_during_mda(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    widget = SpimTimingWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    mmcore.mda.events.sequenceStarted.emit(None, {})
    assert not widget.save_button.isEnabled()
    mmcore.mda.events.sequenceFinished.emit(None)
    assert widget.save_button.isEnabled()
