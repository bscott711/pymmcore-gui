from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from pymmcore_gui._qt.QtWidgets import QMessageBox
from pymmcore_gui.widgets import _spectral_channel_config as scc
from pymmcore_gui.widgets._spectral_channel_config import SpectralChannelConfigWidget

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

SENSOR = (3200, 3200)


@pytest.fixture
def widget(
    qtbot: QtBot, mmcore: CMMCorePlus, monkeypatch: pytest.MonkeyPatch
) -> SpectralChannelConfigWidget:
    monkeypatch.setattr(scc, "sensor_size", lambda _mmc, _label: SENSOR)
    w = SpectralChannelConfigWidget(mmcore=mmcore)
    qtbot.addWidget(w)
    return w


def test_center_on_chip(widget: SpectralChannelConfigWidget) -> None:
    widget.width_spin.setValue(1536)
    widget.height_spin.setValue(540)
    widget.gap_spin.setValue(40)
    widget.center_button.click()
    for r in widget._rows:
        top = r.position_combo.currentText() == "top"
        assert r.position == (832, 1040 if top else 1620)
    assert widget._synced


def test_clamp_rows_to_sensor(widget: SpectralChannelConfigWidget) -> None:
    widget.width_spin.setValue(1536)
    widget.height_spin.setValue(540)
    row = widget._rows[0]
    row.set_position(5000, 4000)
    widget._clamp_row(row)
    assert row.position == (3200 - 1536, 3200 - 540)


def test_sibling_sync_uses_gap_and_clamps(widget: SpectralChannelConfigWidget) -> None:
    widget.gap_spin.setValue(100)
    top = next(r for r in widget._rows if r.position_combo.currentText() == "top")
    widget.width_spin.setValue(1536)
    widget.height_spin.setValue(540)
    widget._on_rect_drawn(top, None, (832, 1000, 1536, 540))  # type: ignore[arg-type]
    bottom = next(r for r in widget._rows if r.position_combo.currentText() == "bottom")
    assert bottom.position == (832, 1000 + 540 + 100)


def test_center_with_oversized_region_warns(
    widget: SpectralChannelConfigWidget, monkeypatch: pytest.MonkeyPatch
) -> None:
    warned: list[str] = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *a, **k: warned.append(a[2]))
    widget.width_spin.setValue(1536)
    widget.height_spin.setValue(1700)  # 2*1700 + 40 > 3200
    widget.center_button.click()
    assert warned
