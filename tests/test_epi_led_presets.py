"""The ``EPI LEDs`` presets survive pymmcore-widgets' ``PresetsWidget``.

Regression test for the first design (one preset per LED, each setting a different
device): ``PresetsWidget`` deletes rows whose (device, property) is not in the first
preset, and ``getCurrentConfig`` returns the first matching preset. The presets now
all set ``Physical Shutter 1`` of a ``Multi Shutter`` selector, which the demo core
can reproduce with demo shutters standing in for the LEDs. No hardware is used.
"""

from __future__ import annotations

import warnings
from typing import TYPE_CHECKING

import pytest
from pymmcore_widgets import PresetsWidget

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

GROUP = "EPI LEDs"
SELECTOR = "EPI LED Selector"
PROP = "Physical Shutter 1"
STAND_INS = {"LED488": "LED:L:37:1", "LED561": "LED:L:37:2", "LED642": "LED:L:37:3"}


@pytest.fixture
def epi_group(mmcore: CMMCorePlus) -> CMMCorePlus:
    try:
        for label in STAND_INS.values():  # demo shutters stand in for the LEDs
            mmcore.loadDevice(label, "DemoCamera", "DShutter")
            mmcore.initializeDevice(label)
        mmcore.loadDevice(SELECTOR, "Utilities", "Multi Shutter")
        mmcore.initializeDevice(SELECTOR)
    except Exception as e:  # pragma: no cover - depends on the test environment
        pytest.skip(f"Utilities Multi Shutter adapter is not loadable here: {e}")
    allowed = mmcore.getAllowedPropertyValues(SELECTOR, PROP)
    if not set(STAND_INS.values()) <= set(allowed):  # pragma: no cover
        pytest.skip(f"Multi Shutter does not offer the stand-in shutters: {allowed}")
    for preset, label in STAND_INS.items():
        mmcore.defineConfig(GROUP, preset, SELECTOR, PROP, label)
    return mmcore


def test_presets_widget_keeps_all_three_selector_presets(
    qtbot: QtBot, epi_group: CMMCorePlus
) -> None:
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        widget = PresetsWidget(GROUP)
        qtbot.addWidget(widget)
    assert [str(w.message) for w in caught] == []
    assert list(epi_group.getAvailableConfigs(GROUP)) == list(STAND_INS)
    for preset, label in STAND_INS.items():
        assert [tuple(r) for r in epi_group.getConfigData(GROUP, preset)] == [
            (SELECTOR, PROP, label)
        ]


def test_each_preset_reads_back_as_itself_and_lights_nothing(
    qtbot: QtBot, epi_group: CMMCorePlus
) -> None:
    widget = PresetsWidget(GROUP)
    qtbot.addWidget(widget)
    for preset, label in STAND_INS.items():
        epi_group.setConfig(GROUP, preset)
        assert epi_group.getCurrentConfig(GROUP) == preset
        assert epi_group.getProperty(SELECTOR, PROP) == label
        # selecting a preset opens none of the shutters it names
        assert not any(epi_group.getShutterOpen(lb) for lb in STAND_INS.values())
