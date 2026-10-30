"""CRISPy's panels as the GUI builds them: the Spot lock section and its settings.

Builds the panels through :class:`CrispWidget` on a small fake core with one CRISP axis
on a piezo and one on a stage. Run with ``PYTEST_QT_API=pyqt6`` and
``QT_QPA_PLATFORM=offscreen``.
"""

from __future__ import annotations

import sys
import types
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock, patch

import pytest
from CRISPy.spot.store import get_settings_store, set_settings_store
from CRISPy.ui import CrispControlPanel
from pymmcore_plus import DeviceType

from pymmcore_gui import _settings
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.widgets._crisp_widget import CrispWidget, SpotSettingsAdapter

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pytestqt.qtbot import QtBot

#: label -> (axis letter, hex address, paired stage label, stage adapter name)
AXES = {
    "CRISPAFocus:Z:32": ("Z", "32", "ZStage:Z:32", "ZStage:Z:32"),
    "CRISPAFocus:P:34": ("P", "34", "PiezoStage:P:34", "PiezoStage:P:34"),
}


class FakeCore:
    """The few CMMCorePlus calls CRISPy's discovery, controller and panels make."""

    def __init__(self) -> None:
        self.events = MagicMock()
        self.mda = MagicMock()
        self.stages = {stage: name for _, _, stage, name in AXES.values()}

    def getLoadedDevices(self) -> list[str]:
        return [*AXES, *self.stages]

    def getLoadedDevicesOfType(self, device_type: DeviceType) -> list[str]:
        return list(self.stages)

    def getDeviceName(self, label: str) -> str:
        return self.stages.get(label, label)

    def getDeviceLibrary(self, label: str) -> str:
        return "ASITiger"

    def getDeviceType(self, label: str) -> DeviceType:
        return DeviceType.AutoFocusDevice if label in AXES else DeviceType.StageDevice

    def hasProperty(self, label: str, prop: str) -> bool:
        return False

    def getProperty(self, label: str, prop: str) -> str:
        if label in AXES:
            axis, addr, _, _ = AXES[label]
            props = {"AxisLetter": axis, "TigerHexAddress": addr, "CRISP State": "Idle"}
            return props[prop]
        raise KeyError(prop)  # the callers suppress read errors

    def setProperty(self, label: str, prop: str, value: Any) -> None:
        pass

    def getPosition(self, label: str) -> float:
        return 0.0

    def waitForDevice(self, label: str) -> None:
        pass


class StubPoller:
    """Stands in for pymmcore-widgets' ``AsyncPoller`` where the installed one lacks it.

    The panels import it lazily; it only schedules telemetry reads, which these
    tests do not need.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        pass

    def poll_now(self, force: bool = False) -> None:
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_interval(self, ms: int) -> None:
        pass


@pytest.fixture(autouse=True)
def _async_poller(monkeypatch: pytest.MonkeyPatch) -> None:
    try:
        import pymmcore_widgets.control._async_poller  # noqa: F401
    except ImportError:
        module = types.ModuleType("pymmcore_widgets.control._async_poller")
        module.AsyncPoller = StubPoller  # type: ignore[attr-defined]
        monkeypatch.setitem(sys.modules, module.__name__, module)


@pytest.fixture
def settings() -> Iterator[SettingsV1]:
    """Fresh GUI settings (no disk writes), swapped in as the global."""
    fresh = SettingsV1()
    with patch.object(_settings, "_GLOBAL_SETTINGS", fresh):
        yield fresh


@pytest.fixture
def crisp(
    qtbot: QtBot, settings: SettingsV1, monkeypatch: pytest.MonkeyPatch
) -> Iterator[CrispWidget]:
    """A ``CrispWidget`` with both CRISP panels built on the fake core."""
    core = FakeCore()
    fake_cls = MagicMock()
    fake_cls.instance.return_value = core
    monkeypatch.setattr("CRISPy.plugin.CMMCorePlus", fake_cls)
    widget = CrispWidget(mmcore=core)  # type: ignore[arg-type]
    qtbot.addWidget(widget)
    yield widget
    widget.close()
    widget.deleteLater()
    CrispWidget._set_spot_store(None)
    set_settings_store(None)


def _panels(widget: CrispWidget) -> dict[str, CrispControlPanel]:
    found = widget.findChildren(CrispControlPanel)
    assert len(found) == 2
    return {p.controller.actuator_type: p for p in found}


def test_only_the_piezo_panel_has_the_spot_lock_section(crisp: CrispWidget) -> None:
    panels = _panels(crisp)
    piezo, stage = panels["piezo"], panels["stage"]
    assert piezo.spot_section is not None and piezo.spot_panel is not None
    assert stage.spot_section is None and stage.spot_panel is None


def test_the_adapter_store_is_installed_and_live(
    crisp: CrispWidget, settings: SettingsV1
) -> None:
    store = get_settings_store()
    assert isinstance(store, SpotSettingsAdapter)
    assert store.settings is settings.spot_focus
    assert store.piezo_label == "PiezoStage:P:34"
    assert store.crisp_label == settings.focus.crisp_label
    assert store.data_dir is None  # while testing
    # the property follows a swapped global, never a captured object
    other = SettingsV1()
    with patch.object(_settings, "_GLOBAL_SETTINGS", other):
        assert store.settings is other.spot_focus
    # an edit made through the store is an edit of the GUI's setting
    store.settings.exposure_ms = 9.0
    assert settings.spot_focus.exposure_ms == 9.0


def test_save_flushes_the_gui_settings(
    crisp: CrispWidget, settings: SettingsV1
) -> None:
    with patch.object(SettingsV1, "flush") as flush:
        get_settings_store().save()
    flush.assert_called_once_with()


def test_destroying_the_widget_clears_the_store(
    crisp: CrispWidget,
) -> None:
    assert isinstance(get_settings_store(), SpotSettingsAdapter)
    crisp._disconnect()
    assert not isinstance(get_settings_store(), SpotSettingsAdapter)


def test_the_led_stays_off_limits_while_crisp_locks_across_a_tool(
    crisp: CrispWidget,
) -> None:
    """With CRISP in Lock, a tool beginning and ending must not re-enable the LED."""
    stage = _panels(crisp)["stage"]
    stage._last_state = "Lock"
    stage._sync_tool_controls()
    assert not stage.btn_led.isEnabled()
    assert stage._begin_tool("dither")
    assert not stage.btn_led.isEnabled()
    stage._end_tool("dither")
    assert not stage.btn_led.isEnabled()
    stage._last_state = "Idle"
    stage._sync_tool_controls()
    assert stage.btn_led.isEnabled()  # control: it is the Lock state that gates it
