"""The stage panel: no "Zero" (set origin) for piezo stages."""

from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import MagicMock

import pytest
import useq
from pymmcore_plus import CMMCorePlus, DeviceType

from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.widgets import _galvo_control
from pymmcore_gui.widgets._galvo_control import OFFSET_PROP, GalvoControlWidget
from pymmcore_gui.widgets._stage_control import (
    StagesControlWidget,
    _Group,
    is_piezo,
)

if TYPE_CHECKING:
    from pytestqt.qtbot import QtBot


def _core(name: str, description: str) -> MagicMock:
    mmc = MagicMock()
    mmc.getDeviceName.return_value = name
    mmc.getDeviceDescription.return_value = description
    return mmc


def test_an_asi_piezo_is_a_piezo() -> None:
    assert is_piezo(_core("PiezoStage:P:34", "ASI Piezo Stage"), "PiezoStage:P:34")


def test_an_asi_z_stage_is_not() -> None:
    assert not is_piezo(_core("ZStage:Z:32", "ASI Z Stage"), "ZStage:Z:32")


def test_an_unreadable_device_is_not() -> None:
    mmc = MagicMock()
    mmc.getDeviceName.side_effect = RuntimeError("no such device")
    assert not is_piezo(mmc, "gone")


def test_the_panel_hides_zero_only_for_piezos(qtbot: QtBot) -> None:
    core = CMMCorePlus()
    core.loadSystemConfiguration()  # the demo config: XY and Z stages, no piezo
    widget = StagesControlWidget(mmcore=core)
    qtbot.addWidget(widget)
    from pymmcore_widgets import StageWidget

    stages = widget.findChildren(StageWidget)
    assert stages
    if not hasattr(stages[0], "enable_set_origin"):
        return  # an older pymmcore-widgets: nothing to hide
    assert all(not s._origin_btn.isHidden() for s in stages)

    # The same panel when the demo Z stage reports itself as a piezo.
    real_description = core.getDeviceDescription
    core.getDeviceDescription = lambda label: (  # type: ignore[method-assign]
        "A Piezo Stage" if label == "Z" else real_description(label)
    )
    widget._on_cfg_loaded()
    hidden = {
        s._device: s._origin_btn.isHidden() for s in widget.findChildren(StageWidget)
    }
    assert hidden["Z"] is True
    assert hidden["XY"] is False


# ---------------------------------------------------------------------------
# Galvo control
# ---------------------------------------------------------------------------

GALVO = "Scanner:AB:33"
SLOPE = HardwareConstants().slice_calibration_slope_um_per_deg


class _FakeGalvo:
    """Property state behind the patched core and ``set_property``."""

    def __init__(self, offset: float = 0.0, limits: tuple[float, float] = (-2.0, 2.0)):
        self.offset = offset
        self.limits = limits
        self.writes: list[float] = []

    def write(self, device: str, prop: str, value: str) -> None:
        assert (device, prop) == (GALVO, OFFSET_PROP)
        self.offset = float(value)
        self.writes.append(float(value))


@pytest.fixture
def galvo(monkeypatch: pytest.MonkeyPatch) -> tuple[CMMCorePlus, _FakeGalvo]:
    """A bare core that pretends to hold a galvo with limits of +/-2 deg."""
    core = CMMCorePlus()
    state = _FakeGalvo()
    real_devices = core.getLoadedDevices
    real_of_type = core.getLoadedDevicesOfType
    monkeypatch.setattr(core, "getLoadedDevices", lambda: (*real_devices(), GALVO))
    monkeypatch.setattr(
        core,
        "getLoadedDevicesOfType",
        lambda t: (GALVO,) if t == DeviceType.Galvo else real_of_type(t),
    )
    monkeypatch.setattr(core, "getProperty", lambda d, p: str(state.offset))
    monkeypatch.setattr(core, "hasPropertyLimits", lambda d, p: True)
    monkeypatch.setattr(core, "getPropertyLowerLimit", lambda d, p: state.limits[0])
    monkeypatch.setattr(core, "getPropertyUpperLimit", lambda d, p: state.limits[1])
    monkeypatch.setattr(_galvo_control, "set_property", state.write)
    return core, state


def test_a_loaded_galvo_gets_a_group(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, _ = galvo
    widget = StagesControlWidget(mmcore=core)
    qtbot.addWidget(widget)
    controls = widget.findChildren(GalvoControlWidget)
    assert len(controls) == 1
    assert controls[0]._device == GALVO
    groups = [g for g in widget.findChildren(_Group) if g.title() == GALVO]
    assert len(groups) == 1
    assert widget._layout.getItemPosition(0)[:2] == (0, 0)


def test_galvo_groups_fill_the_grid_two_per_row(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo], monkeypatch: pytest.MonkeyPatch
) -> None:
    core, _ = galvo
    monkeypatch.setattr(
        core,
        "getLoadedDevicesOfType",
        lambda t: ("G1", "G2", "G3") if t == DeviceType.Galvo else (),
    )
    widget = StagesControlWidget(mmcore=core)
    qtbot.addWidget(widget)
    places = [widget._layout.getItemPosition(i)[:2] for i in range(3)]
    assert places == [(0, 0), (0, 1), (1, 0)]


def test_the_hardware_label_is_used_when_not_typed_as_galvo(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo], monkeypatch: pytest.MonkeyPatch
) -> None:
    core, _ = galvo
    real_of_type = core.getLoadedDevicesOfType
    monkeypatch.setattr(
        core,
        "getLoadedDevicesOfType",
        lambda t: () if t == DeviceType.Galvo else real_of_type(t),
    )
    widget = StagesControlWidget(mmcore=core)
    qtbot.addWidget(widget)
    assert [w._device for w in widget.findChildren(GalvoControlWidget)] == [GALVO]


def test_no_galvo_no_group(qtbot: QtBot) -> None:
    core = CMMCorePlus()  # nothing loaded, so no galvo
    widget = StagesControlWidget(mmcore=core)
    qtbot.addWidget(widget)
    assert not widget.findChildren(GalvoControlWidget)
    assert widget._layout.count() == 0


def test_spin_converts_degrees_and_microns(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    state.offset = 0.25
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    assert w.offset.value() == pytest.approx(0.25 * SLOPE)
    assert "0.2500" in w.degrees.text()

    w.offset.setValue(-80.0)  # um -> deg
    assert state.writes == [pytest.approx(-80.0 / SLOPE)]
    assert "-0.8000" in w.degrees.text()


def test_writes_are_clamped_to_the_limits(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    w.set_offset_deg(5.0)
    assert state.writes == [2.0]
    # Negative control: the out-of-range value must never reach the card.
    assert 5.0 not in state.writes
    w.set_offset_deg(-9.0)
    assert state.writes[-1] == -2.0
    assert -9.0 not in state.writes


def test_buttons_step_and_zero(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    w.step.setValue(10.0)  # um
    w.plus.click()
    assert state.offset == pytest.approx(10.0 / SLOPE)
    w.minus.click()
    w.minus.click()
    assert state.offset == pytest.approx(-10.0 / SLOPE)
    w.zero.click()
    assert state.offset == 0.0


def test_no_write_when_unchanged(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    state.offset = 0.5
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    w.set_offset_deg(0.5)
    w.zero.click()
    w.zero.click()  # already at 0 the second time
    assert state.writes == [0.0]
    # A refresh from the card (propertyChanged) must not write back.
    state.offset = 1.0
    core.events.propertyChanged.emit(GALVO, OFFSET_PROP, "1.0")
    assert state.writes == [0.0]
    assert w.offset.value() == pytest.approx(1.0 * SLOPE)
    # Other devices/properties are ignored.
    state.offset = 1.5
    core.events.propertyChanged.emit("Other", OFFSET_PROP, "1.5")
    assert w.offset.value() == pytest.approx(1.0 * SLOPE)


def test_locked_while_an_mda_runs(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, _ = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    assert w.isEnabled()
    seq = useq.MDASequence()
    core.mda.events.sequenceStarted.emit(seq, {})
    assert not w.isEnabled()
    core.mda.events.sequenceFinished.emit(seq)
    assert w.isEnabled()
