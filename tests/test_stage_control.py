"""The stage panel: no "Zero" (set origin) for piezo stages."""

from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import MagicMock

import pytest
import useq
from pymmcore_plus import CMMCorePlus, DeviceType

from pymmcore_gui.asi_z_stack.common import HardwareConstants
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
    """Property state behind the patched core."""

    def __init__(self, offset: float = 0.0, limits: tuple[float, float] = (-2.0, 2.0)):
        self.offset = offset
        self.limits = limits
        self.writes: list[float] = []
        # The mirror: setGalvoPosition moves it, getGalvoPosition reads it.
        self.position = (0.0, 0.0)
        self.moves: list[tuple[float, float]] = []
        self.reads = 0
        self.moves_ignored = False  # beam off / single-axis mode on

    def write(self, device: str, prop: str, value: str) -> None:
        assert (device, prop) == (GALVO, OFFSET_PROP)
        self.offset = float(value)
        self.writes.append(float(value))

    def move(self, device: str, x: float, y: float) -> None:
        assert device == GALVO
        self.moves.append((x, y))
        if not self.moves_ignored:
            self.position = (x, y)

    def read(self, device: str) -> tuple[float, float]:
        assert device == GALVO
        self.reads += 1
        return self.position


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
    monkeypatch.setattr(core, "hasProperty", lambda d, p: True)
    monkeypatch.setattr(core, "setProperty", state.write)
    monkeypatch.setattr(core, "setGalvoPosition", state.move)
    monkeypatch.setattr(core, "getGalvoPosition", state.read)
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


def test_spin_write_moves_the_mirror_and_keeps_x(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    state.position = (1.5, 0.0)
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    w.offset.setValue(-80.0)  # um -> deg
    assert state.writes == [pytest.approx(-80.0 / SLOPE)]
    assert state.moves == [(1.5, pytest.approx(-80.0 / SLOPE))]
    assert w.mirror.text() == f"mirror {-80.0 / SLOPE:.4f} deg"


def test_go_to_zero_moves_the_mirror_even_if_the_property_is_unchanged(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    state.offset = 0.0
    state.position = (0.0, 0.7)  # property says 0, mirror is elsewhere
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    assert w.mirror.text() == "mirror 0.7000 deg"
    assert w.mirror.toolTip().startswith("The mirror is not at the offset")
    w.zero.click()
    assert state.writes == []  # redundant property write skipped
    assert state.moves == [(0.0, 0.0)]  # but the mirror is re-asserted
    assert w.mirror.text() == "mirror 0.0000 deg"
    assert not w.mirror.toolTip().startswith("The mirror is not at the offset")


def test_refresh_does_not_read_the_mirror_during_an_mda(
    qtbot: QtBot,
    galvo: tuple[CMMCorePlus, _FakeGalvo],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, state = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    monkeypatch.setattr(core.mda, "is_running", lambda: True)
    state.reads = 0
    w.refresh()
    core.events.propertyChanged.emit(GALVO, OFFSET_PROP, "0.0")
    assert state.reads == 0
    # Negative control: once the run is over, refresh reads the mirror again.
    monkeypatch.setattr(core.mda, "is_running", lambda: False)
    w.refresh()
    assert state.reads == 1


def test_sequence_finished_refreshes_the_mirror_label(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    seq = useq.MDASequence()
    core.mda.events.sequenceStarted.emit(seq, {})
    state.position = (0.0, 0.3)  # the engine re-moved the mirror at teardown
    state.offset = 0.3
    core.mda.events.sequenceFinished.emit(seq)
    assert w.isEnabled()
    assert w.mirror.text() == "mirror 0.3000 deg"


def test_mirror_label_flags_a_move_the_card_ignored(
    qtbot: QtBot, galvo: tuple[CMMCorePlus, _FakeGalvo]
) -> None:
    core, state = galvo
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    state.moves_ignored = True
    w.set_offset_deg(0.5)
    assert state.moves == [(0.0, 0.5)]
    assert w.mirror.text() == "mirror 0.0000 deg"
    assert "not at the offset" in w.mirror.toolTip()
    assert "single-axis" in w.mirror.toolTip()


def test_mirror_label_when_unreadable(
    qtbot: QtBot,
    galvo: tuple[CMMCorePlus, _FakeGalvo],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    core, _ = galvo

    def _boom(device: str) -> tuple[float, float]:
        raise RuntimeError("no card")

    monkeypatch.setattr(core, "getGalvoPosition", _boom)
    w = GalvoControlWidget(GALVO, core)
    qtbot.addWidget(w)
    assert w.mirror.text() == "mirror ?"
    w.set_offset_deg(0.5)  # must not raise
    assert w.mirror.text() == "mirror ?"
