"""The spot focus lock's settings and panel, with a real spawned worker.

A ``FakeSpotSource`` stands in for the camera and a fake mmc records every
``setPosition``. The fake LED patch is centred at (320, 240) and is 96 px wide, so
its left edge is at x = 272 and moves along x by ``SLOPE`` px per um of P; the
edge box ``BOX`` lies across that edge.

Run WITHOUT ``--noconftest`` and with ``QT_QPA_PLATFORM=offscreen``: with both
PyQt6 and PySide6 installed, pytest-qt otherwise picks PySide6 and
``qtbot.addWidget`` fails ("Need to pass a QWidget").
"""

from __future__ import annotations

import functools
import os
import re
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar
from unittest.mock import PropertyMock, patch

import pytest
from CRISPy.spot.panel import (
    ALERT_COLOR,
    ATTENTION_COLOR,
    NEED_BOX_TEXT,
    OK_COLOR,
    SpotFocusPanel,
    box_warnings,
    noise_color,
    source_spec,
)
from CRISPy.spot.spot_focus import (
    EDGE_DETECTOR_LABELS,
    EDGE_DETECTORS,
    SIGMA_PX_RANGE,
    EdgeCalibration,
    FocusCalibration,
    measure_edge,
)
from CRISPy.spot.spot_focus_service import SpotFocusService
from CRISPy.spot.store import set_settings_store
from CRISPy.spot.toupcam_source import make_source

from pymmcore_gui import _settings
from pymmcore_gui._qt.QtCore import QPointF, Qt
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.widgets._crisp_widget import SpotSettingsAdapter

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pytestqt.qtbot import QtBot

P_LABEL = "PiezoStage:P:34"
CRISP = "CRISP"
P0 = 50.0
SLOPE = 5.0  # the fake's true image motion along x, px per um of P
#: What the fake camera is built from (its true image motion).
CAL_LIKE = {
    "x0": 320.0,
    "y0": 240.0,
    "ux": 1.0,
    "uy": 0.0,
    "slope_px_per_um": SLOPE,
    "p0_um": P0,
}
#: Across the left edge of the fake patch; clear of its corners and the ghost.
BOX = (232, 210, 80, 60)
#: Across the top edge (y = 194); the rig-shaped cases measure this one.
TOP_BOX = (288, 154, 64, 80)
#: What the rig's saved calibration looked like (a single "edge" dict, P negative).
RIG_P = -45.5
RIG_SLOPE = 1.919
WAIT_MS = 30_000  # a spawned worker needs a few seconds to import


class FakeCrisp:
    """A ``CrispController`` stand-in that records the LED and joystick calls."""

    label = CRISP

    def __init__(self) -> None:
        self.calls: list[tuple[str, bool]] = []

    @property
    def leds(self) -> list[bool]:
        return [v for k, v in self.calls if k == "led"]

    @property
    def joystick(self) -> list[bool]:
        return [v for k, v in self.calls if k == "joystick"]

    def set_led(self, on: bool) -> None:
        self.calls.append(("led", on))

    def set_joystick_enabled(self, enabled: bool) -> None:
        self.calls.append(("joystick", enabled))


class FakeMMC:
    """Records setPosition; never waits; remembers properties."""

    def __init__(self) -> None:
        self.pos = {P_LABEL: P0}
        self.props = {(CRISP, "CRISP State"): "Idle", (CRISP, "LED Intensity"): "0"}
        self.crisp = FakeCrisp()
        self.sets: list[float] = []
        self._lock = threading.Lock()

    def getLoadedDevices(self) -> list[str]:
        return [P_LABEL, CRISP]

    def setPosition(self, label: str, value: float) -> None:
        time.sleep(0.002)
        with self._lock:
            self.pos[label] = float(value)
            self.sets.append(float(value))

    def getPosition(self, label: str) -> float:
        return self.pos[label]

    def waitForDevice(self, label: str) -> None:
        pass

    def getProperty(self, label: str, prop: str) -> str:
        return str(self.props[(label, prop)])

    def setProperty(self, label: str, prop: str, value: object) -> None:
        self.props[(label, prop)] = str(value)


@pytest.fixture(autouse=True)
def _isolated_settings() -> Iterator[SettingsV1]:
    """Fresh default settings pointing at a fake camera (no disk writes)."""
    fresh = SettingsV1()
    fresh.spot_focus.source.kind = "fake"
    fresh.spot_focus.source.fake = {
        "cal_like": dict(CAL_LIKE),
        "noise": 2.0,
        "fps": 50.0,
    }
    fresh.spot_focus.params.avg_frames = 4
    fresh.spot_focus.params.max_rate_hz = 5.0
    fresh.spot_focus.params.strikes = 3
    with patch("pymmcore_gui._settings._GLOBAL_SETTINGS", fresh):
        # the panel reads and saves through CRISPy's store, here the GUI's adapter
        set_settings_store(SpotSettingsAdapter())
        try:
            yield fresh
        finally:
            set_settings_store(None)


@pytest.fixture(autouse=True)
def _child_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make sure the spawned child can import the package under test."""
    import CRISPy

    src = str(Path(CRISPy.__file__).resolve().parent.parent)
    old = os.environ.get("PYTHONPATH", "")
    monkeypatch.setenv("PYTHONPATH", src + (os.pathsep + old if old else ""))


@pytest.fixture
def mmc() -> FakeMMC:
    return FakeMMC()


def _factory(mmc: FakeMMC):
    def make(spec: dict, led: float | None) -> SpotFocusService:
        return SpotFocusService(
            mmc,
            P_LABEL,
            CRISP,
            spec,
            led_intensity=led,
            ready_timeout_s=25.0,
            crisp=mmc.crisp,  # type: ignore[arg-type]
        )

    return make


@pytest.fixture
def panel(qtbot: QtBot, mmc: FakeMMC) -> Iterator[SpotFocusPanel]:
    widget = SpotFocusPanel(mmc, service_factory=_factory(mmc))
    qtbot.addWidget(widget)
    widget.show()  # the docked live view only polls while the panel is visible
    yield widget
    widget.shutdown()


@functools.cache
def _e0() -> float:
    """The edge position the fake shows at P0, as the metric measures it."""
    src = make_source(
        {"kind": "fake", "cal_like": dict(CAL_LIKE), "noise": 2.0, "fps": 50.0}
    )
    fixes = [measure_edge(src.render(0.0), BOX) for _ in range(5)]  # type: ignore[attr-defined]
    assert all(f.ok for f in fixes)
    return float(sum(f.position for f in fixes) / len(fixes))


def _edge_cal() -> EdgeCalibration:
    """A saved edge calibration that matches the fake camera."""
    return EdgeCalibration(
        box=BOX,
        axis="x",
        polarity=1,
        slope_px_per_um=SLOPE,
        e0_px=_e0(),
        p0_um=P0,
        r2=0.999,
        residual_px=0.05,
        linear_range_um=1.0,
        noise_px=0.02,
        angle_deg=0.0,
        contrast=700.0,
        n_points=9,
        created="2026-10-28T00:00:00",
        # _e0() is measured with measure_edge's default detector.
        detector="gaussderiv",
        detector_sigma_px=6.0,
    )


def _focus_cal() -> FocusCalibration:
    """The same calibration as a (one-edge) ``FocusCalibration``."""
    return FocusCalibration(edge=_edge_cal(), p0_um=P0, created="2026-10-28T00:00:00")


def _calibrated(settings: SettingsV1) -> None:
    settings.spot_focus.calibration = _focus_cal().to_dict()
    settings.spot_focus.edge_box = BOX


def _use_diagonal_motion(settings: SettingsV1, sign: float = 1.0) -> None:
    """Make the fake patch move diagonally: 0.8 along x and 0.6 along y per um."""
    fake = dict(settings.spot_focus.source.fake)
    fake["cal_like"] = {
        **CAL_LIKE,
        "ux": 0.8,
        "uy": 0.6,
        "slope_px_per_um": sign * SLOPE,
    }
    settings.spot_focus.source.fake = fake


def _rig_cal_like() -> dict:
    """A fake whose top edge moves along y by the rig's 1.919 px/um, P near -45.5."""
    return {
        "x0": 320.0,
        "y0": 240.0,
        "ux": 0.0,
        "uy": 1.0,
        "slope_px_per_um": RIG_SLOPE,
        "p0_um": RIG_P,
    }


def _rig_saved_edge(e0: float) -> dict:
    """The single "edge" dict an older version saved (no "edges" key)."""
    return {
        "kind": "edge",
        "box": list(TOP_BOX),
        "axis": "y",
        "polarity": 1,
        "slope_px_per_um": RIG_SLOPE,
        "e0_px": e0,
        "p0_um": -45.497,
        "r2": 0.993,
        "residual_px": 0.05,
        "linear_range_um": 1.0,
        "noise_px": 0.02,
        "angle_deg": 0.0,
        "contrast": 700.0,
        "n_points": 9,
        "sweep": [],
        "warnings": [],
        "record_path": "",
        "created": "2026-10-28T13:11:49",
    }


def _rig_e0() -> float:
    """Where the rig-shaped fake shows its top edge at P = -45.5."""
    src = make_source(
        {"kind": "fake", "cal_like": _rig_cal_like(), "noise": 2.0, "fps": 50.0}
    )
    # An older version's saved calibration (no "detector" key) is a half-max one.
    fixes = [
        measure_edge(src.render(0.0), TOP_BOX, detector="halfmax")  # type: ignore[attr-defined]
        for _ in range(5)
    ]
    assert all(f.ok for f in fixes)
    return float(sum(f.position for f in fixes) / len(fixes))


# ---------------------------------------------------------------------------
# settings
# ---------------------------------------------------------------------------


def test_spot_settings_defaults_and_roundtrip() -> None:
    s = SettingsV1()
    cfg = s.spot_focus
    assert (cfg.channel, cfg.source.kind) == ("red", "toupcam")
    assert cfg.calibration is None
    assert cfg.to_calibration() is None
    assert cfg.calibration_status()[0] is None
    assert cfg.edge_box is None
    assert not hasattr(cfg, "edge_box_2")
    assert (cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames) == (2.0, 9, 3)
    params = cfg.params.to_params()
    assert not hasattr(params, "max_edge_disagreement_um")
    assert not hasattr(params, "hold_band_um")
    assert params.deadband_um == pytest.approx(0.03)
    assert params.avg_frames == 4
    assert params.offset_um == 0.0
    assert not hasattr(params, "lateral_limit_px")

    cfg.channel = "sum"
    cfg.roi = (10, 20, 300, 200)
    cfg.led_intensity = 40.0
    cfg.params.kp = 0.3
    cfg.params.offset_um = 0.25
    cfg.params.min_contrast = 25.0
    cfg.calibration = _focus_cal().to_dict()
    cfg.edge_box = BOX
    cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames = 1.5, 7, 2
    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    assert s2.spot_focus == s.spot_focus
    assert s2.spot_focus.roi == (10, 20, 300, 200)
    assert s2.spot_focus.edge_box == BOX
    assert (s2.spot_focus.sweep_span_um, s2.spot_focus.sweep_steps) == (1.5, 7)
    cal = s2.spot_focus.to_calibration()
    assert isinstance(cal, FocusCalibration)
    assert cal.edge.slope_px_per_um == SLOPE
    assert s2.spot_focus.calibration_status()[1] == ""
    p2 = s2.spot_focus.params.to_params()
    assert (p2.kp, p2.offset_um, p2.min_contrast) == (0.3, 0.25, 25.0)


def test_old_settings_with_edge_box_2_and_two_edges_still_load() -> None:
    """A file from the two-box version loads: box 2 and the fusion limits go."""
    old = SettingsV1().model_dump(mode="json")
    sf = old["spot_focus"]
    sf["edge_box"] = list(BOX)
    sf["edge_box_2"] = list(TOP_BOX)
    sf["params"].update(max_edge_disagreement_um=0.45, hold_band_um=0.25, kp=0.3)
    two = {
        "kind": "edges",
        "edges": [_edge_cal().to_dict(), _rig_saved_edge(194.0)],
        "p0_um": P0,
        "created": "2026-10-28T00:00:00",
    }
    sf["calibration"] = two
    # the GUI's loader warns about each key the model no longer has, then drops it
    with pytest.warns(RuntimeWarning, match="not found in model"):
        cleaned = _settings._good_data_only(SettingsV1, old)
    s = SettingsV1.model_validate(cleaned)
    cfg = s.spot_focus
    assert cfg.edge_box == BOX and cfg.params.kp == 0.3
    assert not hasattr(cfg, "edge_box_2")
    cal, message = cfg.calibration_status()
    assert message == ""
    # the first edge is kept; the second is dropped with a warning
    assert isinstance(cal, FocusCalibration)
    assert cal.edge.box == BOX and cal.box == BOX
    assert any("edge 2 dropped" in w for w in cal.warnings)
    # the removed settings are not written back
    again = s.model_dump(mode="json")["spot_focus"]
    assert "edge_box_2" not in again
    assert "hold_band_um" not in again["params"]
    assert "max_edge_disagreement_um" not in again["params"]


def test_rig_shaped_single_edge_calibration_loads_as_one_edge() -> None:
    """The calibration saved before two-edge fusion keeps working."""
    cfg = SettingsV1().spot_focus
    rig = _rig_saved_edge(194.0)
    rig.update(box=[762, 590, 176, 114], slope_px_per_um=1.919)
    cfg.calibration = rig
    cal, message = cfg.calibration_status()
    assert message == ""
    assert isinstance(cal, FocusCalibration)
    assert cal.edge.box == (762, 590, 176, 114)
    assert cal.edge.slope_px_per_um == pytest.approx(1.919)
    assert cal.p0_um == pytest.approx(-45.497)
    assert cal.created == "2026-10-28T13:11:49"
    assert cfg.to_calibration() == cal
    # and it survives a settings round trip, now in the single-edge shape
    s2 = SettingsV1.model_validate_json(SettingsV1(spot_focus=cfg).model_dump_json())
    assert s2.spot_focus.to_calibration() == cal


def test_old_calibration_in_settings_is_ignored_not_raised() -> None:
    cfg = SettingsV1().spot_focus
    # The Gaussian-spot calibration this replaced, and plain garbage.
    for stored in (
        {
            "x0": 320.0,
            "y0": 240.0,
            "ux": 0.8,
            "uy": 0.6,
            "slope_px_per_um": 10.0,
            "p0_um": P0,
        },
        {"kind": "edge", "slope_px_per_um": "steep"},
        {"nonsense": 1},
    ):
        cfg.calibration = stored
        cal, message = cfg.calibration_status()
        assert cal is None
        assert "old spot calibration ignored" in message
        assert cfg.to_calibration() is None


def test_old_settings_files_still_load() -> None:
    """A file written before the spot lock (no ``spot_focus``) loads with defaults."""
    old = {"version": "1.0", "focus": {"enabled": True}}
    cleaned = _settings._good_data_only(SettingsV1, old)
    s = SettingsV1.model_validate(cleaned)
    assert s.focus.enabled is True
    assert s.spot_focus.calibration is None
    assert s.spot_focus.channel == "red"
    # A bad spot section degrades to defaults and keeps the rest.
    bad = {"focus": {"enabled": True}, "spot_focus": {"channel": "purple"}}
    with pytest.warns(RuntimeWarning):
        cleaned = _settings._good_data_only(SettingsV1, bad)
    assert cleaned["focus"] == {"enabled": True}
    assert cleaned.get("spot_focus", {}) == {}


def test_source_spec_from_settings() -> None:
    cfg = SettingsV1().spot_focus
    cfg.source.serial = "abc"
    spec = source_spec(cfg)
    assert spec == {
        "kind": "toupcam",
        "serial": "abc",
        "analysis": {
            "channel": "red",
            "bayer": "RGGB",
            "detector": "gaussderiv",
            "detector_sigma_px": 6.0,
        },
    }
    cfg.source.kind = "fake"
    cfg.source.fake = {"cal_like": dict(CAL_LIKE)}
    assert source_spec(cfg)["kind"] == "fake"


# ---------------------------------------------------------------------------
# panel
# ---------------------------------------------------------------------------


def test_buttons_and_readouts_are_the_edge_workflow(panel: SpotFocusPanel) -> None:
    assert panel.calibrate_button.text() == "Calibrate"
    assert "Focus on the cells first" in panel.calibrate_button.toolTip()
    assert panel.zero_button.text() == "Zero here"
    assert panel.start_button.text() == "Lock"
    assert panel.stop_button.text() == "Stop"
    assert panel.draw_edge_button.text() == "Draw edge box"
    assert not hasattr(panel, "modeForced")
    assert list(panel.readout_labels) == [
        "edge",
        "contrast",
        "fps",
        "error",
        "moves",
        "move_ms",
    ]
    for gone in ("draw_edge2_button", "clear_edge2_button", "edge_box2_label"):
        assert not hasattr(panel, gone)
    # Nothing calibrated yet: Lock and Calibrate try (and ask for a box), Zero waits.
    assert panel.start_button.isEnabled() and panel.calibrate_button.isEnabled()
    assert not panel.zero_button.isEnabled() and not panel.stop_button.isEnabled()
    assert "not calibrated" in panel.calibration_label.text()
    assert panel.edge_box_label.text() == "none"
    assert panel.offset_spin.value() == 0.0
    assert (panel.span_spin.value(), panel.steps_spin.value()) == (2.0, 9)
    assert panel.frames_spin.value() == 3
    for field in ("min_contrast", "max_angle_change_deg", "max_edge_residual_px"):
        assert field in panel.param_spins


def test_lock_and_calibrate_need_an_edge_box(panel: SpotFocusPanel) -> None:
    panel.calibrate()
    assert NEED_BOX_TEXT in panel.fault_label.text()
    assert panel._busy == ""
    assert panel.service is None  # lazy: nothing was created
    panel.fault_label.setText("")
    panel.start()
    assert NEED_BOX_TEXT in panel.fault_label.text()
    assert panel.service is None
    assert panel.state == "idle"
    assert panel.start_button.isEnabled()


def test_old_calibration_in_settings_does_not_crash_the_panel(
    qtbot: QtBot, mmc: FakeMMC
) -> None:
    settings = SettingsV1.instance()
    for stored in (
        {
            "x0": 320.0,
            "y0": 240.0,
            "ux": 0.8,
            "uy": 0.6,
            "slope_px_per_um": 10.0,
            "p0_um": P0,
        },
        {"kind": "edge"},
    ):
        settings.spot_focus.calibration = stored
        widget = SpotFocusPanel(mmc, service_factory=_factory(mmc))
        qtbot.addWidget(widget)
        try:
            assert "old spot calibration ignored" in widget.calibration_label.text()
            assert widget._cal is None
            assert not widget.zero_button.isEnabled()
            assert not widget.live_view._setpoint_item.isVisible()
        finally:
            widget.shutdown()


def test_drawing_an_edge_box_saves_it_and_shows_the_overlay(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    cfg = SettingsV1.instance().spot_focus
    view = panel.live_view
    assert not view._edge_box_item.isVisible()
    view.edgeBoxDrawn.emit(*BOX)
    assert cfg.edge_box == BOX
    assert panel.edge_box_label.text() == "232, 210, 80, 60"
    assert view.edge_box == BOX
    assert view._edge_box_item.isVisible()
    r = view._edge_box_item.rect()
    assert (r.x(), r.y(), r.width(), r.height()) == BOX
    assert panel.service is None  # nothing to tell yet; it is sent on creation
    # a fresh refresh (an app restart) shows the saved box again
    view.set_edge_box(None)
    panel.refresh()
    assert view.edge_box == BOX and view._edge_box_item.isVisible()
    panel.set_edge_box(None)
    assert cfg.edge_box is None and not view._edge_box_item.isVisible()
    assert panel.edge_box_label.text() == "none"


def test_stale_box_warning_follows_the_edge_box(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    settings = SettingsV1.instance()
    _calibrated(settings)
    panel.refresh()
    assert "changed since the calibration" not in panel.calibration_label.text()
    panel.set_edge_box((0, 0, 64, 64))
    assert "edge box changed since the calibration" in panel.calibration_label.text()
    panel.set_edge_box(BOX)
    assert "changed since the calibration" not in panel.calibration_label.text()


def test_telemetry_shows_the_edge_the_error_and_the_note(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    labels = panel.readout_labels
    tele = {
        "edge_px": 272.0,
        "contrast": 700.0,
        "fps": 30.0,
        "along_um": 0.05,
        "note": "",
        "reason": "",
    }
    panel._on_telemetry(tele)
    assert labels["edge"].text() == "272.00 px"
    assert labels["error"].text() == "+0.050 µm"
    assert labels["contrast"].text() == "700"
    assert labels["fps"].text() == "30.0"
    assert panel.note_label.text() == ""

    # a note is shown whether or not there is an error
    panel._on_telemetry({**tele, "note": "holding"})
    assert "holding" in panel.note_label.text()
    assert ATTENTION_COLOR in panel.note_label.text()

    # A bad frame without a note shows its reason, and no error.
    bad = {
        "edge_px": 272.0,
        "contrast": 5.0,
        "along_um": None,
        "reason": "edge too weak (contrast 5)",
    }
    panel._on_telemetry(bad)
    assert "edge too weak" in panel.note_label.text()
    assert labels["error"].text() == "—"
    panel._on_telemetry({**bad, "reason": "", "along_um": 0.0})
    assert panel.note_label.text() == ""
    # an old worker's two-edge fields are ignored
    panel._on_telemetry({**tele, "disagreement_um": 0.4, "edges": [{"ok": True}]})
    assert labels["error"].text() == "+0.050 µm"


def test_loop_parameters_have_no_fusion_spins(panel: SpotFocusPanel) -> None:
    assert "max_edge_disagreement_um" not in panel.param_spins
    assert "hold_band_um" not in panel.param_spins
    assert "deadband_um" in panel.param_spins


def test_preview_error_appears_after_calibrate(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    panel.set_edge_box(BOX)
    labels = panel.readout_labels
    # The edge is measured in the preview, but there is no calibration: no error.
    qtbot.waitUntil(lambda: "px" in labels["edge"].text(), timeout=WAIT_MS)
    assert labels["error"].text() == "—"
    with patch.object(svc, "set_calibration", wraps=svc.set_calibration) as spy:
        panel.calibrate()
        qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
        assert panel.fault_label.text() == "", panel.fault_label.text()
        spy.assert_called()
        sent = spy.call_args[0][0]
        assert isinstance(sent, FocusCalibration) and sent.box == BOX
    # Now the preview shows the error (about 0 um: the fake is at its zero).
    qtbot.waitUntil(lambda: "µm" in labels["error"].text(), timeout=WAIT_MS)
    assert abs(float(labels["error"].text().split()[0])) < 0.5


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_calibrate_gives_a_two_line_label_for_the_one_edge(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC, sign: float
) -> None:
    settings = SettingsV1.instance()
    cfg = settings.spot_focus
    _use_diagonal_motion(settings, sign)
    cfg.edge_box = BOX
    cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames = 1.6, 7, 2
    panel.refresh()
    svc = _preview_on(qtbot, panel)  # the preview then shows the error
    with patch.object(svc, "calibrate", wraps=svc.calibrate) as spy:
        panel.calibrate()
        qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
        assert spy.call_args[0][0] == BOX
    stored = cfg.calibration
    assert stored is not None, panel.fault_label.text()
    assert stored["kind"] == "edge" and "edges" not in stored
    edge = stored["edge"]
    assert edge["slope_px_per_um"] == pytest.approx(sign * SLOPE * 0.8, rel=0.15)
    assert edge["axis"] == "x"
    assert tuple(edge["box"]) == BOX
    text = panel.calibration_label.text()
    lines = text.split("<br>")
    assert len(lines) == 2, text  # the edge line + the zero line
    sgn = "+" if sign > 0 else "-"
    assert re.match(rf"edge axis x \| \{sgn}[34]\.\d\d px/µm \| R²", lines[0]), text
    assert "residual" in lines[0] and "noise" in lines[0] and "µm" in lines[0]
    assert lines[1].startswith("zero P") and "±0.80 µm" in lines[1]
    assert "changed since the calibration" not in text
    # the ring sits on the zero edge point
    view = panel.live_view
    assert view._setpoint_item.pos().x() == pytest.approx(edge["e0_px"] + 0.5)
    # the preview shows the edge and the error
    labels = panel.readout_labels
    qtbot.waitUntil(
        lambda: "px" in labels["edge"].text() and "µm" in labels["error"].text(),
        timeout=WAIT_MS,
    )
    assert mmc.pos[P_LABEL] == pytest.approx(P0)  # P returned to where it was


def test_zero_here_rezeroes_the_edge(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    settings = SettingsV1.instance()
    cfg = settings.spot_focus
    _use_diagonal_motion(settings)
    cfg.edge_box = BOX
    cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames = 1.6, 7, 2
    panel.refresh()
    panel.calibrate()
    qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    before = cfg.calibration
    assert before is not None, panel.fault_label.text()
    mmc.pos[P_LABEL] = P0 + 0.5  # the user refocused by hand
    panel.zero()
    qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    assert panel.fault_label.text() == "", panel.fault_label.text()
    after = cfg.calibration
    assert after is not None and after["kind"] == "edge"
    assert after["p0_um"] == pytest.approx(P0 + 0.5)
    old, new = before["edge"], after["edge"]
    assert new["slope_px_per_um"] == old["slope_px_per_um"]  # kept
    moved = new["e0_px"] - old["e0_px"]
    assert moved == pytest.approx(0.5 * old["slope_px_per_um"], abs=0.5)
    assert isinstance(panel._cal, FocusCalibration)
    assert panel._cal.edge.e0_px == pytest.approx(new["e0_px"])


def test_rig_shaped_single_edge_calibration_locks_at_negative_p(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    settings = SettingsV1.instance()
    cfg = settings.spot_focus
    cfg.source.fake = {**cfg.source.fake, "cal_like": _rig_cal_like()}
    mmc.pos[P_LABEL] = RIG_P
    cfg.calibration = _rig_saved_edge(_rig_e0())
    cfg.edge_box = TOP_BOX
    # The old file loads as a half-max calibration; Lock needs that setting
    # (under the default Gaussian derivative it is refused: recalibrate).
    cfg.detector = "halfmax"
    panel.refresh()
    assert isinstance(panel._cal, FocusCalibration)
    assert panel._cal.box == TOP_BOX
    text = panel.calibration_label.text()
    assert "edge axis y" in text and "+1.92 px/µm" in text
    assert "changed since the calibration" not in text
    assert panel.zero_button.isEnabled() and panel.start_button.isEnabled()

    panel.start()
    qtbot.waitUntil(lambda: panel.state == "running", timeout=WAIT_MS)
    assert panel.fault_label.text() == "", panel.fault_label.text()
    qtbot.waitUntil(lambda: len(panel.chart) > 5, timeout=WAIT_MS)
    assert panel.state == "running"  # no range / excursion fault at P = -45.5
    assert abs(mmc.pos[P_LABEL] - RIG_P) < 1.0
    panel.stop()
    assert panel.state == "stopped"


def test_draw_edge_box_button_drags_a_box_to_the_worker(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    view = panel.live_view
    cfg = SettingsV1.instance().spot_focus
    assert view.draw_target is None
    panel.draw_edge_button.click()
    assert panel.draw_edge_button.isChecked()
    assert view.draw_target == "edge" and view.view.draw_mode
    assert view.draw_edge_button.isChecked()
    # Draw ROI and Draw edge box are exclusive
    view.draw_roi_button.setChecked(True)
    assert view.draw_target == "roi" and not panel.draw_edge_button.isChecked()
    panel.draw_edge_button.click()
    assert view.draw_target == "edge" and not view.draw_roi_button.isChecked()

    with patch.object(svc, "set_edge_box", wraps=svc.set_edge_box) as spy:
        # a drag across the left edge, in scene (sensor) pixels
        view._on_rect_drawn(QPointF(232.3, 210.6), QPointF(311.2, 269.9))
        spy.assert_called_with(BOX)
    assert cfg.edge_box == BOX
    assert not panel.draw_edge_button.isChecked()  # one box per click
    assert view.draw_target is None and not view.view.draw_mode
    assert view._edge_box_item.isVisible()
    # The preview now measures it: the cross lands on the patch's left edge.
    qtbot.waitUntil(
        lambda: view.frame is not None
        and view.frame.spot is not None
        and abs(view.frame.spot[0] - _e0()) < 2
        and abs(view.frame.spot[1] - 240) < 2,
        timeout=WAIT_MS,
    )


def test_start_telemetry_stop_releases_led_and_joystick(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC, tmp_path: Path
) -> None:
    settings = SettingsV1.instance()
    _calibrated(settings)
    panel.refresh()
    assert panel.service is None
    assert panel.start_button.isEnabled()
    assert panel.zero_button.isEnabled()
    assert not panel.stop_button.isEnabled()
    assert "px/µm" in panel.calibration_label.text()

    panel.start()
    assert not panel.start_button.isEnabled()
    qtbot.waitUntil(lambda: panel.state == "running", timeout=WAIT_MS)
    assert mmc.crisp.leds[-1] is True  # lit by a raw command
    # the piezo wheel is off under the lock (the worker's status can lag the panel's)
    qtbot.waitUntil(lambda: mmc.crisp.joystick == [False], timeout=WAIT_MS)
    assert panel.lock_active
    # Calibrate and Zero are only for a stopped loop.
    assert not panel.calibrate_button.isEnabled()
    assert not panel.zero_button.isEnabled()
    assert not panel.draw_edge_button.isEnabled()
    assert panel.stop_button.isEnabled()

    qtbot.waitUntil(lambda: len(panel.chart) > 5, timeout=WAIT_MS)
    # The lock streams frames whether or not a window is open: open it now,
    # while the lock runs, and it fills with frames at the sensor offset.
    assert not panel.live_view.polling
    panel.preview_button.click()
    assert panel.live_view.polling and panel.live_view.isVisible()
    assert panel.preview_button.isChecked()
    qtbot.waitUntil(lambda: panel.live_view.frame is not None, timeout=WAIT_MS)
    view = panel.live_view
    frame = view.frame
    assert frame is not None
    assert frame.spot is not None
    setpoint = (_e0(), 240.0)  # the zero edge point: (e0, box_y + h / 2)
    # The fake edge sits at the zero, so the cross and the ring coincide.
    assert abs(frame.spot[0] - setpoint[0]) < 3
    assert abs(frame.spot[1] - setpoint[1]) < 3
    assert view._setpoint_item.isVisible()
    assert view._setpoint_item.pos().x() == pytest.approx(setpoint[0] + 0.5)
    assert view._setpoint_item.pos().y() == pytest.approx(setpoint[1] + 0.5)
    assert panel.readout_labels["fps"].text() != "-"
    assert "px" in panel.readout_labels["edge"].text()
    assert panel.readout_labels["contrast"].text() != "-"
    assert "edge (" in view.status_label.text()
    # The chart and the live view paint without raising (a paint error aborts Qt).
    panel.resize(500, 800)
    pixmap = panel.grab()
    assert not pixmap.isNull()
    pixmap.save(str(tmp_path / "spot_panel.png"))
    assert not view.grab().isNull()
    seq = frame.seq
    # While the lock runs the live view keeps updating.
    qtbot.waitUntil(
        lambda: view.frame is not None and view.frame.seq > seq + 3, timeout=WAIT_MS
    )
    assert "µm" in panel.readout_labels["error"].text()

    # Editing a parameter while running reaches the settings (and the worker).
    panel.param_spins["deadband_um"].setValue(0.05)
    assert settings.spot_focus.params.deadband_um == pytest.approx(0.05)
    # So does the offset: persisted, and sent to the running loop at once.
    svc = panel.service
    assert svc is not None
    with patch.object(svc, "update_params", wraps=svc.update_params) as spy:
        panel.offset_spin.setValue(0.3)
        spy.assert_called_once()
        assert spy.call_args[0][0].offset_um == pytest.approx(0.3)
    assert settings.spot_focus.params.offset_um == pytest.approx(0.3)
    # A box drawn while the lock runs is refused (the lock keeps its own).
    view.edgeBoxDrawn.emit(0, 0, 40, 40)
    assert settings.spot_focus.edge_box == BOX
    assert "Stop the lock" in panel.fault_label.text()

    sets_before_stop = len(mmc.sets)
    with (
        patch.object(svc, "set_edge_box", wraps=svc.set_edge_box) as box_spy,
        patch.object(svc, "set_calibration", wraps=svc.set_calibration) as cal_spy,
    ):
        panel.stop()
        box_spy.assert_called_with(BOX)  # the preview measures the drawn box again
        cal_spy.assert_called_once()  # ... with the calibration, for the error
        assert isinstance(cal_spy.call_args[0][0], FocusCalibration)
    assert panel.state == "stopped"
    assert not panel.lock_active
    assert mmc.crisp.leds[-1] is False  # the lock's LED is released
    assert mmc.crisp.joystick == [False, True]  # the wheel is back
    assert panel.calibrate_button.isEnabled()
    assert panel.start_button.isEnabled()
    assert panel.zero_button.isEnabled()
    qtbot.wait(300)
    assert len(mmc.sets) - sets_before_stop <= 1  # at most one in flight


def test_calibrate_off_the_gui_thread(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    settings = SettingsV1.instance()
    cfg = settings.spot_focus
    cfg.edge_box = BOX
    panel.refresh()
    # The sweep spins are persisted and drive the sweep.
    panel.span_spin.setValue(1.6)
    panel.steps_spin.setValue(7)
    panel.frames_spin.setValue(2)
    assert (cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames) == (1.6, 7, 2)
    panel.offset_spin.setValue(0.4)
    assert cfg.params.offset_um == pytest.approx(0.4)
    assert panel._record_path() is None  # nothing is written to disk under pytest

    panel.calibrate()
    assert panel._busy == "calibrating"
    assert not panel.calibrate_button.isEnabled()
    assert not panel.start_button.isEnabled()
    assert not panel.zero_button.isEnabled()
    qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    stored = cfg.calibration
    assert stored is not None, panel.fault_label.text()
    assert stored["kind"] == "edge" and "edges" not in stored
    edge = stored["edge"]
    assert edge["slope_px_per_um"] == pytest.approx(SLOPE, rel=0.1)  # signed
    assert edge["axis"] == "x" and edge["polarity"] == 1
    assert edge["n_points"] == 7
    assert edge["linear_range_um"] == pytest.approx(0.8)
    assert stored["p0_um"] == pytest.approx(P0)
    assert edge["e0_px"] == pytest.approx(_e0(), abs=0.5)
    assert tuple(edge["box"]) == BOX
    assert panel.start_button.isEnabled() and panel.zero_button.isEnabled()
    assert mmc.pos[P_LABEL] == pytest.approx(P0)  # P returned to where it was
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"  # never written
    assert mmc.crisp.joystick == []  # calibrating leaves the wheel alone
    text = panel.calibration_label.text()
    assert "px/µm" in text and "R²" in text and "zero P" in text
    assert re.search(r"\+[45]\.\d\d px/µm", text), text  # the right sign
    # a new zero: the offset is back to 0 and the ring sits on the zero edge point
    assert panel.offset_spin.value() == 0.0
    assert cfg.params.offset_um == 0.0
    ring = panel.live_view._setpoint_item
    assert ring.isVisible()
    assert ring.pos().x() == pytest.approx(edge["e0_px"] + 0.5)
    assert ring.pos().y() == pytest.approx(210 + 30 + 0.5)


def test_zero_here_rezeroes_and_resets_the_offset(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    settings = SettingsV1.instance()
    _calibrated(settings)
    panel.refresh()
    panel.offset_spin.setValue(0.3)
    # The user refocused by hand: P is 0.4 um higher, so the fake edge moved.
    mmc.pos[P_LABEL] = P0 + 0.4
    panel.zero()
    assert panel._busy == "zeroing"
    assert not panel.zero_button.isEnabled() and not panel.start_button.isEnabled()
    qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    assert panel.fault_label.text() == "", panel.fault_label.text()
    stored = settings.spot_focus.calibration
    assert stored is not None
    edge = stored["edge"]
    assert stored["p0_um"] == pytest.approx(P0 + 0.4)
    assert edge["e0_px"] == pytest.approx(_e0() + SLOPE * 0.4, abs=0.4)
    assert edge["slope_px_per_um"] == SLOPE  # kept
    assert panel.offset_spin.value() == 0.0
    assert settings.spot_focus.params.offset_um == 0.0
    assert panel._cal is not None and panel._cal.p0_um == pytest.approx(P0 + 0.4)
    assert panel.live_view._setpoint_item.pos().x() == pytest.approx(
        edge["e0_px"] + 0.5
    )
    assert mmc.pos[P_LABEL] == pytest.approx(P0 + 0.4)  # Zero never moves P


def test_lock_without_a_calibration_calibrates_then_locks(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    settings = SettingsV1.instance()
    cfg = settings.spot_focus
    cfg.edge_box = BOX
    cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames = 1.2, 7, 2
    cfg.params.offset_um = 0.2
    panel.refresh()
    assert panel._cal is None
    assert panel.start_button.isEnabled()
    panel.start()
    qtbot.waitUntil(lambda: panel.state == "running", timeout=WAIT_MS)
    assert panel.fault_label.text() == "", panel.fault_label.text()
    stored = cfg.calibration
    assert stored is not None and stored["kind"] == "edge"
    assert stored["edge"]["slope_px_per_um"] == pytest.approx(SLOPE, rel=0.1)
    assert stored["p0_um"] == pytest.approx(P0)  # the present P is the zero
    assert panel._cal is not None
    assert cfg.params.offset_um == 0.0  # a new zero
    assert mmc.crisp.leds[-1] is True  # (calibrating lit and released it first)
    assert panel.lock_active
    qtbot.waitUntil(lambda: len(panel.chart) > 3, timeout=WAIT_MS)
    panel.stop()
    assert panel.state == "stopped"
    assert mmc.crisp.leds[-1] is False


def test_failed_calibration_inside_lock_is_reported_and_nothing_locks(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    cfg = SettingsV1.instance().spot_focus
    cfg.edge_box = (0, 0, 32, 32)  # a corner of the dark background: no edge
    panel.refresh()
    panel.start()
    qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    assert "Calibration failed" in panel.fault_label.text()
    assert panel.state != "running"
    assert cfg.calibration is None
    assert not panel.lock_active
    assert mmc.crisp.joystick == []  # the wheel was never switched off
    assert panel.start_button.isEnabled()


def test_start_failure_is_reported(qtbot: QtBot, mmc: FakeMMC) -> None:
    def broken(spec: dict, led: float | None) -> SpotFocusService:
        raise RuntimeError("no camera")

    widget = SpotFocusPanel(mmc, service_factory=broken)
    qtbot.addWidget(widget)
    _calibrated(SettingsV1.instance())
    widget.refresh()
    widget.start()
    qtbot.waitUntil(lambda: not widget._busy, timeout=WAIT_MS)
    assert "no camera" in widget.fault_label.text()
    assert widget.state == "idle"
    assert not widget.lock_active
    assert widget.start_button.isEnabled()


# ---------------------------------------------------------------------------
# camera section: Preview, live view, exposure / gain / channel, ROI
# ---------------------------------------------------------------------------


def _preview_on(qtbot: QtBot, panel: SpotFocusPanel) -> SpotFocusService:
    panel.preview_button.click()
    assert panel._preview_busy
    assert not panel.preview_button.isEnabled()  # while the camera opens
    qtbot.waitUntil(lambda: not panel._preview_busy, timeout=WAIT_MS)
    assert panel.fault_label.text() == "", panel.fault_label.text()
    svc = panel.service
    assert svc is not None
    qtbot.waitUntil(lambda: panel.live_view.frame is not None, timeout=WAIT_MS)
    return svc


def test_preview_lights_the_led_and_off_switches_it_off(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    """Preview lights the CRISP LED with a raw command; the readout says on/off."""
    assert mmc.crisp.leds == []
    _preview_on(qtbot, panel)
    qtbot.waitUntil(lambda: mmc.crisp.leds == [True], timeout=3000)
    qtbot.waitUntil(lambda: panel.led_button.isChecked(), timeout=WAIT_MS)
    assert mmc.props[(CRISP, "LED Intensity")] == "50.0"  # the default
    assert panel.led_spin.value() == 50
    assert "LED on" in panel.crisp_readout.text()
    assert CRISP in panel.crisp_readout.text()
    panel.preview_button.click()  # off
    assert mmc.crisp.leds == [True, False]
    qtbot.waitUntil(lambda: not panel.led_button.isChecked(), timeout=WAIT_MS)
    assert "LED off" in panel.crisp_readout.text()
    # CRISP's own state is never written
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"


def test_led_button_and_spin_apply_live_and_are_saved(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC, _isolated_settings: SettingsV1
) -> None:
    panel.led_button.click()
    qtbot.waitUntil(lambda: "LED on" in panel.crisp_readout.text(), timeout=WAIT_MS)
    assert mmc.crisp.leds == [True]
    panel.led_spin.setValue(80)
    assert mmc.props[(CRISP, "LED Intensity")] == "80.0"
    assert _isolated_settings.spot_focus.led_intensity == 80.0
    panel.led_button.click()  # off
    qtbot.waitUntil(lambda: "LED off" in panel.crisp_readout.text(), timeout=WAIT_MS)
    assert mmc.crisp.leds == [True, False]


def test_no_crisp_device_is_said_not_silent(qtbot: QtBot, mmc: FakeMMC) -> None:
    def make(spec: dict, led: float | None) -> SpotFocusService:
        return SpotFocusService(mmc, P_LABEL, None, spec, ready_timeout_s=25.0)

    widget = SpotFocusPanel(mmc, service_factory=make)
    qtbot.addWidget(widget)
    try:
        widget.preview_button.click()
        qtbot.waitUntil(lambda: not widget._preview_busy, timeout=WAIT_MS)
        assert "no CRISP device (no CRISP label)" in widget.fault_label.text()
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    finally:
        widget.shutdown()


def test_preview_opens_a_full_resolution_live_view(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    assert panel.service is None  # lazy
    svc = _preview_on(qtbot, panel)
    view = panel.live_view
    assert view.isVisible()
    assert view.windowTitle() == "Spot camera"
    assert panel.preview_button.isChecked()
    assert svc.preview_active
    assert panel.state == "idle"
    frame = view.frame
    assert frame is not None
    # Native size: the pixmap has exactly the ROI's pixels.
    pm = view.pixmap_item.pixmap()
    assert (pm.width(), pm.height()) == (frame.roi[2], frame.roi[3])
    assert view.sensor == tuple(frame.sensor)
    assert "FakeSpotSource" in panel.camera_label.text()
    qtbot.waitUntil(
        lambda: "fps" in panel.camera_readout.text()
        and panel.camera_readout.text() != "-",
        timeout=WAIT_MS,
    )
    assert "saturated" in panel.camera_readout.text()
    # No calibration: the ring is hidden. Calibrating shows it.
    assert not view._setpoint_item.isVisible()

    # Preview off stops the stream and polling; the docked view keeps its space.
    panel.preview_button.click()
    assert not panel.preview_button.isChecked()
    assert view.isVisible()
    assert not view.polling and not view._timer.isActive()
    assert view.frame is None  # cleared
    assert not svc.preview_active


def test_view_is_docked_above_the_controls_by_default(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    view = panel.live_view
    assert not view.isWindow()
    assert view.window() is panel.window()  # not its own top-level window
    assert panel.splitter.indexOf(view) == 0
    assert panel.splitter.orientation() == Qt.Orientation.Vertical
    assert view.minimumHeight() >= 240
    assert not panel.view_floating
    assert not panel.pop_out_button.isChecked()
    assert panel.pop_out_button.text() == "Pop out"
    assert view.isVisible()  # the space is kept even before Preview
    assert not view.polling and not view._timer.isActive()
    assert all(s > 0 for s in panel.splitter.sizes())


def test_pop_out_and_dock_back_keep_the_index_and_the_scene(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    view = panel.live_view
    scene = view.scene
    index = panel.splitter.indexOf(view)
    panel.pop_out_button.click()
    assert view.isWindow() and view.isVisible()
    assert panel.view_floating and panel.pop_out_button.isChecked()
    assert panel.pop_out_button.text() == "Dock"
    assert view.window() is view
    assert view.scene is scene
    assert panel.splitter.indexOf(view) == -1
    assert panel.splitter.count() == 1
    assert view.parentWidget() is panel  # owned by the panel: deleted with it

    panel.pop_out_button.click()  # dock again
    assert not view.isWindow() and view.isVisible()
    assert not panel.view_floating and not panel.pop_out_button.isChecked()
    assert panel.pop_out_button.text() == "Pop out"
    assert panel.splitter.indexOf(view) == index
    assert panel.splitter.count() == 2
    assert view.scene is scene
    # The code path works too, and a repeat is a no-op.
    panel.set_view_floating(True)
    panel.set_view_floating(True)
    assert panel.pop_out_button.isChecked() and view.isWindow()
    panel.set_view_floating(False)
    assert panel.splitter.indexOf(view) == index


def test_polling_follows_preview_docked_and_floating(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    view = panel.live_view
    svc = _preview_on(qtbot, panel)
    assert view.polling and view._timer.isActive()
    panel.pop_out_button.click()
    assert view._timer.isActive()  # still polling while floating
    seq = view.frame.seq  # type: ignore[union-attr]
    qtbot.waitUntil(lambda: view.frame.seq > seq, timeout=WAIT_MS)  # type: ignore[union-attr]
    panel.pop_out_button.click()
    assert view._timer.isActive()  # ... and docked again
    panel.preview_button.click()  # off
    assert not view._timer.isActive() and not svc.preview_active
    # Preview off while floating hides the window and keeps polling off.
    panel.pop_out_button.click()
    assert view.isWindow() and view.isVisible() and not view._timer.isActive()
    panel.preview_button.click()
    qtbot.waitUntil(lambda: panel.live_view.frame is not None, timeout=WAIT_MS)
    assert view._timer.isActive()
    panel.preview_button.click()  # off
    assert not view.isVisible() and not view._timer.isActive()
    panel.pop_out_button.click()  # dock it back
    assert view.isVisible() and not view._timer.isActive()


def test_closing_the_floating_view_docks_it_and_keeps_preview(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    view = panel.live_view
    svc = _preview_on(qtbot, panel)
    panel.pop_out_button.click()
    assert view.isWindow()
    with qtbot.waitSignal(view.windowClosed, timeout=1000):  # still emitted
        view.close()
    qtbot.waitUntil(lambda: not view.isWindow(), timeout=2000)
    assert view.isVisible()
    assert panel.splitter.indexOf(view) == 0
    assert not panel.pop_out_button.isChecked()
    assert panel.preview_button.isChecked()  # closing a window is not Preview off
    assert svc.preview_active and view._timer.isActive()
    seq = view.frame.seq  # type: ignore[union-attr]
    qtbot.waitUntil(lambda: view.frame.seq > seq, timeout=WAIT_MS)  # type: ignore[union-attr]


def test_closing_the_view_while_the_lock_runs_keeps_it_running(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    _calibrated(SettingsV1.instance())
    panel.refresh()
    svc = _preview_on(qtbot, panel)
    panel.start()
    qtbot.waitUntil(lambda: panel.state == "running", timeout=WAIT_MS)
    assert panel.live_view.isVisible()  # still updating under the lock
    seq = panel.live_view.frame.seq  # type: ignore[union-attr]
    qtbot.waitUntil(lambda: panel.live_view.frame.seq > seq + 3, timeout=WAIT_MS)  # type: ignore[union-attr]

    # Preview cannot be switched off while the lock runs.
    panel.preview_button.click()
    assert panel.preview_button.isChecked()
    assert panel.state == "running"

    panel.pop_out_button.click()
    panel.live_view.close()
    qtbot.waitUntil(lambda: not panel.live_view.isWindow(), timeout=2000)
    assert panel.state == "running"
    assert svc.current_state == "running"
    assert panel.preview_button.isChecked()
    panel.stop()
    assert panel.state == "stopped"
    assert panel.preview_button.isChecked()  # the preview was never switched off


def test_draw_button_still_emits_the_edge_box_when_docked(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    view = panel.live_view
    assert not view.isWindow()
    got: list[tuple] = []
    view.edgeBoxDrawn.connect(lambda *a: got.append(a))
    view._set_sensor((640, 480))  # the drag is clamped to the sensor
    a, b = QPointF(232.0, 210.0), QPointF(312.0, 270.0)
    view.draw_edge_button.setChecked(True)
    assert view.draw_target == "edge" and panel.draw_edge_button.isChecked()
    view.view.rectDrawn.emit(a, b)  # what a mouse drag on the image emits
    assert len(got) == 1
    assert SettingsV1.instance().spot_focus.edge_box == tuple(got[0])
    assert not panel.draw_edge_button.isChecked() and view.draw_target is None


def test_shutdown_leaves_no_floating_window(qtbot: QtBot, mmc: FakeMMC) -> None:
    widget = SpotFocusPanel(mmc, service_factory=_factory(mmc))
    qtbot.addWidget(widget)
    widget.show()
    view = widget.live_view
    widget.set_view_floating(True)
    assert view.isVisible()
    widget.shutdown()
    assert not view.isVisible()  # hidden at once, deleted with the panel
    assert not view.polling
    widget.hide()
    widget.show()
    assert not view.isVisible()  # a shut-down panel does not bring it back


def test_floating_view_goes_with_the_panel_when_hidden(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    view = panel.live_view
    panel.set_view_floating(True)
    assert view.isVisible()
    panel.hide()  # a closed dock or another tab
    assert not view.isVisible()
    panel.show()
    assert view.isWindow() and view.isVisible()
    panel.set_view_floating(False)
    panel.hide()
    panel.show()
    assert not view.isWindow() and view.isVisible()


def test_exposure_gain_channel_apply_live_and_are_saved(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    cfg = SettingsV1.instance().spot_focus
    with patch.object(svc, "set_camera", wraps=svc.set_camera) as spy:
        panel.exposure_spin.setValue(12.5)
        spy.assert_called_with(exposure_ms=12.5)
        panel.gain_spin.setValue(300)
        spy.assert_called_with(gain=300)
        panel.channel_combo.setCurrentText("sum")
        spy.assert_called_with(channel="sum")
    assert (cfg.exposure_ms, cfg.gain, cfg.channel) == (12.5, 300, "sum")
    # The channel is live: the worker keeps running, the service is not rebuilt.
    spec = source_spec(cfg)
    assert spec["analysis"]["channel"] == "sum"
    assert panel._owner.service is svc
    panel.refresh()
    assert panel.channel_combo.currentText() == "sum"
    assert panel.gain_spin.value() == 300


# ---------------------------------------------------------------------------
# edge detector, mismatch, per-edge quality
# ---------------------------------------------------------------------------


def _halfmax_cal() -> FocusCalibration:
    """The saved calibration of an older version: made with the half-max method."""
    return FocusCalibration(
        edge=replace(_edge_cal(), detector="halfmax"),
        p0_um=P0,
        created="2026-10-28T00:00:00",
    )


def test_detector_controls_follow_the_settings(panel: SpotFocusPanel) -> None:
    combo, spin = panel.detector_combo, panel.detector_sigma_spin
    assert [combo.itemData(i) for i in range(combo.count())] == list(EDGE_DETECTORS)
    assert [combo.itemText(i) for i in range(combo.count())] == [
        EDGE_DETECTOR_LABELS[d] for d in EDGE_DETECTORS
    ]
    assert "steepest point" in combo.toolTip() and "4x" in combo.toolTip()
    assert (spin.minimum(), spin.maximum()) == SIGMA_PX_RANGE
    assert spin.singleStep() == 0.5
    # the settings' defaults
    assert combo.currentData() == "gaussderiv" and spin.value() == 6.0
    assert spin.isEnabled()
    # saved values show after a refresh; half-max has no width
    cfg = SettingsV1.instance().spot_focus
    cfg.detector, cfg.detector_sigma_px = "halfmax", 9.0
    panel.refresh()
    assert combo.currentData() == "halfmax" and spin.value() == 9.0
    assert not spin.isEnabled()
    assert cfg.detector == "halfmax" and cfg.detector_sigma_px == 9.0  # not rewritten


def test_detector_change_is_saved_sent_live_and_put_in_the_spec(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    cfg = SettingsV1.instance().spot_focus
    combo, spin = panel.detector_combo, panel.detector_sigma_spin
    with patch.object(svc, "set_camera", wraps=svc.set_camera) as spy:
        combo.setCurrentIndex(combo.findData("halfmax"))
        spy.assert_called_with(detector="halfmax", detector_sigma_px=6.0)
        assert not spin.isEnabled()
        combo.setCurrentIndex(combo.findData("gaussderiv"))
        assert spin.isEnabled()
        spin.setValue(8.0)
        spy.assert_called_with(detector="gaussderiv", detector_sigma_px=8.0)
    assert (cfg.detector, cfg.detector_sigma_px) == ("gaussderiv", 8.0)
    assert svc.detector_setting == ("gaussderiv", 8.0)
    analysis = source_spec(cfg)["analysis"]
    assert analysis["detector"] == "gaussderiv" and analysis["detector_sigma_px"] == 8.0
    # live: the worker keeps running, the service is not rebuilt
    assert panel._owner.service is svc
    panel.refresh()
    assert spin.value() == 8.0 and combo.currentData() == "gaussderiv"


def test_detector_mismatch_is_shown_and_blocks_lock(panel: SpotFocusPanel) -> None:
    cfg = SettingsV1.instance().spot_focus
    cfg.calibration = _halfmax_cal().to_dict()
    cfg.edge_box = BOX
    panel.refresh()  # setting gaussderiv, calibration halfmax
    why = "calibrated with halfmax; the detector setting is gaussderiv: recalibrate"
    assert not panel.start_button.isEnabled()
    assert panel.start_button.toolTip() == why
    text = panel.calibration_label.text()
    assert why in text and ATTENTION_COLOR in text
    assert "detector halfmax" in text  # what the calibration was made with
    assert panel.calibrate_button.isEnabled()  # recalibrating is the way out
    panel.start()  # also refuses when called directly
    assert why in panel.fault_label.text() and panel._busy == ""
    # switching the setting to the calibration's detector clears it
    panel.detector_combo.setCurrentIndex(panel.detector_combo.findData("halfmax"))
    assert panel.start_button.isEnabled()
    assert why not in panel.calibration_label.text()
    # a different gaussderiv width is a mismatch too
    cfg.calibration = _focus_cal().to_dict()
    panel.refresh()
    panel.detector_combo.setCurrentIndex(panel.detector_combo.findData("gaussderiv"))
    assert panel.start_button.isEnabled()
    assert "detector gaussderiv 6 px" in panel.calibration_label.text()
    panel.detector_sigma_spin.setValue(3.0)
    assert not panel.start_button.isEnabled()
    assert "sigma 6 px" in panel.start_button.toolTip()


def test_refusal_from_the_service_start_is_shown_not_raised(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    _calibrated(SettingsV1.instance())
    panel.refresh()
    assert panel.start_button.isEnabled()  # the panel's own check passes
    # The service's idea of the setting differs (a race): its start() refuses.
    with patch.object(
        SpotFocusService,
        "detector_setting",
        new_callable=PropertyMock,
        return_value=("halfmax", 6.0),
    ):
        panel.start()
        qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    text = panel.fault_label.text()
    assert "calibrated with gaussderiv; the detector setting is halfmax" in text
    assert ATTENTION_COLOR in text
    assert panel.state != "running" and not panel.lock_active
    assert panel._owner.service is svc and not panel._owner.dead  # no rebuild
    assert panel.start_button.isEnabled()


@pytest.mark.parametrize(
    ("noise", "color"),
    [
        (0.0, OK_COLOR),
        (0.0299, OK_COLOR),
        (0.03, ATTENTION_COLOR),
        (0.0599, ATTENTION_COLOR),
        (0.06, ALERT_COLOR),
        (0.2, ALERT_COLOR),
        (float("nan"), ALERT_COLOR),
    ],
)
def test_noise_colour_thresholds(noise: float, color: str) -> None:
    assert noise_color(noise) == color


@pytest.mark.parametrize(
    ("noise_px", "noise_um", "color"),
    [
        (0.1, "0.020", OK_COLOR),
        (0.225, "0.045", ATTENTION_COLOR),
        (0.5, "0.100", ALERT_COLOR),
    ],
)
def test_edge_quality_colours_in_the_label_and_the_live_readout(
    panel: SpotFocusPanel, noise_px: float, noise_um: str, color: str
) -> None:
    # noise_um = max(noise_px, residual_px, 0.02) / |slope| with slope 5 px/um
    edge = replace(_edge_cal(), noise_px=noise_px, residual_px=0.05)
    panel._cal = FocusCalibration(edge=edge, p0_um=P0, created="2026-10-28T00:00:00")
    panel._refresh_calibration()
    line = panel.calibration_label.text().split("<br>")[0]
    assert f'{color}">noise {noise_um} µm' in line, line
    # the live readout carries the same colour
    panel._on_telemetry({"edge_px": 272.0, "along_um": 0.0})
    text = panel.readout_labels["edge"].text()
    assert text.startswith("272.00 px")
    assert f'{color}">(noise {noise_um} µm)' in text


def test_narrow_box_warning_depends_on_the_detector() -> None:
    def cal(detector: str, box_px: int, width: float) -> FocusCalibration:
        edge = replace(
            _edge_cal(), detector=detector, box_normal_px=box_px, edge_width_px=width
        )
        return FocusCalibration(edge=edge, p0_um=P0, created="x")

    # 38 px box, 20 px edge: narrow for both (needs 80 px for half-max, 40 gauss)
    assert box_warnings(cal("halfmax", 38, 20.0)) == [
        "the box is narrow for this edge (38 px box, 20 px edge): "
        "widen it across the edge"
    ]
    assert len(box_warnings(cal("gaussderiv", 38, 20.0))) == 1
    # 60 px box, 20 px edge: fine for gaussderiv (>= 2x), narrow for half-max (4x)
    assert box_warnings(cal("gaussderiv", 60, 20.0)) == []
    assert len(box_warnings(cal("halfmax", 60, 20.0))) == 1
    # edge width unknown (nan) or box size unknown (old calibration): no warning
    assert box_warnings(cal("halfmax", 38, float("nan"))) == []
    assert box_warnings(cal("halfmax", 0, 20.0)) == []


def test_narrow_box_warning_is_in_the_calibration_label(
    panel: SpotFocusPanel,
) -> None:
    narrow = replace(_edge_cal(), box_normal_px=38, edge_width_px=20.0)
    panel._cal = FocusCalibration(edge=narrow, p0_um=P0, created="2026-10-28T00:00:00")
    panel._refresh_calibration()
    text = panel.calibration_label.text()
    assert "the box is narrow for this edge (38 px box, 20 px edge)" in text
    assert ATTENTION_COLOR in text


def test_apply_roi_and_full_frame(qtbot: QtBot, panel: SpotFocusPanel) -> None:
    svc = _preview_on(qtbot, panel)
    cfg = SettingsV1.instance().spot_focus
    with patch.object(svc, "set_camera", wraps=svc.set_camera) as spy:
        for spin, value in zip(
            panel.roi_spins.values(), (101, 41, 300, 200), strict=True
        ):
            spin.setValue(value)
        panel.apply_roi_button.click()
        spy.assert_called_with(roi=(100, 40, 300, 200))  # snapped to even
        assert cfg.roi == (100, 40, 300, 200)
        assert [s.value() for s in panel.roi_spins.values()] == [100, 40, 300, 200]
        qtbot.waitUntil(
            lambda: panel.live_view.frame is not None
            and tuple(panel.live_view.frame.roi) == (100, 40, 300, 200),
            timeout=WAIT_MS,
        )
        pm = panel.live_view.pixmap_item.pixmap()
        assert (pm.width(), pm.height()) == (300, 200)
        assert (
            panel.live_view.pixmap_item.pos().x(),
            panel.live_view.pixmap_item.pos().y(),
        ) == (100, 40)

        panel.full_frame_button.click()
        spy.assert_called_with(roi=(0, 0, 0, 0))
        assert cfg.roi is None
        sw, sh = panel.live_view.sensor  # type: ignore[misc]
        assert [s.value() for s in panel.roi_spins.values()] == [0, 0, sw, sh]
        qtbot.waitUntil(
            lambda: panel.live_view.frame is not None
            and tuple(panel.live_view.frame.roi) == (0, 0, sw, sh),
            timeout=WAIT_MS,
        )


def test_roi_drawn_in_the_view_fills_the_spins_and_applies(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    cfg = SettingsV1.instance().spot_focus
    with patch.object(svc, "set_camera", wraps=svc.set_camera) as spy:
        panel.live_view.roiDrawn.emit(64, 32, 200, 120)
        spy.assert_called_with(roi=(64, 32, 200, 120))
    assert cfg.roi == (64, 32, 200, 120)
    assert [s.value() for s in panel.roi_spins.values()] == [64, 32, 200, 120]


def test_sdk_folder_is_saved_and_browse_fills_it(
    qtbot: QtBot, panel: SpotFocusPanel, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = SettingsV1.instance().spot_focus
    panel.sdk_edit.setText(str(tmp_path))
    panel.sdk_edit.editingFinished.emit()
    assert cfg.source.sdk_dir == str(tmp_path)
    sub = tmp_path / "sdk"
    monkeypatch.setattr(
        "CRISPy.spot.panel.QFileDialog.getExistingDirectory",
        lambda *a, **k: str(sub),
    )
    panel.browse_button.click()
    assert panel.sdk_edit.text() == str(sub)
    assert cfg.source.sdk_dir == str(sub)
    panel.sdk_edit.setText("")
    panel.sdk_edit.editingFinished.emit()
    assert cfg.source.sdk_dir is None
    # A freshly built panel shows the saved folder.
    cfg.source.sdk_dir = "/opt/toupcam"
    panel.refresh()
    assert panel.sdk_edit.text() == "/opt/toupcam"


def test_camera_error_is_shown_and_preview_can_be_retried(
    qtbot: QtBot, mmc: FakeMMC
) -> None:
    calls = {"n": 0}
    good = _factory(mmc)

    def flaky(spec: dict, led: float | None) -> SpotFocusService:
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("SDK folder not found: /nope")
        return good(spec, led)

    widget = SpotFocusPanel(mmc, service_factory=flaky)
    qtbot.addWidget(widget)
    widget.show()
    try:
        widget.preview_button.click()
        qtbot.waitUntil(lambda: not widget._preview_busy, timeout=WAIT_MS)
        assert "SDK folder not found" in widget.fault_label.text()
        assert not widget.preview_button.isChecked()
        assert widget.preview_button.isEnabled()
        assert not widget.live_view.polling
        # Fix the folder, press Preview again.
        widget.sdk_edit.setText("/fixed")
        widget.sdk_edit.editingFinished.emit()
        widget.preview_button.click()
        qtbot.waitUntil(lambda: not widget._preview_busy, timeout=WAIT_MS)
        qtbot.waitUntil(lambda: widget.live_view.frame is not None, timeout=WAIT_MS)
        assert widget.fault_label.text() == ""
        assert widget.preview_button.isChecked()
        assert widget.live_view.isVisible()
    finally:
        widget.shutdown()


def test_setpoint_ring_is_in_sensor_pixels_with_a_roi(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    """With a ROI set the ring and cross keep sensor coordinates (the ROI offset
    is in the frame, not in the markers)."""
    settings = SettingsV1.instance()
    _calibrated(settings)
    settings.spot_focus.roi = (200, 150, 300, 200)  # contains the box and the patch
    panel.refresh()
    _preview_on(qtbot, panel)
    view = panel.live_view
    qtbot.waitUntil(
        lambda: view.frame is not None and view.frame.spot is not None, timeout=WAIT_MS
    )
    assert tuple(view.frame.roi) == (200, 150, 300, 200)  # type: ignore[union-attr]
    assert view._setpoint_item.pos().x() == pytest.approx(_e0() + 0.5)
    assert view._setpoint_item.pos().y() == pytest.approx(240.0 + 0.5)
    spot = view.frame.spot  # type: ignore[union-attr]
    assert abs(spot[0] - _e0()) < 3 and abs(spot[1] - 240.0) < 3


class FakeQSettings:
    """An in-memory ``QSettings`` so a test never touches the user's store."""

    store: ClassVar[dict[str, object]] = {}

    def __init__(self, *args: object) -> None:
        pass

    def value(self, key: str, default: object = None, type: object = None) -> object:
        return self.store.get(key, default)

    def setValue(self, key: str, value: object) -> None:
        self.store[key] = value


def test_camera_parameters_section_is_collapsible(
    qtbot: QtBot, mmc: FakeMMC, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("CRISPy.spot.panel.QSettings", FakeQSettings)
    monkeypatch.setattr(FakeQSettings, "store", {})
    widget = SpotFocusPanel(mmc, service_factory=_factory(mmc))
    qtbot.addWidget(widget)
    widget.show()
    try:
        section = widget.camera_params_section
        assert section.toggle_btn.text() == "Camera parameters"
        # collapsed by default: the body is hidden, Preview stays visible
        assert not section.toggle_btn.isChecked()
        assert not widget.exposure_spin.isVisible()
        assert not widget.apply_roi_button.isVisible()
        assert widget.preview_button.isVisible() and widget.camera_label.isVisible()
        # the title shows and hides the body
        section.toggle_btn.click()
        assert widget.exposure_spin.isVisible() and widget.apply_roi_button.isVisible()
        assert FakeQSettings.store["camera_params_expanded"] is True
        section.toggle_btn.click()
        assert not widget.exposure_spin.isVisible()
        assert FakeQSettings.store["camera_params_expanded"] is False
    finally:
        widget.shutdown()
    # the choice is remembered by the next panel
    FakeQSettings.store["camera_params_expanded"] = True
    again = SpotFocusPanel(mmc, service_factory=_factory(mmc))
    qtbot.addWidget(again)
    again.show()
    try:
        assert again.camera_params_section.toggle_btn.isChecked()
        assert again.exposure_spin.isVisible()
    finally:
        again.shutdown()
