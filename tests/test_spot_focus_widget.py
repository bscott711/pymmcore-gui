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
from pathlib import Path
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest
import useq

from pymmcore_gui import _settings
from pymmcore_gui._qt.QtCore import QPointF
from pymmcore_gui._qt.QtWidgets import QWidget
from pymmcore_gui._settings import (
    CalibrationRecord,
    FocusBoxSettings,
    SettingsV1,
)
from pymmcore_gui.actions.widget_actions import (
    SpotFocusScrollArea,
    create_image_focus_widget,
)
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.image_focus_events import (
    CALIBRATION_METADATA_KEY,
    spot_focus_flag,
)
from pymmcore_gui.asi_z_stack.image_focus_run import ImageFocusRun
from pymmcore_gui.asi_z_stack.spot_focus import EdgeCalibration, measure_edge
from pymmcore_gui.asi_z_stack.spot_focus_service import SpotFocusService
from pymmcore_gui.asi_z_stack.toupcam_source import make_source
from pymmcore_gui.widgets._spot_focus_panel import (
    NEED_BOX_TEXT,
    SpotFocusPanel,
    source_spec,
)

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
WAIT_MS = 30_000  # a spawned worker needs a few seconds to import


class FakeMMC:
    """Records setPosition; never waits; remembers properties."""

    def __init__(self) -> None:
        self.pos = {P_LABEL: P0}
        self.props = {(CRISP, "CRISP State"): "Idle", (CRISP, "LED Intensity"): "0"}
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
        yield fresh


@pytest.fixture(autouse=True)
def _child_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make sure the spawned child can import the package under test."""
    import pymmcore_gui

    src = str(Path(pymmcore_gui.__file__).resolve().parent.parent)
    old = os.environ.get("PYTHONPATH", "")
    monkeypatch.setenv("PYTHONPATH", src + (os.pathsep + old if old else ""))


@pytest.fixture(autouse=True)
def _flag_off() -> Iterator[None]:
    spot_focus_flag.active = False
    yield
    spot_focus_flag.active = False


@pytest.fixture
def mmc() -> FakeMMC:
    return FakeMMC()


def _factory(mmc: FakeMMC):
    def make(spec: dict, led: float | None) -> SpotFocusService:
        return SpotFocusService(
            mmc, P_LABEL, CRISP, spec, led_intensity=led, ready_timeout_s=25.0
        )

    return make


@pytest.fixture
def panel(qtbot: QtBot, mmc: FakeMMC) -> Iterator[SpotFocusPanel]:
    widget = SpotFocusPanel(mmc, service_factory=_factory(mmc))
    qtbot.addWidget(widget)
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
    )


def _calibrated(settings: SettingsV1) -> None:
    settings.spot_focus.calibration = _edge_cal().to_dict()
    settings.spot_focus.edge_box = BOX


# ---------------------------------------------------------------------------
# settings
# ---------------------------------------------------------------------------


def test_spot_settings_defaults_and_roundtrip() -> None:
    s = SettingsV1()
    assert s.image_focus.mode == "spot"
    cfg = s.spot_focus
    assert (cfg.channel, cfg.source.kind) == ("red", "toupcam")
    assert cfg.calibration is None
    assert cfg.to_calibration() is None
    assert cfg.calibration_status()[0] is None
    assert cfg.edge_box is None
    assert (cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames) == (2.0, 9, 3)
    params = cfg.params.to_params()
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
    cfg.calibration = _edge_cal().to_dict()
    cfg.edge_box = BOX
    cfg.sweep_span_um, cfg.sweep_steps, cfg.sweep_frames = 1.5, 7, 2
    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    assert s2.spot_focus == s.spot_focus
    assert s2.spot_focus.roi == (10, 20, 300, 200)
    assert s2.spot_focus.edge_box == BOX
    assert (s2.spot_focus.sweep_span_um, s2.spot_focus.sweep_steps) == (1.5, 7)
    cal = s2.spot_focus.to_calibration()
    assert cal is not None and cal.slope_px_per_um == SLOPE
    assert s2.spot_focus.calibration_status()[1] == ""
    p2 = s2.spot_focus.params.to_params()
    assert (p2.kp, p2.offset_um, p2.min_contrast) == (0.3, 0.25, 25.0)


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
    """A file written before the spot lock (no ``spot_focus``) loads with defaults.

    An old ``image_focus.mode`` ("crisp" / "image" / "off") is dropped with a
    RuntimeWarning and gives "spot".
    """
    old = {
        "version": "1.0",
        "image_focus": {
            "mode": "image",
            "loop_gain": 0.4,
            "box": {
                "camera": "Camera-1",
                "preset": "488nm",
                "slice_index": 1,
                "rect": [100, 60, 40, 30],
            },
        },
    }
    with pytest.warns(RuntimeWarning):
        cleaned = _settings._good_data_only(SettingsV1, old)
    s = SettingsV1.model_validate(cleaned)
    assert s.image_focus.mode == "spot"
    assert s.image_focus.loop_gain == 0.4
    assert s.spot_focus.calibration is None
    assert s.spot_focus.channel == "red"
    # A bad spot section degrades to defaults and keeps the rest.
    bad = {"image_focus": {"mode": "image"}, "spot_focus": {"channel": "purple"}}
    with pytest.warns(RuntimeWarning):
        cleaned = _settings._good_data_only(SettingsV1, bad)
    assert cleaned["image_focus"] == {}
    assert cleaned.get("spot_focus", {}) == {}


def test_source_spec_from_settings() -> None:
    cfg = SettingsV1().spot_focus
    cfg.source.serial = "abc"
    spec = source_spec(cfg)
    assert spec == {
        "kind": "toupcam",
        "serial": "abc",
        "analysis": {"channel": "red", "bayer": "RGGB"},
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
        "moves",
        "move_ms",
        "edge",
        "contrast",
        "fps",
        "error",
    ]
    assert "lateral" not in panel.readout_labels
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


def test_start_telemetry_stop_restores_crisp(
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
    assert mmc.props[(CRISP, "CRISP State")] == "Ready"
    assert spot_focus_flag.active
    # Calibrate and Zero are only for a stopped loop.
    assert not panel.calibrate_button.isEnabled()
    assert not panel.zero_button.isEnabled()
    assert not panel.draw_edge_button.isEnabled()
    assert panel.stop_button.isEnabled()

    qtbot.waitUntil(lambda: len(panel.chart) > 5, timeout=WAIT_MS)
    # The lock streams frames whether or not a window is open: open it now,
    # while the lock runs, and it fills with frames at the sensor offset.
    assert not panel.live_view.isVisible()
    panel.preview_button.click()
    assert panel.live_view.isVisible()
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
    with patch.object(svc, "set_edge_box", wraps=svc.set_edge_box) as box_spy:
        panel.stop()
        box_spy.assert_called_with(BOX)  # the preview measures the drawn box again
    assert panel.state == "stopped"
    assert not spot_focus_flag.active
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"  # restored
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
    assert stored["kind"] == "edge"
    assert stored["slope_px_per_um"] == pytest.approx(SLOPE, rel=0.1)  # signed
    assert stored["axis"] == "x" and stored["polarity"] == 1
    assert stored["n_points"] == 7
    assert stored["linear_range_um"] == pytest.approx(0.8)
    assert stored["p0_um"] == pytest.approx(P0)
    assert stored["e0_px"] == pytest.approx(_e0(), abs=0.5)
    assert tuple(stored["box"]) == BOX
    assert panel.start_button.isEnabled() and panel.zero_button.isEnabled()
    assert mmc.pos[P_LABEL] == pytest.approx(P0)  # P returned to where it was
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    text = panel.calibration_label.text()
    assert "px/µm" in text and "R²" in text and "zero P" in text
    assert re.search(r"\+[45]\.\d\d px/µm", text), text  # the right sign
    # a new zero: the offset is back to 0 and the ring sits on the zero edge point
    assert panel.offset_spin.value() == 0.0
    assert cfg.params.offset_um == 0.0
    ring = panel.live_view._setpoint_item
    assert ring.isVisible()
    assert ring.pos().x() == pytest.approx(stored["e0_px"] + 0.5)
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
    assert stored["p0_um"] == pytest.approx(P0 + 0.4)
    assert stored["e0_px"] == pytest.approx(_e0() + SLOPE * 0.4, abs=0.4)
    assert stored["slope_px_per_um"] == SLOPE  # kept
    assert panel.offset_spin.value() == 0.0
    assert settings.spot_focus.params.offset_um == 0.0
    assert panel._cal is not None and panel._cal.p0_um == pytest.approx(P0 + 0.4)
    assert panel.live_view._setpoint_item.pos().x() == pytest.approx(
        stored["e0_px"] + 0.5
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
    assert stored["slope_px_per_um"] == pytest.approx(SLOPE, rel=0.1)
    assert stored["p0_um"] == pytest.approx(P0)  # the present P is the zero
    assert panel._cal is not None
    assert cfg.params.offset_um == 0.0  # a new zero
    assert mmc.props[(CRISP, "CRISP State")] == "Ready"
    assert spot_focus_flag.active
    qtbot.waitUntil(lambda: len(panel.chart) > 3, timeout=WAIT_MS)
    panel.stop()
    assert panel.state == "stopped"
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"


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
    assert not spot_focus_flag.active
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"
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
    assert not spot_focus_flag.active
    assert widget.start_button.isEnabled()


# ---------------------------------------------------------------------------
# dock: the panel is hosted directly (Spot is the only mode)
# ---------------------------------------------------------------------------


def _image_cfg() -> _settings.ImageFocusSettingsV1:
    # "image" is no longer a settings value; build the legacy state directly.
    cfg = _settings.ImageFocusSettingsV1.model_construct(mode="image")
    cfg.box = FocusBoxSettings(
        camera="Camera-1", preset="488nm", slice_index=1, rect=(100, 60, 40, 30)
    )
    cfg.calibrations = {
        "Camera-1|488nm|": CalibrationRecord(
            rows_per_um=4.0,
            intercept_rows=0.0,
            r2=0.998,
            range_um=(-1.5, 1.5),
            n_points=7,
            date="2026-10-26T12:00:00",
        )
    }
    return cfg


def _image_run(cfg: _settings.ImageFocusSettingsV1, seq=None) -> ImageFocusRun:
    return ImageFocusRun.for_sequence(
        MagicMock(),
        HardwareConstants(),
        seq or useq.MDASequence(channels=[useq.Channel(config="488nm", exposure=10)]),
        crisp_locked=lambda: False,
        image_focus=cfg,
        spectral=_settings.SpectralChannelSettingsV1(),
    )


def test_image_lock_skips_while_spot_runs(qtbot: QtBot, panel: SpotFocusPanel) -> None:
    cfg = _image_cfg()
    assert _image_run(cfg).active  # control: the image lock arms normally
    cal_seq = useq.MDASequence(
        channels=[useq.Channel(config="488nm", exposure=10)],
        metadata={
            CALIBRATION_METADATA_KEY: {
                "steps_um": [-1.0, 0.0, 1.0],
                "reps": 1,
                "box": {
                    "camera": "Camera-1",
                    "preset": "488nm",
                    "slice_index": 1,
                    "rect": [100, 60, 40, 30],
                },
            }
        },
    )
    assert _image_run(cfg, cal_seq).active

    _calibrated(SettingsV1.instance())
    panel.refresh()
    panel.start()
    qtbot.waitUntil(lambda: panel.state == "running", timeout=WAIT_MS)
    run = _image_run(cfg)  # even if the setting says "image"
    assert not run.active
    assert run.mode == "off"
    assert not run.owns_piezo
    assert not _image_run(cfg, cal_seq).active  # nor a calibration sweep

    panel.stop()
    assert _image_run(cfg).active  # armed again once spot has stopped


def test_spot_mode_does_not_arm_image_lock() -> None:
    cfg = _image_cfg()
    cfg.mode = "spot"
    run = _image_run(cfg)
    assert not run.active
    assert run.mode == "off"


def test_dock_hosts_the_spot_panel_directly(qtbot: QtBot) -> None:
    parent = QWidget()
    qtbot.addWidget(parent)
    widget = create_image_focus_widget(parent)
    assert isinstance(widget, SpotFocusScrollArea)
    panel = widget.spot_panel
    assert isinstance(panel, SpotFocusPanel)
    assert widget.widget() is panel  # no mode radio, no wrapper
    assert not hasattr(widget, "mode_buttons")
    assert panel.start_button.text() == "Lock"
    panel.shutdown()


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


def test_preview_lights_the_led_and_off_restores_crisp(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    """The CRISP LED is lit only in Ready: Preview must put CRISP there."""
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    _preview_on(qtbot, panel)
    # Without the fix CRISP stays Idle here and the camera sees a dark frame.
    qtbot.waitUntil(lambda: mmc.props[(CRISP, "CRISP State")] == "Ready", timeout=3000)
    qtbot.waitUntil(lambda: panel.led_button.isChecked(), timeout=WAIT_MS)
    assert mmc.props[(CRISP, "LED Intensity")] == "50.0"  # the default
    assert panel.led_spin.value() == 50
    assert "Ready" in panel.crisp_readout.text()
    assert CRISP in panel.crisp_readout.text()
    panel.preview_button.click()  # off
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    qtbot.waitUntil(lambda: not panel.led_button.isChecked(), timeout=WAIT_MS)
    assert "Idle" in panel.crisp_readout.text()


def test_led_button_and_spin_apply_live_and_are_saved(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC, _isolated_settings: SettingsV1
) -> None:
    panel.led_button.click()
    qtbot.waitUntil(
        lambda: mmc.props[(CRISP, "CRISP State")] == "Ready", timeout=WAIT_MS
    )
    panel.led_spin.setValue(80)
    assert mmc.props[(CRISP, "LED Intensity")] == "80.0"
    assert _isolated_settings.spot_focus.led_intensity == 80.0
    panel.led_button.click()  # off
    qtbot.waitUntil(
        lambda: mmc.props[(CRISP, "CRISP State")] == "Idle", timeout=WAIT_MS
    )


def test_no_crisp_device_is_said_not_silent(qtbot: QtBot, mmc: FakeMMC) -> None:
    def make(spec: dict, led: float | None) -> SpotFocusService:
        return SpotFocusService(mmc, P_LABEL, None, spec, ready_timeout_s=25.0)

    widget = SpotFocusPanel(mmc, service_factory=make)
    qtbot.addWidget(widget)
    try:
        widget.preview_button.click()
        qtbot.waitUntil(lambda: not widget._preview_busy, timeout=WAIT_MS)
        assert "no CRISP device (focus.crisp_label)" in widget.fault_label.text()
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

    # Preview off stops the stream and hides the window.
    panel.preview_button.click()
    assert not panel.preview_button.isChecked()
    assert not view.isVisible()
    assert not svc.preview_active


def test_closing_the_live_view_turns_preview_off(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    svc = _preview_on(qtbot, panel)
    panel.live_view.close()
    assert not panel.preview_button.isChecked()
    assert not svc.preview_active
    # ... and Preview opens it again.
    panel.preview_button.click()
    qtbot.waitUntil(
        lambda: panel.live_view.isVisible() and svc.preview_active, timeout=WAIT_MS
    )
    assert panel.preview_button.isChecked()


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

    panel.live_view.close()
    assert panel.state == "running"
    assert svc.current_state == "running"
    panel.stop()
    assert not svc.preview_active  # the closed view's stream stops with the lock


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
        "pymmcore_gui.widgets._spot_focus_panel.QFileDialog.getExistingDirectory",
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
    try:
        widget.preview_button.click()
        qtbot.waitUntil(lambda: not widget._preview_busy, timeout=WAIT_MS)
        assert "SDK folder not found" in widget.fault_label.text()
        assert not widget.preview_button.isChecked()
        assert widget.preview_button.isEnabled()
        assert not widget.live_view.isVisible()
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
