"""The spot focus lock's settings and panel, with a real spawned worker.

A ``FakeSpotSource`` stands in for the camera and a fake mmc records every
``setPosition``. Fixtures are self-contained (isolated settings, a bare
``CMMCorePlus``), so this runs with ``--noconftest`` like
``test_image_focus_widget.py``.
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest
import useq
from pymmcore_plus import CMMCorePlus

from pymmcore_gui import _settings
from pymmcore_gui._settings import (
    CalibrationRecord,
    FocusBoxSettings,
    SettingsV1,
)
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.image_focus_events import (
    CALIBRATION_METADATA_KEY,
    spot_focus_flag,
)
from pymmcore_gui.asi_z_stack.image_focus_run import ImageFocusRun
from pymmcore_gui.asi_z_stack.spot_focus_service import SpotFocusService
from pymmcore_gui.widgets._image_focus_widget import ImageFocusWidget
from pymmcore_gui.widgets._spot_focus_panel import SpotFocusPanel, source_spec

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pytestqt.qtbot import QtBot

P_LABEL = "PiezoStage:P:34"
CRISP = "CRISP"
P0 = 50.0
UX, UY = 0.8, 0.6
SLOPE = 10.0
CAL = {
    "x0": 320.0,
    "y0": 240.0,
    "ux": UX,
    "uy": UY,
    "slope_px_per_um": SLOPE,
    "p0_um": P0,
}
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
    fresh.spot_focus.source.fake = {"cal_like": dict(CAL), "noise": 2.0, "fps": 50.0}
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


def _calibrated(settings: SettingsV1) -> None:
    settings.spot_focus.calibration = dict(CAL)


# ---------------------------------------------------------------------------
# settings
# ---------------------------------------------------------------------------


def test_spot_settings_defaults_and_roundtrip() -> None:
    s = SettingsV1()
    assert s.image_focus.mode == "crisp"
    cfg = s.spot_focus
    assert (cfg.channel, cfg.source.kind) == ("red", "toupcam")
    assert cfg.calibration is None
    assert cfg.to_calibration() is None
    assert cfg.params.to_params().deadband_um == pytest.approx(0.03)

    s.image_focus.mode = "spot"
    cfg.channel = "sum"
    cfg.roi = (10, 20, 300, 200)
    cfg.led_intensity = 40.0
    cfg.params.kp = 0.3
    cfg.calibration = dict(CAL)
    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    assert s2.image_focus.mode == "spot"
    assert s2.spot_focus == s.spot_focus
    assert s2.spot_focus.roi == (10, 20, 300, 200)
    assert s2.spot_focus.to_calibration().slope_px_per_um == SLOPE  # type: ignore[union-attr]
    assert s2.spot_focus.params.to_params().kp == pytest.approx(0.3)


def test_old_settings_files_still_load() -> None:
    """A file written before the spot lock (no ``spot_focus``) loads with defaults."""
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
    s = SettingsV1.model_validate(old)
    assert s.image_focus.mode == "image"
    assert s.image_focus.loop_gain == 0.4
    assert s.spot_focus.calibration is None
    assert s.spot_focus.channel == "red"
    # A bad spot section degrades to defaults and keeps the rest.
    bad = {"image_focus": {"mode": "image"}, "spot_focus": {"channel": "purple"}}
    with pytest.warns(RuntimeWarning):
        cleaned = _settings._good_data_only(SettingsV1, bad)
    assert cleaned["image_focus"] == {"mode": "image"}
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
    cfg.source.fake = {"cal_like": dict(CAL)}
    assert source_spec(cfg)["kind"] == "fake"


# ---------------------------------------------------------------------------
# panel
# ---------------------------------------------------------------------------


def test_start_needs_a_calibration(panel: SpotFocusPanel) -> None:
    assert not panel.start_button.isEnabled()
    assert panel.calibrate_button.isEnabled()
    panel.start()
    assert panel.service is None  # lazy: nothing was created
    assert panel.state == "idle"


def test_start_telemetry_stop_restores_crisp(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC, tmp_path: Path
) -> None:
    settings = SettingsV1.instance()
    _calibrated(settings)
    panel.refresh()
    assert panel.service is None
    assert panel.start_button.isEnabled()
    assert not panel.stop_button.isEnabled()

    panel.start()
    assert not panel.start_button.isEnabled()
    assert settings.image_focus.mode == "spot"
    qtbot.waitUntil(lambda: panel.state == "running", timeout=WAIT_MS)
    assert mmc.props[(CRISP, "CRISP State")] == "Ready"
    assert spot_focus_flag.active
    # Calibrate is only for a stopped loop.
    assert not panel.calibrate_button.isEnabled()
    assert panel.stop_button.isEnabled()

    qtbot.waitUntil(lambda: len(panel.chart) > 5, timeout=WAIT_MS)
    qtbot.waitUntil(lambda: panel.thumb_view.has_image, timeout=WAIT_MS)
    spot, setpoint = panel.thumb_view.markers
    assert spot is not None
    assert setpoint is not None
    # The fake spot sits at the setpoint, so the two markers nearly coincide.
    assert abs(spot[0] - setpoint[0]) < 3
    assert abs(spot[1] - setpoint[1]) < 3
    assert panel.readout_labels["fps"].text() != "-"
    # The chart and thumbnail paint without raising (a paint error aborts Qt).
    panel.resize(500, 700)
    pixmap = panel.grab()
    assert not pixmap.isNull()
    pixmap.save(str(tmp_path / "spot_panel.png"))
    assert "µm" in panel.readout_labels["error"].text()

    # Editing a parameter while running reaches the settings (and the worker).
    panel.param_spins["deadband_um"].setValue(0.05)
    assert settings.spot_focus.params.deadband_um == pytest.approx(0.05)

    sets_before_stop = len(mmc.sets)
    panel.stop()
    assert panel.state == "stopped"
    assert not spot_focus_flag.active
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"  # restored
    assert panel.calibrate_button.isEnabled()
    assert panel.start_button.isEnabled()
    qtbot.wait(300)
    assert len(mmc.sets) - sets_before_stop <= 1  # at most one in flight


def test_calibrate_off_the_gui_thread(
    qtbot: QtBot, panel: SpotFocusPanel, mmc: FakeMMC
) -> None:
    panel.calibrate()
    assert panel._busy == "calibrating"
    assert not panel.calibrate_button.isEnabled()
    assert not panel.start_button.isEnabled()
    qtbot.waitUntil(lambda: not panel._busy, timeout=WAIT_MS)
    stored = SettingsV1.instance().spot_focus.calibration
    assert stored is not None, panel.fault_label.text()
    assert stored["slope_px_per_um"] == pytest.approx(SLOPE, rel=0.1)
    assert (stored["ux"], stored["uy"]) == pytest.approx((UX, UY), abs=0.05)
    assert panel.start_button.isEnabled()
    assert mmc.pos[P_LABEL] == pytest.approx(P0)  # P returned to where it was
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    assert "px/µm" in panel.calibration_label.text()


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
# dock: mode and mutual exclusion with the image lock
# ---------------------------------------------------------------------------


def _image_cfg() -> _settings.ImageFocusSettingsV1:
    cfg = _settings.ImageFocusSettingsV1(mode="image")
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


def test_dock_spot_radio_and_exclusion(qtbot: QtBot, mmc: FakeMMC) -> None:
    settings = SettingsV1.instance()
    widget = ImageFocusWidget(CMMCorePlus())
    qtbot.addWidget(widget)
    widget.spot_panel._factory = _factory(mmc)
    assert set(widget.mode_buttons) == {"crisp", "image", "spot", "off"}
    assert widget.spot_panel.isHidden()

    widget.mode_buttons["spot"].setChecked(True)
    assert settings.image_focus.mode == "spot"
    assert not widget.spot_panel.isHidden()

    _calibrated(settings)
    widget.spot_panel.refresh()
    widget.spot_panel.start()
    qtbot.waitUntil(lambda: widget.spot_panel.state == "running", timeout=WAIT_MS)
    assert mmc.props[(CRISP, "CRISP State")] == "Ready"

    # Choosing Image while spot runs stops spot first and restores CRISP.
    widget.mode_buttons["image"].setChecked(True)
    assert settings.image_focus.mode == "image"
    assert widget.spot_panel.state == "stopped"
    assert not spot_focus_flag.active
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    assert widget.spot_panel.isHidden()

    # Start from a different mode switches the dock to Spot.
    widget.spot_panel.start()
    qtbot.waitUntil(lambda: widget.spot_panel.state == "running", timeout=WAIT_MS)
    assert settings.image_focus.mode == "spot"
    assert widget.mode_buttons["spot"].isChecked()
    widget.spot_panel.shutdown()
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"


# ---------------------------------------------------------------------------
# thumbnail markers near the frame edge
# ---------------------------------------------------------------------------

EDGE_CAL = {**CAL, "x0": 8.0, "y0": 470.0}  # corner of a 640 x 480 frame


def test_thumb_message_carries_origin_near_the_edge() -> None:
    import numpy as np

    from pymmcore_gui.asi_z_stack.spot_focus import SpotCalibration
    from pymmcore_gui.asi_z_stack.spot_focus_worker import THUMB_MAX, thumb_message

    frame = np.zeros((480, 640), dtype=np.uint16)
    cal = SpotCalibration.from_dict(EDGE_CAL)
    msg = thumb_message(frame, 8.4, 470.2, cal)
    # The window is clamped into the frame, so the spot is not at its centre.
    assert (msg["x0"], msg["y0"]) == (0, 480 - THUMB_MAX)
    assert msg["scale"] == 1.0
    assert msg["image"].shape == (THUMB_MAX, THUMB_MAX)
    assert msg["spot"] == pytest.approx((8.4, 470.2))
    assert msg["setpoint"] == pytest.approx((8.0, 470.0))
    # The old "centred" assumption would put it 56 px off horizontally.
    assert msg["spot"][0] - msg["x0"] < THUMB_MAX // 2 - 50
    mid = thumb_message(frame, 320.0, 240.0)
    assert (mid["x0"], mid["y0"]) == (320 - THUMB_MAX // 2, 240 - THUMB_MAX // 2)
    assert mid["setpoint"] is None


def test_panel_places_markers_for_a_spot_at_the_edge(
    qtbot: QtBot, panel: SpotFocusPanel
) -> None:
    settings = SettingsV1.instance()
    settings.spot_focus.source.fake = {
        "cal_like": dict(EDGE_CAL),
        "noise": 2.0,
        "fps": 50.0,
        "sigma_px": 3.0,
    }
    settings.spot_focus.calibration = dict(EDGE_CAL)
    panel.refresh()
    panel.start()
    qtbot.waitUntil(lambda: panel.thumb_view.has_image, timeout=WAIT_MS)
    qtbot.waitUntil(lambda: panel.thumb_view.markers[0] is not None, timeout=WAIT_MS)
    spot, setpoint = panel.thumb_view.markers
    assert spot is not None
    assert setpoint is not None
    # Near the left/bottom edge the markers sit inside the crop at its edge,
    # at (8, 470 - 352) = (8, 118), not at its centre (64, 64).
    assert spot[0] == pytest.approx(8.0, abs=2.5)
    assert spot[1] == pytest.approx(118.0, abs=2.5)
    assert setpoint == pytest.approx((8.0, 118.0))
    panel.stop()
