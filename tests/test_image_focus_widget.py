"""Tests for the image focus lock panel and the viewer's focus box.

Fixtures here are self-contained (a bare ``CMMCorePlus`` with no devices, an
isolated settings object) so they also run with ``--noconftest``.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING
from unittest.mock import patch

import ndv
import numpy as np
import pytest
import useq
from pymmcore_plus import CMMCorePlus

from pymmcore_gui import _focus_box as focus_box_module
from pymmcore_gui import _ndv_viewers
from pymmcore_gui._argus_stream._session import CameraGeometry, _build_session_header
from pymmcore_gui._focus_box import (
    ViewerFocusBox,
    bbox_to_focus_box,
    default_bbox,
    focus_box_to_bbox,
)
from pymmcore_gui._ndv_viewers import NDVViewersManager
from pymmcore_gui._qt.QtWidgets import QPushButton, QWidget
from pymmcore_gui._settings import (
    ArgusStreamSettingsV1,
    CalibrationRecord,
    FocusBoxSettings,
    SettingsV1,
)
from pymmcore_gui.asi_z_stack.image_focus import FocusCalibration
from pymmcore_gui.asi_z_stack.image_focus_events import (
    CALIBRATION_METADATA_KEY,
    FocusLockReport,
    focus_lock_signals,
)
from pymmcore_gui.asi_z_stack.image_focus_run import build_calibration_sequence
from pymmcore_gui.widgets._image_focus_widget import (
    ALERT_COLOR,
    ImageFocusWidget,
    age_text,
    calibration_steps,
)

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pytestqt.qtbot import QtBot

BOX = FocusBoxSettings(
    camera="Camera-1", preset="488nm", slice_index=1, rect=(100, 60, 40, 30)
)
KEY = "Camera-1|488nm|"
FIT = FocusCalibration(
    rows_per_um=4.0,
    intercept_rows=0.0,
    r2=0.998,
    range_um=(-1.5, 1.5),
    n_points=7,
    date="2026-10-26T12:00:00",
)


@pytest.fixture(autouse=True)
def _isolated_settings() -> Iterator[SettingsV1]:
    """Fresh default settings (and no disk writes: flush is a no-op in pytest)."""
    fresh = SettingsV1()
    with patch("pymmcore_gui._settings._GLOBAL_SETTINGS", fresh):
        yield fresh


@pytest.fixture
def core() -> CMMCorePlus:
    return CMMCorePlus()


def _sequence() -> useq.MDASequence:
    return useq.MDASequence(
        channels=[
            useq.Channel(config="488nm", exposure=10),
            useq.Channel(config="560nm", exposure=20),
        ],
        z_plan=useq.ZRangeAround(range=2, step=1),
        time_plan=useq.TIntervalLoops(interval=0, loops=3),
    )


def _widget(qtbot: QtBot, core: CMMCorePlus, **kwargs) -> ImageFocusWidget:
    widget = ImageFocusWidget(core, **kwargs)
    qtbot.addWidget(widget)
    return widget


def _report(t: int, *, piezo: float, error_rows: float = 0.0, held: bool = False):
    from pymmcore_gui.asi_z_stack.image_focus import LockDecision

    decision = LockDecision(
        move_um=-0.1,
        target_um=piezo,
        error_rows=error_rows,
        error_um=error_rows / 4.0,
        confidence=0.93,
        held=held,
        reason="low confidence" if held else None,
    )
    return FocusLockReport({"t": t}, "image", piezo, decision, "")


# ---------------------------------------------------------------------------
# panel: settings
# ---------------------------------------------------------------------------


def test_mode_radios_write_settings(qtbot: QtBot, core: CMMCorePlus) -> None:
    cfg = SettingsV1.instance().image_focus
    cfg.mode = "off"
    widget = _widget(qtbot, core)
    assert widget.mode_buttons["off"].isChecked()  # shows what is saved

    widget.mode_buttons["image"].setChecked(True)
    assert cfg.mode == "image"
    widget.mode_buttons["crisp"].setChecked(True)
    assert cfg.mode == "crisp"
    widget.mode_buttons["off"].setChecked(True)
    assert cfg.mode == "off"


def test_limits_write_settings(qtbot: QtBot, core: CMMCorePlus) -> None:
    cfg = SettingsV1.instance().image_focus
    widget = _widget(qtbot, core)
    # the spinboxes start at the saved values
    assert widget.limit_spins["loop_gain"].value() == cfg.loop_gain
    assert widget.limit_spins["max_strikes"].value() == cfg.max_strikes

    widget.limit_spins["loop_gain"].setValue(0.25)
    widget.limit_spins["max_step_um"].setValue(0.15)
    widget.limit_spins["max_strikes"].setValue(7)
    widget.end_of_run.setCurrentIndex(widget.end_of_run.findData("stay"))
    assert cfg.loop_gain == 0.25
    assert cfg.max_step_um == 0.15
    assert cfg.max_strikes == 7
    assert cfg.end_of_run == "stay"
    # the engine's view of the same settings
    assert cfg.to_limits().loop_gain == 0.25


# ---------------------------------------------------------------------------
# panel: live status
# ---------------------------------------------------------------------------


def test_status_follows_reports_from_any_thread(
    qtbot: QtBot, core: CMMCorePlus, monkeypatch: pytest.MonkeyPatch
) -> None:
    on_gui_thread: list[bool] = []
    original = ImageFocusWidget._on_report

    def spy(self: ImageFocusWidget, report: FocusLockReport) -> None:
        on_gui_thread.append(threading.current_thread() is threading.main_thread())
        original(self, report)

    monkeypatch.setattr(ImageFocusWidget, "_on_report", spy)
    widget = _widget(qtbot, core)

    first = FocusLockReport({"t": 0}, "image", 50.0, None, "reference captured")
    worker = threading.Thread(target=focus_lock_signals.report.emit, args=(first,))
    worker.start()
    worker.join()
    qtbot.waitUntil(lambda: "locked" in widget.state_label.text(), timeout=2000)

    focus_lock_signals.report.emit(_report(3, piezo=49.7, error_rows=-1.2))
    qtbot.waitUntil(
        lambda: widget.status_labels["volume"].text() == "t=3", timeout=2000
    )
    labels = widget.status_labels
    assert labels["error"].text() == "-1.20 rows = -0.300 µm"
    assert labels["move"].text() == "-0.100 µm"
    assert labels["total"].text() == "-0.300 µm"  # 49.7 against the first 50.0
    assert labels["confidence"].text() == "0.93"
    assert "locked" in widget.state_label.text()
    assert not widget.state_is_alert

    focus_lock_signals.report.emit(_report(4, piezo=49.7, held=True))
    qtbot.waitUntil(lambda: "held" in widget.state_label.text(), timeout=2000)
    assert "low confidence" in widget.state_label.text()

    assert on_gui_thread and all(on_gui_thread)


def test_stopped_is_red_except_run_finished(qtbot: QtBot, core: CMMCorePlus) -> None:
    widget = _widget(qtbot, core)

    focus_lock_signals.stopped.emit("fell back: low confidence")
    qtbot.waitUntil(lambda: widget.state_is_alert, timeout=2000)
    text = widget.state_label.text()
    assert "stopped: fell back: low confidence" in text
    assert ALERT_COLOR in text

    focus_lock_signals.stopped.emit("run finished")
    qtbot.waitUntil(lambda: not widget.state_is_alert, timeout=2000)
    assert ALERT_COLOR not in widget.state_label.text()
    assert "idle" in widget.state_label.text()


def test_widget_disconnects_when_destroyed(qtbot: QtBot, core: CMMCorePlus) -> None:
    widget = ImageFocusWidget(core)
    relay = widget._relay
    widget.deleteLater()
    qtbot.waitUntil(lambda: not _alive(relay), timeout=2000)
    # An emission after the panel is gone must not reach its (deleted) relay:
    # that would raise out of the engine's emit.
    focus_lock_signals.report.emit(_report(0, piezo=50.0))
    focus_lock_signals.stopped.emit("run finished")
    focus_lock_signals.calibrated.emit(KEY, FIT)
    focus_lock_signals.box_changed.emit()


def _alive(obj: object) -> bool:
    try:
        obj.objectName()  # type: ignore[attr-defined]
    except RuntimeError:
        return False
    return True


# ---------------------------------------------------------------------------
# panel: box and calibration
# ---------------------------------------------------------------------------


def test_box_status_and_clear(qtbot: QtBot, core: CMMCorePlus) -> None:
    cfg = SettingsV1.instance().image_focus
    widget = _widget(qtbot, core)
    assert widget.box_label.text() == "none drawn"
    assert not widget.clear_button.isEnabled()

    cfg.box = BOX
    widget.refresh()
    assert "Camera-1, 488nm, slice 1" in widget.box_label.text()
    assert "not calibrated" in widget.calibration_label.text()

    cfg.calibrations[KEY] = CalibrationRecord.from_calibration(FIT)
    widget.refresh()
    text = widget.calibration_label.text()
    assert "4.00 rows/µm (sign +)" in text
    assert "R² 0.998" in text
    assert "-1.50..+1.50 µm" in text
    assert "2026-10-26 12:00" in text

    assert widget.clear_button.isEnabled()
    widget.clear_button.click()
    assert cfg.box is None
    assert widget.box_label.text() == "none drawn"


def test_calibration_steps_and_age() -> None:
    assert calibration_steps(1.5, 0.5) == [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
    assert calibration_steps(1.4, 0.5) == [-1.0, -0.5, 0.0, 0.5, 1.0]
    from datetime import datetime

    now = datetime(2026, 10, 26, 14, 0)
    assert age_text("2026-10-26T12:00:00", now) == "2026-10-26 12:00 (2 h ago)"
    assert age_text("2026-10-24T12:00:00", now).endswith("(2 d ago)")
    assert age_text("garbage") == "garbage"


def test_calibrate_builds_sequence_and_runs_it(qtbot: QtBot, core: CMMCorePlus) -> None:
    SettingsV1.instance().image_focus.box = BOX
    widget = _widget(qtbot, core, sequence_source=_sequence)
    widget.range_spin.setValue(1.0)
    widget.step_spin.setValue(0.5)
    widget.reps_spin.setValue(3)
    assert widget.calibrate_button.isEnabled()

    with patch.object(core, "run_mda") as run_mda:
        widget.calibrate_button.click()

    run_mda.assert_called_once()
    (sequence,) = run_mda.call_args.args
    payload = sequence.metadata[CALIBRATION_METADATA_KEY]
    assert payload["steps_um"] == [-1.0, -0.5, 0.0, 0.5, 1.0]
    assert payload["reps"] == 3
    assert payload["box"] == {
        "camera": "Camera-1",
        "preset": "488nm",
        "slice_index": 1,
        "rect": [100, 60, 40, 30],
    }
    assert [c.config for c in sequence.channels] == ["488nm"]
    assert sequence.time_plan.loops == 15  # 5 steps x 3 repeats
    assert "Calibrating" in widget.calibration_result.text()


def test_calibrate_defaults_match_the_sweep_default(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    widget = _widget(qtbot, core)
    assert widget.range_spin.value() == 1.5
    assert widget.step_spin.value() == 0.5
    assert widget.reps_spin.value() == 2


def test_calibrate_disabled_without_box_or_while_running(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    cfg = SettingsV1.instance().image_focus
    widget = _widget(qtbot, core, sequence_source=_sequence)
    assert not widget.calibrate_button.isEnabled()  # no box

    cfg.box = BOX
    widget.refresh()
    assert widget.calibrate_button.isEnabled()

    widget._set_mda_running(True)
    assert not widget.calibrate_button.isEnabled()
    widget._set_mda_running(False)
    assert widget.calibrate_button.isEnabled()

    # no acquisition to calibrate against
    assert not _widget(qtbot, core).calibrate_button.isEnabled()


def test_calibrate_reports_a_plan_that_cannot_work(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    box = BOX.model_copy(update={"preset": "405nm"})  # not in the plan
    SettingsV1.instance().image_focus.box = box
    widget = _widget(qtbot, core, sequence_source=_sequence)
    with patch.object(core, "run_mda") as run_mda:
        widget.calibrate()
    run_mda.assert_not_called()
    assert "Not started" in widget.calibration_result.text()
    assert "405nm" in widget.calibration_result.text()


def test_calibration_result_and_failure_are_shown(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    cfg = SettingsV1.instance().image_focus
    cfg.box = BOX
    widget = _widget(qtbot, core, sequence_source=_sequence)
    with patch.object(core, "run_mda"):
        widget.calibrate()

    cfg.calibrations[KEY] = CalibrationRecord.from_calibration(FIT)
    focus_lock_signals.calibrated.emit(KEY, FIT)
    qtbot.waitUntil(
        lambda: "Calibrated" in widget.calibration_result.text(), timeout=2000
    )
    assert "R² 0.998" in widget.calibration_result.text()
    assert "4.00 rows/µm" in widget.calibration_label.text()  # box status refreshed

    # a later failed calibration shows its reason
    with patch.object(core, "run_mda"):
        widget.calibrate()
    focus_lock_signals.stopped.emit(
        "calibration fit r2 0.500 is below 0.900; not saved"
    )
    qtbot.waitUntil(
        lambda: "Calibration failed" in widget.calibration_result.text(), timeout=2000
    )
    assert "r2 0.500" in widget.calibration_result.text()


# ---------------------------------------------------------------------------
# focus box on the viewer
# ---------------------------------------------------------------------------


def test_bbox_roundtrip_with_roi_origin() -> None:
    origin = (100, 50)
    box = FocusBoxSettings(
        camera="Camera-1", preset="488nm", slice_index=2, rect=(150, 100, 80, 40)
    )
    bbox = focus_box_to_bbox(box, origin)
    assert bbox == ((50, 50), (130, 90))  # array indices: the origin is gone

    back = bbox_to_focus_box(
        bbox, origin, camera="Camera-1", preset="488nm", slice_index=2
    )
    assert back == box


def test_bbox_to_focus_box_rounds_orders_and_clips() -> None:
    kwargs = {"camera": "C", "preset": "p", "slice_index": 0}
    # corners in any order, fractional pixels
    box = bbox_to_focus_box(((20.4, 30.6), (10.2, 10.0)), (0, 0), **kwargs)
    assert box is not None
    assert box.rect == (10, 10, 10, 21)
    # clipped to the frame, then moved to full-sensor coordinates
    box = bbox_to_focus_box(
        ((-5, -5), (50, 500)), (100, 200), frame_size=(40, 60), **kwargs
    )
    assert box is not None
    assert box.rect == (100, 200, 40, 60)
    # nothing left, or a zero-size box
    assert (
        bbox_to_focus_box(((100, 0), (120, 10)), (0, 0), frame_size=(40, 60), **kwargs)
        is None
    )
    assert bbox_to_focus_box(((5, 5), (5, 9)), (0, 0), **kwargs) is None


def test_default_bbox_is_centred() -> None:
    assert default_bbox((400, 200)) == ((150, 75), (250, 125))


def _viewer_box(core: CMMCorePlus, camera: str = "Camera-1"):
    viewer = ndv.ArrayViewer()
    button = QPushButton("Focus box")
    button.setCheckable(True)
    with (
        patch.object(focus_box_module, "camera_roi", return_value=(100, 50, 400, 300)),
        patch.object(
            focus_box_module, "physical_camera_labels", return_value=["Camera-1"]
        ),
    ):
        controller = ViewerFocusBox(viewer, _sequence(), camera, button, core)
    return viewer, button, controller


def test_viewer_shows_the_saved_box_for_its_camera(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    SettingsV1.instance().image_focus.box = FocusBoxSettings(
        camera="Camera-1", preset="488nm", slice_index=1, rect=(150, 100, 80, 40)
    )
    viewer, button, _ = _viewer_box(core)
    assert button.isChecked()
    assert viewer.roi is not None
    assert viewer.roi.visible
    assert viewer.roi.bounding_box == ((50, 50), (130, 90))

    # another camera's viewer shows nothing
    viewer2, button2, _ = _viewer_box(core, camera="Camera-2")
    assert viewer2.roi is None
    assert not button2.isChecked()


def test_dragging_the_roi_saves_the_box_for_the_displayed_slice_and_channel(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    cfg = SettingsV1.instance().image_focus
    cfg.box = BOX
    viewer, button, controller = _viewer_box(core)
    seen: list[int] = []
    focus_lock_signals.box_changed.connect(lambda: seen.append(1))
    try:
        # the user moves to slice 2 of channel 1 (560nm), then drags the ROI
        viewer.display_model.current_index.update({"z": 2, "c": 1})
        assert controller.selection() == ("560nm", 2)
        assert viewer.roi is not None
        # what the canvas emits while the user drags a handle (ndv's own path:
        # view signal -> controller -> ROI model -> our debounced commit)
        assert viewer._roi_view is not None
        viewer._roi_view.boundingBoxChanged.emit(((60.0, 70.0), (110.0, 120.0)))
        assert viewer.roi.bounding_box == ((60.0, 70.0), (110.0, 120.0))
        assert cfg.box == BOX  # debounced: still the old box mid-drag
        qtbot.waitUntil(lambda: cfg.box != BOX, timeout=3000)
    finally:
        focus_lock_signals.box_changed.disconnect()
    assert cfg.box == FocusBoxSettings(
        camera="Camera-1",
        preset="560nm",
        slice_index=2,
        rect=(160, 120, 50, 50),  # + the camera ROI origin (100, 50)
    )
    assert seen == [1]


def test_focus_box_button_creates_a_default_box_and_hides_it(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    cfg = SettingsV1.instance().image_focus
    viewer, button, _ = _viewer_box(core)
    assert viewer.roi is None and cfg.box is None

    button.setChecked(True)
    assert viewer.roi is not None and viewer.roi.visible
    assert viewer.roi.bounding_box == ((150, 112), (250, 187))
    assert cfg.box is not None
    assert cfg.box.camera == "Camera-1"
    assert cfg.box.preset == "488nm"  # channel 0 at slice 0
    assert cfg.box.slice_index == 0
    assert cfg.box.rect[2:] == (100, 75)

    button.setChecked(False)
    assert not viewer.roi.visible
    assert cfg.box is not None  # hiding the overlay keeps the box


def test_clearing_the_box_hides_the_roi(qtbot: QtBot, core: CMMCorePlus) -> None:
    SettingsV1.instance().image_focus.box = BOX
    viewer, button, _ = _viewer_box(core)
    assert button.isChecked()
    widget = _widget(qtbot, core)
    widget.clear_button.click()
    assert not button.isChecked()
    assert viewer.roi is not None and not viewer.roi.visible


# ---------------------------------------------------------------------------
# calibration runs open no viewer and are not streamed
# ---------------------------------------------------------------------------


def _manager(core: CMMCorePlus, qtbot: QtBot) -> NDVViewersManager:
    parent = QWidget()
    qtbot.addWidget(parent)
    manager = NDVViewersManager(parent, core)
    manager._test_parent = parent  # type: ignore[attr-defined]  # keep it alive
    return manager


def test_viewers_manager_skips_calibration_sequences(
    qtbot: QtBot, core: CMMCorePlus
) -> None:
    manager = _manager(core, qtbot)
    calibration = build_calibration_sequence(
        _sequence(),
        BOX.to_box(),
        steps_um=(-1.0, 0.0, 1.0),
        reps=1,
    )
    assert CALIBRATION_METADATA_KEY in calibration.metadata

    with patch.object(
        _ndv_viewers, "physical_camera_labels", return_value=["Camera-1"]
    ):
        # a normal run opens a viewer and a display store ...
        manager._on_sequence_started(_sequence(), {})  # type: ignore[arg-type]
        assert len(manager) == 1
        assert manager._own_handler is not None
        first_viewer = manager._active_mda_viewer  # the manager holds it weakly
        manager._on_sequence_finished(_sequence())

        # ... a calibration sweep after it opens nothing and drops the old store
        manager._on_sequence_started(calibration, {})  # type: ignore[arg-type]
    assert len(manager) == 1  # still only the first run's viewer
    assert first_viewer is not None
    assert manager._own_handler is None
    assert manager._active_mda_viewer is None
    assert manager._mda_camera_handlers == {}

    # its frames go nowhere (no crash, nothing written)
    event = next(iter(calibration))
    manager._on_frame_ready(np.zeros((4, 4), "uint16"), event, {})  # type: ignore[arg-type]
    manager._on_sequence_finished(calibration)
    assert not manager._is_mda_running


def test_calibration_sequence_is_not_streamed_to_argus() -> None:
    """It carries no save name/dir, which is what makes the streamer skip it."""
    base = _sequence().model_copy(
        update={
            "metadata": {
                "pymmcore_widgets": {"save_dir": "S:/exp", "save_name": "run.ome.zarr"}
            }
        }
    )
    calibration = build_calibration_sequence(
        base, BOX.to_box(), steps_um=(-1.0, 0.0, 1.0), reps=1
    )
    geometry = CameraGeometry(["Camera-1"], "uint16", 6, 8, 0.11)
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(base, geometry, settings, [])
    assert header is not None  # the base run itself would stream
    header, reason = _build_session_header(calibration, geometry, settings, [])
    assert header is None
    assert "experiment name" in reason
