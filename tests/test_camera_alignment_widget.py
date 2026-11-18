from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from pymmcore_gui import MicroManagerGUI
from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui._spot_tracking import displacement
from pymmcore_gui.actions._action_info import ActionInfo
from pymmcore_gui.actions.widget_actions import WidgetAction
from pymmcore_gui.widgets.camera_alignment import CameraAlignmentWidget
from pymmcore_gui.widgets.camera_alignment import (
    _camera_alignment_widget as widget_module,
)
from pymmcore_gui.widgets.camera_alignment._layers import MODE_CAMERAS, MODE_REGIONS

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

    from pymmcore_gui._qt.QtWidgets import QApplication

REF = "region:mNG_488nm"
K405 = "region:Electra_405nm"
K561 = "region:mScarlet_561nm"
K638 = "region:CF647_638nm"
CAM1, CAM2 = "camera:Camera", "camera:Camera-2"

# Region-local blob centres (x, y) for the four default channels, and where the
# regions sit on a 128x128 frame (top region at y=10, bottom at y=70).
BLOBS = {REF: (20.0, 20.0), K405: (22.0, 19.0), K561: (18.0, 23.0), K638: (23.0, 22.0)}
TOP_Y, BOTTOM_Y, REGION_X, REGION_SIZE = 10, 70, 10, 40


@pytest.fixture
def gui(qtbot: QtBot, qapp: QApplication) -> Iterator[MicroManagerGUI]:
    gui = MicroManagerGUI()
    qtbot.addWidget(gui)
    yield gui


def _blob_frame(shape: tuple[int, int], center: tuple[float, float]) -> np.ndarray:
    """A synthetic uint16 frame (matching real camera bit-depth) with one blob."""
    return _blobs_frame(shape, [center])


def _blobs_frame(
    shape: tuple[int, int], centers: list[tuple[float, float]]
) -> np.ndarray:
    h, w = shape
    ys, xs = np.indices((h, w))
    frame = np.full(shape, 50.0)
    for cx, cy in centers:
        frame += 1000.0 * np.exp(-(((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * 3.0**2)))
    return frame.astype(np.uint16)


def _four_regions(cam_a: str = "Camera", cam_b: str = "Camera-2") -> None:
    """Point the 4 default spectral channels at 2 cameras x top/bottom rects."""
    channels = SettingsV1.instance().spectral.channels
    cams = [cam_a, cam_a, cam_b, cam_b]
    ys = [TOP_Y, BOTTOM_Y, TOP_Y, BOTTOM_Y]
    for ch, cam, y in zip(channels, cams, ys, strict=True):
        ch.camera = cam
        ch.rect = (REGION_X, y, REGION_SIZE, REGION_SIZE)


def _region_frames() -> tuple[np.ndarray, np.ndarray]:
    """One 128x128 frame per camera holding that camera's two region blobs."""

    def at(key: str, y0: int) -> tuple[float, float]:
        return REGION_X + BLOBS[key][0], y0 + BLOBS[key][1]

    return (
        _blobs_frame((128, 128), [at(REF, TOP_Y), at(K405, BOTTOM_Y)]),
        _blobs_frame((128, 128), [at(K561, TOP_Y), at(K638, BOTTOM_Y)]),
    )


def _push(gui: MicroManagerGUI, cam: str, frame: np.ndarray) -> None:
    gui.viewers_manager.get_or_create_camera_preview(cam).append(frame)


def _make(
    gui: MicroManagerGUI, qtbot: QtBot, *, multi: bool = True
) -> CameraAlignmentWidget:
    if multi:
        gui.mmcore.setProperty("Core", "Camera", "Multi Camera")
    widget = CameraAlignmentWidget(parent=gui, mmcore=gui.mmcore)
    qtbot.addWidget(widget)
    return widget


def _pick_cameras(gui: MicroManagerGUI, widget: CameraAlignmentWidget) -> None:
    """Whole-camera setup: blobs at (60, 60) / (64, 65), polled, then picked."""
    _push(gui, "Camera", _blob_frame((128, 128), (60.0, 60.0)))
    _push(gui, "Camera-2", _blob_frame((128, 128), (64.0, 65.0)))
    widget._on_poll()
    # Seed picks via the real done-callbacks (exercises world->pixel conversion),
    # bypassing real pointer events -- no existing test drives pygfx pointer
    # events through Qt, matching that precedent.
    widget._on_pick_done(CAM1, (60.0 - 0.5, 60.0 - 0.5))
    widget._on_pick_done(CAM2, (64.0 - 0.5, 65.0 - 0.5))


def _push_cameras(gui: MicroManagerGUI, d1: float = 0.0, d2: float = 0.0) -> None:
    _push(gui, "Camera", _blob_frame((128, 128), (60.0 + d1, 60.0)))
    _push(gui, "Camera-2", _blob_frame((128, 128), (64.0 + d2, 65.0)))


def _region_widget(gui: MicroManagerGUI, qtbot: QtBot) -> CameraAlignmentWidget:
    """A widget on the 4-region setup, frames pushed and polled once."""
    _four_regions()
    widget = _make(gui, qtbot)
    f1, f2 = _region_frames()
    _push(gui, "Camera", f1)
    _push(gui, "Camera-2", f2)
    widget._on_poll()
    return widget


def _pick_all_regions(widget: CameraAlignmentWidget) -> None:
    widget._on_pick_all_done((21.0 - 0.5, 21.0 - 0.5))


# ----------------------------------------------------------------- basics


def test_camera_alignment_action_registered() -> None:
    """The alignment widget is registered as a widget action, like CRISP/Spectral."""
    infos = ActionInfo.widget_actions()
    assert WidgetAction.CAMERA_ALIGNMENT.value in infos
    assert infos[WidgetAction.CAMERA_ALIGNMENT.value].text == "Camera Alignment"


def test_disabled_with_default_single_camera(qtbot: QtBot, mmcore: CMMCorePlus) -> None:
    """`test_config.cfg`'s default active camera is a single physical camera."""
    widget = CameraAlignmentWidget(mmcore=mmcore)
    qtbot.addWidget(widget)

    assert not widget._pick_all_btn.isEnabled()
    assert not widget._clear_btn.isEnabled()
    assert not widget._poll_timer.isActive()
    assert "found 1 camera" in widget._status.text()
    assert "0 regions" in widget._status.text()


def test_enabled_with_multi_camera_active(qtbot: QtBot, mmcore: CMMCorePlus) -> None:
    mmcore.setProperty("Core", "Camera", "Multi Camera")
    widget = CameraAlignmentWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    widget._on_config_loaded()

    assert widget._pick_all_btn.isEnabled()
    assert widget._clear_btn.isEnabled()
    # No regions drawn -> whole-camera mode.
    assert widget._mode == MODE_CAMERAS
    assert [s.key for s in widget._specs] == [CAM1, CAM2]
    assert all(r._pick_btn.isEnabled() for r in widget._panel.rows())


def test_overlay_shows_frames_before_any_pick(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    """The overlay must render as soon as 2 layers resolve, before any pick.

    Picking is only needed for displacement tracking, not for the overlay
    view itself.
    """
    widget = _make(gui, qtbot)
    assert widget._poll_timer.isActive()

    _push(gui, "Camera", _blob_frame((128, 128), (60.0, 60.0)))
    _push(gui, "Camera-2", _blob_frame((128, 128), (64.0, 65.0)))
    widget._on_poll()

    for key in (CAM1, CAM2):
        assert widget._overlay.layer_data(key) is not None
        assert widget._overlay._layers[key].node.visible
    # No picks yet -- no tracking should have happened.
    assert widget._readout.text() == ""
    assert len(widget._chart._samples) == 0


def test_pick_and_track_updates_readout_and_chart(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _make(gui, qtbot)
    assert widget._ref_key == CAM1 and widget._chart_key == CAM2
    _pick_cameras(gui, widget)
    assert widget._poll_timer.isActive()

    _push_cameras(gui)
    widget._on_poll()

    assert widget._readout.text() != ""
    assert len(widget._chart._samples) == 1
    sample = widget._chart._samples[-1]
    assert sample.dx == pytest.approx(4.0, abs=0.2)
    assert sample.dy == pytest.approx(5.0, abs=0.2)
    assert sample.mag == pytest.approx((4.0**2 + 5.0**2) ** 0.5, abs=0.3)

    widget._on_clear_clicked()
    # Clear resets tracking state but the live overlay keeps polling.
    assert widget._poll_timer.isActive()
    assert widget._spots == {}
    assert len(widget._chart._samples) == 0
    assert widget._readout.text() == ""


def test_no_new_sample_without_new_frame(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    """Polling again on the same (unchanged) frames must not advance the chart.

    Acquisition being paused/stopped means the preview keeps returning the
    same frame object tick after tick; the strip chart should hold its last
    point rather than keep stamping fresh wall-clock times onto stale data.
    """
    widget = _make(gui, qtbot)
    _pick_cameras(gui, widget)
    _push_cameras(gui)

    widget._on_poll()
    assert len(widget._chart._samples) == 1

    # No new frames pushed -- simulates acquisition being paused. Repeated
    # polling must not add more samples.
    widget._on_poll()
    widget._on_poll()
    assert len(widget._chart._samples) == 1

    # A genuinely new frame arrives -- the chart should advance again.
    _push(gui, "Camera", _blob_frame((128, 128), (61.0, 60.0)))
    widget._on_poll()
    assert len(widget._chart._samples) == 2


def test_paused_during_mda(
    gui: MicroManagerGUI, qtbot: QtBot, monkeypatch: pytest.MonkeyPatch
) -> None:
    mmc = gui.mmcore
    widget = _make(gui, qtbot)
    _pick_cameras(gui, widget)
    _push_cameras(gui)

    # Scoped context, not a bare setattr: this must revert before the test
    # returns, not merely by end-of-test teardown. If it were still active when
    # the `gui` fixture closes the window, MicroManagerGUI.closeEvent would see
    # a (fake) running MDA and pop a real, blocking QMessageBox.exec() asking
    # to confirm cancellation -- hanging the whole test run on a click nobody
    # can give in an automated run.
    with monkeypatch.context() as m:
        m.setattr(mmc.mda, "is_running", lambda: True)
        widget._on_poll()

    assert len(widget._chart._samples) == 0
    assert "Paused during MDA" in widget._status.text()


# ------------------------------------------------------------ region layers


def test_four_regions_default_to_region_mode(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _region_widget(gui, qtbot)

    assert widget._mode == MODE_REGIONS
    assert widget._mode_combo.currentText() == MODE_REGIONS
    assert [s.key for s in widget._specs] == [REF, K405, K561, K638]
    assert widget._overlay.layer_keys() == [REF, K405, K561, K638]
    assert [r.key for r in widget._panel.rows()] == [REF, K405, K561, K638]
    # The green (488 nm) layer is the default reference.
    assert widget._ref_key == REF
    assert widget._ref_combo.currentData() == REF
    assert widget._chart_key == K405
    assert widget._chart_combo.findData(REF) == -1
    for key in widget._overlay.layer_keys():
        data = widget._overlay.layer_data(key)
        assert data is not None
        assert data.shape == (REGION_SIZE, REGION_SIZE)
        assert widget._overlay._layers[key].node.visible
    assert widget._overlay.color(REF) == "#00ff00"


def test_roi_origin_offset_is_honoured(
    gui: MicroManagerGUI, qtbot: QtBot, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Regions are in full-sensor coords; a cropped camera's frame starts at ROI."""
    cam = physical_camera_labels(gui.mmcore)[0]
    monkeypatch.setattr(widget_module, "roi_origin", lambda mmc, label: (100, 200))
    channels = SettingsV1.instance().spectral.channels
    channels[0].camera = channels[1].camera = cam
    channels[0].rect = (110, 210, 40, 40)
    channels[1].rect = (110, 270, 40, 40)
    widget = _make(gui, qtbot, multi=False)
    assert widget._mode == MODE_REGIONS

    # The sensor ROI is 128x128 starting at (100, 200); blob at frame (30, 30)
    # is region-local (20, 20) for the first region.
    _push(gui, cam, _blobs_frame((128, 128), [(30.0, 30.0), (35.0, 95.0)]))
    widget._on_poll()

    top = widget._overlay.layer_data(REF)
    bottom = widget._overlay.layer_data(K405)
    assert top is not None and bottom is not None
    assert top.shape == bottom.shape == (40, 40)
    assert divmod(int(np.argmax(top)), 40) == (20, 20)
    assert divmod(int(np.argmax(bottom)), 40) == (25, 25)


def test_region_outside_frame_shows_hint(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    cam = physical_camera_labels(gui.mmcore)[0]
    channels = SettingsV1.instance().spectral.channels
    channels[0].camera = channels[1].camera = cam
    channels[0].rect = (10, 10, 40, 40)
    channels[1].rect = (200, 200, 40, 40)  # off a 128x128 frame
    widget = _make(gui, qtbot, multi=False)

    _push(gui, cam, _blob_frame((128, 128), (30.0, 30.0)))
    widget._on_poll()  # must not raise

    bad = widget._panel.row(K405)
    good = widget._panel.row(REF)
    assert bad is not None and good is not None
    assert "outside frame" in bad._status_label.text()
    assert good._status_label.text() == ""
    assert widget._overlay.layer_data(K405) is None
    assert widget._overlay.layer_data(REF) is not None

    # A bigger frame (e.g. ROI reset) makes the region valid again.
    _push(gui, cam, _blob_frame((256, 256), (30.0, 30.0)))
    widget._on_poll()
    assert bad._status_label.text() == ""
    assert widget._overlay.layer_data(K405) is not None


def test_single_camera_top_and_bottom_regions(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    """The same-camera case: top and bottom regions of ONE camera."""
    cam = physical_camera_labels(gui.mmcore)[0]
    channels = SettingsV1.instance().spectral.channels
    channels[0].camera = channels[1].camera = cam
    channels[0].rect = (REGION_X, TOP_Y, REGION_SIZE, REGION_SIZE)
    channels[1].rect = (REGION_X, BOTTOM_Y, REGION_SIZE, REGION_SIZE)
    widget = _make(gui, qtbot, multi=False)

    assert widget._mode == MODE_REGIONS
    assert [s.key for s in widget._specs] == [REF, K405]
    assert widget._pick_all_btn.isEnabled()
    assert widget._poll_timer.isActive()

    f1, _ = _region_frames()
    _push(gui, cam, f1)
    widget._on_poll()
    _pick_all_regions(widget)
    _push(gui, cam, _region_frames()[0].copy())
    widget._on_poll()

    dx, dy, _ = displacement(widget._spots[REF], widget._spots[K405])
    assert dx == pytest.approx(2.0, abs=0.2)
    assert dy == pytest.approx(-1.0, abs=0.2)
    assert len(widget._chart._samples) == 1


def test_pick_all_four_regions_readout_relative_to_reference(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _region_widget(gui, qtbot)
    _pick_all_regions(widget)

    assert set(widget._spots) == {REF, K405, K561, K638}
    text = widget._readout.text()
    assert "Reference" in text and "mNG_488nm" in text
    assert text.count("dx = ") == 3
    for name in ("Electra_405nm", "mScarlet_561nm", "CF647_638nm"):
        assert name in text
    ref_spot = widget._spots[REF]
    for key in (K405, K561, K638):
        bx, by = BLOBS[key]
        dx, dy, mag = displacement(ref_spot, widget._spots[key])
        assert dx == pytest.approx(bx - BLOBS[REF][0], abs=0.2)
        assert dy == pytest.approx(by - BLOBS[REF][1], abs=0.2)
        assert f"{dx:+.2f}" in text and f"{dy:+.2f}" in text and f"{mag:.2f}" in text

    # Live tracking on new frames updates the chart for the chart layer (405).
    f1, f2 = _region_frames()
    _push(gui, "Camera", f1.copy())
    _push(gui, "Camera-2", f2.copy())
    widget._on_poll()
    assert len(widget._chart._samples) == 1
    assert widget._chart._samples[-1].dx == pytest.approx(2.0, abs=0.2)
    assert widget._chart._samples[-1].dy == pytest.approx(-1.0, abs=0.2)


def test_per_row_pick_button_seeds_only_that_layer(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _region_widget(gui, qtbot)
    row = widget._panel.row(K561)
    assert row is not None

    row._pick_btn.click()
    assert widget._overlay._pick_handler is not None
    widget._overlay._detach_pick_handler()  # restore pan/zoom; no real pointer events

    widget._on_pick_done(K561, (18.0 - 0.5, 23.0 - 0.5))
    assert set(widget._spots) == {K561}
    assert widget._spots[K561].x == pytest.approx(18.0, abs=0.2)
    assert widget._spots[K561].y == pytest.approx(23.0, abs=0.2)
    # The reference has no spot -> nothing to report relative to.
    assert widget._readout.text() == ""


# --------------------------------------------------------- layer controls


def test_toggle_visibility_hides_node_marker_and_readout(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _make(gui, qtbot)
    _pick_cameras(gui, widget)
    _push_cameras(gui)
    widget._on_poll()
    assert "dx = " in widget._readout.text()
    row = widget._panel.row(CAM2)
    assert row is not None
    shown = widget._overlay.layer_data(CAM2)
    assert shown is not None
    old_data = shown.copy()

    row._visible_cb.setChecked(False)

    assert not widget._overlay._layers[CAM2].node.visible
    assert widget._overlay._markers[CAM2]
    assert all(not obj.visible for obj in widget._overlay._markers[CAM2])
    assert widget._overlay._layers[CAM1].node.visible
    assert widget._readout.text() == ""  # the only other layer is hidden
    assert not widget._displays[CAM2].visible

    # Hidden layers are skipped entirely: a new frame is not pushed to them
    # and adds no chart sample.
    samples = len(widget._chart._samples)
    new_frame = _blob_frame((128, 128), (70.0, 70.0))
    _push(gui, "Camera-2", new_frame)
    widget._on_poll()
    stale = widget._overlay.layer_data(CAM2)
    assert stale is not None and np.array_equal(stale, old_data)
    assert len(widget._chart._samples) == samples

    # Re-showing re-pushes the latest frame on the next poll and resumes.
    row._visible_cb.setChecked(True)
    widget._on_poll()
    assert widget._overlay._layers[CAM2].node.visible
    assert all(obj.visible for obj in widget._overlay._markers[CAM2])
    fresh = widget._overlay.layer_data(CAM2)
    assert fresh is not None and np.array_equal(fresh, new_frame)
    assert "dx = " in widget._readout.text()


def test_color_change_recolors_layer_and_marker(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _make(gui, qtbot)
    _pick_cameras(gui, widget)
    row = widget._panel.row(CAM1)
    assert row is not None
    old_marker = tuple(widget._overlay._markers[CAM1][0].material.color)

    row._color_combo.setCurrentIndex(row._color_combo.findData("#ff0000"))

    assert widget._overlay.color(CAM1) == "#ff0000"
    assert widget._displays[CAM1].color == "#ff0000"
    marker_color = tuple(widget._overlay._markers[CAM1][0].material.color)
    assert marker_color != old_marker
    assert marker_color == pytest.approx((1.0, 0.0, 0.0, 1.0))


def test_manual_vs_auto_clim(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    widget = _make(gui, qtbot)
    frame = _blob_frame((128, 128), (60.0, 60.0))
    _push(gui, "Camera", frame)
    _push(gui, "Camera-2", _blob_frame((128, 128), (64.0, 65.0)))
    widget._on_poll()
    row = widget._panel.row(CAM1)
    assert row is not None
    overlay = widget._overlay

    assert overlay.is_auto_clim(CAM1)
    assert overlay.clim(CAM1) == (float(frame.min()), float(frame.max()))
    # The row mirrors the live auto range.
    assert (row._min_spin.value(), row._max_spin.value()) == (
        int(frame.min()),
        int(frame.max()),
    )

    row._auto_cb.setChecked(False)
    row._min_spin.setValue(100)
    row._max_spin.setValue(900)
    assert not overlay.is_auto_clim(CAM1)
    assert overlay.clim(CAM1) == (100.0, 900.0)
    assert not widget._displays[CAM1].auto_contrast
    assert widget._displays[CAM1].clim == (100.0, 900.0)

    # A new frame does not move a manual range.
    _push(gui, "Camera", _blob_frame((128, 128), (61.0, 60.0)))
    widget._on_poll()
    assert overlay.clim(CAM1) == (100.0, 900.0)

    row._auto_cb.setChecked(True)
    data = overlay.layer_data(CAM1)
    assert data is not None
    assert overlay.is_auto_clim(CAM1)
    assert overlay.clim(CAM1) == (float(data.min()), float(data.max()))


def test_reference_switching_changes_baseline_and_clears_chart(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _region_widget(gui, qtbot)
    _pick_all_regions(widget)
    f1, f2 = _region_frames()
    _push(gui, "Camera", f1.copy())
    _push(gui, "Camera-2", f2.copy())
    widget._on_poll()
    assert len(widget._chart._samples) == 1
    assert "mNG_488nm" in widget._readout.text().split("<br>")[0]

    widget._ref_combo.setCurrentIndex(widget._ref_combo.findData(K561))

    assert widget._ref_key == K561
    assert len(widget._chart._samples) == 0
    assert widget._chart_combo.findData(K561) == -1  # no layer vs itself
    assert widget._chart_key != K561
    text = widget._readout.text()
    assert text.split("<br>")[0].count("mScarlet_561nm") == 1
    assert text.count("dx = ") == 3
    dx, dy, _ = displacement(widget._spots[K561], widget._spots[REF])
    assert f"mNG_488nm: dx = {dx:+.2f}" in text.replace("</span>", "")

    # Changing the chart layer also restarts the trace.
    _push(gui, "Camera", f1.copy())
    widget._on_poll()
    assert len(widget._chart._samples) == 1
    widget._chart_combo.setCurrentIndex(widget._chart_combo.findData(K638))
    assert widget._chart_key == K638
    assert len(widget._chart._samples) == 0


def test_mode_switch_to_cameras_and_back(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    widget = _region_widget(gui, qtbot)
    _pick_all_regions(widget)
    assert widget._mode == MODE_REGIONS

    widget._mode_combo.setCurrentIndex(widget._mode_combo.findText(MODE_CAMERAS))

    assert widget._mode == MODE_CAMERAS
    assert widget._mode_user_chosen
    assert widget._overlay.layer_keys() == [CAM1, CAM2]
    assert [r.key for r in widget._panel.rows()] == [CAM1, CAM2]
    assert widget._spots == {}  # tracking is reset on a mode switch
    assert widget._poll_timer.isActive()
    # Whole-camera layers keep the historical magenta / green defaults.
    assert widget._overlay.color(CAM1) == "#ff00ff"
    assert widget._overlay.color(CAM2) == "#00ff00"
    _push(gui, "Camera", _region_frames()[0].copy())
    widget._on_poll()
    data = widget._overlay.layer_data(CAM1)
    assert data is not None and data.shape == (128, 128)

    widget._mode_combo.setCurrentIndex(widget._mode_combo.findText(MODE_REGIONS))

    assert widget._mode == MODE_REGIONS
    assert widget._overlay.layer_keys() == [REF, K405, K561, K638]
    assert widget._ref_key == REF


def test_spectral_change_triggers_rebuild_on_next_poll(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    widget = _region_widget(gui, qtbot)
    old_sig = widget._sig
    data = widget._overlay.layer_data(REF)
    assert data is not None and data.shape == (40, 40)

    channels = SettingsV1.instance().spectral.channels
    channels[0].rect = (REGION_X, TOP_Y, 30, 30)
    widget._on_poll()  # same frames, but the layer set changed -> rebuild + re-push

    assert widget._sig != old_sig
    spec = next(s for s in widget._specs if s.key == REF)
    assert spec.rect == (REGION_X, TOP_Y, 30, 30)
    data = widget._overlay.layer_data(REF)
    assert data is not None and data.shape == (30, 30)

    # Removing a region's rect drops its layer.
    channels[3].rect = None
    widget._on_poll()
    assert widget._overlay.layer_keys() == [REF, K405, K561]
    assert [r.key for r in widget._panel.rows()] == [REF, K405, K561]


def test_display_state_survives_refresh(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    widget = _region_widget(gui, qtbot)
    row = widget._panel.row(K561)
    assert row is not None
    row._color_combo.setCurrentIndex(row._color_combo.findData("#ffff00"))
    row._auto_cb.setChecked(False)
    row._min_spin.setValue(10)
    row._max_spin.setValue(500)
    hidden = widget._panel.row(K638)
    assert hidden is not None
    hidden._visible_cb.setChecked(False)

    widget._refresh_btn.click()

    assert widget._overlay.color(K561) == "#ffff00"
    assert not widget._overlay.is_auto_clim(K561)
    assert widget._overlay.clim(K561) == (10.0, 500.0)
    assert not widget._overlay.is_visible(K638)
    new_row = widget._panel.row(K561)
    assert new_row is not None and new_row is not row
    assert new_row._color_combo.currentData() == "#ffff00"
    assert not new_row._auto_cb.isChecked()
    assert (new_row._min_spin.value(), new_row._max_spin.value()) == (10, 500)
    hidden_row = widget._panel.row(K638)
    assert hidden_row is not None and not hidden_row._visible_cb.isChecked()
