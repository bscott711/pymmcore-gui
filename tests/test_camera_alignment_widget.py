from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from pymmcore_gui import MicroManagerGUI
from pymmcore_gui.actions._action_info import ActionInfo
from pymmcore_gui.actions.widget_actions import WidgetAction
from pymmcore_gui.widgets.camera_alignment import CameraAlignmentWidget

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

    from pymmcore_gui._qt.QtWidgets import QApplication


@pytest.fixture
def gui(qtbot: QtBot, qapp: QApplication) -> Iterator[MicroManagerGUI]:
    gui = MicroManagerGUI()
    qtbot.addWidget(gui)
    yield gui


def _blob_frame(shape: tuple[int, int], center: tuple[float, float]) -> np.ndarray:
    """A synthetic uint16 frame (matching real camera bit-depth) with one blob."""
    h, w = shape
    ys, xs = np.indices((h, w))
    cx, cy = center
    frame = 50.0 + 1000.0 * np.exp(-(((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * 3.0**2)))
    return frame.astype(np.uint16)


def test_camera_alignment_action_registered() -> None:
    """The alignment widget is registered as a widget action, like CRISP/Spectral."""
    infos = ActionInfo.widget_actions()
    assert WidgetAction.CAMERA_ALIGNMENT.value in infos
    assert infos[WidgetAction.CAMERA_ALIGNMENT.value].text == "Camera Alignment"


def test_disabled_with_default_single_camera(qtbot: QtBot, mmcore: CMMCorePlus) -> None:
    """`test_config.cfg`'s default active camera is a single physical camera."""
    widget = CameraAlignmentWidget(mmcore=mmcore)
    qtbot.addWidget(widget)

    assert not widget._pick1_btn.isEnabled()
    assert not widget._pick2_btn.isEnabled()
    assert "found 1" in widget._status.text()


def test_enabled_with_multi_camera_active(qtbot: QtBot, mmcore: CMMCorePlus) -> None:
    mmcore.setProperty("Core", "Camera", "Multi Camera")
    widget = CameraAlignmentWidget(mmcore=mmcore)
    qtbot.addWidget(widget)
    widget._on_config_loaded()

    assert widget._pick1_btn.isEnabled()
    assert widget._pick2_btn.isEnabled()
    assert widget._cam1 == "Camera"
    assert widget._cam2 == "Camera-2"


def test_overlay_shows_frames_before_any_pick(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    """The overlay must render as soon as 2 cameras resolve, before any pick.

    Picking is only needed for displacement tracking, not for the overlay
    view itself.
    """
    mmc = gui.mmcore
    mmc.setProperty("Core", "Camera", "Multi Camera")

    widget = CameraAlignmentWidget(parent=gui, mmcore=mmc)
    qtbot.addWidget(widget)
    widget._on_config_loaded()
    assert widget._cam1 is not None and widget._cam2 is not None
    assert widget._poll_timer.isActive()

    preview1 = gui.viewers_manager.get_or_create_camera_preview(widget._cam1)
    preview2 = gui.viewers_manager.get_or_create_camera_preview(widget._cam2)
    preview1.append(_blob_frame((128, 128), (60.0, 60.0)))
    preview2.append(_blob_frame((128, 128), (64.0, 65.0)))

    widget._on_poll()

    assert widget._overlay._layer1.data is not None
    assert widget._overlay._layer2.data is not None
    assert widget._overlay._layer1.node.visible
    assert widget._overlay._layer2.node.visible
    # No picks yet -- no tracking should have happened.
    assert widget._readout.text() == ""
    assert len(widget._chart._samples) == 0


def test_pick_and_track_updates_readout_and_chart(
    gui: MicroManagerGUI, qtbot: QtBot
) -> None:
    mmc = gui.mmcore
    mmc.setProperty("Core", "Camera", "Multi Camera")

    widget = CameraAlignmentWidget(parent=gui, mmcore=mmc)
    qtbot.addWidget(widget)
    widget._on_config_loaded()
    assert widget._cam1 is not None and widget._cam2 is not None

    preview1 = gui.viewers_manager.get_or_create_camera_preview(widget._cam1)
    preview2 = gui.viewers_manager.get_or_create_camera_preview(widget._cam2)
    frame1 = _blob_frame((128, 128), (60.0, 60.0))
    frame2 = _blob_frame((128, 128), (64.0, 65.0))
    preview1.append(frame1)
    preview2.append(frame2)

    # Seed picks via the real done-callbacks (exercises world->pixel conversion),
    # bypassing real pointer events -- no existing test drives pygfx pointer
    # events through Qt, matching that precedent.
    widget._on_pick1_done((60.0 - 0.5, 60.0 - 0.5))
    widget._on_pick2_done((64.0 - 0.5, 65.0 - 0.5))
    assert widget._poll_timer.isActive()

    widget._on_poll()

    assert widget._readout.text() != ""
    assert len(widget._chart._samples) == 1
    dx, dy, mag = (
        widget._chart._samples[-1].dx,
        widget._chart._samples[-1].dy,
        widget._chart._samples[-1].mag,
    )
    assert dx == pytest.approx(4.0, abs=0.2)
    assert dy == pytest.approx(5.0, abs=0.2)
    assert mag == pytest.approx((4.0**2 + 5.0**2) ** 0.5, abs=0.3)

    widget._on_clear_clicked()
    # Clear resets tracking state but the live overlay keeps polling.
    assert widget._poll_timer.isActive()
    assert widget._spot1 is None and widget._spot2 is None
    assert len(widget._chart._samples) == 0


def test_no_new_sample_without_new_frame(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    """Polling again on the same (unchanged) frames must not advance the chart.

    Acquisition being paused/stopped means the preview keeps returning the
    same frame object tick after tick; the strip chart should hold its last
    point rather than keep stamping fresh wall-clock times onto stale data.
    """
    mmc = gui.mmcore
    mmc.setProperty("Core", "Camera", "Multi Camera")

    widget = CameraAlignmentWidget(parent=gui, mmcore=mmc)
    qtbot.addWidget(widget)
    widget._on_config_loaded()
    assert widget._cam1 is not None and widget._cam2 is not None

    preview1 = gui.viewers_manager.get_or_create_camera_preview(widget._cam1)
    preview2 = gui.viewers_manager.get_or_create_camera_preview(widget._cam2)
    preview1.append(_blob_frame((128, 128), (60.0, 60.0)))
    preview2.append(_blob_frame((128, 128), (64.0, 65.0)))
    widget._on_pick1_done((60.0 - 0.5, 60.0 - 0.5))
    widget._on_pick2_done((64.0 - 0.5, 65.0 - 0.5))

    widget._on_poll()
    assert len(widget._chart._samples) == 1

    # No new frames pushed -- simulates acquisition being paused. Repeated
    # polling must not add more samples.
    widget._on_poll()
    widget._on_poll()
    assert len(widget._chart._samples) == 1

    # A genuinely new frame arrives -- the chart should advance again.
    preview1.append(_blob_frame((128, 128), (61.0, 60.0)))
    widget._on_poll()
    assert len(widget._chart._samples) == 2


def test_paused_during_mda(
    gui: MicroManagerGUI, qtbot: QtBot, monkeypatch: pytest.MonkeyPatch
) -> None:
    mmc = gui.mmcore
    mmc.setProperty("Core", "Camera", "Multi Camera")

    widget = CameraAlignmentWidget(parent=gui, mmcore=mmc)
    qtbot.addWidget(widget)
    widget._on_config_loaded()
    assert widget._cam1 is not None and widget._cam2 is not None

    preview1 = gui.viewers_manager.get_or_create_camera_preview(widget._cam1)
    preview2 = gui.viewers_manager.get_or_create_camera_preview(widget._cam2)
    preview1.append(_blob_frame((128, 128), (60.0, 60.0)))
    preview2.append(_blob_frame((128, 128), (64.0, 65.0)))
    widget._on_pick1_done((60.0 - 0.5, 60.0 - 0.5))
    widget._on_pick2_done((64.0 - 0.5, 65.0 - 0.5))

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
