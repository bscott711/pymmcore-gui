from __future__ import annotations

import sys
from typing import TYPE_CHECKING, cast
from unittest.mock import patch

import ndv
import pytest
import useq
from PyQt6.QtWidgets import QApplication, QDialog

from pymmcore_gui import MicroManagerGUI
from pymmcore_gui._app import MMQApplication
from pymmcore_gui._notification_manager import NotificationManager
from pymmcore_gui.actions import CoreAction, WidgetAction
from pymmcore_gui.widgets._toolbars import ShuttersToolbar

if TYPE_CHECKING:
    from collections.abc import Iterator

    from PyQt6Ads import CDockAreaWidget
    from pytestqt.qtbot import QtBot

    from pymmcore_gui._settings import Settings


@pytest.fixture
def gui(qtbot: QtBot, qapp: QApplication) -> Iterator[MicroManagerGUI]:
    gui = MicroManagerGUI()
    qtbot.addWidget(gui)
    yield gui


@pytest.mark.parametrize("w_action", list(WidgetAction))
def test_main_window_widget_actions(
    gui: MicroManagerGUI, w_action: WidgetAction
) -> None:
    gui = MicroManagerGUI()
    action = gui.get_action(w_action)
    with patch.object(QDialog, "exec", lambda x: x.show()):
        wdg = gui.get_widget(w_action)
        assert w_action in gui._qactions
    if isinstance(wdg, QDialog):
        ...
    else:
        assert w_action in gui._action_widgets
        assert action.isChecked()
        gui.get_dock_widget(w_action).toggleView(False)
        assert not action.isChecked()


@pytest.mark.parametrize("c_action", list(CoreAction))
def test_main_window_core_actions(gui: MicroManagerGUI, c_action: CoreAction) -> None:
    gui = MicroManagerGUI()
    with patch.object(QDialog, "exec", lambda x: x.show()):
        _ = gui.get_action(c_action)
    assert c_action in gui._qactions


def test_shutter_toolbar(gui: MicroManagerGUI) -> None:
    sh_toolbar = ShuttersToolbar(gui._mmc, gui)

    # in our test cfg we have 3 shutters
    assert (layout := sh_toolbar.layout()) is not None
    assert layout.count() == 3
    assert len(sh_toolbar.actions()) == 3

    # loading default cfg
    gui._mmc.loadSystemConfiguration()
    # in our test cfg we have 2 shutters
    assert layout.count() == 2
    assert len(sh_toolbar.actions()) == 2


def test_save_restore_state(gui: MicroManagerGUI, settings: Settings) -> None:
    assert not gui._open_widgets()
    settings.window.open_widgets.clear()

    # save the state
    assert not settings.window.open_widgets
    assert not settings.window.geometry
    gui._save_state()
    assert settings.window.geometry

    # add a widget
    gui.get_widget(WidgetAction.STAGE_CONTROL)
    assert WidgetAction.STAGE_CONTROL in gui._open_widgets()
    # restore the state
    assert not settings.window.open_widgets
    gui.restore_state()
    assert WidgetAction.STAGE_CONTROL not in gui._open_widgets()


def test_ndv_viewers_in_main_window(gui: MicroManagerGUI) -> None:
    central_area = cast("CDockAreaWidget", gui._central_dock_area)
    assert central_area.dockWidgetsCount() == 1
    gui.mmcore.mda.run(
        useq.MDASequence(
            time_plan=useq.TIntervalLoops(interval=1, loops=2),  # pyright: ignore
            channels=["DAPI", "FITC"],  #  pyright: ignore
        ),
    )
    assert central_area.dockWidgetsCount() == 2


def test_main_window_notifications(gui: MicroManagerGUI) -> None:
    """Test that notifications are created and removed correctly."""
    assert isinstance(gui.nm, NotificationManager)

    with patch.object(gui.nm, "show_error_message") as mock_show_error:
        err = ValueError("Boom!")
        app = QApplication.instance()
        assert isinstance(app, MMQApplication)
        app.exceptionRaised.emit(err)
        mock_show_error.assert_called_once()
        assert mock_show_error.call_args[0][0] == "Boom!"


def test_snap(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    """Test that snapping creates and updates a single image preview."""
    vm = gui._viewers_manager
    assert not vm._camera_previews
    core = gui._mmc
    # generous timeout: the first preview build does a one-time GPU init.
    with qtbot.waitSignal(vm.previewViewerCreated, timeout=15000):
        core.snapImage()
    # One preview dock for the single camera.
    assert len(vm._camera_previews) == 1
    cam_label = core.getCameraDevice()
    assert cam_label in vm._camera_previews
    preview_dw = vm._camera_previews[cam_label]
    assert preview_dw.widget().data is not None
    assert len(vm._preview_dock_widgets) == 1

    # A pixel-type / shape change is handled in place by the fast preview (the GPU
    # texture is recreated on the fly) — the *same* dock is reused, no churn.
    core.setProperty(core.getCameraDevice(), "PixelType", "32bitRGB")
    with qtbot.waitSignal(core.events.imageSnapped):
        core.snapImage()
    qtbot.wait(20)
    assert len(vm._camera_previews) == 1
    assert vm._camera_previews[cam_label] is preview_dw
    assert len(vm._preview_dock_widgets) == 1

    # an exposure change likewise does not spawn a new preview
    core.setProperty(core.getCameraDevice(), "Exposure", "42")
    with qtbot.waitSignal(core.events.imageSnapped):
        core.snapImage()
    assert len(vm._camera_previews) == 1
    assert len(vm._preview_dock_widgets) == 1


@pytest.mark.skipif(
    bool(sys.platform == "darwin"),
    reason="need to debug hanging test on macOS CI",
)
def test_stream(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    """Test that streaming creates an image preview that receives frames."""
    from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

    vm = gui._viewers_manager
    assert not vm._camera_previews
    core = gui._mmc
    with qtbot.waitSignal(vm.previewViewerCreated, timeout=15000):
        core.startContinuousSequenceAcquisition()

    assert vm._camera_previews
    cam_label = core.getCameraDevice()
    preview = vm._camera_previews[cam_label].widget()
    assert isinstance(preview, PygfxPreview)

    # let a few frames stream and confirm the preview received data
    qtbot.wait(150)
    assert preview.data is not None

    # changing the exposure should not break streaming
    core.setExposure(11)
    qtbot.wait(50)
    core.stopSequenceAcquisition()


def test_mda(gui: MicroManagerGUI, qtbot: QtBot) -> None:
    vm = gui._viewers_manager
    assert vm._active_mda_viewer is None
    core = gui._mmc
    with qtbot.waitSignal(vm.mdaViewerCreated):
        core.mda.run(
            useq.MDASequence(
                time_plan=useq.TIntervalLoops(interval=1, loops=2),  # pyright: ignore
                channels=["DAPI", "FITC"],  #  pyright: ignore
            ),
        )
    assert vm._active_mda_viewer is not None
