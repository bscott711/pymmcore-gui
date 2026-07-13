from __future__ import annotations

import logging
import sys
from collections.abc import Callable
from contextlib import suppress
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Literal, cast, overload
from weakref import WeakValueDictionary

from pymmcore_plus import CMMCorePlus
from pymmcore_widgets import ConfigWizard
from PyQt6.QtCore import QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QCloseEvent, QGuiApplication, QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDialog,
    QHBoxLayout,
    QMainWindow,
    QMenu,
    QMenuBar,
    QMessageBox,
    QPushButton,
    QStatusBar,
    QToolBar,
    QVBoxLayout,
    QWidget,
)
from PyQt6Ads import (
    CDockAreaWidget,
    CDockManager,
    CDockWidget,
    DockWidgetArea,
    SideBarLocation,
)
from superqt import QIconifyIcon

from ._mmcore_shutdown import shutdown_mmcore, track_mda_thread
from ._ndv_viewers import NDVViewersManager
from ._notification_manager import NotificationManager
from ._settings import Settings
from .actions import CoreAction, QCoreAction, WidgetAction, WidgetActionInfo
from .actions._action_info import ActionInfo
from .widgets._toolbars import OCToolBar

if TYPE_CHECKING:
    from collections.abc import Mapping

    import ndv
    from pymmcore_widgets import (
        CameraRoiWidget,
        ConfigWizard,
        GroupPresetTableWidget,
        InstallWidget,
        MDAWidget,
        PixelConfigurationWidget,
        PropertyBrowser,
    )
    from useq import MDASequence

    from pymmcore_gui.widgets._about_widget import AboutWidget
    from pymmcore_gui.widgets._exception_log import ExceptionLog
    from pymmcore_gui.widgets._mm_console import MMConsole
    from pymmcore_gui.widgets._stage_control import StagesControlWidget

    from ._app import MMQApplication


logger = logging.getLogger("pymmcore_gui")

RESOURCES = Path(__file__).parent / "resources"
ICON = RESOURCES / ("icon.ico" if sys.platform.startswith("win") else "logo.png")


class Menu(str, Enum):
    """Menu names."""

    PYMM_GUI = "pymmcore-gui"
    WINDOW = "Window"
    DEVICE = "Devices"
    PLUGINS = "Plugins"
    HELP = "Help"

    def __str__(self) -> str:
        return str(self.value)


class Toolbar(str, Enum):
    """Toolbar names."""

    CAMERA_ACTIONS = "Camera Actions"
    OPTICAL_CONFIGS = "Optical Configs"
    WIDGETS = "Widgets"
    SHUTTERS = "Shutters"

    def __str__(self) -> str:
        return str(self.value)


ToolDictValue = list[str | None] | Callable[[CMMCorePlus, "MicroManagerGUI"], QToolBar]
MenuDictValue = list[str | None] | Callable[[CMMCorePlus, "MicroManagerGUI"], QMenu]


def _create_window_menu(mmc: CMMCorePlus, parent: MicroManagerGUI) -> QMenu:
    """
    Create the Window menu, containing all WidgetActions not in other menus.

    This function assumes lazy evaluation, i.e. that all Actions that want to be on
    other menus are already there.
    """
    all_actions = set(ActionInfo.widget_actions())

    # Ignore those already in other menus
    parented_actions: set[str] = set()
    for other_menu in parent.MENUS.values():
        if isinstance(other_menu, list):
            parented_actions.update(str(action) for action in other_menu)
    parentless_actions = all_actions - parented_actions

    # Create a new menu with the remaining parentless actions
    menu = QMenu(Menu.WINDOW.value, parent)
    for action in sorted(parentless_actions):
        menu.addAction(parent.get_action(action))
    return menu


class MicroManagerGUI(QMainWindow):
    """Micro-Manager minimal GUI."""

    # Emitted (possibly more than once, on every config load/reload) each
    # time the session's ASI/PLogic circular-buffer grow is confirmed
    # finished -- immediately, if no growth was needed. create_mmgui uses
    # this to delay showing the main window at startup until it's actually
    # safe to interact with; see _on_system_config_loaded.
    bufferReady = pyqtSignal()

    # Toolbars are a mapping of strings to either a list of ActionKeys or a callable
    # that takes a CMMCorePlus instance and QMainWindow and returns a QToolBar.
    TOOLBARS: Mapping[str, ToolDictValue] = {
        Toolbar.CAMERA_ACTIONS: [
            CoreAction.SNAP,
            CoreAction.TOGGLE_LIVE,
        ],
        Toolbar.OPTICAL_CONFIGS: OCToolBar,
        # Toolbar.SHUTTERS: ShuttersToolbar,
        Toolbar.WIDGETS: [
            WidgetAction.CONSOLE,
            WidgetAction.PROP_BROWSER,
            WidgetAction.MDA_WIDGET,
            WidgetAction.STAGE_CONTROL,
            WidgetAction.CAMERA_ROI,
        ],
    }
    # Menus are a mapping of strings to either a list of ActionKeys or a callable
    # that takes a CMMCorePlus instance and QMainWindow and returns a QMenu.
    MENUS: Mapping[str, MenuDictValue] = {
        Menu.PYMM_GUI: [WidgetAction.ABOUT],
        Menu.WINDOW: _create_window_menu,
        Menu.DEVICE: [
            WidgetAction.PROP_BROWSER,
            WidgetAction.CONFIG_WIZARD,
            None,
            CoreAction.LOAD_CONFIG,
            CoreAction.LOAD_DEMO,
            CoreAction.SAVE_CONFIG,
            None,
            WidgetAction.INSTALL_DEVICES,
        ],
        Menu.PLUGINS: [
            WidgetAction.CRISP,
            WidgetAction.SPECTRAL_CHANNELS,
        ],
        Menu.HELP: [],
    }

    def __init__(self, *, mmcore: CMMCorePlus | None = None) -> None:
        super().__init__()
        self.setWindowTitle("pyMM")
        self.setWindowIcon(QIcon(str(ICON)))
        self.setObjectName("MicroManagerGUI")

        # Serves to cache created QAction objects so that they can be re-used
        # when the same action is requested multiple times. This is useful to
        # synchronize the state of actions that may appear in multiple menus or
        # toolbars.
        self._qactions = WeakValueDictionary[str, QAction]()
        # widgets that are associated with a QAction
        self._action_widgets = WeakValueDictionary[str, QWidget]()
        # the wrapping QDockWidget for widgets that are associated with a QAction
        self._dock_widgets = WeakValueDictionary[str, CDockWidget]()

        # get global CMMCorePlus instance
        self._mmc = mmcore or CMMCorePlus.instance()
        self._mmc.events.systemConfigurationLoaded.connect(
            self._on_system_config_loaded
        )
        # so that closeEvent can reliably wait for any in-progress MDA to
        # actually finish before unloading devices (see _mmcore_shutdown.py)
        track_mda_thread(self._mmc)

        self._viewers_manager = NDVViewersManager(self, self._mmc)
        self._viewers_manager.mdaViewerCreated.connect(self._on_mda_viewer_created)
        self._viewers_manager.previewViewerCreated.connect(self._on_previewer_created)
        self._notification_manager = NotificationManager(self)
        if app := QApplication.instance():
            if hasattr(app, "exceptionRaised"):
                cast("MMQApplication", app).exceptionRaised.connect(self._on_exception)

        # Status bar -----------------------------------------

        self._status_bar = QStatusBar(self)
        self._status_bar.setMaximumHeight(26)
        self.setStatusBar(self._status_bar)

        self.bell_button = QPushButton(QIconifyIcon("codicon:bell"), None)
        self.bell_button.setFixedWidth(20)
        self.bell_button.setFlat(True)  # Make it blend nicely
        self._status_bar.addPermanentWidget(self.bell_button)

        # MENUS ====================================
        # To add menus or menu items, add them to the MENUS dict above

        for name, entry in self.MENUS.items():
            self._add_menubar(name, entry)

        # TOOLBARS =================================
        # To add toolbars or toolbar items, add them to the TOOLBARS dict above

        for name, tb_entry in self.TOOLBARS.items():
            self._add_toolbar(name, tb_entry)

        # LAYOUT ======================================

        # Create the dock manager. Because the parent parameter is a QMainWindow
        # the dock manager registers itself as the central widget.
        # It controls *all* widgets that are owned by the QMainWindow (both those that
        # are docked and floating).
        CDockManager.setConfigFlag(
            CDockManager.eConfigFlag.DockAreaHasCloseButton, False
        )
        CDockManager.setConfigFlag(CDockManager.eConfigFlag.OpaqueSplitterResize, True)
        # Split new docks evenly with their neighbor on insertion (e.g. two
        # camera preview docks end up 50/50 instead of sized off widget hints).
        CDockManager.setConfigFlag(CDockManager.eConfigFlag.EqualSplitOnInsertion, True)
        CDockManager.setAutoHideConfigFlag(
            CDockManager.eAutoHideFlag.AutoHideFeatureEnabled, True
        )
        self.dock_manager = CDockManager(self)

        self._central = CDockWidget("Viewers", self)
        self._central.setFeature(CDockWidget.DockWidgetFeature.NoTab, True)
        blank = QWidget()
        blank.setObjectName("blank")
        blank.setStyleSheet(
            "background-color: qlineargradient("
            "x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #333, stop: 1 #111);"
        )
        self._central.setWidget(blank)
        self._central_dock_area = self.dock_manager.setCentralWidget(self._central)

        # Per-camera preview dock AREAS (not just widgets), keyed by physical
        # camera label -- populated in _on_previewer_created, consulted in
        # _on_mda_viewer_created so each camera's MDA tab joins that camera's
        # own preview area instead of a single shared central area. Reset
        # fresh every session (not persisted) -- see _ensure_camera_previews.
        self._camera_preview_areas: dict[str, CDockAreaWidget] = {}

        # QTimer.singleShot(0, self._restore_state)

    # --------------------- Properties ----------------------

    @property
    def nm(self) -> NotificationManager:
        """A callable that can be used to show a message in the status bar."""
        return self._notification_manager

    @property
    def mmcore(self) -> CMMCorePlus:
        return self._mmc

    @property
    def viewers_manager(self) -> NDVViewersManager:
        return self._viewers_manager

    # --------------------- Public methods ----------------------
    # -----------------------------------------------------------

    def get_action(self, key: str, create: bool = True) -> QAction:
        """Create a QAction from this key."""
        if key not in self._qactions:
            if not create:  # pragma: no cover
                raise KeyError(
                    f"Action {key} has not been created yet, and 'create' is False"
                )
            # create and cache it
            info = ActionInfo.for_key(key)
            self._qactions[key] = action = info.to_qaction(self._mmc, self)
            # connect WidgetActions to toggle their widgets
            if isinstance(info, WidgetActionInfo):
                action.triggered.connect(self._toggle_action_widget)

        return self._qactions[key]

    # TODO: it's possible this could be expressed using Generics...
    # which would avoid the need for the manual overloads
    # fmt: off
    @overload
    def get_widget(self, key: Literal[WidgetAction.ABOUT], create: bool = ...) -> AboutWidget: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.CAMERA_ROI], create: bool = ...) -> CameraRoiWidget: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.CONFIG_GROUPS], create: bool = ...) -> GroupPresetTableWidget: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.CONFIG_WIZARD], create: bool = ...) -> ConfigWizard: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.CONSOLE], create: bool = ...) -> MMConsole: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.EXCEPTION_LOG], create: bool = ...) -> ExceptionLog: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.INSTALL_DEVICES], create: bool = ...) -> InstallWidget: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.MDA_WIDGET], create: bool = ...) -> MDAWidget: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.PIXEL_CONFIG], create: bool = ...) -> PixelConfigurationWidget: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.PROP_BROWSER], create: bool = ...) -> PropertyBrowser: ...  # noqa: E501
    @overload
    def get_widget(self, key: Literal[WidgetAction.STAGE_CONTROL], create: bool = ...) -> StagesControlWidget: ...  # noqa: E501
    # generic fallback
    @overload
    def get_widget(self, key: str, create: bool = ...) -> QWidget: ...
    # fmt: on
    def get_widget(self, key: str, create: bool = True) -> QWidget:
        """Get (or create) widget for `key` ensuring that it is linked to its QAction.

        If the widget has been "closed" (hidden), it will be re-shown.

        Note that all widgets created this way are singletons, so calling this method
        multiple times will return the same widget instance.

        Parameters
        ----------
        key : WidgetAction
            The widget to get.
        create : bool, optional
            Whether to create the widget if it doesn't exist yet, by default True.

        Raises
        ------
        KeyError
            If the widget doesn't exist and `create` is False.
        """
        if key not in self._action_widgets:
            if not create:  # pragma: no cover
                raise KeyError(
                    f"Widget {key} has not been created yet, and 'create' is False"
                )
            info: WidgetActionInfo = WidgetActionInfo.for_key(key)
            area = info.dock_area
            widget = info.create_widget(self)
            widget.setObjectName(info.key)
            if isinstance(widget, QDialog):
                widget.exec()
                return widget

            self._action_widgets[key] = widget

            action = self.get_action(key)
            dock = CDockWidget(info.text, self)
            dock.setWidget(widget, info.scroll_mode)
            dock.setObjectName(f"docked_{info.key}")
            dock.setToggleViewAction(action)
            dock.setMinimumSize(widget.minimumSize())
            dock.setIcon(action.icon())
            dock.resize(widget.sizeHint())
            self._dock_widgets[key] = dock
            if area is None:
                self.dock_manager.addDockWidgetFloating(dock)
            elif isinstance(area, SideBarLocation):
                if container := self.dock_manager.addAutoHideDockWidget(area, dock):
                    dock.toggleView(True)
                    if area in {
                        SideBarLocation.SideBarLeft,
                        SideBarLocation.SideBarRight,
                    }:
                        size = widget.sizeHint().width()
                    else:
                        size = widget.sizeHint().height()
                    container.setSize(size + 5)
            else:
                self.dock_manager.addDockWidget(area, dock)

            # Set the action checked since the widget is now “open.”
            action.setChecked(True)

        return self._action_widgets[key]

    def get_dock_widget(self, key: str) -> CDockWidget:
        """Get the QDockWidget for `key`.

        Note, you can also get the QDockWidget by calling `get_widget(key)`, and then
        calling `widget.parent()`.  The parent will *either* be an instance of
        `QDockWidget` (if it's actually a docked widget), or `MicroManagerGUI`, if
        it's not docked.  You *should* use `isisinstance` in this case to check.

        Parameters
        ----------
        key : WidgetAction
            The key for the *inner* widget owned by the requested QDockWidget.

        Raises
        ------
        KeyError
            If the widget doesn't exist.
        """
        if key not in self._dock_widgets:
            raise KeyError(  # pragma: no cover
                f"Dock widget for {key} has not been created yet, "
                "or it is not owned by a dock widget"
            )
        return self._dock_widgets[key]

    # --------------------- Private methods ---------------------
    # -----------------------------------------------------------

    def _on_system_config_loaded(self) -> None:
        settings = Settings.instance()
        if cfg := self._mmc.systemConfigurationFile():
            settings.last_config = Path(cfg)
        else:
            settings.last_config = None
        settings.flush()

        # On ASI/PLogic configs, raise the fiber-optic global shutter and set up
        # the always-on cell so software snap/live can gate the laser BNCs, and
        # enable the galvo's beam once for the session (matching the
        # microscope-control sibling repo's confirmed-working engine, which
        # does this once at startup rather than per-MDA-run). Also
        # pre-allocate a large circular buffer once here -- see
        # ensure_circular_buffer_capacity_async's docstring for why this must
        # never happen mid-acquisition. All three are no-ops for demo /
        # non-ASI configs.
        from pymmcore_gui.asi_z_stack.asi_controller import (
            ensure_beam_enabled,
            ensure_circular_buffer_capacity_async,
            ensure_global_shutter_open,
        )

        ensure_global_shutter_open()
        ensure_beam_enabled()

        # The buffer grow (if any) runs on a background thread and can take a
        # few seconds for tens of GB -- disable Live for that window so it
        # can't start a sequence acquisition against the main-process core
        # while its circular buffer is mid-resize (see
        # asi_controller.circular_buffer_growing). Resolves synchronously,
        # with no visible flicker, when no growth is actually needed.
        live_action = self.get_action(CoreAction.TOGGLE_LIVE)
        live_action.setEnabled(False)

        def _on_buffer_ready() -> None:
            with suppress(RuntimeError):
                live_action.setEnabled(bool(self._mmc.getCameraDevice()))
            self.bufferReady.emit()

        ensure_circular_buffer_capacity_async(on_done=_on_buffer_ready)

        self._register_mda_engine()

    def _register_mda_engine(self) -> None:
        """Select the MDA engine to match the loaded configuration.

        On ASI/PLogic configs, install the PLogic-triggered
        :class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` so MDA
        z-stacks step the galvo via hardware triggers instead of running a
        software stage-stepping z-stack. On demo / non-ASI configs, restore
        the stock :class:`~pymmcore_plus.mda.MDAEngine`. Idempotent: the
        engine is only swapped when it doesn't already match the current
        hardware, so demo/test sessions keep the default engine untouched.
        """
        from pymmcore_plus.mda import MDAEngine

        from pymmcore_gui.asi_z_stack.asi_controller import (
            asi_zstack_hardware_available,
        )
        from pymmcore_gui.asi_z_stack.common import HardwareConstants
        from pymmcore_gui.asi_z_stack.engine import ASISPIMEngine

        engine = self._mmc.mda.engine
        if asi_zstack_hardware_available():
            if not isinstance(engine, ASISPIMEngine):
                self._mmc.mda.set_engine(ASISPIMEngine(self._mmc, HardwareConstants()))
        elif isinstance(engine, ASISPIMEngine):
            # returning to a demo / non-ASI config: restore the stock engine
            self._mmc.mda.set_engine(MDAEngine(self._mmc))

    def _add_toolbar(self, name: str, tb_entry: ToolDictValue) -> None:
        if callable(tb_entry):
            tb = tb_entry(self._mmc, self)
            self.addToolBar(tb)
        else:
            tb = cast("QToolBar", self.addToolBar(name))
            for action in tb_entry:
                if action is None:
                    tb.addSeparator()
                else:
                    tb.addAction(self.get_action(action))
        tb.setObjectName(name)

    def _add_menubar(self, name: str, menu_entry: MenuDictValue) -> None:
        mb = cast("QMenuBar", self.menuBar())
        if callable(menu_entry):
            menu = menu_entry(self._mmc, self)
            mb.addMenu(menu)
        else:
            menu = cast("QMenu", mb.addMenu(name))
            for action in menu_entry:
                if action is None:
                    menu.addSeparator()
                else:
                    menu.addAction(self.get_action(action))

    def closeEvent(self, a0: QCloseEvent | None) -> None:
        if self._mmc.mda.is_running() and not self._confirm_close_with_running_mda():
            if a0 is not None:
                a0.ignore()
            return
        self._save_state()
        try:
            shutdown_mmcore(self._mmc)
        except Exception:
            logger.exception("Error during mmcore shutdown on close")
        return super().closeEvent(a0)

    def _confirm_close_with_running_mda(self) -> bool:
        box = QMessageBox(
            QMessageBox.Icon.Warning,
            "Acquisition in progress",
            "An acquisition (MDA) is currently running.\n\n"
            "Closing pyMM now will cancel it immediately. Continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            self,
        )
        box.setDefaultButton(QMessageBox.StandardButton.No)
        box.setEscapeButton(QMessageBox.StandardButton.No)
        return box.exec() == QMessageBox.StandardButton.Yes

    def restore_state(self, *, show: bool = False) -> None:
        """Restore the state of the window from settings (or load default state).

        show is added as a convenience here because it may be a common use case to
        restore the state in a single shot timer and (only) then show the window.
        This avoids the window flashing on the screen before it is properly positioned.
        """
        settings = Settings.instance()
        open_widgets = settings.window.open_widgets
        for widget in self._open_widgets():
            if widget not in open_widgets:
                # if the widget is not in the settings, close it
                with suppress(KeyError):
                    dw = self.get_dock_widget(widget)
                    dw.toggleView(False)

        # we need to create the widgets first, before calling restoreState.
        for key in open_widgets:
            try:
                self.get_widget(key)
            except KeyError:
                self.nm.show_warning_message(
                    f"Unable to reload widget key stored in settings: {key!r}",
                )

        # restore position and size of the main window
        if geo := settings.window.geometry:
            self.restoreGeometry(geo)
        elif screen := QGuiApplication.primaryScreen():
            # if no geometry is saved, center the window taking up 90% of the screen
            percent = 0.9
            ageo = screen.availableGeometry()
            ageo.setSize(ageo.size() * percent)
            margin = (1 - percent) / 2
            ageo.translate(int(ageo.width() * margin), int(ageo.height() * margin))
            self.setGeometry(ageo)

        # restore state of toolbars and dockwidgets, but only after event loop start
        # https://forum.qt.io/post/794120
        if open_widgets and (state := settings.window.dock_manager_state):
            self.dock_manager.restoreState(state)
            for key in self._open_widgets():
                self.get_action(key).setChecked(True)
            if wdg := self.dock_manager.centralWidget():
                self._central_dock_area = wdg.dockAreaWidget()

        if show:
            self.show()
            self.nm.reposition_notifications()
            # Defer past this event-loop iteration so the window actually
            # paints before we potentially block on the one-time GPU/wgpu
            # init that creating a camera preview triggers (see
            # _ensure_camera_previews) -- otherwise that pause would land
            # before the user ever sees the window at all.
            QTimer.singleShot(0, self._ensure_camera_previews)

    def _ensure_camera_previews(self) -> None:
        """Proactively create + auto-arrange camera preview docks at startup.

        Without this, Camera-1/Camera-2 preview docks only appear after the
        user manually clicks Snap or Live, and have to be manually
        rearranged into a split view every session. This also moves the
        one-time GPU/wgpu initialization (shared with the first MDA's ndv
        viewer) to this deliberate startup pause instead of blocking the
        event loop mid-Acquire on the first MDA of a session.
        """
        self._viewers_manager.create_default_camera_previews()

    def _save_state(self) -> None:
        """Save the state of the window to settings."""
        # save position and size of the main window
        settings = Settings.instance()
        settings.window.geometry = self.saveGeometry().data()
        # remember which widgets are open, and preserve their state.
        settings.window.open_widgets = open_ = self._open_widgets()
        if open_:
            # note that dock_manager.saveState mostly replaces QMainWindow.saveState
            # the one thing it doesn't capture is the Toolbar state.
            # so we will need to add that separately if that is desired.
            settings.window.dock_manager_state = self.dock_manager.saveState().data()
        else:
            settings.window.dock_manager_state = None
        # write to disk, blocking up to 5 seconds
        settings.flush(timeout=5000)

    def _open_widgets(self) -> set[str]:
        """Return the set of open widgets."""
        return {
            key
            for key, widget in self._dock_widgets.items()
            if (action := widget.toggleViewAction()) and action.isChecked()
        }

    def _toggle_action_widget(self, checked: bool) -> None:
        """Callback that toggles the visibility of a widget.

        This is connected to the triggered signal of WidgetAction QActions above in
        `get_action`, so it is assumed that the sender is a QCoreAction with a
        WidgetAction key.  Calling otherwise will do nothing.
        """
        if not (isinstance(action := self.sender(), QCoreAction)):
            return

        # if the widget is a dock widget, we want to toggle the dock widget
        # rather than the inner widget
        if action.key in self._dock_widgets:
            widget: QWidget = self.get_dock_widget(action.key)
        else:
            # this will create the widget if it doesn't exist yet,
            # e.g. for a click event on a Toolbutton that doesn't yet have a widget
            widget = self.get_widget(action.key)
        widget.setVisible(checked)
        if checked:
            widget.raise_()

    def _on_mda_viewer_created(
        self, ndv_viewer: ndv.ArrayViewer, sequence: MDASequence, camera_label: str = ""
    ) -> None:
        q_viewer = cast("QWidget", ndv_viewer.widget())

        sha = str(sequence.uid)[:8]
        suffix = f"-{camera_label}" if camera_label else ""
        q_viewer.setObjectName(f"ndv-{sha}{suffix}")
        title = f"MDA {sha}" + (f" — {camera_label}" if camera_label else "")
        q_viewer.setWindowTitle(title)

        # Small header row with a "Lock current slice" toggle -- lets the
        # user pin whichever z-index is currently displayed, so the view
        # stops jumping to every new frame and only updates when a new
        # frame arrives at that same z (see NDVViewersManager.set_viewer_z_locked
        # / _update_mda_viewer). A no-op for sequences with no z axis.
        container = QWidget(self)
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)
        lock_row = QWidget(container)
        lock_layout = QHBoxLayout(lock_row)
        lock_layout.setContentsMargins(4, 2, 4, 2)
        lock_checkbox = QCheckBox("Lock current slice", lock_row)
        lock_checkbox.toggled.connect(
            lambda checked, v=ndv_viewer: self._viewers_manager.set_viewer_z_locked(
                v, checked
            )
        )
        lock_layout.addWidget(lock_checkbox)
        lock_layout.addStretch()
        container_layout.addWidget(lock_row)
        container_layout.addWidget(q_viewer)

        # NOTE: don't call q_viewer.setWindowFlags(Qt.WindowType.Dialog) here --
        # changing window flags on an already-parented widget forces Qt to hide
        # and rebuild its native window handle, which (combined with the
        # parentless CDockWidget below) was the cause of the whole main window
        # visibly flickering/disappearing every time an MDA started. The
        # viewer is going straight into a dock, not shown as a standalone
        # dialog, so it doesn't need Dialog window flags.
        dw = CDockWidget(f"ndv-{sha}{suffix}", self)
        # small hack ... we need to retain a pointer to the viewer
        # otherwise the viewer will be garbage collected
        dw._viewer = ndv_viewer
        dw.setWidget(container)
        dw.setFeature(dw.DockWidgetFeature.DockWidgetFloatable, False)

        # Route each camera's MDA tab to that camera's own preview area, so
        # multi-camera MDAs don't all pile up as tabs in one shared area.
        # Falls back to the shared central area for the single-camera case
        # or if the tracked area reference has gone stale (e.g. the preview
        # dock was closed/reparented in the meantime).
        area = self._camera_preview_areas.get(camera_label) if camera_label else None
        if area is not None:
            try:
                self.dock_manager.addDockWidgetTabToArea(dw, area)
                return
            except RuntimeError:
                pass
        self.dock_manager.addDockWidgetTabToArea(dw, self._central_dock_area)

    def _on_previewer_created(
        self, dock_widget: CDockWidget, camera_label: str = ""
    ) -> None:
        if self._camera_preview_areas:
            # split beside the most recently placed camera preview, extending
            # the row left-to-right as more cameras are discovered
            target = next(reversed(self._camera_preview_areas.values()))
            area = self.dock_manager.addDockWidget(
                DockWidgetArea.RightDockWidgetArea, dock_widget, target
            )
        else:
            area = self.dock_manager.addDockWidgetTabToArea(
                dock_widget, self._central_dock_area
            )
        if area is not None and camera_label:
            self._camera_preview_areas[camera_label] = area

    def _on_exception(self, exc: BaseException) -> None:
        """Show a notification when an exception is raised."""
        see_tb = "See traceback"

        def _open_traceback(choice: str | None) -> None:
            if choice == see_tb:
                log = self.get_widget(WidgetAction.EXCEPTION_LOG)
                log.show_exception(exc)
                log.show()

        self._notification_manager.show_error_message(
            str(exc), see_tb, on_action=_open_traceback
        )
