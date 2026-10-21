"""Defines actions that act on the global CMMCore instance."""

from __future__ import annotations

from contextlib import suppress
from pathlib import Path
from typing import TYPE_CHECKING

from ._action_info import ActionInfo, ActionKey

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from ._core_qaction import QCoreAction


# ######################## Functions acting on the Core #########################
class CoreAction(ActionKey):
    """Actions that act on the global CMMCore instance."""

    SNAP = "pymmcore_gui.snap_image"
    TOGGLE_LIVE = "pymmcore_gui.toggle_live"
    SAVE_IMAGE = "pymmcore_gui.save_image"
    SAVE_IMAGE_AS = "pymmcore_gui.save_image_as"
    LOAD_DEMO = "pymmcore_gui.load_demo_config"
    LOAD_CONFIG = "pymmcore_gui.load_config"
    SAVE_CONFIG = "pymmcore_gui.save_config"


# TODO: perhaps have alternate signatures for these functions that take a
# CMMCorePlus instance, rather than needing to extract it from the QCoreAction.
def snap_image(action: QCoreAction, checked: bool) -> None:
    """Snap an image, firing the selected laser(s) for the exposure."""
    from pymmcore_gui.asi_z_stack.asi_controller import (
        close_selected_lasers,
        open_selected_lasers,
    )
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

    mmc = action.mmc

    if (svc := CameraWorkerService.get_active()) is not None:
        # Camera-1/Camera-2 live permanently in worker processes -- route
        # through the persistent pool instead of mmc.snapImage(), which has
        # no camera loaded to act on. svc.snap() is synchronous (matches
        # today's blocking Snap UX) and emits svc.frameReady per frame as it
        # arrives, which NDVViewersManager is already listening for.
        open_selected_lasers()
        try:
            svc.snap()
        finally:
            close_selected_lasers()
        return

    if mmc.isSequenceRunning():
        mmc.stopSequenceAcquisition()
    open_selected_lasers()
    try:
        mmc.snapImage()
    finally:
        close_selected_lasers()


def toggle_live(action: QCoreAction, checked: bool) -> None:
    """Start or stop live mode, firing the selected laser(s) while live."""
    from pymmcore_gui.asi_z_stack import asi_controller
    from pymmcore_gui.asi_z_stack.asi_controller import (
        close_all_lasers,
        open_selected_lasers,
    )
    from pymmcore_gui.asi_z_stack.camera_worker_service import (
        CameraWorkerService,
        CameraWorkerServiceState,
    )

    mmc = action.mmc

    if (svc := CameraWorkerService.get_active()) is not None:
        # Camera-1/Camera-2 live permanently in worker processes -- route
        # through the persistent pool instead of
        # mmc.startContinuousSequenceAcquisition(0), which has no camera
        # loaded to act on. The toolbar action's checked state is kept in
        # sync via svc.liveStateChanged (see _init_toggle_live) rather than
        # the mmc sequence-acquisition events used below, which never fire
        # for worker-owned cameras.
        if svc.state is CameraWorkerServiceState.LIVE:
            svc.stop_live()
            close_all_lasers()
        elif svc.state is CameraWorkerServiceState.IDLE:
            open_selected_lasers()
            svc.start_live()
        else:
            # SPAWNING or MDA -- not ready for Live right now.
            action.setChecked(False)
        return

    if mmc.isSequenceRunning():
        mmc.stopSequenceAcquisition()
        close_all_lasers()
    elif asi_controller.circular_buffer_growing:
        # Belt-and-suspenders: the action is normally disabled for this
        # window (see MicroManagerGUI._on_system_config_loaded), but guard
        # here too in case of a click that lands right as it's re-enabling.
        action.setChecked(False)
    else:
        open_selected_lasers()
        mmc.startContinuousSequenceAcquisition(0)


def _save_latest_frames(action: QCoreAction, *, ask_for_folder: bool) -> None:
    """Save each camera's newest Snap/Live frame (see ``_snapshot_save``)."""
    from qtpy.QtWidgets import QFileDialog

    from pymmcore_gui._settings import SettingsV1
    from pymmcore_gui._snapshot_save import collect_scope_state, save_frames

    window = action.parent()
    viewers = getattr(window, "viewers_manager", None)
    notify = getattr(window, "nm", None)
    frames = viewers.latest_frames() if viewers is not None else []
    if not frames:
        if notify is not None:
            notify.show_warning_message(
                "Nothing to save yet -- Snap or start Live first."
            )
        return

    settings = SettingsV1.instance()
    folder = settings.snapshot.save_dir
    if ask_for_folder or folder is None:
        chosen = QFileDialog.getExistingDirectory(
            None, "Folder for saved images", str(folder or "")
        )
        if not chosen:
            return
        settings.snapshot.save_dir = folder = Path(chosen)
        settings.flush()

    try:
        paths = save_frames(frames, folder, collect_scope_state(action.mmc))
    except OSError as exc:
        if notify is not None:
            notify.show_error_message(f"Could not save image: {exc}")
        return
    if notify is not None:
        names = ", ".join(p.name for p in paths)
        notify.show_info_message(f"Saved {len(paths)} image(s) to {folder}: {names}")


def save_image(action: QCoreAction, checked: bool) -> None:
    """Save the newest Snap/Live frame of every camera to the save folder."""
    _save_latest_frames(action, ask_for_folder=False)


def save_image_as(action: QCoreAction, checked: bool) -> None:
    """Choose a folder, then save the newest Snap/Live frame of every camera."""
    _save_latest_frames(action, ask_for_folder=True)


def _init_snap_image(action: QCoreAction) -> None:
    from pymmcore_gui.asi_z_stack.camera_worker_service import has_camera

    mmc = action.mmc

    def _on_load() -> None:
        # the action's underlying Qt widget may already be gone (e.g. a core
        # event fired during app shutdown, after this action's window closed)
        with suppress(RuntimeError):
            action.setEnabled(has_camera(mmc))

    mmc.events.systemConfigurationLoaded.connect(_on_load)

    _on_load()


def _init_toggle_live(action: QCoreAction) -> None:
    from pymmcore_gui.asi_z_stack.camera_worker_service import (
        CameraWorkerService,
        has_camera,
    )

    mmc = action.mmc
    connected_service: CameraWorkerService | None = None

    def _sync_service_connection() -> None:
        # The mmc sequence-acquisition events below never fire once Live
        # routes through worker-owned cameras, so the toggle button's
        # checked state instead follows svc.liveStateChanged -- needed
        # especially for autonomous stops this action didn't itself
        # trigger (an MDA preempting Live, a worker dying, a config
        # reload). Re-evaluated on every config load since the active
        # service instance can change across a reload.
        nonlocal connected_service
        svc = CameraWorkerService.get_active()
        if svc is connected_service:
            return
        if connected_service is not None:
            with suppress(RuntimeError, TypeError):
                connected_service.liveStateChanged.disconnect(action.setChecked)
        if svc is not None:
            svc.liveStateChanged.connect(action.setChecked)
        connected_service = svc

    def _on_load() -> None:
        with suppress(RuntimeError):
            action.setEnabled(has_camera(mmc))
        _sync_service_connection()

    mmc.events.systemConfigurationLoaded.connect(_on_load)

    def _on_change() -> None:
        with suppress(RuntimeError):
            action.setChecked(mmc.isSequenceRunning())

    mmc.events.sequenceAcquisitionStarted.connect(_on_change)
    mmc.events.continuousSequenceAcquisitionStarted.connect(_on_change)
    mmc.events.sequenceAcquisitionStopped.connect(_on_change)

    _on_load()


def _prepare_for_reload(mmc: CMMCorePlus) -> bool:
    """Tear down an active camera worker service before a config (re)load.

    pymmcore-plus has no pre-load event, and a new config's
    ``initializeDevice`` for Camera-1/Camera-2 will fail if a previous
    session's workers still hold those PVCAM handles open (driver
    exclusivity) -- see ``CameraWorkerService.prepare_for_reload``. Returns
    ``False`` (and shows a message instead of raising into Qt's signal
    machinery) if an MDA is currently running, so the caller can bail out of
    the load entirely.
    """
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

    try:
        CameraWorkerService.prepare_for_reload(mmc)
    except RuntimeError as exc:
        from qtpy.QtWidgets import QMessageBox

        QMessageBox.warning(None, "Cannot load configuration", str(exc))
        return False
    return True


def load_demo_config(action: QCoreAction, checked: bool) -> None:
    """Load the demo configuration."""
    if not _prepare_for_reload(action.mmc):
        return
    action.mmc.loadSystemConfiguration()


def load_sys_config_dialog(action: QCoreAction, checked: bool) -> None:
    """Open a dialog to load a system configuration."""
    from qtpy.QtWidgets import QFileDialog

    (path, _filter) = QFileDialog.getOpenFileName(
        None,
        "Select a Micro-Manager configuration file",
        "",
        "cfg(*.cfg)",
    )
    if path and _prepare_for_reload(action.mmc):
        action.mmc.loadSystemConfiguration(path)


def save_sys_config_dialog(action: QCoreAction, checked: bool) -> None:
    """Open a dialog to save a system configuration."""
    from qtpy.QtWidgets import QFileDialog

    (path, _filter) = QFileDialog.getSaveFileName(
        None,
        "Save Micro-Manager configuration file",
        "",
        "cfg(*.cfg)",
    )
    if path:
        action.mmc.saveSystemConfiguration(path)


# ########################## Action Info Instances #############################


snap_action = ActionInfo(
    key=CoreAction.SNAP,
    text="Snap Image",
    shortcut="Ctrl+K",
    auto_repeat=True,
    icon="mdi-light:camera",
    on_triggered=snap_image,
    on_created=_init_snap_image,
)


toggle_live_action = ActionInfo(
    key=CoreAction.TOGGLE_LIVE,
    text="Toggle Live",
    shortcut="Ctrl+L",
    auto_repeat=True,
    icon="mdi:video-outline",
    checkable=True,
    on_triggered=toggle_live,
    on_created=_init_toggle_live,
)

save_image_action = ActionInfo(
    key=CoreAction.SAVE_IMAGE,
    text="Save Image",
    shortcut="Ctrl+S",
    icon="mdi:content-save-outline",
    tooltip="Save the newest Snap/Live frame of each camera (OME-TIFF + JSON)",
    on_triggered=save_image,
)

save_image_as_action = ActionInfo(
    key=CoreAction.SAVE_IMAGE_AS,
    text="Save Image To...",
    icon="mdi:content-save-move-outline",
    tooltip="Choose a folder, then save the newest Snap/Live frame of each camera",
    on_triggered=save_image_as,
)

load_demo_action = ActionInfo(
    key=CoreAction.LOAD_DEMO,
    text="Load Demo Configuration",
    on_triggered=load_demo_config,
)

load_config_action = ActionInfo(
    key="pymmcore_gui.load_config",
    text="Load System Configuration...",
    on_triggered=load_sys_config_dialog,
)

load_config_action = ActionInfo(
    key="pymmcore_gui.save_config",
    text="Save System Configuration...",
    on_triggered=save_sys_config_dialog,
)
