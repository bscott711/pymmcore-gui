"""Snapshot, release, and reload the main process's cameras around a handoff.

For the duration of one hardware-triggered MDA run, the main process's
``CMMCorePlus`` lets go of ``Camera-1``/``Camera-2`` (and the ``Multi Camera``
composite, if present) so that :mod:`~pymmcore_gui.asi_z_stack.worker_pool`
subprocesses can each open one of them exclusively. :func:`release_cameras_for_workers`
captures everything needed to put the main process back exactly how it was
(:class:`CameraHandoffSnapshot`) and then unloads the camera devices;
:func:`reload_cameras_after_handoff` reverses it once the worker pool has shut
down. Live/Snap/the device-property browser/the Dual-ROI widget see no
difference across an MDA run other than a brief window where the cameras
aren't loaded.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from pymmcore_gui._multi_camera_handler import physical_camera_labels

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from .common import HardwareConstants

_ADAPTER_MODULE = "PVCAM"
_MULTI_CAMERA_LIBRARY = "Utilities"


@dataclass
class _CameraSnapshot:
    """Per-camera state captured before release, to restore after reload."""

    trigger_mode: str | None
    roi: tuple[int, int, int, int] | None
    property_values: dict[str, str]


@dataclass
class CameraHandoffSnapshot:
    """Everything the main process needs cached while its cameras are detached.

    ``n_cameras``/``camera_labels``/image-geometry fields exist purely so code
    that runs *during* the detached window (e.g.
    :meth:`~pymmcore_gui.asi_z_stack.engine._ASITriggerEngineBase.exec_event`)
    never has to call a Camera-role-dependent ``CMMCorePlus`` method (which
    would misbehave or raise once the devices are unloaded) -- they're read
    once, here, while the cameras are still loaded.
    """

    n_cameras: int
    camera_labels: tuple[str, ...]
    core_camera_role: str
    image_width: int
    image_height: int
    bytes_per_pixel: int
    n_components: int
    per_camera: dict[str, _CameraSnapshot] = field(default_factory=dict)

    @property
    def dtype_str(self) -> str:
        """The numpy dtype string implied by ``bytes_per_pixel``/``n_components``."""
        return f"uint{(self.bytes_per_pixel // self.n_components) * 8}"


def _snapshot_camera(mmc: CMMCorePlus, label: str) -> _CameraSnapshot:
    """Capture *label*'s trigger mode, ROI, and settable property values.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main process's core, with *label* still loaded.
    label : str
        The physical camera device label to snapshot.
    """
    trigger_mode = (
        mmc.getProperty(label, "TriggerMode")
        if mmc.hasProperty(label, "TriggerMode")
        else None
    )
    roi: tuple[int, int, int, int] | None
    try:
        x, y, w, h = mmc.getROI(label)
        roi = (x, y, w, h)
    except Exception:
        roi = None

    values: dict[str, str] = {}
    for prop in mmc.getDevicePropertyNames(label):
        try:
            if mmc.isPropertyReadOnly(label, prop) or mmc.isPropertyPreInit(
                label, prop
            ):
                continue
            values[prop] = mmc.getProperty(label, prop)
        except Exception:
            continue
    return _CameraSnapshot(trigger_mode=trigger_mode, roi=roi, property_values=values)


def release_cameras_for_workers(
    mmc: CMMCorePlus, hw: HardwareConstants
) -> CameraHandoffSnapshot:
    """Snapshot and unload every physical camera, in composite-first order.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main process's core.
    hw : HardwareConstants
        Unused directly (physical camera labels come from the currently
        loaded configuration via
        :func:`~pymmcore_gui._multi_camera_handler.physical_camera_labels`,
        not hardcoded), accepted for symmetry with
        :func:`reload_cameras_after_handoff` and future use.

    Returns
    -------
    CameraHandoffSnapshot
        Everything needed to restore the main process's camera state later.
    """
    del hw  # not needed here; see docstring
    labels = tuple(physical_camera_labels(mmc))
    n_cameras = len(labels)
    core_role = mmc.getCameraDevice()

    snapshot = CameraHandoffSnapshot(
        n_cameras=n_cameras,
        camera_labels=labels,
        core_camera_role=core_role,
        image_width=mmc.getImageWidth(),
        image_height=mmc.getImageHeight(),
        bytes_per_pixel=mmc.getBytesPerPixel(),
        n_components=mmc.getNumberOfComponents(),
        per_camera={label: _snapshot_camera(mmc, label) for label in labels},
    )

    loaded = mmc.getLoadedDevices()
    # Multi Camera holds references to the physical cameras behind it and must
    # be unloaded first; the physical cameras themselves unload in reverse of
    # their (arbitrary but consistent) load order.
    if n_cameras > 1 and core_role in loaded:
        mmc.unloadDevice(core_role)
    for label in reversed(labels):
        if label in mmc.getLoadedDevices():
            mmc.unloadDevice(label)

    return snapshot


def _restore_camera(mmc: CMMCorePlus, label: str, snap: _CameraSnapshot) -> None:
    """Best-effort reapply *snap*'s properties/ROI/trigger-mode to *label*.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main process's core, with *label* freshly reloaded.
    label : str
        The physical camera device label to restore.
    snap : _CameraSnapshot
        The pre-release state to reapply.
    """
    for prop, value in snap.property_values.items():
        try:
            mmc.setProperty(label, prop, value)
        except Exception as exc:
            print(
                f"[camera_handoff] {label}: could not restore {prop}={value!r}: {exc}"
            )
    if snap.roi is not None:
        try:
            mmc.setROI(label, *snap.roi)
        except Exception as exc:
            print(f"[camera_handoff] {label}: could not restore ROI {snap.roi}: {exc}")
    if snap.trigger_mode is not None:
        try:
            mmc.setProperty(label, "TriggerMode", snap.trigger_mode)
        except Exception as exc:
            print(f"[camera_handoff] {label}: could not restore TriggerMode: {exc}")


def reload_cameras_after_handoff(
    mmc: CMMCorePlus, hw: HardwareConstants, snapshot: CameraHandoffSnapshot
) -> None:
    """Reload every physical camera (and the composite, if any) and restore state.

    Written defensively: safe to call even if *snapshot* reflects only a
    partially released state (e.g. ``setup_sequence`` raised after releasing
    the cameras but before a worker pool ever started), since it only acts on
    devices that aren't already loaded.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main process's core.
    hw : HardwareConstants
        Unused directly, accepted for symmetry with
        :func:`release_cameras_for_workers`.
    snapshot : CameraHandoffSnapshot
        The state captured by :func:`release_cameras_for_workers`.
    """
    del hw  # not needed here; see docstring
    for label in snapshot.camera_labels:
        if label in mmc.getLoadedDevices():
            continue
        try:
            mmc.loadDevice(label, _ADAPTER_MODULE, label)
            mmc.initializeDevice(label)
        except Exception as exc:
            print(f"[camera_handoff] failed to reload {label}: {exc}")
            continue
        cam_snapshot = snapshot.per_camera.get(label)
        if cam_snapshot is not None:
            _restore_camera(mmc, label, cam_snapshot)

    if (
        snapshot.n_cameras > 1
        and snapshot.core_camera_role not in mmc.getLoadedDevices()
    ):
        try:
            # "Physical Camera N" are NOT pre-init properties, despite being
            # documented as such elsewhere -- confirmed empirically on the
            # real rig: mmc.getDevicePropertyNames("Multi Camera") does not
            # list them until *after* initializeDevice() has run (Multi
            # Camera's C++ Initialize() registers them, not its
            # constructor). Setting them before initializeDevice() raises
            # "Cannot set property" because the property doesn't exist yet.
            mmc.loadDevice(
                snapshot.core_camera_role,
                _MULTI_CAMERA_LIBRARY,
                snapshot.core_camera_role,
            )
            mmc.initializeDevice(snapshot.core_camera_role)
            for i, label in enumerate(snapshot.camera_labels, start=1):
                mmc.setProperty(
                    snapshot.core_camera_role, f"Physical Camera {i}", label
                )
        except Exception as exc:
            print(
                f"[camera_handoff] failed to reload {snapshot.core_camera_role}: {exc}"
            )

    if (
        snapshot.core_camera_role in mmc.getLoadedDevices()
        and mmc.getCameraDevice() != snapshot.core_camera_role
    ):
        mmc.setCameraDevice(snapshot.core_camera_role)
