"""Crop each camera's hardware ROI to the bounding box of its spectral regions.

A Kinetix's rolling readout time scales with the number of rows read, and
every pixel outside the image-splitter regions is thrown away by
:class:`~pymmcore_gui._spectral_channel_handler.SpectralChannelHandler`
anyway -- reading only the regions' bounding box shortens each frame's
readout (faster slices, especially in the camera's All Rows mode) and cuts
the data moved and saved per frame. The regions themselves stay in
full-sensor coordinates; every consumer shifts by the ROI origin (see
:func:`~pymmcore_gui._spectral_channel_handler.roi_origin`).
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._spectral_channel_handler import (
    regions_bounding_box,
    regions_outside_roi,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._settings import SpectralChannelSettingsV1

logger = logging.getLogger(__name__)

Roi = tuple[int, int, int, int]


def crop_targets(
    spectral: SpectralChannelSettingsV1, cameras: list[str]
) -> dict[str, Roi | None]:
    """The ROI each camera should get: its regions' bounding box, or ``None``.

    ``None`` means "full frame" -- cropping is off, or that camera has no
    drawn regions.
    """
    crop = spectral.enabled and spectral.crop_camera_roi
    return {
        label: regions_bounding_box(spectral.channels, label) if crop else None
        for label in cameras
    }


def apply_camera_crop(
    mmc: CMMCorePlus, spectral: SpectralChannelSettingsV1
) -> dict[str, Roi]:
    """Set every camera's ROI per :func:`crop_targets`; return what took effect.

    On the worker rig Live is stopped around the change and restarted (ROI
    changes need an idle camera); an MDA in progress makes this raise. The
    camera may round the requested ROI; regions that end up outside the ROI
    it actually applied are logged (an MDA refuses to start with them).

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    spectral : SpectralChannelSettingsV1
        The spectral-channel settings holding the regions and the toggle.
    """
    from pymmcore_gui.asi_z_stack.camera_worker_service import (
        CameraWorkerService,
        CameraWorkerServiceState,
    )

    applied: dict[str, Roi] = {}
    svc = CameraWorkerService.get_active()
    if svc is not None:
        if svc.state is CameraWorkerServiceState.MDA:
            raise RuntimeError("Cannot change the camera ROI while an MDA runs.")
        was_live = svc.state is CameraWorkerServiceState.LIVE
        if was_live:
            svc.stop_live()
        try:
            for label, target in crop_targets(
                spectral, list(svc.camera_labels)
            ).items():
                roi = target or svc.full_frame_roi(label)
                applied[label] = svc.set_roi(label, *roi)
        finally:
            if was_live:
                svc.start_live()
    else:
        cameras = physical_camera_labels(mmc)
        for label, target in crop_targets(spectral, cameras).items():
            if target is None:
                mmc.clearROI()
                x, y, w, h = mmc.getROI(label)
            else:
                mmc.setROI(label, *target)
                x, y, w, h = mmc.getROI(label)
            applied[label] = (int(x), int(y), int(w), int(h))

    for label, roi in applied.items():
        logger.info(f"{label} ROI is now {roi}.")
    rois: dict[str, Roi | None] = dict(applied)
    if outside := regions_outside_roi(spectral.channels, rois):
        logger.warning(
            f"Spectral regions {outside} fall outside the ROI the camera applied "
            f"({applied}); widen the regions' bounding box or turn cropping off."
        )
    return applied
