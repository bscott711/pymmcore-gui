"""Save the latest Snap/Live frame of each camera to disk.

One OME-TIFF per camera plus a JSON sidecar holding the camera's own image
tags and the microscope state at save time (exposure, laser preset, galvo
offset, focus/XY position, CRISP state), so a saved Live/Snap frame can be
compared like-for-like with MDA output.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

import tifffile

from pymmcore_gui._multi_camera_handler import per_camera_path

if TYPE_CHECKING:
    from collections.abc import Sequence
    from datetime import datetime

    import numpy as np
    from pymmcore_plus import CMMCorePlus

logger = logging.getLogger(__name__)

FrameSource = Literal["snap", "live"]


@dataclass(frozen=True)
class LatestFrame:
    """The most recent frame a camera delivered to its preview.

    Attributes
    ----------
    camera_label : str
        Physical camera device label, e.g. ``"Camera-1"``.
    frame : np.ndarray
        The pixels, exactly as the camera delivered them.
    source : {"snap", "live"}
        Which acquisition mode produced the frame.
    timestamp : datetime
        When the GUI received the frame.
    camera_metadata : dict[str, Any]
        The camera's own image tags (empty when the delivery path has none).
    """

    camera_label: str
    frame: np.ndarray
    source: FrameSource
    timestamp: datetime
    camera_metadata: dict[str, Any] = field(default_factory=dict)


def _try(fn: Any, *args: Any) -> Any:
    """Return ``fn(*args)``, or ``None`` if it raises (state is best-effort)."""
    try:
        return fn(*args)
    except Exception:
        return None


def collect_scope_state(mmc: CMMCorePlus) -> dict[str, Any]:
    """Read the microscope state worth recording next to a saved frame.

    Every field is best-effort: a device that isn't loaded (or a read that
    fails) is recorded as ``None`` rather than blocking the save.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    """
    from pymmcore_gui._settings import SettingsV1
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService
    from pymmcore_gui.asi_z_stack.common import HardwareConstants

    hw = HardwareConstants()
    loaded = set(_try(mmc.getLoadedDevices) or ())

    def prop(device: str, name: str) -> str | None:
        if device not in loaded:
            return None
        return _try(mmc.getProperty, device, name)  # type: ignore[no-any-return]

    svc = CameraWorkerService.get_active()
    exposure = svc.last_known_exposure_ms if svc is not None else _try(mmc.getExposure)
    channel_group = _try(mmc.getChannelGroup) or hw.laser_config_group
    focus_device = _try(mmc.getFocusDevice) or ""
    crisp_label = SettingsV1.instance().focus.crisp_label
    xy = _try(mmc.getXYPosition) if _try(mmc.getXYStageDevice) else None

    return {
        "exposure_ms": exposure,
        "channel_group": channel_group,
        "channel_preset": _try(mmc.getCurrentConfig, channel_group),
        "pixel_size_um": _try(mmc.getPixelSizeUm),
        "focus_device": focus_device or None,
        "focus_um": _try(mmc.getPosition, focus_device) if focus_device else None,
        "xy_um": list(xy) if xy is not None else None,
        "galvo_offset_deg": prop(hw.galvo_a_label, "SingleAxisYOffset(deg)"),
        "crisp_state": prop(crisp_label, "CRISP State"),
    }


def _unique_path(path: Path) -> Path:
    """Return *path*, or a ``_1``/``_2``/... sibling if it already exists."""
    if not path.exists():
        return path
    name = path.name
    stem, dot, suffix = name.partition(".")
    for i in range(1, 10_000):
        candidate = path.with_name(f"{stem}_{i}{dot}{suffix}")
        if not candidate.exists():
            return candidate
    raise FileExistsError(f"Could not find a free filename near {path}")


def save_frames(
    frames: Sequence[LatestFrame],
    directory: str | Path,
    scope_state: dict[str, Any] | None = None,
) -> list[Path]:
    """Write each frame as an OME-TIFF with a JSON metadata sidecar.

    Files are named ``{YYYYmmdd-HHMMSS}_{source}_{camera}.ome.tif`` (and
    ``.json``), with a numeric suffix if that name is already taken.

    Parameters
    ----------
    frames : Sequence[LatestFrame]
        The frames to save, typically one per camera.
    directory : str | Path
        Destination folder; created if missing.
    scope_state : dict[str, Any] | None
        Microscope state to record in every sidecar (see
        :func:`collect_scope_state`).

    Returns
    -------
    list[Path]
        The OME-TIFF paths written, in the order of *frames*.
    """
    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)
    state = scope_state or {}
    pixel_size = state.get("pixel_size_um")

    written: list[Path] = []
    for item in frames:
        stamp = item.timestamp.strftime("%Y%m%d-%H%M%S")
        base = out_dir / f"{stamp}_{item.source}.ome.tif"
        tif_path = _unique_path(Path(per_camera_path(base, item.camera_label)))

        ome_meta: dict[str, Any] = {"axes": "YX", "Name": item.camera_label}
        if isinstance(pixel_size, int | float) and pixel_size > 0:
            ome_meta.update(
                PhysicalSizeX=pixel_size,
                PhysicalSizeXUnit="µm",
                PhysicalSizeY=pixel_size,
                PhysicalSizeYUnit="µm",
            )
        tifffile.imwrite(tif_path, item.frame, ome=True, metadata=ome_meta)

        sidecar = {
            "camera": item.camera_label,
            "source": item.source,
            "received": item.timestamp.isoformat(timespec="milliseconds"),
            "shape": list(item.frame.shape),
            "dtype": str(item.frame.dtype),
            **state,
            "camera_metadata": item.camera_metadata,
        }
        json_path = tif_path.with_name(tif_path.name.removesuffix(".ome.tif") + ".json")
        json_path.write_text(json.dumps(sidecar, indent=2, default=str))
        logger.info(
            "Saved %s frame from %s to %s", item.source, item.camera_label, tif_path
        )
        written.append(tif_path)
    return written
