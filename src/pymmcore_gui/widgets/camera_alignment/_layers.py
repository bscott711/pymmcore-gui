"""Qt-free layer model for the Camera Alignment widget.

The alignment overlay shows N tinted layers. A layer is either a whole camera
frame or a crop (spectral region) of one camera's live frame; this module holds
the data model and helpers, with no Qt and no camera access.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import TYPE_CHECKING

from pymmcore_gui._spectral_channel_handler import crop_region

if TYPE_CHECKING:
    from collections.abc import Collection, Sequence

    import numpy as np

    from pymmcore_gui._settings import SpectralChannelConfig

MODE_REGIONS = "Spectral regions"
MODE_CAMERAS = "Whole cameras"

PALETTE: dict[str, str] = {
    "Green": "#00ff00",
    "Magenta": "#ff00ff",
    "Red": "#ff0000",
    "Cyan": "#00ffff",
    "Yellow": "#ffff00",
    "Orange": "#ff8000",
    "Blue": "#3c78ff",
    "White": "#ffffff",
    "Gray": "#808080",
}
"""Ordered display name -> hex colour for the per-layer colour picker."""

_NUMBER = re.compile(r"\d+")


@dataclass(frozen=True)
class LayerSpec:
    """Immutable description of one overlay layer.

    Parameters
    ----------
    key : str
        Unique id, ``"region:<channel name>"`` or ``"camera:<camera label>"``.
    name : str
        Display name (channel name, or camera label).
    camera : str
        Physical camera label supplying the frames.
    rect : tuple[int, int, int, int] | None
        ``(x, y, w, h)`` full-sensor crop; ``None`` means the whole frame.
    laser_preset : str
        Laser config preset this layer maps to (e.g. ``"488nm"``), or ``""``.
    default_color : str
        Hex colour the layer starts with.
    """

    key: str
    name: str
    camera: str
    rect: tuple[int, int, int, int] | None = None
    laser_preset: str = ""
    default_color: str = "#808080"


@dataclass
class LayerDisplay:
    """Mutable per-layer display state.

    Parameters
    ----------
    color : str
        Hex tint colour.
    visible : bool
        Whether the layer is drawn.
    auto_contrast : bool
        Whether the contrast range follows the data.
    clim : tuple[float, float]
        Manual contrast range, used when *auto_contrast* is off.
    """

    color: str
    visible: bool = True
    auto_contrast: bool = True
    clim: tuple[float, float] = (0.0, 65535.0)


def _wavelength(laser_preset: str) -> int | None:
    """Return the first plausible wavelength (nm) found in *laser_preset*."""
    for m in _NUMBER.findall(laser_preset):
        value = int(m)
        if 300 <= value <= 1100:
            return value
    return None


def _first_unused(used: Collection[str]) -> str:
    used_lower = {u.lower() for u in used}
    for color in PALETTE.values():
        if color.lower() not in used_lower:
            return color
    return PALETTE["Gray"]


def default_color_for(laser_preset: str, used: Collection[str] = ()) -> str:
    """Pick a default tint colour from the wavelength in a laser preset name.

    Parameters
    ----------
    laser_preset : str
        Preset name such as ``"488nm"``; the first number in 300..1100 is taken
        as the wavelength (``<450`` cyan, ``450-530`` green, ``530-600``
        magenta, ``>=600`` red).
    used : Collection[str]
        Hex colours already taken. If the heuristic colour is among them, or no
        wavelength parses, the first unused palette colour is returned instead
        (Gray when the palette is exhausted).

    Returns
    -------
    str
        Hex colour string.
    """
    wl = _wavelength(laser_preset)
    if wl is not None:
        if wl < 450:
            color = PALETTE["Cyan"]
        elif wl < 530:
            color = PALETTE["Green"]
        elif wl < 600:
            color = PALETTE["Magenta"]
        else:
            color = PALETTE["Red"]
        if color.lower() not in {u.lower() for u in used}:
            return color
    return _first_unused(used)


def region_layer_specs(
    channels: Sequence[SpectralChannelConfig], cameras: Sequence[str]
) -> list[LayerSpec]:
    """Build one layer spec per drawn spectral region.

    Parameters
    ----------
    channels : Sequence[SpectralChannelConfig]
        Configured spectral channels, in config order.
    cameras : Sequence[str]
        Physical camera labels currently available; channels on other cameras
        and channels without a rect are skipped.

    Returns
    -------
    list[LayerSpec]
        Specs keyed ``"region:<channel name>"``, colours from
        :func:`default_color_for` with a running set of used colours.
    """
    specs: list[LayerSpec] = []
    used: set[str] = set()
    for ch in channels:
        if ch.rect is None or ch.camera not in cameras:
            continue
        color = default_color_for(ch.laser_preset, used)
        used.add(color)
        specs.append(
            LayerSpec(
                key=f"region:{ch.name}",
                name=ch.name,
                camera=ch.camera,
                rect=ch.rect,
                laser_preset=ch.laser_preset,
                default_color=color,
            )
        )
    return specs


def camera_layer_specs(cameras: Sequence[str]) -> list[LayerSpec]:
    """Build one whole-frame layer spec per camera.

    Parameters
    ----------
    cameras : Sequence[str]
        Physical camera labels, in order. The first is tinted Magenta and the
        second Green (the historical behaviour), the rest take the first unused
        palette colours.

    Returns
    -------
    list[LayerSpec]
        Specs keyed ``"camera:<label>"`` with ``rect=None``.
    """
    first = [PALETTE["Magenta"], PALETTE["Green"]]
    specs: list[LayerSpec] = []
    used: set[str] = set()
    for i, cam in enumerate(cameras):
        color = first[i] if i < len(first) else _first_unused(used)
        used.add(color)
        specs.append(
            LayerSpec(key=f"camera:{cam}", name=cam, camera=cam, default_color=color)
        )
    return specs


def default_reference_key(
    specs: Sequence[LayerSpec], hidden: Collection[str] = ()
) -> str | None:
    """Choose the default reference layer.

    Parameters
    ----------
    specs : Sequence[LayerSpec]
        Candidate layers, in order.
    hidden : Collection[str]
        Keys of hidden layers, which are never chosen.

    Returns
    -------
    str | None
        Key of the first visible spec whose laser preset contains ``"488"``,
        else the first visible spec, else ``None``.
    """
    visible = [s for s in specs if s.key not in hidden]
    for s in visible:
        if "488" in s.laser_preset:
            return s.key
    return visible[0].key if visible else None


def crop_layer_frame(
    frame: np.ndarray, spec: LayerSpec, origin: tuple[int, int]
) -> np.ndarray | None:
    """Extract a layer's pixels from a camera frame.

    Parameters
    ----------
    frame : np.ndarray
        The camera's live frame.
    spec : LayerSpec
        The layer; a ``None`` rect returns *frame* itself (no copy).
    origin : tuple[int, int]
        Full-sensor ``(x, y)`` of the frame's top-left pixel (the camera ROI
        origin), subtracted from the spec's rect.

    Returns
    -------
    np.ndarray | None
        The cropped pixels, or ``None`` when the rect is not fully inside the
        frame.
    """
    if spec.rect is None:
        return frame
    return crop_region(frame, spec.rect, origin)


def spectral_signature(channels: Sequence[SpectralChannelConfig]) -> tuple:
    """Return a hashable summary of the channel config for change detection.

    Parameters
    ----------
    channels : Sequence[SpectralChannelConfig]
        Configured spectral channels.

    Returns
    -------
    tuple
        ``(name, camera, laser_preset, rect)`` per channel.
    """
    return tuple((c.name, c.camera, c.laser_preset, c.rect) for c in channels)
