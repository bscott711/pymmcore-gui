"""Pure-numpy sub-pixel spot tracking for the Camera Alignment widget.

Deliberately has no Qt/pygfx dependency and no plugin/strategy abstraction:
:data:`TrackFunc` is just the call signature every tracking implementation
must match, and :func:`centroid_track` is the only one that exists today.
Swapping in FFT cross-correlation later means writing a second function with
the same signature and changing one assignment in
``CameraAlignmentWidget.__init__`` -- not touching this module's public shape.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

DEFAULT_BOX_RADIUS = 15  # px, half-width of the re-centered search box
DEFAULT_THRESHOLD_SIGMA = 4.0  # robust sigmas above local median to count as signal
_MAX_ITERATIONS = 4
_CONVERGENCE_TOL = 0.05  # px; stop iterating once the shift drops below this
_MAD_TO_SIGMA = 1.4826  # consistency constant for a Gaussian noise assumption


@dataclass(frozen=True, slots=True)
class TrackedSpot:
    """A tracked feature's current sub-pixel position and search-box size.

    Parameters
    ----------
    x : float
        Sub-pixel column coordinate in the source frame.
    y : float
        Sub-pixel row coordinate in the source frame.
    box_radius : int
        Half-width (pixels) of the square search box used to re-track this
        spot on the next frame.
    """

    x: float
    y: float
    box_radius: int = DEFAULT_BOX_RADIUS


def centroid_track(
    frame: np.ndarray,
    spot: TrackedSpot,
    threshold_sigma: float = DEFAULT_THRESHOLD_SIGMA,
) -> TrackedSpot | None:
    """Re-center `spot` on `frame` via a background-thresholded intensity centroid.

    Extracts a ``(2*box_radius+1)``-square box, estimates a robust local
    background from the box's median and MAD (median absolute deviation --
    unlike a fixed percentile, this adapts to how much of the box is
    actually signal vs. noise), keeps only pixels more than
    *threshold_sigma* robust-sigmas above that background, and computes
    their intensity-weighted (sub-pixel) centroid. The box is then
    re-cropped around that new estimate and the process repeats (up to a
    small internal cap) until the shift between iterations is negligible --
    so a single call converges onto the true local peak even if the seed
    position (e.g. a fresh user click) was off by several pixels, rather
    than only nudging closer to it.

    Parameters
    ----------
    frame : np.ndarray
        Current 2-D frame from one camera.
    spot : TrackedSpot
        The previous frame's (or freshly picked) position + box size to
        re-center on.
    threshold_sigma : float
        Number of robust sigmas above the local median a pixel must exceed
        to count as signal. Higher is more selective (better rejects
        sensor noise, worse for very dim features).

    Returns
    -------
    TrackedSpot | None
        Updated position (same ``box_radius``), or ``None`` if the box
        would extend past the frame edge on the very first iteration, or
        the thresholded signal is ~0 (degenerate/lost spot) before any
        iteration succeeded. If a later iteration would run off the frame
        edge, the last good result is returned instead of ``None`` --
        callers should hold the last good position rather than jump to a
        garbage value.
    """
    h, w = frame.shape[-2:]
    r = spot.box_radius
    cx, cy = spot.x, spot.y
    result: TrackedSpot | None = None

    for _ in range(_MAX_ITERATIONS):
        icx, icy = round(cx), round(cy)
        x0, x1 = icx - r, icx + r + 1
        y0, y1 = icy - r, icy + r + 1
        if x0 < 0 or y0 < 0 or x1 > w or y1 > h:
            return result

        box = frame[y0:y1, x0:x1].astype(np.float64)
        median = float(np.median(box))
        mad = float(np.median(np.abs(box - median)))
        threshold = median + threshold_sigma * _MAD_TO_SIGMA * mad
        sub = np.clip(box - threshold, 0, None)
        total = sub.sum()
        if total <= 1e-9:
            return result

        ys, xs = np.indices(box.shape)
        new_x = float((sub * xs).sum() / total) + x0
        new_y = float((sub * ys).sum() / total) + y0

        result = TrackedSpot(x=new_x, y=new_y, box_radius=r)
        shift = float(np.hypot(new_x - cx, new_y - cy))
        cx, cy = new_x, new_y
        if shift < _CONVERGENCE_TOL:
            break

    return result


TrackFunc = Callable[[np.ndarray, TrackedSpot], "TrackedSpot | None"]


def displacement(
    cam1_spot: TrackedSpot, cam2_spot: TrackedSpot
) -> tuple[float, float, float]:
    """Return ``(dx, dy, magnitude)`` = camera2 position minus camera1 position.

    All values in pixels. No baseline-zeroing: this is the raw pixel
    difference, so it starts at the true physical misalignment and trends
    toward zero as hardware is adjusted.

    Parameters
    ----------
    cam1_spot : TrackedSpot
        Tracked spot position on camera 1.
    cam2_spot : TrackedSpot
        Tracked spot position on camera 2.
    """
    dx = cam2_spot.x - cam1_spot.x
    dy = cam2_spot.y - cam1_spot.y
    return dx, dy, float(np.hypot(dx, dy))
