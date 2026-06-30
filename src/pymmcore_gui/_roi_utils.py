"""Helpers for multi-ROI (dual-ROI) camera readout.

Some cameras (notably PVCAM / Photometrics) can read out several sub-regions of the
sensor in a single exposure via MMCore's multi-ROI API
(:meth:`~pymmcore_plus.CMMCorePlus.setMultiROI` /
:meth:`~pymmcore_plus.CMMCorePlus.getMultiROI`).  When multiple ROIs are active the
camera returns a single *packed composite* image containing only the ROI pixels — the
full chip is never transferred.

This module provides Qt-free helpers to:

* convert a drawn rectangle to an integer pixel ROI (:func:`rect_to_pixel_roi`),
* apply / clear ROIs on the core (:func:`apply_dual_roi`, :func:`clear_roi`),
* split the packed composite back into one array per ROI (:func:`split_composite`).

Coordinate convention
----------------------
ROIs are ``(x, y, w, h)`` integer tuples in **sensor pixel** coordinates, where
``x``/``y`` is the top-left corner — matching
:meth:`~pymmcore_plus.CMMCorePlus.getROI` / ``setROI``.
"""

from __future__ import annotations

import warnings
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    from pymmcore_plus import CMMCorePlus

# (x, y, width, height) in sensor pixels, top-left origin.
PixelROI = tuple[int, int, int, int]


def rois_from_multiroi_tuple(
    xs: list[int], ys: list[int], ws: list[int], hs: list[int]
) -> list[PixelROI]:
    """Convert the 4-list form returned by ``getMultiROI`` to a list of ROIs."""
    return [
        (int(x), int(y), int(w), int(h))
        for x, y, w, h in zip(xs, ys, ws, hs, strict=False)
    ]


def rect_to_pixel_roi(
    top_left: tuple[float, float],
    bot_right: tuple[float, float],
    *,
    max_w: int | None = None,
    max_h: int | None = None,
) -> PixelROI:
    """Convert two opposite rectangle corners to an integer pixel ROI.

    Parameters
    ----------
    top_left, bot_right : tuple[float, float]
        Opposite corners ``(x, y)`` of the rectangle in sensor-pixel coordinates.
        The two points may be given in any order; the result is normalized so that
        ``w`` and ``h`` are positive.
    max_w, max_h : int | None
        Optional sensor width / height used to clamp the ROI inside the chip.

    Returns
    -------
    tuple[int, int, int, int]
        ``(x, y, w, h)`` with ``w >= 1`` and ``h >= 1``.
    """
    x0, y0 = top_left
    x1, y1 = bot_right
    x = round(min(x0, x1))
    y = round(min(y0, y1))
    w = round(abs(x1 - x0))
    h = round(abs(y1 - y0))

    x = max(x, 0)
    y = max(y, 0)
    w = max(w, 1)
    h = max(h, 1)

    if max_w is not None:
        x = min(x, max(max_w - 1, 0))
        w = min(w, max_w - x)
    if max_h is not None:
        y = min(y, max(max_h - 1, 0))
        h = min(h, max_h - y)

    return (x, y, max(w, 1), max(h, 1))


def apply_dual_roi(mmc: CMMCorePlus, rois: list[PixelROI]) -> None:
    """Set multiple ROIs on the current camera device.

    Operates on the core's *current* camera.  Note that, unlike single-ROI
    ``setROI``, ``setMultiROI`` does **not** emit a ``roiSet`` event — callers that
    need a UI/preview refresh should snap a frame afterwards.

    Parameters
    ----------
    mmc : CMMCorePlus
        The core instance.
    rois : list[tuple[int, int, int, int]]
        One or more ``(x, y, w, h)`` ROIs.

    Raises
    ------
    RuntimeError
        If the current camera does not support multiple ROIs.
    ValueError
        If *rois* is empty or any width/height is non-positive.
    """
    if not rois:
        raise ValueError("At least one ROI is required.")
    if not mmc.isMultiROISupported():
        raise RuntimeError(
            f"Camera {mmc.getCameraDevice()!r} does not support multiple ROIs."
        )
    if any(w <= 0 or h <= 0 for _, _, w, h in rois):
        raise ValueError(f"All ROI widths/heights must be positive, got {rois!r}.")

    xs = [x for x, _, _, _ in rois]
    ys = [y for _, y, _, _ in rois]
    ws = [w for _, _, w, _ in rois]
    hs = [h for _, _, _, h in rois]
    mmc.setMultiROI(xs, ys, ws, hs)


def clear_roi(mmc: CMMCorePlus, *, label: str | None = None) -> None:
    """Reset the camera to full-chip readout and emit a ``roiSet`` event.

    Restores full-frame readout (clearing any single- or multi-ROI) and re-applies it
    via ``setROI`` so that listeners relying on the ``roiSet`` signal (previews, the
    Camera ROI widget) refresh — ``clearROI`` alone is silent.
    """
    cam = label or mmc.getCameraDevice()
    if not cam:
        return
    # clearROI() resets to full chip (silently); getROI then reports full sensor size.
    mmc.clearROI()
    x, y, w, h = mmc.getROI(cam)
    # Re-apply the full-chip ROI through setROI so a roiSet event is emitted.
    mmc.setROI(cam, x, y, w, h)


def split_composite(frame: np.ndarray, rois: list[PixelROI]) -> dict[int, np.ndarray]:
    """Split a packed multi-ROI composite into one sub-array per ROI.

    The exact way a camera packs multiple ROIs into a single buffer is
    device-specific, so the layout is **inferred and validated** from the composite's
    dimensions rather than assumed.  Three layouts are recognized:

    * *bounding-box* (e.g. the demo camera): the composite is the bounding box of all
      ROIs, each ROI placed at its offset within it;
    * *horizontal tight-pack*: equal-height ROIs concatenated left-to-right;
    * *vertical tight-pack*: equal-width ROIs concatenated top-to-bottom.

    Parameters
    ----------
    frame : np.ndarray
        The packed composite.  ROIs are sliced from the last two axes ``(..., H, W)``.
    rois : list[tuple[int, int, int, int]]
        The ROI geometry, as returned by :func:`rois_from_multiroi_tuple`.

    Returns
    -------
    dict[int, np.ndarray]
        ``{roi_index: sub_array}``.  For a single ROI returns ``{0: frame}``.

    Raises
    ------
    ValueError
        If the composite dimensions match none of the known layouts.  Callers should
        catch this and fall back to writing the composite unsplit (never guess).
    """
    if len(rois) <= 1:
        return {0: frame}

    h_img, w_img = frame.shape[-2], frame.shape[-1]
    xs = [r[0] for r in rois]
    ys = [r[1] for r in rois]
    ws = [r[2] for r in rois]
    hs = [r[3] for r in rois]

    xmin, ymin = min(xs), min(ys)
    bbox_w = max(x + w for x, w in zip(xs, ws, strict=False)) - xmin
    bbox_h = max(y + h for y, h in zip(ys, hs, strict=False)) - ymin

    # 1) Bounding-box layout: each ROI lives at its offset within the bounding box.
    if (h_img, w_img) == (bbox_h, bbox_w):
        out: dict[int, np.ndarray] = {}
        for i, (x, y, w, h) in enumerate(rois):
            r0, c0 = y - ymin, x - xmin
            out[i] = frame[..., r0 : r0 + h, c0 : c0 + w]
        return out

    # 2) Horizontal tight-pack: equal heights, widths summed along x.
    if len(set(hs)) == 1 and h_img == hs[0] and w_img == sum(ws):
        out = {}
        c0 = 0
        for i, w in enumerate(ws):
            out[i] = frame[..., :, c0 : c0 + w]
            c0 += w
        return out

    # 3) Vertical tight-pack: equal widths, heights summed along y.
    if len(set(ws)) == 1 and w_img == ws[0] and h_img == sum(hs):
        out = {}
        r0 = 0
        for i, h in enumerate(hs):
            out[i] = frame[..., r0 : r0 + h, :]
            r0 += h
        return out

    raise ValueError(
        f"Cannot determine multi-ROI packing: composite shape {(h_img, w_img)} "
        f"does not match bounding-box {(bbox_h, bbox_w)}, horizontal {(hs[0], sum(ws))}"
        f", or vertical {(sum(hs), ws[0])} layouts for ROIs {rois!r}."
    )


def safe_split_composite(
    frame: np.ndarray, rois: list[PixelROI]
) -> dict[int, np.ndarray] | None:
    """Like :func:`split_composite` but return ``None`` (with a warning) on failure.

    Convenience wrapper for the save pipeline: when the layout cannot be determined
    the composite should be written unsplit rather than risk scrambling pixels.
    """
    try:
        return split_composite(frame, rois)
    except ValueError as exc:  # pragma: no cover - exercised via handler tests
        warnings.warn(str(exc), RuntimeWarning, stacklevel=2)
        return None
