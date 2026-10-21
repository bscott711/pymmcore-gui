"""Offline image checks for comparing Live/Snap against MDA, and for drift.

Pure numpy (plus ``tifffile`` for loading), no hardware access, so these run
anywhere -- the embedded console, a notebook, or tests. Re-exported from
:mod:`~pymmcore_gui.asi_z_stack.diagnostics` for bench use.

- :func:`row_profile_ratio` -- MDA/Snap brightness per band of sensor rows.
  A rolling-shutter camera fired by a short laser pulse leaves later-read
  rows under-lit, so the ratio falls off along the readout direction; with
  the camera's pseudo-global "All Rows" mode it should be flat near 1.
- :func:`volume_drift` -- sub-pixel shift of each timepoint's volume against
  the first, by phase correlation, to quantify drift across a timelapse.
"""

from __future__ import annotations

import itertools
import logging
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Sequence

logger = logging.getLogger(__name__)


def _as_array(source: str | Path | np.ndarray) -> np.ndarray:
    if isinstance(source, str | Path):
        import tifffile

        return np.asarray(tifffile.imread(source))
    return np.asarray(source)


def _as_plane(source: str | Path | np.ndarray, plane: int | None) -> np.ndarray:
    """Load *source* and reduce it to one 2D (rows, cols) plane."""
    data = _as_array(source)
    while data.ndim > 2:
        index = data.shape[0] // 2 if plane is None else plane
        data = data[index]
        plane = None
    return data.astype(np.float64)


def row_profile_ratio(
    snap: str | Path | np.ndarray,
    mda: str | Path | np.ndarray,
    bands: int = 16,
    *,
    mda_plane: int | None = None,
    offset: float | None = None,
) -> np.ndarray:
    """Return the MDA/Snap mean-intensity ratio for each band of sensor rows.

    Parameters
    ----------
    snap : str | Path | np.ndarray
        The Live/Snap frame (a saved ``.ome.tif`` or an array).
    mda : str | Path | np.ndarray
        The MDA frame of the same plane. A stack is reduced to its middle
        plane unless *mda_plane* is given.
    bands : int
        Number of equal-height row bands to compare (rows are the camera's
        rolling-readout axis).
    mda_plane : int | None
        Which plane of a multi-plane *mda* to use.
    offset : float | None
        Camera offset (dark level) subtracted from both images before
        comparing. ``None`` estimates it per image as the 1st percentile.

    Returns
    -------
    np.ndarray
        One ratio per band, top band first. Flat near 1.0 means MDA matches
        Snap; a ratio that falls off with row position is the rolling-shutter
        signature of a laser pulse shorter than the rows' exposure spread.
    """
    a = _as_plane(snap, None)
    b = _as_plane(mda, mda_plane)
    if a.shape != b.shape:
        raise ValueError(f"Snap {a.shape} and MDA {b.shape} frames differ in shape")
    a = a - (np.percentile(a, 1) if offset is None else offset)
    b = b - (np.percentile(b, 1) if offset is None else offset)

    edges = np.linspace(0, a.shape[0], bands + 1).astype(int)
    ratios = np.empty(bands)
    for i, (r0, r1) in enumerate(itertools.pairwise(edges)):
        denom = a[r0:r1].mean()
        ratios[i] = b[r0:r1].mean() / denom if denom > 0 else np.nan
    lines = [
        f"  rows {r0:5d}-{r1 - 1:5d}: MDA/Snap = {ratio:.3f}"
        for r0, r1, ratio in zip(edges[:-1], edges[1:], ratios, strict=True)
    ]
    logger.info("Row-band MDA/Snap ratios:\n" + "\n".join(lines))
    return ratios


def phase_shift(reference: np.ndarray, moving: np.ndarray) -> np.ndarray:
    """Return the sub-pixel shift of *moving* relative to *reference*.

    Phase correlation over all axes, refined per axis with a parabolic fit
    around the correlation peak. A positive component means *moving*'s
    content sits at a larger index along that axis than *reference*'s.

    Parameters
    ----------
    reference : np.ndarray
        The reference image or volume.
    moving : np.ndarray
        Same shape as *reference*.
    """
    if reference.shape != moving.shape:
        raise ValueError(f"Shapes differ: {reference.shape} vs {moving.shape}")
    ref = reference.astype(np.float64)
    mov = moving.astype(np.float64)
    cross = np.fft.fftn(mov - mov.mean()) * np.conj(np.fft.fftn(ref - ref.mean()))
    cross /= np.abs(cross) + 1e-12
    corr = np.fft.ifftn(cross).real
    peak = np.unravel_index(int(np.argmax(corr)), corr.shape)

    shift = np.empty(corr.ndim)
    for axis, p in enumerate(peak):
        n = corr.shape[axis]
        lo, hi = list(peak), list(peak)
        lo[axis], hi[axis] = (p - 1) % n, (p + 1) % n
        c0, cm, cp = corr[peak], corr[tuple(lo)], corr[tuple(hi)]
        denom = cm - 2 * c0 + cp
        frac = 0.5 * (cm - cp) / denom if denom != 0 else 0.0
        s = p + frac
        shift[axis] = s - n if s > n / 2 else s
    return shift


def _bin(volume: np.ndarray, factor: int) -> np.ndarray:
    """Average-bin the last two (y, x) axes by *factor*."""
    if factor <= 1:
        return volume
    *lead, ny, nx = volume.shape
    ny, nx = ny - ny % factor, nx - nx % factor
    trimmed = volume[..., :ny, :nx]
    blocks = trimmed.reshape(*lead, ny // factor, factor, nx // factor, factor)
    return np.asarray(blocks.mean(axis=(-3, -1)))


def volume_drift(
    volumes: np.ndarray | Sequence[np.ndarray],
    *,
    bin_yx: int = 4,
) -> np.ndarray:
    """Return each timepoint's shift against the first, as ``(T, ndim)``.

    Parameters
    ----------
    volumes : np.ndarray | Sequence[np.ndarray]
        A ``(T, Z, Y, X)`` array (or ``(T, Y, X)`` for single planes), or a
        sequence of per-timepoint volumes of equal shape.
    bin_yx : int
        Average-bin Y and X by this factor first (speed; drift is reported in
        unbinned pixels).

    Returns
    -------
    np.ndarray
        Row *t* is the ``(dz, dy, dx)`` (or ``(dy, dx)``) shift of timepoint
        *t* relative to timepoint 0, in slices/pixels. Row 0 is all zeros.
    """
    stack = [_bin(np.asarray(v, dtype=np.float64), bin_yx) for v in volumes]
    scale = np.ones(stack[0].ndim)
    scale[-2:] = max(bin_yx, 1)
    shifts = np.zeros((len(stack), stack[0].ndim))
    for t in range(1, len(stack)):
        shifts[t] = phase_shift(stack[0], stack[t]) * scale
    lines = [
        f"  t={t:3d}: " + ", ".join(f"{s:+7.2f}" for s in row)
        for t, row in enumerate(shifts)
    ]
    axes = "(dz, dy, dx)" if stack[0].ndim == 3 else "(dy, dx)"
    logger.info(f"Drift vs t=0 {axes}, slices/pixels:\n" + "\n".join(lines))
    return shifts
