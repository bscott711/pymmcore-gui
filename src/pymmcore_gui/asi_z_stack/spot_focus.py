"""Spot focus lock: measure one straight edge of the CRISP LED patch, drive the piezo.

Pure numpy, no Qt, no MMCore and no hardware access, so the whole loop can be
unit-tested and replayed on saved frames. The CRISP LED patch is imaged through
O2/O3 onto a small camera; one of its straight edges moves, with a known sign,
when O3 drifts out of focus. The user draws a small "edge box" across that
edge. Inside it we find, row by row, where the brightness crosses half way
between the dark outside and the bright inside, fit a straight line through
those crossings and report where the line crosses the box centre: one signed,
sub-pixel number that needs no search.

- :class:`EdgeFix` / :func:`measure_edge` -- sub-pixel edge position from a raw
  (mono or Bayer) frame, with quality flags.
- :class:`EdgeCalibration` / :func:`fit_edge_calibration` /
  :func:`load_calibration` / :func:`rezero` -- zero edge position, signed
  px/um and linear range from a piezo sweep; persistence that never raises.
- :func:`edge_error` / :func:`check_fix` -- the focus error in um and the
  per-frame sanity checks.
- :class:`SpotLoopParams` / :class:`SpotFocusController` -- dead band, PI
  control, step clamp, rate cap and fault logic. It only ever returns targets;
  sending them to the stage is somebody else's job.

Sign convention used throughout: ``along_um`` is the focus error, such that
moving the piezo by **-along_um** returns the edge to the zero (plus offset).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields, replace
from datetime import datetime
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Mapping, Sequence

# Offsets (row, col) of the red site inside the 2x2 Bayer cell.
_RED_SITE = {"RGGB": (0, 0), "BGGR": (1, 1), "GRBG": (0, 1), "GBRG": (1, 0)}
_BAND = 0.15  # outer fraction of the box (along the normal) used for the levels
_OLD_CAL_MSG = "old spot calibration ignored; recalibrate"
_NO_CAL_MSG = "no spot calibration yet; press Calibrate"


# ---------------------------------------------------------------------------
# Edge measurement
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class EdgeFix:
    """One edge measurement. Positions are in sensor pixels."""

    position: float
    """Edge position along the edge normal (the 50 % crossing at the box centre)."""
    x: float
    """Overlay point column: the edge point at the box centre."""
    y: float
    """Overlay point row."""
    axis: str
    """Normal of the edge: ``"x"`` (a vertical-ish edge) or ``"y"``."""
    polarity: int
    """+1 if the bright side is at larger coordinates along the normal, else -1."""
    contrast: float
    """Bright level minus dark level (raw counts)."""
    low: float
    """Dark-side level (median of the outer band, raw counts)."""
    high: float
    """Bright-side level."""
    angle_deg: float
    """Edge tilt: ``atan(d position / d along-edge coordinate)`` in degrees."""
    residual_px: float
    """RMS distance of the inlier crossings from the fitted line, sensor px."""
    valid_fraction: float
    """Fraction of rows that had a 50 % crossing."""
    saturated_frac: float
    """Fraction of raw pixels in the box at the saturation level."""
    peak: float
    """Brightest raw pixel in the box (counts)."""
    ok: bool
    """False if the fix must not be used for control."""
    reason: str = ""
    """Why ``ok`` is False; empty when ok."""


def snap_box(box: Sequence[float], *, min_size: int = 16) -> tuple[int, int, int, int]:
    """Snap an edge box ``(x, y, w, h)`` to even sensor pixels.

    Parameters
    ----------
    box : Sequence[float]
        ``(x, y, w, h)`` in sensor pixels.
    min_size : int
        Smallest width / height after snapping.

    Returns
    -------
    tuple[int, int, int, int]
        ``x`` and ``y`` rounded down to even, ``w`` and ``h`` rounded down to
        even and at least *min_size* (so the box holds whole 2x2 Bayer cells).
    """
    x, y, w, h = (math.floor(float(v)) for v in box)
    return (
        x // 2 * 2,
        y // 2 * 2,
        max(w // 2 * 2, min_size),
        max(h // 2 * 2, min_size),
    )


def _failed(
    reason: str,
    *,
    box: tuple[int, int, int, int],
    axis: str,
    polarity: int,
    **info: float,
) -> EdgeFix:
    """Build a not-ok :class:`EdgeFix` that still has finite numbers."""
    bx, by, bw, bh = box
    cx, cy = bx + bw / 2.0, by + bh / 2.0
    vals: dict[str, float] = {
        "position": cx if axis == "x" else cy,
        "contrast": 0.0,
        "low": 0.0,
        "high": 0.0,
        "angle_deg": 0.0,
        "residual_px": 0.0,
        "valid_fraction": 0.0,
        "saturated_frac": 0.0,
        "peak": 0.0,
    }
    vals.update(info)
    return EdgeFix(
        x=vals["position"] if axis == "x" else cx,
        y=cy if axis == "x" else vals["position"],
        axis=axis,
        polarity=polarity,
        ok=False,
        reason=reason,
        **vals,
    )


def measure_edge(
    frame: np.ndarray,
    box: Sequence[float],
    *,
    roi_offset: tuple[int, int] = (0, 0),
    channel: str = "sum",
    bayer: str = "RGGB",
    axis: str | None = None,
    polarity: int | None = None,
    saturation: float | None = None,
    max_saturated_frac: float = 0.002,
    min_contrast: float = 10.0,
    min_valid_fraction: float = 0.5,
) -> EdgeFix:
    """Measure the sub-pixel position of one straight edge inside a box.

    The box is cropped, reduced to a half-resolution grey image aligned to the
    2x2 Bayer cell (which removes the colour mosaic), and for every row along
    the edge the first crossing of the half-way level (between the dark and
    the bright side) is found by linear interpolation. A robust straight-line
    fit through the crossings gives the position at the box centre. Half-way
    levels make the result independent of exposure, gain and LED power, and a
    symmetric defocus blur widens the edge without moving its 50 % point.

    Parameters
    ----------
    frame : np.ndarray
        2-D raw frame (uint8 / uint16 / float), mono or the Bayer mosaic.
    box : Sequence[float]
        ``(x, y, w, h)`` of the edge box in sensor pixels (snapped to even
        values, at least 16 x 16, see :func:`snap_box`).
    roi_offset : tuple[int, int]
        Sensor ``(x, y)`` of the frame's first pixel (hardware ROI offset).
    channel : str
        ``"sum"`` (mean of every 2x2 cell) or ``"red"`` (the red site only,
        for the 720 nm LED).
    bayer : str
        Bayer pattern, used for ``channel="red"``.
    axis : str | None
        ``"x"`` / ``"y"`` normal of the edge; detected from the gradients
        when None.
    polarity : int | None
        +1 / -1 side of the bright part; detected when None.
    saturation : float | None
        Saturation level in raw counts; the dtype maximum by default (floats:
        never saturated).
    max_saturated_frac : float
        More saturated pixels than this fraction of the box -> not ok.
    min_contrast : float
        Bright minus dark level (raw counts) below this -> "edge lost".
    min_valid_fraction : float
        Fewer rows with a crossing than this -> "edge lost".

    Returns
    -------
    EdgeFix
        ``ok`` False with a ``reason`` when the box lies outside the frame, the
        box is saturated, the edge is missing or has walked to the box border.
    """
    arr = np.asarray(frame)
    if arr.ndim != 2:
        raise ValueError(f"Expected a 2D raw frame, got shape {arr.shape}")
    if channel not in ("sum", "red"):
        raise ValueError(f"channel must be 'sum' or 'red', not {channel!r}")
    if channel == "red" and bayer not in _RED_SITE:
        raise ValueError(f"Unknown Bayer pattern {bayer!r}")
    if axis not in (None, "x", "y"):
        raise ValueError(f"axis must be 'x' or 'y', not {axis!r}")
    sbox = snap_box(box)
    bx, by, bw, bh = sbox
    rx, ry = int(roi_offset[0]), int(roi_offset[1])
    fh, fw = arr.shape
    ax0 = axis or "x"
    pol0 = polarity or 1
    if bx < rx or by < ry or bx + bw > rx + fw or by + bh > ry + fh:
        return _failed(
            "edge box outside the camera ROI", box=sbox, axis=ax0, polarity=pol0
        )
    crop = arr[by - ry : by - ry + bh, bx - rx : bx - rx + bw]

    peak = float(crop.max())
    if saturation is None:
        saturation = float(np.iinfo(arr.dtype).max) if arr.dtype.kind in "ui" else 0.0
    sat_frac = (
        float(np.count_nonzero(crop >= saturation)) / crop.size
        if saturation > 0
        else 0.0
    )
    if sat_frac > max_saturated_frac:
        return _failed(
            "saturated",
            box=sbox,
            axis=ax0,
            polarity=pol0,
            peak=peak,
            saturated_frac=sat_frac,
        )

    # Half-resolution grey image, one value per 2x2 Bayer cell. Binned pixel
    # (i, j) sits at sensor (bx + 2j + off_x, by + 2i + off_y).
    if channel == "red":
        site_y, site_x = _RED_SITE[bayer]
        grey = crop[site_y::2, site_x::2].astype(np.float64)
        off_x, off_y = float(site_x), float(site_y)
    else:
        grey = crop.reshape(bh // 2, 2, bw // 2, 2).mean(axis=(1, 3), dtype=np.float64)
        off_x, off_y = 0.5, 0.5

    if axis is None:
        gx = float(np.abs(np.diff(grey, axis=1)).mean())
        gy = float(np.abs(np.diff(grey, axis=0)).mean())
        axis = "x" if gx >= gy else "y"
    # rows = along the edge, columns = along the normal
    g = grey if axis == "x" else grey.T
    off_n, off_a = (off_x, off_y) if axis == "x" else (off_y, off_x)
    n_rows, n_cols = g.shape
    if polarity is None:
        q = max(n_cols // 4, 1)
        polarity = 1 if g[:, -q:].mean() >= g[:, :q].mean() else -1
    s = g if polarity > 0 else g[:, ::-1]  # dark side first

    sm = s.copy()
    sm[:, 1:-1] = 0.25 * s[:, :-2] + 0.5 * s[:, 1:-1] + 0.25 * s[:, 2:]
    band = max(1, round(_BAND * n_cols))
    low = float(np.median(sm[:, :band]))
    high = float(np.median(sm[:, -band:]))
    contrast = high - low
    common: dict[str, float] = {
        "contrast": contrast,
        "low": low,
        "high": high,
        "peak": peak,
        "saturated_frac": sat_frac,
    }
    if contrast < min_contrast:
        return _failed(
            f"edge lost (contrast {contrast:.1f} < {min_contrast:g})",
            box=sbox,
            axis=axis,
            polarity=polarity,
            **common,
        )

    # First crossing of the half-way level in every row, linearly interpolated.
    threshold = low + 0.5 * contrast
    above = sm >= threshold
    first = above.argmax(axis=1)
    has = above[np.arange(n_rows), first] & (first >= 1)
    rows = np.nonzero(has)[0]
    valid_fraction = rows.size / n_rows
    common["valid_fraction"] = valid_fraction
    if valid_fraction < min_valid_fraction or rows.size < 3:
        return _failed(
            "edge lost (no crossing in most rows)",
            box=sbox,
            axis=axis,
            polarity=polarity,
            **common,
        )
    i1 = first[rows]
    v0, v1 = sm[rows, i1 - 1], sm[rows, i1]
    cross = (i1 - 1) + (threshold - v0) / (v1 - v0)

    # Robust line c(r) = a + b r, r = distance from the box centre row.
    r = rows.astype(np.float64) - (n_rows - off_a) / 2.0
    keep = np.ones(r.size, dtype=bool)
    a = b = 0.0
    res = np.zeros(r.size)
    for k in range(4):
        rk, ck = r[keep], cross[keep]
        rm, cm = rk.mean(), ck.mean()
        sxx = float(((rk - rm) ** 2).sum())
        b = float(((rk - rm) * (ck - cm)).sum() / sxx) if sxx > 0 else 0.0
        a = float(cm - b * rm)
        res = cross - (a + b * r)
        if k == 3:
            break
        rk_res = res[keep]
        mad = float(np.median(np.abs(rk_res - np.median(rk_res))))
        new_keep = np.abs(res) <= max(3.0 * 1.4826 * mad, 0.5)
        if new_keep.sum() < 3:
            break
        keep = new_keep
    residual_px = 2.0 * float(np.sqrt(np.mean(res[keep] ** 2)))
    common["residual_px"] = residual_px

    a_dark_first = a  # column index with the dark side first (band test)
    if polarity < 0:  # back to the original column direction
        a, b = (n_cols - 1) - a, -b
    common["angle_deg"] = math.degrees(math.atan(b))
    if a_dark_first < band - 0.5 or a_dark_first > n_cols - band - 0.5:
        return _failed(
            "edge left the box", box=sbox, axis=axis, polarity=polarity, **common
        )

    origin = bx if axis == "x" else by
    position = origin + 2.0 * a + off_n
    cx, cy = bx + bw / 2.0, by + bh / 2.0
    return EdgeFix(
        position=position,
        x=position if axis == "x" else cx,
        y=cy if axis == "x" else position,
        axis=axis,
        polarity=polarity,
        ok=True,
        reason="",
        **common,
    )


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


@dataclass
class EdgeCalibration:
    """Where the edge sits at focus and how it moves per um of piezo P."""

    box: tuple[int, int, int, int]
    """Edge box ``(x, y, w, h)`` in sensor pixels (as snapped)."""
    axis: str
    """Normal of the edge, ``"x"`` or ``"y"``."""
    polarity: int
    """+1 / -1: bright side at larger / smaller coordinates along the normal."""
    slope_px_per_um: float
    """Signed ``d(edge position) / dP`` in sensor px per um."""
    e0_px: float
    """Zero (setpoint): the edge position that counts as in focus."""
    p0_um: float
    """Piezo P at the zero."""
    r2: float = 1.0
    """Coefficient of determination of the sweep line."""
    residual_px: float = 0.0
    """RMS distance of the sweep points from the line."""
    linear_range_um: float = 1.0
    """Half-width of the calibrated range around ``p0_um``."""
    noise_px: float = 0.0
    """Single-measurement SD of the edge position."""
    angle_deg: float = 0.0
    """Reference edge tilt (a change flags a different feature)."""
    contrast: float = 0.0
    """Reference contrast (raw counts)."""
    n_points: int = 0
    sweep: list[tuple[float, float]] = field(default_factory=list)
    """The ``(p_um, edge_px)`` points the line was fitted to."""
    warnings: list[str] = field(default_factory=list)
    record_path: str = ""
    """Where the raw calibration frames were saved, if anywhere."""
    created: str = ""
    """ISO timestamp of the fit."""
    kind: str = "edge"

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-friendly dict."""
        d = asdict(self)
        d["box"] = [int(v) for v in self.box]
        d["sweep"] = [[float(p), float(e)] for p, e in self.sweep]
        d["warnings"] = [str(w) for w in self.warnings]
        return d

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> EdgeCalibration:
        """Build from :meth:`to_dict` output; unknown keys are ignored.

        Raises
        ------
        ValueError
            If ``kind`` is not ``"edge"`` (for example an old Gaussian spot
            calibration), a required key is missing or a value is malformed.
        """
        try:
            if d.get("kind") != "edge":
                raise ValueError("not an edge calibration")
            names = {f.name for f in fields(cls)}
            kw: dict[str, Any] = {k: v for k, v in d.items() if k in names}
            box = tuple(int(v) for v in kw["box"])
            if len(box) != 4:
                raise ValueError("box needs 4 values")
            axis = str(kw["axis"])
            polarity = int(kw["polarity"])
            if axis not in ("x", "y") or polarity not in (-1, 1):
                raise ValueError("bad axis / polarity")
            kw.update(
                box=box,
                axis=axis,
                polarity=polarity,
                slope_px_per_um=float(kw["slope_px_per_um"]),
                e0_px=float(kw["e0_px"]),
                p0_um=float(kw["p0_um"]),
                sweep=[(float(p), float(e)) for p, e in kw.get("sweep", [])],
                warnings=[str(w) for w in kw.get("warnings", [])],
            )
            for key in (
                "r2",
                "residual_px",
                "linear_range_um",
                "noise_px",
                "angle_deg",
                "contrast",
            ):
                if key in kw:
                    kw[key] = float(kw[key])
            if "n_points" in kw:
                kw["n_points"] = int(kw["n_points"])
            cal = cls(**kw)
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError(f"malformed edge calibration: {exc!r}") from exc
        if not (
            math.isfinite(cal.slope_px_per_um)
            and cal.slope_px_per_um != 0.0
            and math.isfinite(cal.e0_px)
            and math.isfinite(cal.p0_um)
        ):
            raise ValueError("edge calibration has a non-finite or zero slope")
        return cal


def load_calibration(d: Mapping[str, Any] | None) -> tuple[EdgeCalibration | None, str]:
    """Load a persisted calibration; never raises.

    Parameters
    ----------
    d : Mapping[str, Any] | None
        What :meth:`EdgeCalibration.to_dict` produced (or anything else).

    Returns
    -------
    tuple[EdgeCalibration | None, str]
        ``(calibration, "")`` or ``(None, message)``. An old Gaussian-spot
        calibration, a malformed dict or garbage give "old spot calibration
        ignored; recalibrate"; None or an empty dict give a "no calibration
        yet" message.
    """
    if d is None or (isinstance(d, dict) and not d):
        return None, _NO_CAL_MSG
    try:
        return EdgeCalibration.from_dict(d), ""
    except Exception:
        return None, _OLD_CAL_MSG


def fit_edge_calibration(
    p_um: Sequence[float],
    edge_px: Sequence[float],
    *,
    box: tuple[int, int, int, int],
    axis: str,
    polarity: int,
    p0_um: float,
    e0_px: float | None = None,
    noise_px: float = 0.0,
    angle_deg: float = 0.0,
    contrast: float = 0.0,
    linear_range_um: float | None = None,
    min_slope_px_per_um: float = 0.3,
    min_r2: float = 0.95,
) -> EdgeCalibration:
    """Fit edge position against piezo P with a straight line.

    Parameters
    ----------
    p_um : Sequence[float]
        Piezo positions of the sweep points.
    edge_px : Sequence[float]
        Measured edge positions; non-finite values (a failed measurement) are
        dropped.
    box : tuple[int, int, int, int]
        Edge box used, stored in the result.
    axis : str
        Edge normal, ``"x"`` or ``"y"``.
    polarity : int
        +1 / -1 bright side.
    p0_um : float
        Piezo P of the zero (the P the user focused at).
    e0_px : float | None
        Zero edge position; the fitted position at *p0_um* by default.
    noise_px : float
        Single-measurement SD of the edge position (also used by the
        monotonic check).
    angle_deg : float
        Reference edge tilt.
    contrast : float
        Reference contrast.
    linear_range_um : float | None
        Half-width of the valid range; half the sweep span by default.
    min_slope_px_per_um : float
        Smaller ``|slope|`` -> "edge barely moves".
    min_r2 : float
        Smaller R2 -> "not linear".

    Returns
    -------
    EdgeCalibration
        With the signed slope, R2, residual and warnings (R2 < 0.98,
        ``|slope| < 1 px/um``, dropped points).

    Raises
    ------
    ValueError
        With a plain message: fewer than 5 good points, an edge that barely
        moves, a non-linear or non-monotonic response.
    """
    p_all = np.asarray(p_um, dtype=np.float64)
    e_all = np.asarray(edge_px, dtype=np.float64)
    if p_all.shape != e_all.shape or p_all.ndim != 1:
        raise ValueError("p_um and edge_px must be 1-D and the same length")
    good = np.isfinite(p_all) & np.isfinite(e_all)
    p, e = p_all[good], e_all[good]
    if p.size < 5:
        raise ValueError(
            f"only {p.size} good sweep points, need at least 5; "
            "check the box and the LED"
        )
    order = np.argsort(p, kind="stable")
    p, e = p[order], e[order]
    q = p - float(p0_um)
    qm, em = float(q.mean()), float(e.mean())
    sqq = float(((q - qm) ** 2).sum())
    if sqq <= 0:
        raise ValueError("sweep has no piezo travel")
    slope = float(((q - qm) * (e - em)).sum() / sqq)
    e_c = em - slope * qm  # fitted edge at p0
    resid = e - (e_c + slope * q)
    ss_res = float((resid**2).sum())
    ss_tot = float(((e - em) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rms = math.sqrt(ss_res / p.size)

    if abs(slope) < min_slope_px_per_um:
        raise ValueError(
            f"edge barely moves ({slope:+.3f} px/um); wrong edge or span too small"
        )
    if r2 < min_r2:
        raise ValueError(
            f"edge position is not linear in P (R2 {r2:.3f}); reduce the span"
        )
    step = float(np.median(np.diff(p)))
    if abs(slope) * step > 2.0 * noise_px:
        moves = np.sign(np.diff(e)) == np.sign(slope)
        if moves.mean() < 0.75:
            raise ValueError(
                "edge position is not monotonic in P; reduce the span or "
                "check for a moving ghost / second edge in the box"
            )

    warns: list[str] = []
    if r2 < 0.98:
        warns.append(f"R2 {r2:.3f} is below 0.98; the response is only roughly linear")
    if abs(slope) < 1.0:
        warns.append(
            f"slope {slope:+.2f} px/um is small; the edge position will be noisy in um"
        )
    n_dropped = int(p_all.size - p.size)
    if n_dropped:
        warns.append(f"{n_dropped} sweep point(s) dropped (edge not measurable)")
    span = float(p.max() - p.min())
    return EdgeCalibration(
        box=tuple(int(v) for v in box),  # type: ignore[arg-type]
        axis=axis,
        polarity=int(polarity),
        slope_px_per_um=slope,
        e0_px=float(e_c if e0_px is None else e0_px),
        p0_um=float(p0_um),
        r2=r2,
        residual_px=rms,
        linear_range_um=float(
            span / 2.0 if linear_range_um is None else linear_range_um
        ),
        noise_px=float(noise_px),
        angle_deg=float(angle_deg),
        contrast=float(contrast),
        n_points=int(p.size),
        sweep=[(float(a), float(b)) for a, b in zip(p, e, strict=True)],
        warnings=warns,
        created=datetime.now().isoformat(timespec="seconds"),
    )


def rezero(cal: EdgeCalibration, *, e0_px: float, p0_um: float) -> EdgeCalibration:
    """Return *cal* with a new zero; slope, range and sweep are kept.

    Parameters
    ----------
    cal : EdgeCalibration
        The calibration to move.
    e0_px : float
        Edge position that now counts as in focus.
    p0_um : float
        Piezo P at that zero.

    Returns
    -------
    EdgeCalibration
        A copy with ``e0_px`` and ``p0_um`` replaced.
    """
    return replace(cal, e0_px=float(e0_px), p0_um=float(p0_um))


def edge_error(fix: EdgeFix, cal: EdgeCalibration, offset_um: float = 0.0) -> float:
    """Return the focus error ``along_um`` of an edge fix.

    ``(position - e0) / slope - offset``: moving P by **-along_um** returns the
    edge to the zero plus offset. At ``P = p0 + offset`` the edge sits at
    ``e0 + slope * offset`` and the error is 0.

    Parameters
    ----------
    fix : EdgeFix
        A good fix (see :func:`check_fix`).
    cal : EdgeCalibration
        The calibration (zero and signed slope).
    offset_um : float
        Wanted focus offset from the zero, in um of P.

    Returns
    -------
    float
        Error in um.
    """
    return float((fix.position - cal.e0_px) / cal.slope_px_per_um - offset_um)


def check_fix(fix: EdgeFix, cal: EdgeCalibration, params: SpotLoopParams) -> str:
    """Return why *fix* must not be used for control, or "" when it is fine.

    Parameters
    ----------
    fix : EdgeFix
        The measurement.
    cal : EdgeCalibration
        The calibration it is compared with.
    params : SpotLoopParams
        Limits ``max_angle_change_deg`` and ``max_edge_residual_px``.

    Returns
    -------
    str
        A short reason, or "".
    """
    if not fix.ok:
        return fix.reason or "edge lost"
    if abs(fix.angle_deg - cal.angle_deg) > params.max_angle_change_deg:
        return "edge rotated; the feature changed"
    if fix.residual_px > params.max_edge_residual_px:
        return "edge not straight"
    return ""


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


@dataclass
class SpotLoopParams:
    """Tuning and safety limits of the spot focus loop."""

    kp: float = 0.6
    ki: float = 0.05
    deadband_um: float = 0.03
    max_step_um: float = 0.5
    max_rate_hz: float = 2.0
    avg_frames: int = 4
    offset_um: float = 0.0
    """Wanted focus offset from the zero, in um of P."""
    min_contrast: float = 10.0
    max_saturated_frac: float = 0.002
    min_valid_fraction: float = 0.5
    max_angle_change_deg: float = 5.0
    max_edge_residual_px: float = 3.0
    p_min_um: float = 0.0
    p_max_um: float = 150.0
    max_excursion_um: float = 5.0
    """Largest allowed distance of the commanded P from the start P."""
    strikes: int = 5
    """Consecutive bad measurements before a fault."""

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-friendly dict."""
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SpotLoopParams:
        """Build from :meth:`to_dict` output; unknown keys are ignored."""
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})


class SpotFocusController:
    """Turn averaged focus errors into rare, bounded piezo targets.

    Proportional-integral control on the *averaged* error. Inside the dead band
    nothing moves and the integral is held. Outside it a step
    ``-(kp*e + ki*sum(e))`` is clamped to ``max_step_um`` and rate-limited to
    ``max_rate_hz``; the integral only accumulates when the step is not clamped
    and the error is inside the calibrated linear range. Beyond the linear
    range the sign of the error is still right, so the step is a full
    ``-sign(e) * max_step_um`` (no search needed). A target outside the piezo
    range or further than ``max_excursion_um`` from the start sets
    :attr:`fault` instead of moving, and after a fault :meth:`update` never
    returns a target again.

    Parameters
    ----------
    params : SpotLoopParams
        Gains and limits.
    p_start_um : float
        Piezo P when the lock starts.
    linear_range_um : float | None
        Largest ``|error|`` the calibration covers; None = no limit.
    """

    def __init__(
        self,
        params: SpotLoopParams,
        p_start_um: float,
        *,
        linear_range_um: float | None = None,
    ) -> None:
        self.params = params
        self.p_start_um = float(p_start_um)
        self.linear_range_um = linear_range_um
        self._p_cmd = float(p_start_um)
        self._integral = 0.0
        self._last_move_s: float | None = None
        self._n_moves = 0
        self._strikes = 0
        self._fault: str | None = None

    @property
    def p_cmd_um(self) -> float:
        """Last commanded target (the start P before any move)."""
        return self._p_cmd

    @property
    def fault(self) -> str | None:
        """Fault reason, or None while healthy."""
        return self._fault

    @property
    def n_moves(self) -> int:
        """Number of targets returned so far."""
        return self._n_moves

    def update(self, along_um: float, now_s: float) -> float | None:
        """Return the new target P in um, or None for no move."""
        p = self.params
        if self._fault is not None:
            return None
        if abs(along_um) <= p.deadband_um:
            return None
        if self._last_move_s is not None and now_s - self._last_move_s < 1.0 / max(
            p.max_rate_hz, 1e-9
        ):
            return None
        out_of_range = (
            self.linear_range_um is not None and abs(along_um) > self.linear_range_um
        )
        if out_of_range:
            step = -math.copysign(p.max_step_um, along_um)
        else:
            integral = self._integral + along_um
            if p.ki > 0:  # the I term alone never exceeds one step
                lim = p.max_step_um / p.ki
                integral = max(-lim, min(lim, integral))
            raw = -(p.kp * along_um + p.ki * integral)
            step = max(-p.max_step_um, min(p.max_step_um, raw))
            if step == raw:  # anti-windup: only integrate unclamped steps
                self._integral = integral
        target = self._p_cmd + step
        if not p.p_min_um <= target <= p.p_max_um:
            self._fault = f"range limit: target {target:.2f} um outside piezo range"
            return None
        if abs(target - self.p_start_um) > p.max_excursion_um:
            self._fault = (
                f"range limit: target {target:.2f} um is more than "
                f"{p.max_excursion_um:g} um from the start"
            )
            return None
        self._p_cmd = target
        self._last_move_s = now_s
        self._n_moves += 1
        return target

    def bad(self, reason: str) -> None:
        """Count one bad measurement; enough in a row set the fault."""
        self._strikes += 1
        if self._fault is None and self._strikes >= self.params.strikes:
            self._fault = reason

    def good(self) -> None:
        """A good measurement resets the strike count."""
        self._strikes = 0
