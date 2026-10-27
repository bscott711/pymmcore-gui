"""Spot focus lock: find the CRISP LED spot, convert its motion to a piezo move.

Pure numpy, no Qt, no MMCore and no hardware access, so the whole loop can be
unit-tested and replayed on saved frames. The CRISP LED beam goes through O2/O3
to a small camera; when O3 drifts out of focus the spot moves along one fixed
direction on that camera. We locate the spot with sub-pixel precision, project
its displacement from the setpoint onto the calibrated direction, and ask the
controller for a new piezo target.

- :class:`SpotFix` / :func:`locate_spot` -- sub-pixel spot position from a raw
  (mono or Bayer) frame, with quality flags.
- :class:`SpotCalibration` / :func:`fit_spot_calibration` -- setpoint, motion
  direction and px/um from a piezo sweep.
- :func:`focus_error` -- displacement split into along-direction (um) and
  lateral (px) parts.
- :class:`SpotLoopParams` / :class:`SpotFocusController` -- dead band, PI
  control, step clamp, rate cap and fault logic. It only ever returns targets;
  sending them to the stage is somebody else's job.

Sign convention used throughout: ``along_um`` is the focus error, such that
moving the piezo by **-along_um** returns the spot to the setpoint.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields
from datetime import datetime
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Sequence

# Offsets (row, col) of the red site inside the 2x2 Bayer cell.
_RED_SITE = {"RGGB": (0, 0), "BGGR": (1, 1), "GRBG": (0, 1), "GBRG": (1, 0)}
_SQRT_2PI = math.sqrt(2.0 * math.pi)
_BLUR = 5  # box-blur width (pixels) used to find the peak, robust to hot pixels


# ---------------------------------------------------------------------------
# Spot measurement
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SpotFix:
    """One spot measurement. Positions are in full-frame pixels."""

    x: float
    """Column of the spot centre (pixel centres are integers)."""
    y: float
    """Row of the spot centre."""
    amplitude: float
    """Fitted Gaussian height above the background (counts)."""
    background: float
    """Median of the ROI border (counts)."""
    sigma_x: float
    """Fitted Gaussian width along columns, full-frame pixels."""
    sigma_y: float
    """Fitted Gaussian width along rows, full-frame pixels."""
    peak: float
    """Brightest raw pixel inside the ROI (counts, background included)."""
    saturated: bool
    """True if any pixel in the ROI reached the saturation level."""
    ok: bool
    """False if the fix must not be used for control."""
    reason: str = ""
    """Why ``ok`` is False; empty when ok."""


def _box_blur(img: np.ndarray, k: int = _BLUR) -> np.ndarray:
    """Return the *k*-wide separable box mean of *img* (edges use zero padding)."""
    pad = k // 2
    out = np.pad(img, pad)
    cols = sum(out[:, i : i + img.shape[1] + 2 * pad - k + 1] for i in range(k))
    rows = sum(cols[i : i + img.shape[0]] for i in range(k))
    return rows / (k * k)


def _moments(profile: np.ndarray) -> tuple[float, float]:
    """Return (centre, sigma) of the above-baseline part of a 1D profile."""
    n = profile.size
    q = np.clip(profile - np.median(profile), 0.0, None)
    q[q < 0.2 * q.max()] = 0.0
    total = float(q.sum())
    if total <= 0:
        return (n - 1) / 2.0, max(n / 6.0, 1.0)
    idx = np.arange(n)
    mu = float(np.dot(idx, q) / total)
    var = float(np.dot((idx - mu) ** 2, q) / total)
    return mu, max(math.sqrt(var), 0.5)


def _fit_gauss_1d(profile: np.ndarray) -> tuple[float, float, float, bool]:
    """Fit ``A exp(-(i-mu)^2 / 2 s^2) + c`` to a 1D profile.

    Returns ``(A, mu, sigma, converged)``. Levenberg-Marquardt in numpy, started
    from the moments. On any failure the moment estimates are returned with
    ``converged=False`` (the centroid fallback).
    """
    y = np.asarray(profile, dtype=np.float64)
    n = y.size
    x = np.arange(n, dtype=np.float64)
    mu0, s0 = _moments(y)
    c0 = float(np.median(y))
    a0 = float(y.max() - c0)
    fallback = (a0, mu0, s0, False)
    if a0 <= 0:
        return fallback

    def model(th: np.ndarray) -> np.ndarray:
        return th[0] * np.exp(-((x - th[1]) ** 2) / (2 * th[2] ** 2)) + th[3]

    theta = np.array([a0, mu0, s0, c0])
    cost = float(np.sum((y - model(theta)) ** 2))
    lam = 1e-2
    for _ in range(40):
        a, mu, s, _c = theta
        g = np.exp(-((x - mu) ** 2) / (2 * s**2))
        jac = np.stack(
            [g, a * g * (x - mu) / s**2, a * g * (x - mu) ** 2 / s**3, np.ones(n)],
            axis=1,
        )
        resid = y - model(theta)
        hess = jac.T @ jac
        grad = jac.T @ resid
        try:
            step = np.linalg.solve(hess + lam * np.diag(np.diag(hess) + 1e-12), grad)
        except np.linalg.LinAlgError:
            return fallback
        cand = theta + step
        if cand[2] <= 0.2 or not np.all(np.isfinite(cand)):
            lam *= 10
            continue
        new_cost = float(np.sum((y - model(cand)) ** 2))
        if new_cost < cost:
            small = np.max(np.abs(step[:3])) < 1e-6
            theta, cost, lam = cand, new_cost, max(lam / 10, 1e-9)
            if small:
                break
        else:
            lam *= 10
            if lam > 1e8:
                break
    a, mu, s, _c = theta
    if a <= 0 or not (0 <= mu <= n - 1) or not (0.3 <= s <= n):
        return fallback
    return float(a), float(mu), float(s), True


def locate_spot(
    frame: np.ndarray,
    *,
    roi_half: int = 50,
    channel: str = "sum",
    bayer: str = "RGGB",
    saturation: float | None = None,
    min_amplitude: float = 0.0,
) -> SpotFix:
    """Locate the bright spot in a raw camera frame to sub-pixel precision.

    *frame* is a 2D uint8 or uint16 array (mono or the Bayer mosaic). With
    ``channel="red"`` only the red sites of the *bayer* pattern are used (the
    LED is 720 nm, so green and blue carry no signal); ``"sum"`` uses every
    pixel. Either way the returned positions and widths are in full-frame
    pixels. *roi_half* is the ROI half-size in full-frame pixels.
    *saturation* defaults to the dtype maximum. The fix is not ok when the ROI
    is saturated or the fitted amplitude is below *min_amplitude*.
    """
    arr = np.asarray(frame)
    if arr.ndim != 2:
        raise ValueError(f"Expected a 2D raw frame, got shape {arr.shape}")
    if saturation is None:
        saturation = float(np.iinfo(arr.dtype).max) if arr.dtype.kind in "ui" else 0.0
    if channel == "red":
        if bayer not in _RED_SITE:
            raise ValueError(f"Unknown Bayer pattern {bayer!r}")
        oy, ox = _RED_SITE[bayer]
        step = 2
    elif channel == "sum":
        oy = ox = 0
        step = 1
    else:
        raise ValueError(f"channel must be 'sum' or 'red', not {channel!r}")

    plane = arr[oy::step, ox::step].astype(np.float64)
    half = max(int(roi_half) // step, 3)
    rows, cols = plane.shape

    py, px = np.unravel_index(int(np.argmax(_box_blur(plane))), plane.shape)
    r0, r1 = max(py - half, 0), min(py + half + 1, rows)
    c0, c1 = max(px - half, 0), min(px + half + 1, cols)
    roi = plane[r0:r1, c0:c1]

    border = np.concatenate([roi[0], roi[-1], roi[1:-1, 0], roi[1:-1, -1]])
    bg = float(np.median(border))
    data = roi - bg
    peak = float(roi.max())
    saturated = bool(saturation > 0 and peak >= saturation)

    a_c, mu_x, sx, _ = _fit_gauss_1d(data.sum(axis=0))
    a_r, mu_y, sy, _ = _fit_gauss_1d(data.sum(axis=1))
    # A projection of a 2D Gaussian is A * sqrt(2 pi) * sigma_other, so the
    # two projections give two estimates of the 2D height.
    amp = 0.5 * (a_c / (_SQRT_2PI * sy) + a_r / (_SQRT_2PI * sx))

    reason = ""
    if saturated:
        reason = "saturated"
    elif not np.isfinite(amp) or amp <= 0 or amp < min_amplitude:
        reason = "spot lost (amplitude too low)"
    elif min(sx, sy) < 0.8:
        reason = "spot lost (too narrow, hot pixel?)"

    return SpotFix(
        x=float(ox + step * (c0 + mu_x)),
        y=float(oy + step * (r0 + mu_y)),
        amplitude=float(amp),
        background=bg,
        sigma_x=float(sx * step),
        sigma_y=float(sy * step),
        peak=peak,
        saturated=saturated,
        ok=not reason,
        reason=reason,
    )


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


@dataclass
class SpotCalibration:
    """Where the spot sits at focus and how it moves per um of piezo P."""

    x0: float
    """Setpoint column (spot position at ``p0_um``)."""
    y0: float
    """Setpoint row."""
    ux: float
    """x component of the unit vector of spot motion per +1 um of P."""
    uy: float
    """y component of the same unit vector."""
    slope_px_per_um: float
    """Spot displacement per um of P, along ``(ux, uy)``."""
    residual_px: float = 0.0
    """RMS distance of the sweep points from the fitted line."""
    p0_um: float = 0.0
    """Piezo P at the setpoint."""
    n_points: int = 0
    created: str = ""
    """ISO timestamp of the fit."""

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-friendly dict."""
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SpotCalibration:
        """Build from :meth:`to_dict` output; unknown keys are ignored."""
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})


def fit_spot_calibration(
    p_um: Sequence[float],
    xs: Sequence[float],
    ys: Sequence[float],
    *,
    p0_um: float | None = None,
) -> SpotCalibration:
    """Fit a line of spot position against piezo P.

    The direction is the slope vector ``(dx/dp, dy/dp)`` and the slope is its
    norm. The setpoint is the fitted position at *p0_um* (default: the middle
    of the sweep). Raises ``ValueError`` for fewer than 3 points or a slope
    under 0.2 px/um.
    """
    p = np.asarray(p_um, dtype=np.float64)
    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    if not (p.size == x.size == y.size) or p.size < 3:
        raise ValueError("Need at least 3 matching (p, x, y) points")
    if p0_um is None:
        p0_um = float((p.min() + p.max()) / 2.0)
    q = p - p0_um
    design = np.stack([q, np.ones_like(q)], axis=1)
    (sx, x0), *_ = np.linalg.lstsq(design, x, rcond=None)
    (sy, y0), *_ = np.linalg.lstsq(design, y, rcond=None)
    slope = float(math.hypot(sx, sy))
    if slope < 0.2:
        raise ValueError(f"Spot barely moves ({slope:.3f} px/um); is the beam on?")
    resid = float(np.sqrt(np.mean((x - (sx * q + x0)) ** 2 + (y - (sy * q + y0)) ** 2)))
    return SpotCalibration(
        x0=float(x0),
        y0=float(y0),
        ux=float(sx / slope),
        uy=float(sy / slope),
        slope_px_per_um=slope,
        residual_px=resid,
        p0_um=float(p0_um),
        n_points=int(p.size),
        created=datetime.now().isoformat(timespec="seconds"),
    )


def focus_error(fix: SpotFix, cal: SpotCalibration) -> tuple[float, float]:
    """Return ``(along_um, lateral_px)`` for a spot fix.

    ``along_um`` is the spot displacement from the setpoint projected on the
    calibrated direction, divided by the slope. Moving P by **-along_um**
    returns the spot to the setpoint. ``lateral_px`` is the perpendicular
    component, which focus cannot explain (beam walk, a bad fit).
    """
    dx, dy = fix.x - cal.x0, fix.y - cal.y0
    along = (dx * cal.ux + dy * cal.uy) / cal.slope_px_per_um
    lateral = -dx * cal.uy + dy * cal.ux
    return float(along), float(lateral)


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
    avg_frames: int = 8
    lateral_limit_px: float = 8.0
    min_amplitude: float = 20.0
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
    ``max_rate_hz``. A target outside the piezo range or further than
    ``max_excursion_um`` from the start sets :attr:`fault` instead of moving,
    and after a fault :meth:`update` never returns a target again.
    """

    def __init__(self, params: SpotLoopParams, p_start_um: float) -> None:
        self.params = params
        self.p_start_um = float(p_start_um)
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
        self._integral += along_um
        if p.ki > 0:  # anti-windup: the I term alone never exceeds one step
            lim = p.max_step_um / p.ki
            self._integral = max(-lim, min(lim, self._integral))
        step = -(p.kp * along_um + p.ki * self._integral)
        step = max(-p.max_step_um, min(p.max_step_um, step))
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
