"""Image-based focus lock: measure a row shift, convert it to a piezo move.

Pure numpy, no Qt and no hardware access, so the whole loop can be unit-tested
and replayed on saved data. The idea: when focus drifts, the raw camera content
on a given slice translates rigidly along the camera **row** axis. The user
draws a box on one (camera, laser preset, slice); for every volume we take the
row-intensity profile inside that box, measure its sub-pixel row shift against a
reference profile, convert the shift to micrometres with a calibration (signed
rows per um of piezo), and move the piezo between volumes.

- :class:`FocusBox` / :func:`row_profile` -- cut the box out of a frame and
  reduce it to a 1D profile along the rows.
- :func:`measure_shift` -- sub-pixel shift of one profile against a reference.
- :func:`fit_calibration` -- rows per um from a piezo sweep, with the
  saturating ends of the sweep trimmed off.
- :class:`FocusLockController` -- gates the measurements and turns the good
  ones into bounded, relative piezo moves, falling back for good when the
  measurements stop being trustworthy.

Sign convention used throughout: a **positive shift** means the content sits at
a *higher* row index than in the reference.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import datetime
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Sequence

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Box and profile
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FocusBox:
    """Where the lock looks: one box on one (camera, laser preset, slice)."""

    camera: str
    """Physical camera device label, e.g. ``"Camera-1"``."""
    preset: str
    """Single-laser preset whose frames are measured, e.g. ``"488nm"``."""
    slice_index: int
    """Which slice of the volume to measure."""
    rect: tuple[int, int, int, int]
    """``(x, y, w, h)`` in full-sensor pixels (x = column, y = row), the same
    convention as ``SpectralChannelConfig.rect``."""

    def crop(
        self, frame: np.ndarray, roi_origin: tuple[int, int] = (0, 0)
    ) -> np.ndarray:
        """Return a view of *frame* for this box.

        Frames start at the camera's hardware-ROI origin ``roi_origin = (x, y)``
        (full-sensor pixels), which is subtracted from the box. The box is
        clipped to the frame; ``ValueError`` if nothing is left.
        """
        x, y, w, h = self.rect
        ox, oy = roi_origin
        rows, cols = frame.shape[-2:]
        c0, c1 = max(x - ox, 0), min(x - ox + w, cols)
        r0, r1 = max(y - oy, 0), min(y - oy + h, rows)
        if c1 <= c0 or r1 <= r0:
            raise ValueError(
                f"Focus box {self.rect} is empty on a {cols}x{rows} frame "
                f"with ROI origin {roi_origin}"
            )
        return frame[..., r0:r1, c0:c1]


def row_profile(crop: np.ndarray, *, background_percentile: float = 50.0) -> np.ndarray:
    """Return the mean-over-columns intensity of *crop* for each row.

    The crop's *background_percentile* is subtracted first and negatives are
    clipped to zero, so a flat background does not wash out the profile.
    Length is the number of rows of *crop*.
    """
    data = np.asarray(crop, dtype=np.float64)
    if data.ndim != 2:
        raise ValueError(f"Expected a 2D crop, got shape {data.shape}")
    background = np.percentile(data, background_percentile)
    return np.clip(data - background, 0.0, None).mean(axis=1)


# ---------------------------------------------------------------------------
# Shift measurement
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ShiftMeasurement:
    """Row shift of one profile against a reference (gating lives elsewhere)."""

    shift_rows: float
    """Sub-pixel shift from the correlation peak; positive = content moved
    toward a HIGHER row index."""
    centroid_shift_rows: float
    """Shift of the intensity centroid, an independent cross-check."""
    confidence: float
    """Peak normalised cross-correlation, 0..1 (0 for a flat profile)."""
    signal: float
    """Mean of the profile (same units as the profile)."""


def _centroid(profile: np.ndarray) -> float:
    profile = profile - profile.min()  # a pedestal would pull it to the middle
    total = float(profile.sum())
    if not np.isfinite(total) or total <= 0:
        return 0.0
    return float(np.dot(np.arange(profile.size), profile) / total)


def _ncc(ref: np.ndarray, prof: np.ndarray, lag: int) -> float:
    """Normalised cross-correlation at an integer *lag* over the overlap.

    ``prof[i + lag]`` is compared with ``ref[i]``, so the peak sits at the lag
    by which the content moved toward higher indices.
    """
    n = ref.size
    a = ref[max(0, -lag) : n - max(0, lag)]
    b = prof[max(0, lag) : n - max(0, -lag)]
    a = a - a.mean()
    b = b - b.mean()
    denom = float(np.sqrt(np.dot(a, a) * np.dot(b, b)))
    if denom < 1e-12:
        return 0.0
    return float(np.dot(a, b) / denom)


def measure_shift(
    reference: np.ndarray,
    profile: np.ndarray,
    *,
    max_shift_rows: int | None = None,
) -> ShiftMeasurement:
    """Measure the row shift of *profile* relative to *reference*.

    Zero-mean normalised cross-correlation over integer lags
    ``|lag| <= max_shift_rows`` (default ``len // 4``), refined to sub-pixel
    with a parabola through the peak and its two neighbours. A flat or all-zero
    profile gives confidence 0 and shift 0 -- never NaN -- so a gate can
    reject it.
    """
    ref = np.asarray(reference, dtype=np.float64)
    prof = np.asarray(profile, dtype=np.float64)
    if ref.shape != prof.shape or ref.ndim != 1:
        raise ValueError(
            f"Profiles must be 1D and equal length: {ref.shape} vs {prof.shape}"
        )
    n = ref.size
    signal = float(prof.mean()) if n else 0.0
    if n < 4 or not (np.isfinite(ref).all() and np.isfinite(prof).all()):
        return ShiftMeasurement(0.0, 0.0, 0.0, signal if np.isfinite(signal) else 0.0)

    max_lag = n // 4 if max_shift_rows is None else int(max_shift_rows)
    max_lag = max(1, min(max_lag, n // 2 - 1))
    lags = np.arange(-max_lag, max_lag + 1)
    corr = np.array([_ncc(ref, prof, int(lag)) for lag in lags])

    peak = int(np.argmax(corr))
    confidence = float(np.clip(corr[peak], 0.0, 1.0))
    if confidence <= 0.0:
        return ShiftMeasurement(0.0, 0.0, 0.0, signal)

    frac = 0.0
    if 0 < peak < lags.size - 1:
        cm, c0, cp = corr[peak - 1], corr[peak], corr[peak + 1]
        denom = cm - 2 * c0 + cp
        if denom < 0:  # a real maximum; otherwise leave the integer lag
            frac = float(np.clip(0.5 * (cm - cp) / denom, -1.0, 1.0))
    shift = float(lags[peak]) + frac
    centroid = _centroid(prof) - _centroid(ref)
    return ShiftMeasurement(shift, float(centroid), confidence, signal)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FocusCalibration:
    """Linear map from piezo position to measured row shift."""

    rows_per_um: float
    """Signed: rows of content shift per +1 um of piezo."""
    intercept_rows: float
    """Shift at piezo = 0 of the fitted line."""
    r2: float
    """Coefficient of determination of the fit over the kept points."""
    range_um: tuple[float, float]
    """(min, max) piezo value of the points kept, i.e. the linear range."""
    n_points: int
    """Number of points kept in the fit."""
    date: str
    """ISO timestamp the calibration was made."""


def fit_calibration(
    piezo_um: Sequence[float] | np.ndarray,
    shift_rows: Sequence[float] | np.ndarray,
    *,
    date: str | None = None,
) -> FocusCalibration:
    """Fit ``shift_rows = rows_per_um * piezo_um + intercept`` to a sweep.

    The response saturates (or the profile leaves the box) at large offsets,
    so points are trimmed from the **ends** of the sweep -- the one with the
    largest ``|piezo - median|`` first -- while its residual exceeds
    ``max(0.5 row, 3 x MAD of the residuals)`` and at least 3 points would
    remain. The residuals are measured against a line fitted to the inner half
    of the kept points only: saturating ends would otherwise drag a
    whole-sweep fit toward themselves and hide their own error. The reported
    fit is least squares over the points kept, and ``range_um`` is their span.

    Pass the piezo as an offset from the sweep's start/centre if the result
    will feed :class:`FocusLockController`, which uses ``max(|range_um|)`` to
    bound the error it will act on.

    Raises ``ValueError`` for fewer than 3 points or a zero slope.
    """
    x = np.asarray(piezo_um, dtype=np.float64)
    y = np.asarray(shift_rows, dtype=np.float64)
    if x.shape != y.shape or x.ndim != 1:
        raise ValueError("piezo_um and shift_rows must be 1D and equal length")
    good = np.isfinite(x) & np.isfinite(y)
    x, y = x[good], y[good]
    if x.size < 3:
        raise ValueError(f"Need at least 3 points to calibrate, got {x.size}")

    keep = np.ones(x.size, dtype=bool)
    while keep.sum() > 3:
        idx = np.flatnonzero(keep)
        dist = np.abs(x[idx] - np.median(x[idx]))
        order = np.argsort(dist, kind="stable")
        inner = idx[order[: max(3, (idx.size + 1) // 2)]]
        if np.ptp(x[inner]) == 0:
            break
        slope, intercept = np.polyfit(x[inner], y[inner], 1)
        resid = y - (slope * x + intercept)
        mad = float(np.median(np.abs(resid[inner] - np.median(resid[inner]))))
        limit = max(0.5, 3.0 * mad)
        # Each end of the sweep, the one farthest from the middle first.
        ends = sorted(
            {
                int(np.argmin(np.where(keep, x, np.inf))),
                int(np.argmax(np.where(keep, x, -np.inf))),
            },
            key=lambda i: -abs(x[i] - np.median(x[idx])),
        )
        bad = [i for i in ends if abs(resid[i]) > limit]
        if not bad:
            break
        keep[bad[0]] = False

    xk, yk = x[keep], y[keep]
    if np.ptp(xk) == 0:
        raise ValueError("All piezo positions are identical")
    slope, intercept = np.polyfit(xk, yk, 1)
    if abs(slope) < 1e-12:
        raise ValueError("Calibration slope is zero: no response to the piezo")
    ss_res = float(np.sum((yk - (slope * xk + intercept)) ** 2))
    ss_tot = float(np.sum((yk - yk.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    if date is None:
        date = datetime.now().isoformat(timespec="seconds")
    return FocusCalibration(
        rows_per_um=float(slope),
        intercept_rows=float(intercept),
        r2=r2,
        range_um=(float(xk.min()), float(xk.max())),
        n_points=int(keep.sum()),
        date=date,
    )


def calibration_key(camera: str, preset: str, window: str = "") -> str:
    """Key a calibration by what was measured: ``"camera|preset|window"``.

    *window* names the spectral region (``""`` when spectral mode is off).
    """
    return f"{camera}|{preset}|{window}"


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LockLimits:
    """Tuning and safety limits for :class:`FocusLockController`."""

    loop_gain: float = 0.5
    """Fraction of the measured error corrected per update."""
    deadband_rows: float = 0.5
    """Errors up to this size are left alone (not a strike)."""
    max_step_um: float = 0.3
    """Largest piezo move in a single update."""
    max_excursion_um: float = 3.0
    """Largest distance the piezo may end up from where the run started."""
    min_confidence: float = 0.5
    """Minimum peak correlation to trust a measurement."""
    max_centroid_disagreement_rows: float = 3.0
    """Largest allowed gap between the correlation and centroid shifts."""
    min_signal: float = 1.0
    """Minimum profile mean (background-subtracted counts)."""
    max_strikes: int = 3
    """Consecutive rejected measurements before falling back."""
    max_saturated: int = 3
    """Consecutive moves clipped by the excursion/travel limits before
    falling back."""
    min_r2: float = 0.9
    """Calibrations fitting worse than this are refused."""


@dataclass(frozen=True)
class LockDecision:
    """What the controller wants done after one volume."""

    move_um: float
    """Relative piezo move to apply now (0 if holding)."""
    target_um: float
    """Absolute piezo position after the move."""
    error_rows: float
    """Measured shift against the reference."""
    error_um: float
    """The same error in piezo um (``error_rows / rows_per_um``)."""
    confidence: float
    """Peak correlation of the measurement."""
    held: bool = False
    """The measurement was rejected; nothing moved."""
    saturated: bool = False
    """The move was clipped by the excursion or travel limits."""
    fell_back: bool = False
    """The lock has given up; the caller should hand focus back."""
    reason: str | None = None
    """Why it held / fell back / stayed in the deadband."""


class FocusLockController:
    """Turns row-profile shifts into bounded relative piezo moves.

    Sign check: content that shifted ``+r`` rows with ``rows_per_um = +k``
    means the piezo is ``r / k`` um too far along its axis, so the move is
    ``-loop_gain * r / k`` um, which reduces the error. With ``k < 0`` the
    same formula flips the move, as it should.

    Every measurement passes gates (signal, confidence, agreement of the
    correlation and centroid shifts, and an error no larger than the
    calibration covers). A failed gate holds the piezo and counts a strike;
    ``max_strikes`` in a row -- or ``max_saturated`` moves in a row clipped by
    the excursion/travel limits -- makes the lock fall back for good: every
    later :meth:`update` holds with ``fell_back=True``.

    Parameters
    ----------
    calibration : FocusCalibration
        Rows per um; refused if its ``r2`` is below ``limits.min_r2``.
    limits : LockLimits
        Gains and safety limits.
    start_position_um : float
        Piezo position when the lock engaged; the excursion limit is
        measured from it.
    travel_limits_um : tuple[float, float] | None
        Absolute piezo travel limits, if known.
    """

    def __init__(
        self,
        calibration: FocusCalibration,
        limits: LockLimits | None = None,
        *,
        start_position_um: float,
        travel_limits_um: tuple[float, float] | None = None,
    ) -> None:
        limits = limits or LockLimits()
        if calibration.r2 < limits.min_r2:
            raise ValueError(
                f"Calibration r2 {calibration.r2:.3f} is below the minimum "
                f"{limits.min_r2:.3f}; recalibrate"
            )
        if calibration.rows_per_um == 0:
            raise ValueError("Calibration rows_per_um is zero")
        self.calibration = calibration
        self.limits = limits
        self._start = float(start_position_um)
        self._travel = travel_limits_um
        self._reference: np.ndarray | None = None
        self._position = self._start
        self._strikes = 0
        self._saturations = 0
        self._fallen_back = False

    # ----------------------------- state -----------------------------

    @property
    def has_reference(self) -> bool:
        """Whether :meth:`set_reference` has been called."""
        return self._reference is not None

    @property
    def total_offset_um(self) -> float:
        """Piezo position as last seen, relative to where the lock started."""
        return self._position - self._start

    @property
    def strikes(self) -> int:
        """Current run of consecutive rejected measurements."""
        return self._strikes

    @property
    def fallen_back(self) -> bool:
        """Whether the lock has given up."""
        return self._fallen_back

    def set_reference(self, profile: np.ndarray) -> None:
        """Take *profile* as the in-focus reference (a copy is stored)."""
        self._reference = np.array(profile, dtype=np.float64)

    # ----------------------------- update ----------------------------

    def update(self, profile: np.ndarray, current_position_um: float) -> LockDecision:
        """Decide this volume's piezo move from its row *profile*."""
        if self._reference is None:
            raise RuntimeError("set_reference() must be called before update()")
        current = float(current_position_um)
        self._position = current
        if self._fallen_back:
            return self._hold(current, 0.0, 0.0, 0.0, "fallen back", fell_back=True)

        lim = self.limits
        m = measure_shift(self._reference, profile)
        error_rows = m.shift_rows
        error_um = error_rows / self.calibration.rows_per_um
        bound = max(abs(v) for v in self.calibration.range_um)

        reason = None
        if m.signal < lim.min_signal:
            reason = f"signal {m.signal:.2f} < {lim.min_signal:.2f}"
        elif m.confidence < lim.min_confidence:
            reason = f"confidence {m.confidence:.2f} < {lim.min_confidence:.2f}"
        elif (
            abs(m.shift_rows - m.centroid_shift_rows)
            > lim.max_centroid_disagreement_rows
        ):
            reason = (
                f"correlation shift {m.shift_rows:.2f} and centroid shift "
                f"{m.centroid_shift_rows:.2f} rows disagree"
            )
        elif abs(error_um) > bound:
            reason = f"error {error_um:.2f} um outside calibrated range {bound:.2f} um"
        if reason is not None:
            self._strikes += 1
            fell = self._strikes >= lim.max_strikes
            if fell:
                self._fallen_back = True
                reason = f"{self._strikes} rejected measurements in a row ({reason})"
            return self._hold(
                current, error_rows, error_um, m.confidence, reason, fell_back=fell
            )

        self._strikes = 0
        if abs(error_rows) <= lim.deadband_rows:
            self._saturations = 0
            return LockDecision(
                move_um=0.0,
                target_um=current,
                error_rows=error_rows,
                error_um=error_um,
                confidence=m.confidence,
                reason="within deadband",
            )

        move = -lim.loop_gain * error_rows / self.calibration.rows_per_um
        move = float(np.clip(move, -lim.max_step_um, lim.max_step_um))
        lo = self._start - lim.max_excursion_um
        hi = self._start + lim.max_excursion_um
        if self._travel is not None:
            lo, hi = max(lo, self._travel[0]), min(hi, self._travel[1])
        target = float(np.clip(current + move, lo, hi))
        saturated = not np.isclose(target, current + move)
        if not saturated:
            self._saturations = 0
            return LockDecision(
                move_um=target - current,
                target_um=target,
                error_rows=error_rows,
                error_um=error_um,
                confidence=m.confidence,
            )

        self._saturations += 1
        if self._saturations >= lim.max_saturated:
            self._fallen_back = True
            return self._hold(
                current,
                error_rows,
                error_um,
                m.confidence,
                f"{self._saturations} moves in a row hit the excursion/travel limit",
                fell_back=True,
                saturated=True,
            )
        return LockDecision(
            move_um=target - current,
            target_um=target,
            error_rows=error_rows,
            error_um=error_um,
            confidence=m.confidence,
            saturated=True,
            reason="move clipped by excursion/travel limit",
        )

    @staticmethod
    def _hold(
        current: float,
        error_rows: float,
        error_um: float,
        confidence: float,
        reason: str,
        *,
        fell_back: bool = False,
        saturated: bool = False,
    ) -> LockDecision:
        return LockDecision(
            move_um=0.0,
            target_um=current,
            error_rows=error_rows,
            error_um=error_um,
            confidence=confidence,
            held=True,
            saturated=saturated,
            fell_back=fell_back,
            reason=reason,
        )
