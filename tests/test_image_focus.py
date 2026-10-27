"""Tests for the Qt-free image-based focus lock logic."""

from __future__ import annotations

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.image_focus import (
    FocusBox,
    FocusCalibration,
    FocusLockController,
    LockLimits,
    calibration_key,
    fit_calibration,
    measure_shift,
    row_profile,
)

N_ROWS = 120


def _profile(shift: float = 0.0, *, noise: float = 0.0, seed: int = 0) -> np.ndarray:
    """Sum of Gaussians, moved toward higher rows by *shift* (sub-pixel)."""
    rows = np.arange(N_ROWS, dtype=float)
    p = (
        100 * np.exp(-0.5 * ((rows - 55 - shift) / 4.0) ** 2)
        + 60 * np.exp(-0.5 * ((rows - 75 - shift) / 6.0) ** 2)
        + 30 * np.exp(-0.5 * ((rows - 38 - shift) / 3.0) ** 2)
        + 5.0
    )
    if noise:
        p = p + np.random.default_rng(seed).normal(0, noise, rows.size)
    return p


def _calibration(rows_per_um: float = 2.0, **kw: object) -> FocusCalibration:
    args: dict = {
        "rows_per_um": rows_per_um,
        "intercept_rows": 0.0,
        "r2": 0.99,
        "range_um": (-2.0, 2.0),
        "n_points": 9,
        "date": "2026-10-26T00:00:00",
    }
    args.update(kw)
    return FocusCalibration(**args)


# ------------------------------ box / profile ------------------------------


def test_crop_subtracts_roi_origin() -> None:
    frame = np.arange(100 * 200).reshape(100, 200)
    box = FocusBox("Camera-1", "488nm", 5, (60, 40, 30, 20))
    crop = box.crop(frame, roi_origin=(50, 30))
    assert crop.shape == (20, 30)
    assert crop[0, 0] == frame[10, 10]
    assert np.shares_memory(crop, frame)  # a view, not a copy


def test_crop_clips_and_raises_when_empty() -> None:
    frame = np.zeros((100, 200))
    box = FocusBox("c", "p", 0, (180, 90, 50, 50))
    assert box.crop(frame).shape == (10, 20)
    # Left/top of the frame clip too.
    assert FocusBox("c", "p", 0, (-10, -10, 30, 30)).crop(frame).shape == (20, 20)
    with pytest.raises(ValueError, match="empty"):
        FocusBox("c", "p", 0, (500, 0, 10, 10)).crop(frame)
    with pytest.raises(ValueError, match="empty"):
        box.crop(frame, roi_origin=(400, 0))


def test_row_profile_subtracts_background_and_averages_columns() -> None:
    crop = np.full((10, 8), 100.0)
    crop[3, :] = 140.0
    prof = row_profile(crop, background_percentile=50.0)
    assert prof.shape == (10,)
    assert prof[3] == pytest.approx(40.0)
    assert prof.sum() == pytest.approx(40.0)
    assert (prof >= 0).all()


# ------------------------------ measure_shift ------------------------------


@pytest.mark.parametrize("shift", [-7.3, -0.4, 0.0, 2.6])
def test_measure_shift_known_subpixel_shifts(shift: float) -> None:
    ref = _profile(0.0)
    m = measure_shift(ref, _profile(shift))
    assert abs(m.shift_rows - shift) < 0.05
    assert abs(m.centroid_shift_rows - shift) < 0.5
    assert m.confidence > 0.95
    # Noisy: the reference and the moved profile each carry their own noise.
    ref_n = _profile(0.0, noise=2.0, seed=1)
    m = measure_shift(ref_n, _profile(shift, noise=2.0, seed=2))
    assert abs(m.shift_rows - shift) < 0.2


def test_measure_shift_sign_positive_is_higher_rows() -> None:
    m = measure_shift(_profile(0.0), _profile(3.0))
    assert m.shift_rows > 2.5
    assert m.centroid_shift_rows > 0


@pytest.mark.parametrize("flat", [np.zeros(N_ROWS), np.full(N_ROWS, 7.0)])
def test_measure_shift_flat_profile_is_zero_confidence(flat: np.ndarray) -> None:
    for ref, prof in ((_profile(), flat), (flat, _profile()), (flat, flat)):
        m = measure_shift(ref, prof)
        assert m.confidence == 0.0
        assert np.isfinite([m.shift_rows, m.centroid_shift_rows, m.signal]).all()


def test_measure_shift_shape_mismatch_and_nan() -> None:
    with pytest.raises(ValueError):
        measure_shift(np.zeros(10), np.zeros(11))
    bad = _profile()
    bad[3] = np.nan
    m = measure_shift(_profile(), bad)
    assert m.confidence == 0.0 and np.isfinite(m.shift_rows)


# ------------------------------ calibration ------------------------------


def test_fit_calibration_linear() -> None:
    um = np.linspace(-2, 2, 9)
    for gain in (1.8, -2.4):
        cal = fit_calibration(um, gain * um + 0.3, date="2026-10-26")
        assert cal.rows_per_um == pytest.approx(gain)
        assert cal.intercept_rows == pytest.approx(0.3)
        assert cal.r2 == pytest.approx(1.0)
        assert cal.range_um == (-2.0, 2.0)
        assert cal.n_points == 9
        assert cal.date == "2026-10-26"


def test_fit_calibration_trims_saturating_ends() -> None:
    um = np.linspace(-3, 3, 13)
    # Linear within +-2 um; beyond that the response saturates.
    shift = np.clip(2.0 * um, -4.0, 4.0)
    cal = fit_calibration(um, shift)
    assert cal.rows_per_um == pytest.approx(2.0, abs=0.05)
    assert cal.range_um == (-2.0, 2.0)
    assert cal.n_points == 9
    assert cal.r2 > 0.99
    # Without trimming the slope would be well under 2.
    assert np.polyfit(um, shift, 1)[0] < 1.8


def test_fit_calibration_trims_rollover_ends() -> None:
    um = np.linspace(-4, 4, 17)
    a = np.abs(um)
    # Linear to +-2.5 um, then the response turns back.
    shift = np.sign(um) * np.where(a <= 2.5, 2 * a, 5 - 2 * (a - 2.5))
    cal = fit_calibration(um, shift)
    assert cal.rows_per_um == pytest.approx(2.0, abs=0.01)
    assert cal.range_um == (-2.5, 2.5)


def test_fit_calibration_errors() -> None:
    with pytest.raises(ValueError, match="3 points"):
        fit_calibration([0.0, 1.0], [0.0, 1.0])
    with pytest.raises(ValueError, match="zero"):
        fit_calibration([0.0, 1.0, 2.0], [1.0, 1.0, 1.0])


def test_calibration_key() -> None:
    assert calibration_key("Camera-1", "488nm", "mNG") == "Camera-1|488nm|mNG"
    assert calibration_key("Camera-1", "488nm", "") == "Camera-1|488nm|"


# ------------------------------ controller ------------------------------


def _controller(
    rows_per_um: float = 2.0, *, start: float = 50.0, **limits: float
) -> FocusLockController:
    ctrl = FocusLockController(
        _calibration(rows_per_um),
        LockLimits(**limits),  # type: ignore[arg-type]
        start_position_um=start,
    )
    ctrl.set_reference(_profile(0.0))
    return ctrl


def test_controller_rejects_poor_calibration() -> None:
    with pytest.raises(ValueError, match="r2"):
        FocusLockController(_calibration(r2=0.5), LockLimits(), start_position_um=0.0)


def test_controller_requires_reference() -> None:
    ctrl = FocusLockController(_calibration(), LockLimits(), start_position_um=0.0)
    assert not ctrl.has_reference
    with pytest.raises(RuntimeError):
        ctrl.update(_profile(), 0.0)
    ctrl.set_reference(_profile())
    assert ctrl.has_reference


def test_controller_deadband_is_not_a_strike() -> None:
    ctrl = _controller()
    d = ctrl.update(_profile(0.3), 50.0)
    assert d.move_um == 0.0 and not d.held and not d.fell_back
    assert ctrl.strikes == 0


def test_controller_move_formula_and_sign() -> None:
    ctrl = _controller(2.0)
    d = ctrl.update(_profile(1.0), 50.0)  # +1 row, +2 rows/um, gain 0.5
    assert d.move_um == pytest.approx(-0.25, abs=0.01)
    assert d.target_um == pytest.approx(50.0 + d.move_um)
    assert d.error_um == pytest.approx(0.5, abs=0.02)
    # Negative calibration flips the move.
    ctrl = _controller(-2.0)
    d = ctrl.update(_profile(1.0), 50.0)
    assert d.move_um == pytest.approx(+0.25, abs=0.01)


def test_controller_step_clamp() -> None:
    ctrl = _controller(2.0)
    d = ctrl.update(_profile(3.9), 50.0)  # wants -0.97 um
    assert d.move_um == pytest.approx(-0.3)
    assert not d.saturated


def test_controller_excursion_clamp_then_saturation_fallback() -> None:
    ctrl = _controller(2.0, max_excursion_um=0.5, max_saturated=3)
    pos = 50.0
    d = ctrl.update(_profile(3.9), pos)  # -0.3 -> 49.7, within +-0.5
    assert not d.saturated
    pos += d.move_um
    d = ctrl.update(_profile(3.9), pos)  # wants 49.4, clipped to 49.5
    assert d.saturated and not d.fell_back
    assert d.target_um == pytest.approx(49.5)
    pos = d.target_um
    d = ctrl.update(_profile(3.9), pos)  # 2nd consecutive saturation
    assert d.saturated and not d.fell_back
    d = ctrl.update(_profile(3.9), pos)  # 3rd -> fall back
    assert d.fell_back and d.move_um == 0.0 and ctrl.fallen_back
    assert ctrl.total_offset_um == pytest.approx(-0.5)


def test_controller_travel_limits_saturate() -> None:
    ctrl = FocusLockController(
        _calibration(2.0),
        LockLimits(),
        start_position_um=50.0,
        travel_limits_um=(49.9, 60.0),
    )
    ctrl.set_reference(_profile(0.0))
    d = ctrl.update(_profile(3.9), 50.0)
    assert d.saturated and d.target_um == pytest.approx(49.9)


def test_controller_strikes_then_terminal_fallback() -> None:
    ctrl = _controller()
    flat = np.zeros(N_ROWS)
    d1 = ctrl.update(flat, 50.0)
    d2 = ctrl.update(flat, 50.0)
    assert d1.held and d2.held and not d2.fell_back and d1.reason
    assert ctrl.strikes == 2
    d3 = ctrl.update(flat, 50.0)
    assert d3.held and d3.fell_back and ctrl.fallen_back
    # Terminal: even a perfect measurement now just holds.
    d4 = ctrl.update(_profile(2.0), 50.0)
    assert d4.held and d4.fell_back and d4.move_um == 0.0


def test_controller_good_measurement_resets_strikes() -> None:
    ctrl = _controller()
    flat = np.zeros(N_ROWS)
    ctrl.update(flat, 50.0)
    ctrl.update(flat, 50.0)
    assert ctrl.strikes == 2
    ctrl.update(_profile(1.0), 50.0)
    assert ctrl.strikes == 0 and not ctrl.fallen_back


def test_controller_gates() -> None:
    # min_signal
    ctrl = _controller(min_signal=1e6)
    assert "signal" in (ctrl.update(_profile(1.0), 50.0).reason or "")
    # confidence: reference vs unrelated noise
    ctrl = _controller(min_confidence=0.999)
    d = ctrl.update(_profile(1.0, noise=20.0, seed=3), 50.0)
    assert d.held and "confidence" in (d.reason or "")
    # error outside the calibrated range: 10 rows / 2 = 5 um > 2 um
    ctrl = _controller()
    d = ctrl.update(_profile(10.0), 50.0)
    assert d.held and "outside" in (d.reason or "")
    # centroid disagreement
    ctrl = _controller(max_centroid_disagreement_rows=0.01)
    ref = _profile(0.0)
    skewed = _profile(2.0)
    skewed[:10] += 40.0  # drags the centroid, not the correlation peak
    d = ctrl.update(skewed, 50.0)
    assert ref is not None and d.held and "disagree" in (d.reason or "")


@pytest.mark.parametrize("rows_per_um", [2.0, -2.0, 0.7, -3.5])
def test_controller_converges_on_simulated_plant(rows_per_um: float) -> None:
    """Content shift = k * (piezo - best_focus); the loop walks it to ~0."""
    start = 50.0
    best_focus = start + 0.8  # the focus drifted
    ctrl = FocusLockController(
        _calibration(rows_per_um, range_um=(-3.0, 3.0)),
        LockLimits(max_step_um=1.0, deadband_rows=0.02),
        start_position_um=start,
    )
    ctrl.set_reference(_profile(0.0))
    pos = start
    errors = []
    for _ in range(10):
        # The reference was taken at best focus, so the shift is zero there.
        shift = rows_per_um * (pos - best_focus)
        d = ctrl.update(_profile(shift), pos)
        errors.append(abs(d.error_rows))
        pos += d.move_um
    assert abs(pos - best_focus) < 0.05
    assert errors[-1] < errors[0]
    assert errors[-1] < 0.2
