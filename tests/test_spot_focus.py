"""Spot focus lock: spot measurement, calibration, controller, fake camera.

Pure numpy, no hardware and no MMCore. Run with ``--noconftest``.
"""

from __future__ import annotations

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.spot_focus import (
    SpotCalibration,
    SpotFix,
    SpotFocusController,
    SpotLoopParams,
    fit_spot_calibration,
    focus_error,
    locate_spot,
)
from pymmcore_gui.asi_z_stack.toupcam_source import (
    FakeSpotSource,
    ToupcamSource,
    make_source,
)

SHAPE = (480, 640)
CAL = SpotCalibration(
    x0=320.0,
    y0=240.0,
    ux=0.6,
    uy=0.8,
    slope_px_per_um=10.0,
    p0_um=50.0,
)


def render(x, y, *, amp=800.0, sigma=5.0, bg=100.0, noise=0.0, seed=0, shape=SHAPE):
    """Return a uint16 frame with a Gaussian spot centred at (x, y)."""
    yy, xx = np.mgrid[0 : shape[0], 0 : shape[1]]
    img = bg + amp * np.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * sigma**2))
    if noise:
        img = img + np.random.default_rng(seed).normal(0, noise, img.shape)
    return np.clip(np.rint(img), 0, 65535).astype(np.uint16)


# ---------------------------------------------------------------------------
# locate_spot
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("x", "y"), [(320.0, 240.0), (123.37, 301.81), (500.5, 100.25), (40.9, 60.1)]
)
def test_locate_spot_subpixel(x, y):
    fix = locate_spot(render(x, y, noise=2.0))
    assert fix.ok, fix.reason
    assert fix.x == pytest.approx(x, abs=0.05)
    assert fix.y == pytest.approx(y, abs=0.05)
    assert fix.sigma_x == pytest.approx(5.0, rel=0.05)
    assert fix.amplitude == pytest.approx(800.0, rel=0.05)
    assert fix.background == pytest.approx(100.0, abs=3)
    assert not fix.saturated


def test_locate_spot_uint8_and_asymmetric():
    yy, xx = np.mgrid[0 : SHAPE[0], 0 : SHAPE[1]]
    img = 10 + 150 * np.exp(
        -((xx - 200.4) ** 2) / (2 * 4.0**2) - (yy - 99.7) ** 2 / (2 * 8.0**2)
    )
    fix = locate_spot(np.rint(img).astype(np.uint8))
    assert fix.ok
    assert (fix.x, fix.y) == pytest.approx((200.4, 99.7), abs=0.08)
    assert fix.sigma_x == pytest.approx(4.0, rel=0.1)
    assert fix.sigma_y == pytest.approx(8.0, rel=0.1)


@pytest.mark.parametrize("bayer", ["RGGB", "BGGR", "GRBG", "GBRG"])
def test_locate_spot_bayer_red_channel(bayer):
    """Only the red sites carry the LED; the other sites hold a flat floor."""
    x, y = 211.3, 155.6
    full = render(x, y, amp=900.0, noise=1.0)
    oy, ox = {"RGGB": (0, 0), "BGGR": (1, 1), "GRBG": (0, 1), "GBRG": (1, 0)}[bayer]
    mosaic = np.full_like(full, 100)
    mosaic[oy::2, ox::2] = full[oy::2, ox::2]
    fix = locate_spot(mosaic, channel="red", bayer=bayer)
    assert fix.ok, fix.reason
    assert fix.x == pytest.approx(x, abs=0.15)
    assert fix.y == pytest.approx(y, abs=0.15)
    assert fix.sigma_x == pytest.approx(5.0, rel=0.1)  # full-frame pixels
    # Wrong pattern reads the dark sites instead and finds nothing useful.
    wrong = {"RGGB": "BGGR", "BGGR": "RGGB", "GRBG": "GBRG", "GBRG": "GRBG"}[bayer]
    assert not locate_spot(mosaic, channel="red", bayer=wrong, min_amplitude=50).ok


def test_locate_spot_flags():
    sat = locate_spot(render(300, 200, amp=70000.0), saturation=65535)
    assert sat.saturated
    assert not sat.ok
    assert sat.reason == "saturated"
    assert locate_spot(render(300, 200, amp=100.0), min_amplitude=200).reason
    flat = locate_spot(np.full(SHAPE, 100, dtype=np.uint16))
    assert not flat.ok
    hot = np.full(SHAPE, 100, dtype=np.uint16)
    hot[50, 50] = 5000
    assert not locate_spot(hot).ok  # a hot pixel is not a spot


def test_locate_spot_rejects_bad_input():
    with pytest.raises(ValueError):
        locate_spot(np.zeros((4, 4, 3), dtype=np.uint8))
    with pytest.raises(ValueError):
        locate_spot(render(10, 10), channel="blue")


# ---------------------------------------------------------------------------
# Calibration and sign convention
# ---------------------------------------------------------------------------


def _sweep(p0=50.0, slope=10.0, ux=0.6, uy=0.8, n=11, noise=0.0):
    p = np.linspace(p0 - 1.25, p0 + 1.25, n)
    rng = np.random.default_rng(1)
    xs = 320 + ux * slope * (p - p0) + rng.normal(0, noise, n)
    ys = 240 + uy * slope * (p - p0) + rng.normal(0, noise, n)
    return p, xs, ys


def test_calibration_round_trip():
    p, xs, ys = _sweep(noise=0.05)
    cal = fit_spot_calibration(p, xs, ys, p0_um=50.0)
    assert cal.slope_px_per_um == pytest.approx(10.0, rel=0.02)
    assert (cal.ux, cal.uy) == pytest.approx((0.6, 0.8), abs=0.01)
    assert (cal.x0, cal.y0) == pytest.approx((320, 240), abs=0.1)
    assert cal.n_points == 11
    assert cal.residual_px < 0.2
    assert cal.created
    again = SpotCalibration.from_dict(cal.to_dict() | {"junk": 1})
    assert again == cal


def test_calibration_default_setpoint_is_sweep_middle():
    p, xs, ys = _sweep()
    cal = fit_spot_calibration(p, xs, ys)
    assert cal.p0_um == pytest.approx(50.0)


def test_calibration_errors():
    p, xs, ys = _sweep()
    with pytest.raises(ValueError):
        fit_spot_calibration(p[:2], xs[:2], ys[:2])
    with pytest.raises(ValueError):
        fit_spot_calibration(p, xs * 0 + 5, ys * 0 + 5)  # spot does not move
    # direction may point any way (negative slope along x)
    cal = fit_spot_calibration(p, 320 - 0.6 * 10 * (p - 50), 240 - 0.8 * 10 * (p - 50))
    assert (cal.ux, cal.uy) == pytest.approx((-0.6, -0.8), abs=1e-6)


def _fix(x, y):
    return SpotFix(x, y, 800, 100, 5, 5, 900, False, True)


def test_focus_error_sign_convention():
    """Spot at P=p0+d reads along=+d: moving P by -d returns it."""
    d = 0.4
    along, lateral = focus_error(_fix(320 + 0.6 * 10 * d, 240 + 0.8 * 10 * d), CAL)
    assert along == pytest.approx(d)
    assert lateral == pytest.approx(0.0, abs=1e-9)
    # Moving P by -along puts the spot back at the setpoint.
    x, y = 320 + 0.6 * 10 * (d - along), 240 + 0.8 * 10 * (d - along)
    assert (x, y) == pytest.approx((320, 240))


def test_lateral_negative_control():
    """A purely perpendicular shift is no focus error and all lateral."""
    px = 6.0
    along, lateral = focus_error(_fix(320 - 0.8 * px, 240 + 0.6 * px), CAL)
    assert along == pytest.approx(0.0, abs=1e-9)
    assert abs(lateral) == pytest.approx(px)
    ctrl = SpotFocusController(SpotLoopParams(), 50.0)
    assert ctrl.update(along, 0.0) is None  # no move for a lateral shift
    assert ctrl.fault is None


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


def test_deadband_holds_integral():
    ctrl = SpotFocusController(SpotLoopParams(deadband_um=0.05), 50.0)
    for i in range(20):
        assert ctrl.update(0.04, i) is None
    assert ctrl.n_moves == 0
    # the integral was held, so the first real move is plain kp-ish
    t = ctrl.update(0.2, 100.0)
    assert t == pytest.approx(50.0 - (0.6 * 0.2 + 0.05 * 0.2))


def test_update_sign_and_step_clamp():
    ctrl = SpotFocusController(SpotLoopParams(max_step_um=0.5), 50.0)
    assert ctrl.update(+0.3, 0.0) < 50.0  # spot says +, move P down
    big = SpotFocusController(SpotLoopParams(max_step_um=0.5), 50.0)
    assert big.update(-3.0, 0.0) == pytest.approx(50.5)
    assert big.p_cmd_um == pytest.approx(50.5)


def test_rate_cap():
    ctrl = SpotFocusController(SpotLoopParams(max_rate_hz=2.0, max_step_um=0.1), 50.0)
    moves = [
        t for t in np.arange(0, 10, 0.05) if ctrl.update(1.0, float(t)) is not None
    ]
    assert len(moves) == 20  # 2 Hz over 10 s
    assert np.diff(moves).min() >= 0.5 - 1e-9


def test_range_fault_and_excursion_fault():
    near = SpotFocusController(SpotLoopParams(p_min_um=0.0, max_step_um=0.5), 0.2)
    assert near.update(+1.0, 0.0) is None
    assert near.fault and "range" in near.fault
    assert near.update(-1.0, 10.0) is None  # stays dead after a fault
    assert near.n_moves == 0

    p = SpotLoopParams(max_excursion_um=1.0, max_step_um=0.5, ki=0.0)
    far = SpotFocusController(p, 50.0)
    t = 0.0
    while far.fault is None and t < 100:
        far.update(-5.0, t)
        t += 1.0
    assert far.fault and "start" in far.fault
    assert far.p_cmd_um <= 51.0 + 1e-9
    assert far.n_moves == 2  # 0.5 + 0.5, the third would exceed 1 um


def test_strikes_and_good_reset():
    ctrl = SpotFocusController(SpotLoopParams(strikes=3), 50.0)
    ctrl.bad("a")
    ctrl.bad("a")
    ctrl.good()
    ctrl.bad("a")
    ctrl.bad("a")
    assert ctrl.fault is None
    ctrl.bad("spot lost")
    assert ctrl.fault == "spot lost"
    assert ctrl.update(1.0, 0.0) is None


def test_params_round_trip():
    p = SpotLoopParams(kp=0.3, avg_frames=4)
    q = SpotLoopParams.from_dict(p.to_dict() | {"junk": 1})
    assert q == p
    assert SpotLoopParams().max_rate_hz == 2.0


def _simulate(ctrl, sign, *, offset=0.0, drift=0.0, dt=0.5, steps=120):
    """Closed loop on an ideal plant; returns the along_um history.

    ``along = offset + drift*t + (P_cmd - P_start)``: moving P up moves the spot
    up. *sign* = -1 feeds the controller the wrong-signed error.
    """
    p_start, hist = ctrl.p_cmd_um, []
    for k in range(steps):
        along = offset + drift * k * dt + (ctrl.p_cmd_um - p_start)
        hist.append(along)
        ctrl.update(sign * along, k * dt)
    return hist


def test_closed_loop_converges_within_deadband():
    ctrl = SpotFocusController(SpotLoopParams(max_excursion_um=50.0), 50.0)
    hist = _simulate(ctrl, +1, offset=1.5)
    assert abs(hist[-1]) <= 0.03 + 1e-9
    assert ctrl.fault is None
    assert ctrl.n_moves < 30  # settles, then stops moving


def test_closed_loop_tracks_drift():
    ctrl = SpotFocusController(SpotLoopParams(max_excursion_um=50.0), 50.0)
    hist = _simulate(ctrl, +1, drift=0.05)
    assert max(abs(h) for h in hist[50:]) < 0.2
    assert ctrl.fault is None


def test_sign_flipped_diverges():
    """Negative control: with the error sign flipped the loop runs away."""
    p = SpotLoopParams(max_excursion_um=3.0)
    flip = SpotFocusController(p, 50.0)
    hist = _simulate(flip, -1, offset=0.5)
    assert flip.fault is not None  # ran into the excursion limit
    assert abs(hist[-1]) > abs(hist[0]) + 2.0
    good = SpotFocusController(p, 50.0)
    assert abs(_simulate(good, +1, offset=0.5)[-1]) <= 0.03 + 1e-9


# ---------------------------------------------------------------------------
# FakeSpotSource
# ---------------------------------------------------------------------------


def _fake(**kw):
    return FakeSpotSource(CAL.to_dict(), fps=1000.0, **kw)


def test_fake_spot_follows_actuator():
    src = _fake(noise=1.0)
    src.open()
    try:
        first = src.latest(1.0)
        assert first is not None
        frame, stamp = first
        assert frame.dtype == np.uint16
        assert frame.shape == SHAPE
        fix0 = locate_spot(frame)
        assert (fix0.x, fix0.y) == pytest.approx((320, 240), abs=0.1)
        for dp in (-1.0, 0.5, 2.0):
            src.set_actuator_um(CAL.p0_um + dp)
            fix = locate_spot(src.latest(1.0)[0])
            assert fix.x == pytest.approx(320 + 0.6 * 10 * dp, abs=0.1)
            assert fix.y == pytest.approx(240 + 0.8 * 10 * dp, abs=0.1)
            along, lateral = focus_error(fix, CAL)
            assert along == pytest.approx(dp, abs=0.02)
            assert lateral == pytest.approx(0.0, abs=0.15)
        assert src.latest(1.0)[1] > stamp
    finally:
        src.close()


def test_fake_drift_and_lateral_drift():
    src = _fake(noise=0.5, drift_um_per_s=0.0, lateral_px_per_s=0.0)
    src.open()
    assert src.spot_position(2.0) == pytest.approx((320, 240))
    src.drift_um_per_s = 0.5
    assert src.spot_position(2.0) == pytest.approx((320 + 0.6 * 10, 240 + 0.8 * 10))
    src.drift_um_per_s = 0.0
    src.lateral_px_per_s = 2.0
    x, y = src.spot_position(3.0)
    assert (x, y) == pytest.approx((320 - 0.8 * 6, 240 + 0.6 * 6))
    along, lateral = focus_error(locate_spot(src.render(3.0)), CAL)
    assert along == pytest.approx(0.0, abs=0.02)
    assert lateral == pytest.approx(6.0, abs=0.15)


def test_fake_roi_info_and_pacing():
    src = _fake(noise=0.0)
    assert src.info()["kind"] == "fake"
    src.open()
    src.set_roi(200, 100, 300, 250)
    frame = src.latest(1.0)[0]
    assert frame.shape == (250, 300)
    assert src.info()["roi"] == (200, 100, 300, 250)
    src.set_exposure_ms(5)
    src.set_gain(150)
    assert src.info()["gain"] == 150
    slow = FakeSpotSource(CAL.to_dict(), fps=2.0)
    slow.open()
    slow.latest(1.0)  # first frame is immediate
    assert slow.latest(0.05) is None  # next one is 0.5 s away


def test_fake_is_seeded_and_picklable():
    import pickle

    spec = {"kind": "fake", "cal_like": CAL.to_dict(), "seed": 3, "fps": 1000.0}
    pickle.loads(pickle.dumps(spec))
    a, b = make_source(spec), make_source(spec)
    a.open()
    b.open()
    assert np.array_equal(a.render(0.0), b.render(0.0))


def test_make_source_and_missing_sdk(monkeypatch, tmp_path):
    assert isinstance(make_source({"kind": "toupcam", "serial": "x"}), ToupcamSource)
    with pytest.raises(ValueError):
        make_source({"kind": "nope"})
    monkeypatch.delenv("TOUPCAM_SDK_DIR", raising=False)
    with pytest.raises(RuntimeError, match="TOUPCAM_SDK_DIR"):
        ToupcamSource().open()
    with pytest.raises(RuntimeError, match="toupcam.py"):
        ToupcamSource(sdk_dir=str(tmp_path)).open()
