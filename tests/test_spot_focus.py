"""Spot focus lock: edge measurement, calibration, controller, fake camera.

Pure numpy, no hardware and no MMCore. Run with ``--noconftest``.
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.spot_focus import (
    DEFAULT_DETECTOR,
    EDGE_DETECTOR_LABELS,
    EDGE_DETECTORS,
    EdgeCalibration,
    EdgeFix,
    FocusCalibration,
    FusedError,
    SpotFocusController,
    SpotLoopParams,
    check_fix,
    detector_mismatch,
    edge_error,
    edge_noise_um,
    fit_edge_calibration,
    fuse_edges,
    load_calibration,
    measure_edge,
    rezero,
    snap_box,
)
from pymmcore_gui.asi_z_stack.spot_synthetic import add_noise, render_led_patch
from pymmcore_gui.asi_z_stack.toupcam_source import (
    FakeSpotSource,
    ToupcamSource,
    make_source,
)

SHAPE = (480, 640)
CX, CY = 320.0, 240.0
W, H = 96, 92  # default patch: edges at x = 272 / 368, y = 194 / 286
HALF_W, HALF_H = W / 2, H / 2

# Edge boxes (x, y, w, h): the edge sits 40 px inside, well clear of the
# corner radius (11 px) and of the outer 15 % bands.
BOX_LEFT = (232, 210, 80, 60)  # bright side at larger x   (polarity +1)
BOX_RIGHT = (328, 210, 80, 60)  # bright side at smaller x  (polarity -1)
BOX_TOP = (288, 154, 64, 80)  # axis y, polarity +1
BOX_BOTTOM = (288, 246, 64, 80)  # axis y, polarity -1


def clean(cx=CX, cy=CY, **kw):
    """Noise-free float patch without tail, ghost or texture."""
    kw = {"texture": 0.0, "tail": 0.0, "ghost": 0.0} | kw
    return render_led_patch(SHAPE, (cx, cy), **kw)


def as_uint(img, bits=16, seed=0, noise=0.0, hot_pixels=0):
    return add_noise(
        img,
        noise=noise,
        rng=np.random.default_rng(seed),
        hot_pixels=hot_pixels,
        bits=bits,
    )


def edge_positions(dx=0.0, dy=0.0):
    """True edge positions for a patch shifted by (dx, dy)."""
    return {
        "left": CX + dx - HALF_W,
        "right": CX + dx + HALF_W,
        "top": CY + dy - HALF_H,
        "bottom": CY + dy + HALF_H,
    }


# ---------------------------------------------------------------------------
# measure_edge: accuracy
# ---------------------------------------------------------------------------


def test_snap_box():
    assert snap_box((11, 7, 41, 33)) == (10, 6, 40, 32)
    assert snap_box((10, 10, 3, 5)) == (10, 10, 16, 16)  # minimum size
    assert snap_box((10, 10, 3, 5), min_size=8) == (10, 10, 8, 8)


@pytest.mark.parametrize("shift", [0.0, 0.1, 0.25, 0.37, 0.5, 0.73, 0.9, -0.6])
def test_subpixel_bias_both_axes_both_polarities(shift):
    # x edges move with dx, y edges move with dy.
    img = clean(CX + shift, CY + shift)
    truth = edge_positions(shift, shift)
    for box, key, axis, pol in [
        (BOX_LEFT, "left", "x", +1),
        (BOX_RIGHT, "right", "x", -1),
        (BOX_TOP, "top", "y", +1),
        (BOX_BOTTOM, "bottom", "y", -1),
    ]:
        fix = measure_edge(img, box)
        assert fix.ok, fix.reason
        assert (fix.axis, fix.polarity) == (axis, pol)
        assert fix.position == pytest.approx(truth[key], abs=0.05), key
        assert fix.contrast == pytest.approx(100.0, rel=0.01)
        assert abs(fix.angle_deg) < 0.2
        assert fix.valid_fraction == 1.0


def test_overlay_point_is_the_edge_at_the_box_centre():
    fix = measure_edge(clean(), BOX_LEFT)
    assert (fix.x, fix.y) == pytest.approx((272.0, 240.0), abs=0.05)
    fix = measure_edge(clean(), BOX_TOP)
    assert (fix.x, fix.y) == pytest.approx((320.0, 194.0), abs=0.05)


def test_given_axis_and_polarity_match_the_detected_ones():
    img = clean(CX + 0.3, CY)
    auto = measure_edge(img, BOX_RIGHT)
    given = measure_edge(img, BOX_RIGHT, axis="x", polarity=-1)
    assert given.position == pytest.approx(auto.position, abs=1e-9)
    # the wrong polarity finds no bright-to-dark edge
    wrong = measure_edge(img, BOX_RIGHT, axis="x", polarity=+1)
    assert not wrong.ok


def test_noisy_uint8_single_frame_sd():
    """Screenshot-like noise (0.4 counts SD on a ~100 count patch)."""
    base = clean(CX + 0.31, CY, level=90.0, background=1.0, texture=0.15)
    truth = measure_edge(base, BOX_LEFT).position
    pos = [
        measure_edge(as_uint(base, bits=8, seed=s, noise=0.4), BOX_LEFT).position
        for s in range(40)
    ]
    assert float(np.std(pos)) < 0.1
    assert float(np.mean(pos)) == pytest.approx(truth, abs=0.05)
    pos_hard = [
        measure_edge(as_uint(base, bits=8, seed=s, noise=2.0), BOX_LEFT).position
        for s in range(40)
    ]
    assert float(np.std(pos_hard)) < 0.1
    print(
        f"single-frame SD: {np.std(pos):.4f} px (0.4 counts), "
        f"{np.std(pos_hard):.4f} px (2 counts)"
    )


def _big_frame(seed=0, dx=0.0, texture=0.15, **kw):
    """2048 x 1536 uint8 frame modelled on the screenshot."""
    cx, cy = 1000.3 + dx, 700.7
    img = render_led_patch(
        (1536, 2048),
        (cx, cy),
        size=(308, 284),
        edge_sigma=12.0,
        level=90.0,
        background=1.0,
        texture=texture,
        tail=0.04,
        ghost=0.3,
        seed=seed,
        **kw,
    )
    frame = add_noise(img, noise=0.4, rng=np.random.default_rng(seed), hot_pixels=40)
    return frame, cx, cy


def test_full_scale_frame_edges_and_speed():
    frame, cx, cy = _big_frame()
    assert frame.dtype == np.uint8 and frame.shape == (1536, 2048)
    left_box = (780, 600, 140, 200)  # across the left edge (x = cx - 154)
    top_box = (900, 500, 200, 120)  # across the top edge (y = cy - 142)
    left = measure_edge(frame, left_box)
    top = measure_edge(frame, top_box)
    assert left.ok, left.reason
    assert top.ok, top.reason
    assert (left.axis, left.polarity) == ("x", +1)
    assert (top.axis, top.polarity) == ("y", +1)
    # The texture changes the local brightness at the edge, so the 50 % point
    # sits a few px off the geometric edge (soft edge, sigma = 12 px). That is a
    # constant of the patch, not noise: see test_full_scale_frame_follows_a_shift.
    assert left.position == pytest.approx(cx - 154, abs=4.0)
    assert top.position == pytest.approx(cy - 142, abs=4.0)
    # Without the texture the edge is found exactly, in the same frame layout.
    flat, _, _ = _big_frame(texture=0.0)
    flat_left = measure_edge(flat, left_box)
    flat_top = measure_edge(flat, top_box)
    assert flat_left.position == pytest.approx(cx - 154, abs=0.1)
    assert flat_top.position == pytest.approx(cy - 142, abs=0.1)

    measure_edge(frame, left_box)  # warm up
    times = []
    for box in (left_box, top_box) * 10:
        t0 = time.perf_counter()
        measure_edge(frame, box)
        times.append(time.perf_counter() - t0)
    median_ms = 1e3 * float(np.median(times))
    assert median_ms < 20.0
    print(
        f"full-frame measure_edge: {median_ms:.2f} ms median, "
        f"left err {flat_left.position - (cx - 154):+.3f} px (no texture), "
        f"{left.position - (cx - 154):+.3f} px (texture); "
        f"top err {flat_top.position - (cy - 142):+.3f} px (no texture), "
        f"{top.position - (cy - 142):+.3f} px (texture)"
    )


def test_full_scale_frame_follows_a_shift():
    """Texture moves with the patch, so a shift is measured almost exactly."""
    box = (780, 600, 140, 200)
    a = measure_edge(_big_frame(dx=0.0)[0], box)
    b = measure_edge(_big_frame(dx=2.4)[0], box)
    assert b.position - a.position == pytest.approx(2.4, abs=0.2)


def test_brightness_invariance():
    ref = measure_edge(as_uint(clean(CX + 0.3, level=1000.0)), BOX_LEFT).position
    for scale in (0.3, 3.0):
        img = as_uint(clean(CX + 0.3, level=1000.0 * scale, background=1.0))
        assert measure_edge(img, BOX_LEFT).position == pytest.approx(ref, abs=0.1)
    # same with a texture, because the texture scales with the patch
    t_ref = measure_edge(as_uint(clean(CX + 0.3, texture=0.15, level=1000)), BOX_LEFT)
    t_big = measure_edge(as_uint(clean(CX + 0.3, texture=0.15, level=3000)), BOX_LEFT)
    assert t_big.position == pytest.approx(t_ref.position, abs=0.1)
    assert t_big.contrast == pytest.approx(3 * t_ref.contrast, rel=0.02)


@pytest.mark.parametrize("bayer", ["RGGB", "BGGR", "GRBG", "GBRG"])
def test_bayer_red_channel(bayer):
    """Only the red sites carry the LED; the other sites hold a flat floor."""
    from pymmcore_gui.asi_z_stack.spot_focus import _RED_SITE

    oy, ox = _RED_SITE[bayer]
    shift = 0.37
    img = clean(CX + shift, CY + shift, level=200.0, background=5.0)
    mosaic = np.full(SHAPE, 5.0)
    mosaic[oy::2, ox::2] = img[oy::2, ox::2]
    mosaic = np.rint(mosaic).astype(np.uint16)
    truth = edge_positions(shift, shift)
    for box, key in [(BOX_LEFT, "left"), (BOX_TOP, "top")]:
        fix = measure_edge(mosaic, box, channel="red", bayer=bayer)
        assert fix.ok, fix.reason
        assert fix.position == pytest.approx(truth[key], abs=0.1), (bayer, key)
    with pytest.raises(ValueError):
        measure_edge(mosaic, BOX_LEFT, channel="blue")
    with pytest.raises(ValueError):
        measure_edge(mosaic, BOX_LEFT, channel="red", bayer="XXXX")
    with pytest.raises(ValueError):
        measure_edge(np.zeros((4, 4, 3), dtype=np.uint8), BOX_LEFT)


def test_symmetric_blur_does_not_move_the_edge():
    pos = [
        measure_edge(clean(CX + 0.4, edge_sigma=s), BOX_LEFT).position
        for s in (2.0, 3.0, 4.0, 6.0)
    ]
    assert max(pos) - min(pos) < 0.3
    assert pos[0] == pytest.approx(CX + 0.4 - HALF_W, abs=0.05)


def test_ghost_and_tail_outside_the_box_have_no_effect():
    plain = measure_edge(clean(CX + 0.2), BOX_LEFT).position
    full = render_led_patch(
        SHAPE, (CX + 0.2, CY), texture=0.0, tail=0.04, ghost=0.3, level=100.0
    )
    assert full[300:360, 260:380].max() > 15  # the ghost really is there
    fix = measure_edge(full, BOX_LEFT)
    assert fix.position == pytest.approx(plain, abs=0.02)


def test_tail_inside_the_box_barely_matters():
    """The tail is 4 % of the patch: far below the 50 % level."""
    clean_pos = measure_edge(clean(CX + 0.2), BOX_RIGHT).position
    tailed = render_led_patch(
        SHAPE, (CX + 0.2, CY), texture=0.0, tail=0.04, ghost=0.0, level=100.0
    )
    fix = measure_edge(tailed, BOX_RIGHT)
    assert fix.ok
    assert fix.position == pytest.approx(clean_pos, abs=0.3)


def test_hot_pixels_inside_the_box():
    img = clean(CX + 0.3)
    ref = measure_edge(img, BOX_LEFT).position
    rng = np.random.default_rng(5)
    hot = img.copy()
    ys = rng.choice(np.arange(212, 268), size=12, replace=False)
    xs = rng.integers(236, 308, size=12)  # dark side
    hot[ys, xs] = 600.0
    bad = hot.copy()
    bad[rng.integers(212, 268, 6), rng.integers(280, 308, 6)] = 0.0  # cold, bright
    for frame in (hot, bad):
        fix = measure_edge(frame, BOX_LEFT)
        assert fix.ok, fix.reason
        assert fix.position == pytest.approx(ref, abs=0.1)


@pytest.mark.parametrize("angle", [5.0, -5.0, 3.0])
def test_tilted_edge(angle):
    img = clean(angle_deg=angle)
    th = math.radians(angle)
    fix = measure_edge(img, BOX_LEFT)
    assert fix.ok, fix.reason
    assert fix.angle_deg == pytest.approx(angle, abs=0.5)
    yc = BOX_LEFT[1] + BOX_LEFT[3] / 2
    expect = CX + (-HALF_W + (yc - CY) * math.sin(th)) / math.cos(th)
    assert fix.position == pytest.approx(expect, abs=0.1)
    top = measure_edge(img, BOX_TOP)  # horizontal edge: dy/dx = -tan(angle)
    assert top.angle_deg == pytest.approx(-angle, abs=0.5)
    xc = BOX_TOP[0] + BOX_TOP[2] / 2
    expect_top = CY + (-HALF_H - (xc - CX) * math.sin(th)) / math.cos(th)
    assert top.position == pytest.approx(expect_top, abs=0.1)


def test_texture_moves_with_the_patch():
    a = measure_edge(clean(CX, texture=0.15, seed=3), BOX_LEFT).position
    b = measure_edge(clean(CX + 0.77, texture=0.15, seed=3), BOX_LEFT).position
    assert b - a == pytest.approx(0.77, abs=0.1)


# ---------------------------------------------------------------------------
# measure_edge: faults and the ROI
# ---------------------------------------------------------------------------


def test_led_off_is_edge_lost():
    flat = clean(level=0.0)
    fix = measure_edge(flat, BOX_LEFT)
    assert not fix.ok
    assert fix.reason.startswith("edge lost")
    noisy = as_uint(flat * 0 + 3.0, bits=8, noise=0.4)
    assert measure_edge(noisy, BOX_LEFT).reason.startswith("edge lost")


def test_saturated():
    img = as_uint(clean(level=400.0), bits=8)
    fix = measure_edge(img, BOX_LEFT)
    assert not fix.ok
    assert fix.reason == "saturated"
    assert fix.saturated_frac > 0.2
    assert fix.peak == 255.0
    # a stray hot pixel is below the limit and does not trip it
    ok = as_uint(clean(level=100.0), bits=8)
    ok[230, 290] = 255
    assert measure_edge(ok, BOX_LEFT).ok
    # explicit saturation level
    img16 = as_uint(clean(level=1000.0))
    assert measure_edge(img16, BOX_LEFT, saturation=500).reason == "saturated"
    assert measure_edge(img16, BOX_LEFT).ok


def test_edge_left_the_box():
    img = clean()
    near = (264, 210, 80, 60)  # the edge (x = 272) is 8 px from the left side
    fix = measure_edge(img, near)
    assert not fix.ok
    assert fix.reason == "edge left the box"
    far = (200, 210, 80, 60)  # the edge is 72 px in, 8 px from the right side
    assert measure_edge(img, far).reason == "edge left the box"
    assert measure_edge(img, (232, 210, 80, 60)).ok


def test_box_outside_the_roi():
    img = clean()
    for box in [(600, 400, 80, 80), (-20, 100, 80, 80), (100, 450, 80, 80)]:
        fix = measure_edge(img, box)
        assert not fix.ok
        assert "outside" in fix.reason
    # inside the frame but outside the ROI that the frame came from
    sub = img[100:400, 200:500]
    out = measure_edge(sub, BOX_LEFT, roi_offset=(200, 100))
    assert out.ok  # BOX_LEFT lies inside the ROI
    far = measure_edge(sub, (100, 210, 80, 60), roi_offset=(200, 100))
    assert not far.ok and "outside" in far.reason


@pytest.mark.parametrize("offset", [(200, 100), (201, 101), (0, 100), (200, 0)])
def test_roi_offset_is_in_sensor_coordinates(offset):
    img = clean(CX + 0.4, CY + 0.4)
    ox, oy = offset
    sub = img[oy : oy + 300, ox : ox + 400]
    for box in (BOX_LEFT, BOX_TOP):
        full = measure_edge(img, box)
        part = measure_edge(sub, box, roi_offset=offset)
        assert part.ok, part.reason
        assert part.position == pytest.approx(full.position, abs=1e-9)
        assert (part.x, part.y) == pytest.approx((full.x, full.y))


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


def _sweep(p0=50.0, slope=-4.0, e0=272.0, n=9, span=2.0, noise=0.0, seed=1):
    p = p0 + np.linspace(-span / 2, span / 2, n)
    rng = np.random.default_rng(seed)
    return p, e0 + slope * (p - p0) + rng.normal(0, noise, n)


def _fit(p, e, **kw):
    args = {"box": BOX_LEFT, "axis": "x", "polarity": 1, "p0_um": 50.0} | kw
    return fit_edge_calibration(p, e, **args)


def test_fit_recovers_signed_slope_and_zero():
    p, e = _sweep(slope=-4.0, noise=0.03)
    cal = _fit(p, e, noise_px=0.03, angle_deg=0.4, contrast=90.0)
    assert cal.slope_px_per_um == pytest.approx(-4.0, rel=0.01)
    assert cal.e0_px == pytest.approx(272.0, abs=0.05)
    assert cal.r2 > 0.999
    assert cal.residual_px < 0.06
    assert cal.linear_range_um == pytest.approx(1.0)
    assert cal.n_points == 9 and len(cal.sweep) == 9
    assert (cal.noise_px, cal.angle_deg, cal.contrast) == (0.03, 0.4, 90.0)
    assert cal.created and cal.kind == "edge" and not cal.warnings
    p, e = _sweep(slope=+7.5)
    assert _fit(p, e).slope_px_per_um == pytest.approx(7.5)
    # explicit zero and range
    cal = _fit(p, e, e0_px=270.0, linear_range_um=0.6)
    assert (cal.e0_px, cal.linear_range_um) == (270.0, 0.6)


def test_fit_errors():
    p, e = _sweep()
    with pytest.raises(ValueError, match="at least 5"):
        _fit(p[:4], e[:4])
    with pytest.raises(ValueError, match="barely moves"):
        _fit(p, e * 0 + 272.0)
    with pytest.raises(ValueError, match="barely moves"):
        _fit(p, 272.0 + 0.1 * (p - 50.0))
    q = p - 50.0
    with pytest.raises(ValueError, match="not linear"):
        _fit(p, 272.0 + 8.0 * (q > 0))  # a step, not a ramp
    zig = 272.0 + 10.0 * q + 3.0 * np.where(np.arange(9) % 2, 1.0, -1.0)
    with pytest.raises(ValueError, match="monotonic"):
        _fit(p, zig, min_r2=0.0, noise_px=0.1)
    # too few good points once failed measurements are dropped
    bad = e.copy()
    bad[[0, 3, 5, 7, 8]] = np.nan
    with pytest.raises(ValueError, match="at least 5"):
        _fit(p, bad)
    with pytest.raises(ValueError):
        _fit(p, e[:-1])


def test_fit_warnings():
    p, e = _sweep(slope=4.0)
    q = p - 50.0
    cal = _fit(p, e + 1.5 * q**2)  # bowed: R2 between 0.95 and 0.98
    assert 0.95 <= cal.r2 < 0.98
    assert any("R2" in w for w in cal.warnings)
    weak = _fit(p, 272.0 + 0.5 * q)
    assert any("small" in w for w in weak.warnings)
    bad = e.copy()
    bad[2] = np.nan
    dropped = _fit(p, bad)
    assert dropped.n_points == 8
    assert any("dropped" in w for w in dropped.warnings)


def test_calibration_round_trip_and_json():
    p, e = _sweep(noise=0.02)
    cal = _fit(p, e, noise_px=0.02)
    cal.warnings.append("hello")
    cal.record_path = "x.npz"
    again = EdgeCalibration.from_dict(cal.to_dict() | {"junk": 1})
    assert again == cal
    via_json = EdgeCalibration.from_dict(json.loads(json.dumps(cal.to_dict())))
    assert via_json == cal
    assert isinstance(via_json.box, tuple) and isinstance(via_json.sweep[0], tuple)
    loaded, msg = load_calibration(json.loads(json.dumps(cal.to_dict())))
    assert isinstance(loaded, FocusCalibration) and msg == ""
    assert loaded.edges == [cal] and loaded.p0_um == cal.p0_um
    assert loaded.created == cal.created and loaded.warnings == ["hello"]
    assert loaded.record_path == "x.npz"


def test_from_dict_rejects_bad_dicts():
    good = _fit(*_sweep()).to_dict()
    for bad in (
        good | {"kind": "spot"},
        {k: v for k, v in good.items() if k != "kind"},
        {k: v for k, v in good.items() if k != "box"},
        good | {"box": [1, 2, 3]},
        good | {"axis": "z"},
        good | {"slope_px_per_um": 0.0},
        good | {"slope_px_per_um": "fast"},
    ):
        with pytest.raises(ValueError):
            EdgeCalibration.from_dict(bad)


def test_load_calibration_never_raises():
    old = {
        "x0": 320,
        "y0": 240,
        "ux": 0.6,
        "uy": 0.8,
        "slope_px_per_um": 10,
        "p0_um": 50,
    }
    for junk in (old, {}, None, "garbage", 5, [1, 2], {"kind": "edge"}):
        cal, msg = load_calibration(junk)
        assert cal is None and msg
    assert load_calibration(old)[1] == "old spot calibration ignored; recalibrate"
    assert load_calibration({"kind": "edge", "box": "no"})[0] is None


def test_rezero_keeps_slope():
    cal = _fit(*_sweep(slope=-4.0))
    new = rezero(cal, e0_px=280.5, p0_um=51.25)
    assert new.slope_px_per_um == cal.slope_px_per_um
    assert (new.e0_px, new.p0_um) == (280.5, 51.25)
    assert (cal.e0_px, cal.p0_um) != (280.5, 51.25)  # the original is untouched
    assert new.sweep == cal.sweep and new.linear_range_um == cal.linear_range_um


# ---------------------------------------------------------------------------
# Error sign convention and per-frame checks
# ---------------------------------------------------------------------------


def _cal(slope=-4.0, e0=272.0, p0=50.0, **kw):
    return EdgeCalibration(
        box=BOX_LEFT,
        axis="x",
        polarity=1,
        slope_px_per_um=slope,
        e0_px=e0,
        p0_um=p0,
        linear_range_um=1.0,
        **kw,
    )


def _fix(position, **kw):
    base = {
        "position": position,
        "x": position,
        "y": 240.0,
        "axis": "x",
        "polarity": 1,
        "contrast": 90.0,
        "low": 1.0,
        "high": 91.0,
        "angle_deg": 0.0,
        "residual_px": 0.1,
        "valid_fraction": 1.0,
        "saturated_frac": 0.0,
        "peak": 91.0,
        "ok": True,
    }
    return EdgeFix(**(base | kw))


@pytest.mark.parametrize("slope", [-4.0, 4.0])
def test_edge_error_sign_convention(slope):
    cal = _cal(slope=slope)

    def edge_at(p):  # the plant: the edge moves with the signed slope
        return cal.e0_px + slope * (p - cal.p0_um)

    for offset in (0.0, 0.3, -0.5):
        # At P = p0 + offset the error is zero.
        err0 = edge_error(_fix(edge_at(50.0 + offset)), cal, offset)
        assert err0 == pytest.approx(0.0, abs=1e-12)
        for p in (49.2, 50.0, 50.9):
            err = edge_error(_fix(edge_at(p)), cal, offset)
            # moving P by -error lands on p0 + offset
            assert p - err == pytest.approx(50.0 + offset)
    assert edge_error(_fix(272.0 + slope * 0.4), cal) == pytest.approx(0.4)


def test_check_fix():
    cal = _cal(angle_deg=1.0)
    params = SpotLoopParams()
    assert check_fix(_fix(272.0, angle_deg=2.0), cal, params) == ""
    assert "rotated" in check_fix(_fix(272.0, angle_deg=7.0), cal, params)
    assert "rotated" in check_fix(_fix(272.0, angle_deg=-5.0), cal, params)
    assert "straight" in check_fix(_fix(272.0, residual_px=3.5), cal, params)
    ok_wide = SpotLoopParams(max_angle_change_deg=10.0, max_edge_residual_px=5.0)
    assert check_fix(_fix(272.0, angle_deg=7.0, residual_px=3.5), cal, ok_wide) == ""
    assert check_fix(_fix(272.0, ok=False, reason="saturated"), cal, params) == (
        "saturated"
    )
    assert check_fix(_fix(272.0, ok=False), cal, params) == "edge lost"


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
    assert ctrl.update(+0.3, 0.0) < 50.0  # edge says +, move P down
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
    assert "[0, inf]" in near.fault and "start P 0.20 um" in near.fault
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


def test_negative_start_p_is_not_a_range_fault():
    # Regression: the rig's piezo P is about -45.5 um; the old 0..150 default
    # faulted the first correction with "outside piezo range".
    p = SpotLoopParams(deadband_um=0.0, kp=1.0, ki=0.0)
    assert p.p_min_um is None and p.p_max_um is None
    ctrl = SpotFocusController(p, -45.5)
    assert ctrl.update(+0.02, 0.0) == pytest.approx(-45.52)
    assert ctrl.fault is None and ctrl.n_moves == 1


def test_excursion_fault_names_the_start_p():
    p = SpotLoopParams(max_excursion_um=1.0, max_step_um=0.5, ki=0.0)
    ctrl = SpotFocusController(p, -45.5)
    t = 0.0
    while ctrl.fault is None and t < 100:
        ctrl.update(-5.0, t)
        t += 1.0
    assert ctrl.fault is not None
    assert "target -44.00 um" in ctrl.fault  # the third step
    assert "more than 1 um from the start P -45.50 um" in ctrl.fault


def test_explicit_limits_fault_with_the_range_in_the_message():
    p = SpotLoopParams(p_min_um=-100.0, p_max_um=0.0, max_excursion_um=500.0)
    ctrl = SpotFocusController(p, -99.8)
    assert ctrl.update(+1.0, 0.0) is None  # -99.8 - 0.5 = -100.3 < -100
    assert ctrl.fault is not None
    assert "[-100" in ctrl.fault and "start P -99.80 um" in ctrl.fault
    # Only the set bound is checked.
    half = SpotFocusController(SpotLoopParams(p_max_um=-45.0), -45.5)
    assert half.update(-1.0, 0.0) == pytest.approx(-45.0)  # a +0.5 step
    assert half.update(-1.0, 1.0) is None
    assert "[-inf, -45]" in (half.fault or "")


def test_strikes_and_good_reset():
    ctrl = SpotFocusController(SpotLoopParams(strikes=3), 50.0)
    ctrl.bad("a")
    ctrl.bad("a")
    ctrl.good()
    ctrl.bad("a")
    ctrl.bad("a")
    assert ctrl.fault is None
    ctrl.bad("edge lost")
    assert ctrl.fault == "edge lost"
    assert ctrl.update(1.0, 0.0) is None


def test_params_round_trip():
    p = SpotLoopParams(kp=0.3, avg_frames=6, offset_um=0.25)
    q = SpotLoopParams.from_dict(p.to_dict() | {"junk": 1, "lateral_limit_px": 8})
    assert q == p
    d = SpotLoopParams()
    assert d.max_rate_hz == 2.0 and d.avg_frames == 4 and d.offset_um == 0.0
    assert (d.min_contrast, d.max_saturated_frac, d.min_valid_fraction) == (
        10.0,
        0.002,
        0.5,
    )
    assert (d.max_angle_change_deg, d.max_edge_residual_px) == (5.0, 3.0)
    assert not hasattr(d, "lateral_limit_px") and not hasattr(d, "min_amplitude")


def test_outside_linear_range_takes_a_full_signed_step():
    p = SpotLoopParams(max_step_um=0.5, max_rate_hz=1000.0)
    ctrl = SpotFocusController(p, 50.0, linear_range_um=1.0)
    assert ctrl.update(+2.0, 0.0) == pytest.approx(49.5)  # -sign(e) * max_step
    assert ctrl.update(-1.5, 1.0) == pytest.approx(50.0)
    # No integral was accumulated: the first in-range move is plain kp + ki*e.
    assert ctrl.update(0.2, 2.0) == pytest.approx(50.0 - (0.6 * 0.2 + 0.05 * 0.2))
    # Without a range there is no such branch (the step is only clamped).
    free = SpotFocusController(p, 50.0)
    assert free.update(+2.0, 0.0) == pytest.approx(49.5)
    assert free.update(+0.3, 1.0) == pytest.approx(49.5 - (0.6 * 0.3 + 0.05 * 0.3))


def test_integral_not_accumulated_while_clamped():
    p = SpotLoopParams(max_step_um=0.5, max_rate_hz=1000.0, ki=0.2)
    ctrl = SpotFocusController(p, 50.0)
    for k in range(5):
        ctrl.update(+3.0, float(k))  # always clamped
    # integral still empty: the small move is plain kp*e + ki*e
    t0 = ctrl.p_cmd_um
    assert ctrl.update(0.1, 10.0) == pytest.approx(t0 - (0.6 * 0.1 + 0.2 * 0.1))


def _simulate(ctrl, sign, *, offset=0.0, drift=0.0, dt=0.5, steps=120):
    """Closed loop on an ideal plant; returns the along_um history.

    ``along = offset + drift*t + (P_cmd - P_start)``: moving P up moves the edge
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


def _plant_loop(sign, *, start_off=0.8, slope=4.0, steps=40, max_excursion=3.0):
    """P -> synthetic image -> measure_edge -> edge_error -> controller."""
    p0 = 50.0
    x_ref = CX - HALF_W  # edge position at focus

    def measure(p):
        img = clean(CX + slope * (p - p0), edge_sigma=2.0 + abs(p - p0))
        return measure_edge(img, BOX_LEFT)

    ref = measure(p0)
    cal = _cal(slope=slope, e0=ref.position, p0=p0)
    assert ref.position == pytest.approx(x_ref, abs=0.05)
    ctrl = SpotFocusController(
        SpotLoopParams(max_excursion_um=max_excursion, avg_frames=1),
        p0 + start_off,
        linear_range_um=1.0,
    )
    history = [ctrl.p_cmd_um]
    for k in range(steps):
        fix = measure(ctrl.p_cmd_um)
        if not fix.ok:
            ctrl.bad(fix.reason)
            break
        ctrl.update(sign * edge_error(fix, cal), k * 0.5)
        history.append(ctrl.p_cmd_um)
        if ctrl.fault:
            break
    return ctrl, np.asarray(history) - p0


def test_closed_loop_on_synthetic_images_converges_without_search():
    ctrl, hist = _plant_loop(+1)
    assert ctrl.fault is None
    assert abs(hist[-1]) < 0.03 + 0.01
    assert ctrl.n_moves <= 8
    assert hist[0] == pytest.approx(0.8)
    assert np.all(np.diff(np.abs(hist)) <= 1e-9)  # straight in, no overshoot
    # and from the other side of focus
    ctrl2, hist2 = _plant_loop(+1, start_off=-0.8)
    assert abs(hist2[-1]) < 0.04


def test_closed_loop_with_the_sign_flipped_diverges():
    ctrl, hist = _plant_loop(-1)
    assert ctrl.fault is not None or abs(hist[-1]) > abs(hist[0]) + 1.0
    assert abs(hist[-1]) > abs(hist[0]) + 1.0


# ---------------------------------------------------------------------------
# FocusCalibration: one or two edges
# ---------------------------------------------------------------------------

RIG_EDGE = {
    "kind": "edge",
    "box": [762, 590, 176, 114],
    "axis": "y",
    "polarity": 1,
    "slope_px_per_um": 1.919,
    "e0_px": 641.37,
    "p0_um": -45.497,
    "r2": 0.993,
    "residual_px": 0.41,
    "linear_range_um": 1.0,
    "noise_px": 0.033,
    "angle_deg": 0.3,
    "contrast": 62.0,
    "n_points": 9,
    "sweep": [[-46.497 + 0.25 * k, 639.4 + 0.48 * k] for k in range(9)],
    "warnings": ["slope 1.92 px/um is small"],
    "record_path": "C:/data/spot_calibration_20261028_131149.npz",
    "created": "2026-10-28T13:11:49",
}


def _edge(box, axis, slope, e0, *, p0=50.0, noise_px=0.05, **kw):
    return EdgeCalibration(
        box=box,
        axis=axis,
        polarity=1,
        slope_px_per_um=slope,
        e0_px=e0,
        p0_um=p0,
        noise_px=noise_px,
        **kw,
    )


def _two(**kw):
    """Left edge (x, 2.1 px/um) and top edge (y, 1.9 px/um) of the synthetic patch."""
    left = _edge(BOX_LEFT, "x", 2.1, CX - HALF_W, **kw)
    top = _edge(BOX_TOP, "y", 1.9, CY - HALF_H, **kw)
    return FocusCalibration(edges=[left, top], p0_um=50.0, created="t")


def test_focus_calibration_round_trip_and_json():
    cal = _two()
    cal.warnings.append("careful")
    cal.record_path = "r.npz"
    d = json.loads(json.dumps(cal.to_dict()))
    assert d["kind"] == "edges" and len(d["edges"]) == 2
    assert d["edges"][0] == cal.edges[0].to_dict()
    again = FocusCalibration.from_dict(d | {"junk": 1})
    assert again == cal
    assert isinstance(again.edges[0].box, tuple)
    loaded, msg = load_calibration(d)
    assert loaded == cal and msg == ""
    one = FocusCalibration(edges=[cal.edges[0]], p0_um=50.0)
    assert load_calibration(json.loads(json.dumps(one.to_dict())))[0] == one


def test_focus_calibration_boxes_and_linear_range():
    cal = _two()
    assert cal.boxes == [BOX_LEFT, BOX_TOP]
    assert cal.linear_range_um == 1.0
    cal.edges[1].linear_range_um = 0.6
    assert cal.linear_range_um == 0.6
    assert FocusCalibration(edges=[cal.edges[0]], p0_um=50.0).linear_range_um == 1.0


def test_single_edge_dict_is_wrapped():
    cal, msg = load_calibration(json.loads(json.dumps(RIG_EDGE)))
    assert msg == "" and isinstance(cal, FocusCalibration)
    assert len(cal.edges) == 1 and cal.kind == "edges"
    e = cal.edges[0]
    assert e == EdgeCalibration.from_dict(RIG_EDGE)
    assert (e.axis, e.slope_px_per_um, e.box) == ("y", 1.919, (762, 590, 176, 114))
    assert cal.p0_um == -45.497
    assert cal.created == "2026-10-28T13:11:49"
    assert cal.record_path == RIG_EDGE["record_path"]
    assert cal.warnings == RIG_EDGE["warnings"]
    assert cal.boxes == [(762, 590, 176, 114)]
    # saving the wrapped calibration gives the new format, which loads again
    assert load_calibration(cal.to_dict())[0] == cal


def test_load_calibration_legacy_and_garbage():
    old = {"x0": 320, "y0": 240, "ux": 0.6, "uy": 0.8, "slope_px_per_um": 10}
    good = _two().to_dict()
    bad_edge = RIG_EDGE | {"box": [1, 2, 3]}
    junk = (
        old,
        "garbage",
        5,
        [1, 2],
        {"kind": "edges"},
        {"kind": "edges", "edges": []},
        {"kind": "edges", "edges": "no", "p0_um": 1.0},
        {"kind": "edges", "edges": [good["edges"][0]] * 3, "p0_um": 50.0},
        {"kind": "edges", "edges": [bad_edge], "p0_um": 1.0},
        good | {"p0_um": "far"},
        good | {"p0_um": float("nan")},
        {k: v for k, v in good.items() if k != "p0_um"},
        {"kind": "spot", "edges": good["edges"], "p0_um": 50.0},
        bad_edge,
        RIG_EDGE | {"slope_px_per_um": 0.0},
        {"kind": "edge"},
    )
    for item in junk:
        cal, msg = load_calibration(item)
        assert cal is None and msg == "old spot calibration ignored; recalibrate", item
    for empty in (None, {}):
        cal, msg = load_calibration(empty)
        assert cal is None and msg == "no spot calibration yet; press Calibrate"
    with pytest.raises(ValueError):
        FocusCalibration.from_dict(old)


def test_focus_calibration_rezeroed():
    cal = _two()
    cal.warnings.append("w")
    new = cal.rezeroed([281.5, 190.25], 51.5)
    assert new.p0_um == 51.5
    assert [e.e0_px for e in new.edges] == [281.5, 190.25]
    assert [e.p0_um for e in new.edges] == [51.5, 51.5]
    assert [e.slope_px_per_um for e in new.edges] == [2.1, 1.9]
    assert new.edges[0].sweep == cal.edges[0].sweep
    assert (new.created, new.warnings) == (cal.created, ["w"])
    # the original is untouched
    assert cal.p0_um == 50.0 and cal.edges[0].e0_px == CX - HALF_W
    with pytest.raises(ValueError):
        cal.rezeroed([1.0], 50.0)


def test_edge_noise_um():
    assert edge_noise_um(_edge(BOX_LEFT, "x", 2.0, 0.0, noise_px=0.1)) == 0.05
    # the line residual dominates when it is larger than the static noise
    e = _edge(BOX_LEFT, "x", -4.0, 0.0, noise_px=0.05, residual_px=0.4)
    assert edge_noise_um(e) == pytest.approx(0.1)
    # and a floor of 0.02 px keeps a perfect-looking edge from getting infinite weight
    e = _edge(BOX_LEFT, "x", 1.0, 0.0, noise_px=0.0, residual_px=0.0)
    assert edge_noise_um(e) == pytest.approx(0.02)


# ---------------------------------------------------------------------------
# fuse_edges
# ---------------------------------------------------------------------------


def _fuse_cal():
    """Edge 1: 2 px/um, noise 0.05 um. Edge 2: 4 px/um, noise 0.1 um."""
    e1 = _edge(BOX_LEFT, "x", 2.0, 272.0, noise_px=0.1)
    e2 = _edge(BOX_TOP, "y", 4.0, 194.0, noise_px=0.4)
    return FocusCalibration(edges=[e1, e2], p0_um=50.0)


def _fixes(a1, a2, cal=None):
    """Fixes whose per-edge errors are exactly a1 and a2 um."""
    cal = cal or _fuse_cal()
    e1, e2 = cal.edges
    return [
        _fix(e1.e0_px + e1.slope_px_per_um * a1),
        _fix(e2.e0_px + e2.slope_px_per_um * a2, axis="y"),
    ]


def test_fuse_two_consistent_edges_is_the_inverse_variance_mean():
    cal, params = _fuse_cal(), SpotLoopParams()
    out = fuse_edges(_fixes(0.30, 0.40), cal, params)
    assert isinstance(out, FusedError)
    # weights 1/0.05**2 = 400 and 1/0.1**2 = 100
    assert out.along_um == pytest.approx((400 * 0.30 + 100 * 0.40) / 500)
    assert out.sigma_um == pytest.approx(1 / math.sqrt(500))
    assert out.sigma_um < min(0.05, 0.1)
    assert out.used == (True, True)
    assert out.per_edge_um == pytest.approx((0.30, 0.40))
    assert out.disagreement_um == pytest.approx(0.10)
    assert out.reason == "" and out.note == ""


def test_fuse_equal_edges_average_and_sigma_drops_by_sqrt2():
    e1 = _edge(BOX_LEFT, "x", 2.0, 272.0, noise_px=0.1)
    e2 = _edge(BOX_TOP, "y", 2.0, 194.0, noise_px=0.1)
    cal = FocusCalibration(edges=[e1, e2], p0_um=50.0)
    out = fuse_edges(_fixes(-0.2, -0.1, cal), cal, SpotLoopParams())
    assert out.along_um == pytest.approx(-0.15)
    assert out.sigma_um == pytest.approx(0.05 / math.sqrt(2))


def test_fuse_subtracts_the_offset_once():
    cal = _fuse_cal()
    base = fuse_edges(_fixes(0.30, 0.40), cal, SpotLoopParams())
    out = fuse_edges(_fixes(0.30, 0.40), cal, SpotLoopParams(offset_um=0.1))
    assert out.along_um == pytest.approx(base.along_um - 0.1)
    assert out.per_edge_um == base.per_edge_um  # per-edge values are raw
    # a plant sitting at P = p0 + offset reads edges moved by slope * offset
    at_offset = fuse_edges(_fixes(0.1, 0.1), cal, SpotLoopParams(offset_um=0.1))
    assert at_offset.along_um == pytest.approx(0.0, abs=1e-12)


def test_fuse_disagreement_is_a_bad_measurement():
    cal = _fuse_cal()
    params = SpotLoopParams(max_edge_disagreement_um=0.3)
    out = fuse_edges(_fixes(0.30, -0.30), cal, params)  # 0.6 > max(0.3, 0.447)
    assert out.along_um is None  # neither edge is within the hold band
    assert out.disagreement_um == pytest.approx(0.6)
    assert out.reason.startswith("edges disagree by 0.60 um")
    assert "lateral drift" in out.reason
    assert out.used == (True, True)
    assert out.held_edge is None
    # just under the limit still fuses
    assert fuse_edges(_fixes(0.0, 0.44), cal, params).reason == ""
    # the default limit (1.0 um) lets the same pair through
    assert fuse_edges(_fixes(0.30, -0.30), cal, SpotLoopParams()).reason == ""
    # noisy edges widen the limit: 4 * sqrt(0.5**2 + 0.5**2) = 2.83 um
    noisy = FocusCalibration(
        edges=[
            _edge(BOX_LEFT, "x", 2.0, 272.0, noise_px=1.0),
            _edge(BOX_TOP, "y", 2.0, 194.0, noise_px=1.0),
        ],
        p0_um=50.0,
    )
    out = fuse_edges(_fixes(1.2, -1.2, noisy), noisy, SpotLoopParams())
    assert out.reason == "" and out.along_um == pytest.approx(0.0, abs=1e-12)
    assert out.held_edge is None


def test_fuse_default_limit_is_about_one_micron():
    cal, params = _fuse_cal(), SpotLoopParams()
    assert fuse_edges(_fixes(0.40, -0.55), cal, params).reason == ""  # 0.95
    out = fuse_edges(_fixes(0.60, -0.60), cal, params)  # 1.2, neither near
    assert out.along_um is None and out.reason.startswith("edges disagree by 1.20")


def test_fuse_disagreement_holds_on_edge_1_when_it_is_at_target():
    cal, params = _fuse_cal(), SpotLoopParams()
    out = fuse_edges(_fixes(0.04, 1.50), cal, params)
    assert out.reason == ""
    assert out.held_edge == 0
    assert out.along_um == pytest.approx(0.04)
    assert out.sigma_um == pytest.approx(edge_noise_um(cal.edges[0]))
    assert out.used == (True, True)
    assert out.disagreement_um == pytest.approx(1.46)
    assert out.note == (
        "edges disagree by 1.46 um; holding on edge 1 (0.04 um from target)"
    )


def test_fuse_disagreement_holds_on_edge_2_when_it_is_at_target():
    cal, params = _fuse_cal(), SpotLoopParams()
    out = fuse_edges(_fixes(-1.20, -0.10), cal, params)
    assert out.reason == ""
    assert out.held_edge == 1
    assert out.along_um == pytest.approx(-0.10)
    assert out.sigma_um == pytest.approx(edge_noise_um(cal.edges[1]))
    assert out.used == (True, True)
    assert "holding on edge 2 (0.10 um from target)" in out.note


def test_fuse_hold_band_edges_and_param():
    cal = _fuse_cal()
    # just outside the default 0.15 um band: bad
    assert fuse_edges(_fixes(0.20, 1.5), cal, SpotLoopParams()).along_um is None
    # a wider band holds it
    wide = SpotLoopParams(hold_band_um=0.25)
    assert fuse_edges(_fixes(0.20, 1.5), cal, wide).held_edge == 0
    # a zero band never holds
    none = SpotLoopParams(hold_band_um=0.0)
    assert fuse_edges(_fixes(0.04, 1.5), cal, none).along_um is None


def test_fuse_hold_respects_the_offset():
    cal = _fuse_cal()
    fixes = _fixes(0.50, 2.00)
    # target 0.5 um: edge 1 sits on it, so hold on edge 1 (error ~0)
    out = fuse_edges(fixes, cal, SpotLoopParams(offset_um=0.5))
    assert out.held_edge == 0
    assert out.along_um == pytest.approx(0.0, abs=1e-12)
    # target 0: edge 1 is 0.5 away, edge 2 further: bad
    out = fuse_edges(fixes, cal, SpotLoopParams())
    assert out.held_edge is None and out.along_um is None
    assert out.reason.startswith("edges disagree")
    # edge 2 close to a target of 2.0
    out = fuse_edges(fixes, cal, SpotLoopParams(offset_um=2.0))
    assert out.held_edge == 1
    assert out.along_um == pytest.approx(0.0, abs=1e-12)


def _run_controller(fixes, params, frames=60):
    ctrl = SpotFocusController(params, 50.0)
    cal = _fuse_cal()
    k = 0
    for k in range(frames):
        out = fuse_edges(fixes, cal, params)
        if out.reason:
            ctrl.bad(out.reason)
        else:
            ctrl.good()
            assert out.along_um is not None
            ctrl.update(out.along_um, k * 0.5)
        if ctrl.fault:
            break
    return ctrl, k


def test_persistent_disagreement_with_neither_edge_near_faults_after_strikes():
    params = SpotLoopParams(strikes=5, deadband_um=0.1)
    ctrl, k = _run_controller(_fixes(0.60, -0.60), params)
    assert ctrl.fault is not None and "edges disagree" in ctrl.fault
    assert k == 4  # the 5th consecutive bad frame
    assert ctrl.n_moves == 0


def test_persistent_disagreement_with_one_edge_near_never_faults():
    params = SpotLoopParams(strikes=5, deadband_um=0.1)
    ctrl, _ = _run_controller(_fixes(0.04, 1.50), params, frames=200)
    assert ctrl.fault is None
    assert ctrl.n_moves == 0  # held inside the dead band


def test_a_held_frame_resets_the_strike_count():
    params = SpotLoopParams(strikes=3, deadband_um=0.1)
    ctrl = SpotFocusController(params, 50.0)
    cal = _fuse_cal()
    for _ in range(2):
        ctrl.bad(fuse_edges(_fixes(0.60, -0.60), cal, params).reason)
    held = fuse_edges(_fixes(0.04, 1.50), cal, params)
    assert held.reason == "" and held.along_um is not None
    ctrl.good()
    ctrl.update(held.along_um, 0.0)
    for _ in range(2):
        ctrl.bad(fuse_edges(_fixes(0.60, -0.60), cal, params).reason)
    assert ctrl.fault is None


def test_fuse_one_bad_edge_uses_the_other_with_a_note():
    cal, params = _fuse_cal(), SpotLoopParams(offset_um=0.05)
    f1, f2 = _fixes(0.30, 0.40)
    lost = _fix(f2.position, ok=False, reason="edge lost (contrast 1.0 < 10)")
    out = fuse_edges([f1, lost], cal, params)
    assert out.reason == ""
    assert out.along_um == pytest.approx(0.30 - 0.05)
    assert out.sigma_um == pytest.approx(edge_noise_um(cal.edges[0]))
    assert out.used == (True, False)
    assert out.per_edge_um[1] is None and out.per_edge_um[0] == pytest.approx(0.30)
    assert out.disagreement_um is None
    assert out.note == "edge 2 lost (edge lost (contrast 1.0 < 10)); using edge 1"
    # edge 1 lost; a rotated edge counts as lost too
    rotated = _fix(f1.position, angle_deg=20.0)
    out = fuse_edges([rotated, f2], cal, params)
    assert out.reason == ""
    assert out.along_um == pytest.approx(0.40 - 0.05)
    assert out.sigma_um == pytest.approx(0.1)
    assert out.used == (False, True)
    assert out.note == "edge 1 lost (edge rotated; the feature changed); using edge 2"
    assert fuse_edges([f1, _fix(f2.position, residual_px=5.0)], cal, params).used == (
        True,
        False,
    )


def test_fuse_both_bad_is_a_strike_with_the_reason_of_edge_1():
    cal, params = _fuse_cal(), SpotLoopParams()
    f1 = _fix(272.0, ok=False, reason="saturated")
    f2 = _fix(194.0, ok=False, reason="edge lost")
    out = fuse_edges([f1, f2], cal, params)
    assert out.along_um is None and out.reason == "saturated"
    assert out.used == (False, False) and out.per_edge_um == (None, None)
    assert math.isnan(out.sigma_um) and out.disagreement_um is None
    assert out.note == ""


def test_fuse_single_edge_calibration_matches_the_old_path():
    e = _edge(BOX_LEFT, "x", -4.0, 272.0, noise_px=0.08, residual_px=0.2)
    cal = FocusCalibration(edges=[e], p0_um=50.0)
    params = SpotLoopParams(offset_um=0.2)
    fix = _fix(272.0 - 4.0 * 0.35)
    out = fuse_edges([fix], cal, params)
    assert out.along_um == pytest.approx(edge_error(fix, e, params.offset_um))
    assert out.along_um == pytest.approx(0.15)
    assert out.sigma_um == pytest.approx(0.2 / 4.0)
    assert out.used == (True,) and out.note == "" and out.reason == ""
    assert out.disagreement_um is None
    bad = _fix(272.0, ok=False, reason="edge left the box")
    out = fuse_edges([bad], cal, params)
    assert (out.along_um, out.reason, out.note) == (None, "edge left the box", "")
    assert out.used == (False,)


def test_fuse_edges_needs_one_fix_per_edge():
    cal = _fuse_cal()
    with pytest.raises(ValueError):
        fuse_edges(_fixes(0, 0)[:1], cal, SpotLoopParams())
    with pytest.raises(ValueError):
        fuse_edges(_fixes(0, 0) * 2, cal, SpotLoopParams())


def test_params_have_the_disagreement_limit():
    assert SpotLoopParams().max_edge_disagreement_um == 1.0
    assert SpotLoopParams().hold_band_um == 0.15
    p = SpotLoopParams(max_edge_disagreement_um=0.5, hold_band_um=0.2)
    back = SpotLoopParams.from_dict(p.to_dict())
    assert back.max_edge_disagreement_um == 0.5
    assert back.hold_band_um == 0.2
    # an old dict without the new key loads with the default
    old = {k: v for k, v in p.to_dict().items() if k != "hold_band_um"}
    assert SpotLoopParams.from_dict(old).hold_band_um == 0.15


# ---------------------------------------------------------------------------
# Two edges on synthetic images
# ---------------------------------------------------------------------------

SX, SY = 2.1, 1.9  # image motion of the patch in px per um of P


def _patch_fixes(p, p0=50.0, dx=0.0, dy=0.0):
    """Both edge fixes of a patch that follows P diagonally (plus a lateral shift)."""
    q = p - p0
    img = clean(CX + SX * q + dx, CY + SY * q + dy, edge_sigma=2.0 + abs(q))
    return [measure_edge(img, BOX_LEFT), measure_edge(img, BOX_TOP)]


def _synthetic_cal():
    ref = _patch_fixes(50.0)
    assert all(f.ok for f in ref)
    left = _edge(BOX_LEFT, "x", SX, ref[0].position, noise_px=0.05)
    top = _edge(BOX_TOP, "y", SY, ref[1].position, noise_px=0.05)
    return FocusCalibration(edges=[left, top], p0_um=50.0)


@pytest.mark.parametrize("q", [-0.8, -0.3, 0.0, 0.45, 0.9])
def test_diagonal_motion_gives_agreeing_estimates(q):
    cal, params = _synthetic_cal(), SpotLoopParams()
    out = fuse_edges(_patch_fixes(50.0 + q), cal, params)
    assert out.reason == "" and out.note == ""
    assert out.per_edge_um[0] == pytest.approx(q, abs=0.03)
    assert out.per_edge_um[1] == pytest.approx(q, abs=0.03)
    assert out.along_um == pytest.approx(q, abs=0.03)
    assert out.disagreement_um < 0.05


def test_lateral_move_gives_a_disagreement():
    cal, params = _synthetic_cal(), SpotLoopParams()
    # P is 0.5 um off, so neither edge is at the target once x shifts too
    out = fuse_edges(_patch_fixes(50.5, dx=3.0), cal, params)  # only x moves
    assert out.per_edge_um[0] == pytest.approx(0.5 + 3.0 / SX, abs=0.03)
    assert out.per_edge_um[1] == pytest.approx(0.5, abs=0.03)
    assert out.along_um is None
    assert out.reason.startswith("edges disagree by")
    # a vertical shift disagrees in the other direction
    out = fuse_edges(_patch_fixes(50.5, dy=-3.0), cal, params)
    assert out.along_um is None and out.reason
    # a 1 px shift stays under the default limit and still fuses
    assert fuse_edges(_patch_fixes(50.0, dx=1.0), cal, params).reason == ""


def test_lateral_move_holds_on_the_edge_that_is_at_target():
    cal, params = _synthetic_cal(), SpotLoopParams()
    out = fuse_edges(_patch_fixes(50.0, dx=3.0), cal, params)  # only x moves
    assert out.reason == "" and out.held_edge == 1  # the y edge did not move
    assert out.along_um == pytest.approx(0.0, abs=0.03)
    out = fuse_edges(_patch_fixes(50.0, dy=-3.0), cal, params)
    assert out.reason == "" and out.held_edge == 0


def _two_edge_loop(*, start_off=0.8, steps=40, dx=0.0):
    cal, p0 = _synthetic_cal(), 50.0
    ctrl = SpotFocusController(
        SpotLoopParams(max_excursion_um=3.0, avg_frames=1),
        p0 + start_off,
        linear_range_um=cal.linear_range_um,
    )
    params = ctrl.params
    history = [ctrl.p_cmd_um]
    for k in range(steps):
        out = fuse_edges(_patch_fixes(ctrl.p_cmd_um, dx=dx), cal, params)
        if out.reason:
            ctrl.bad(out.reason)
        else:
            ctrl.good()
            assert out.along_um is not None
            ctrl.update(out.along_um, k * 0.5)
        history.append(ctrl.p_cmd_um)
        if ctrl.fault:
            break
    return ctrl, np.asarray(history) - p0


def test_two_edge_closed_loop_converges_without_search():
    ctrl, hist = _two_edge_loop()
    assert ctrl.fault is None
    assert hist[0] == pytest.approx(0.8)
    assert abs(hist[-1]) < 0.04
    assert ctrl.n_moves <= 8
    assert np.all(np.diff(np.abs(hist)) <= 1e-9)  # straight in, no overshoot
    ctrl2, hist2 = _two_edge_loop(start_off=-0.8)
    assert ctrl2.fault is None and abs(hist2[-1]) < 0.04


def test_two_edge_loop_stops_on_a_lateral_shift():
    """A lateral shift is a strike, not a move: the piezo is held, then faults."""
    ctrl, hist = _two_edge_loop(start_off=0.5, dx=3.0)
    assert ctrl.n_moves == 0
    assert ctrl.fault is not None and "edges disagree" in ctrl.fault
    assert np.all(hist == 0.5)


def test_two_edge_loop_holds_through_a_lateral_shift_at_target():
    """The y edge stays at target, so the loop holds on it and never faults."""
    ctrl, hist = _two_edge_loop(start_off=0.0, dx=3.0, steps=60)
    assert ctrl.fault is None
    assert ctrl.n_moves == 0
    assert np.all(np.abs(hist) < 0.04)


# ---------------------------------------------------------------------------
# FakeSpotSource
# ---------------------------------------------------------------------------

CAL_LIKE = {
    "x0": CX,
    "y0": CY,
    "ux": 1.0,
    "uy": 0.0,
    "slope_px_per_um": 4.0,
    "p0_um": 50.0,
}


def _fake(cal_like=None, **kw):
    return FakeSpotSource(cal_like or CAL_LIKE, fps=1000.0, **kw)


def test_fake_edge_follows_actuator():
    src = _fake(noise=1.0)
    src.open()
    try:
        first = src.latest(1.0)
        assert first is not None
        frame, stamp = first
        assert frame.dtype == np.uint16
        assert frame.shape == SHAPE
        base = measure_edge(frame, BOX_LEFT)
        assert base.ok, base.reason
        assert base.position == pytest.approx(CX - HALF_W, abs=1.0)
        for dp in (-1.0, 0.5, 1.0, -0.4):
            src.set_actuator_um(50.0 + dp)
            fix = measure_edge(src.latest(1.0)[0], BOX_LEFT)
            assert fix.ok, (dp, fix.reason)
            # the edge moves by slope x dP along the box normal (x)
            assert fix.position - base.position == pytest.approx(4.0 * dp, abs=0.25)
        assert src.latest(1.0)[1] > stamp
    finally:
        src.close()


def test_fake_diagonal_motion_and_blur():
    cal = CAL_LIKE | {"ux": 0.6, "uy": 0.8, "slope_px_per_um": 5.0}
    src = _fake(cal, noise=0.0, ghost=0.0, tail=0.0)
    src.open()
    f0 = measure_edge(src.render(0.0), BOX_LEFT)
    top0 = measure_edge(src.render(0.0), BOX_TOP)
    src.set_actuator_um(50.5)
    f1 = measure_edge(src.render(0.0), BOX_LEFT)
    top1 = measure_edge(src.render(0.0), BOX_TOP)
    assert f1.position - f0.position == pytest.approx(0.6 * 5.0 * 0.5, abs=0.15)
    assert top1.position - top0.position == pytest.approx(0.8 * 5.0 * 0.5, abs=0.15)
    # the blur grows with |P - focus|: the edge widens, the contrast stays
    assert f1.contrast == pytest.approx(f0.contrast, rel=0.05)


def test_fake_drift_and_lateral_drift():
    src = _fake(noise=0.5, drift_um_per_s=0.0, lateral_px_per_s=0.0)
    src.open()
    assert src.spot_position(2.0) == pytest.approx((CX, CY))
    src.drift_um_per_s = 0.5
    assert src.spot_position(2.0) == pytest.approx((CX + 4.0 * 1.0, CY))
    a = measure_edge(src.render(0.0), BOX_LEFT).position
    b = measure_edge(src.render(2.0), BOX_LEFT).position
    assert b - a == pytest.approx(4.0, abs=0.25)
    src.drift_um_per_s = 0.0
    src.lateral_px_per_s = 2.0
    x, y = src.spot_position(3.0)
    assert (x, y) == pytest.approx((CX, CY + 6.0))  # perpendicular to u = (1, 0)
    c = measure_edge(src.render(3.0), BOX_TOP).position
    d = measure_edge(src.render(0.0), BOX_TOP).position
    assert c - d == pytest.approx(6.0, abs=0.25)


def test_fake_level_and_led_off():
    src = _fake(noise=0.5)
    src.open()
    ref = measure_edge(src.render(0.0), BOX_LEFT)
    src.set_level(2.0)
    bright = measure_edge(src.render(0.0), BOX_LEFT)
    assert bright.contrast == pytest.approx(2 * ref.contrast, rel=0.03)
    assert bright.position == pytest.approx(ref.position, abs=0.1)
    src.set_level(0.0)
    off = measure_edge(src.render(0.0), BOX_LEFT)
    assert not off.ok and off.reason.startswith("edge lost")
    src.set_level(1.0)
    src.set_exposure_ms(20.0)  # exposure scales brightness like a camera
    assert measure_edge(src.render(0.0), BOX_LEFT).contrast == pytest.approx(
        2 * ref.contrast, rel=0.03
    )


def test_fake_ghost_tail_and_hot_pixels():
    src = _fake(noise=0.0, hot_pixels=20, seed=4)
    frame = src.render(0.0)
    assert int((frame == int(0.7 * 65535)).sum()) >= 15  # static hot pixels
    assert np.array_equal(frame, src.render(0.0))  # and they stay put
    ghost_row = frame[364, 300:340]  # ghost body ~124 px below the centre
    assert ghost_row.mean() > 100 + 0.1 * 800 * 0.5
    assert measure_edge(frame, BOX_LEFT).ok


def test_fake_uint8():
    src = _fake(noise=0.4, bits=8, amplitude=800.0)
    frame = src.render(0.0)
    assert frame.dtype == np.uint8
    assert int(frame.max()) < 255  # the level is capped below full scale
    fix = measure_edge(frame, BOX_LEFT)
    assert fix.ok, fix.reason
    assert 40 < fix.contrast < 110


def test_fake_render_speed():
    src = _fake(noise=2.0)
    src.render(0.0)
    times = []
    for k in range(30):
        src.set_actuator_um(50.0 + 0.01 * k)
        t0 = time.perf_counter()
        src.render(0.0)
        times.append(time.perf_counter() - t0)
    median_ms = 1e3 * float(np.median(times))
    assert median_ms < 15.0
    print(f"FakeSpotSource.render 640x480: {median_ms:.2f} ms median")


def test_fake_roi_info_and_pacing():
    src = _fake(noise=0.0)
    assert src.info()["kind"] == "fake"
    src.open()
    src.set_roi(200, 100, 300, 250)
    frame = src.latest(1.0)[0]
    assert frame.shape == (250, 300)
    assert src.info()["roi"] == (200, 100, 300, 250)
    # edges keep their sensor coordinates inside the ROI
    fix = measure_edge(frame, (232, 210, 80, 60), roi_offset=(200, 100))
    assert fix.ok and fix.position == pytest.approx(CX - HALF_W, abs=1.0)
    src.set_exposure_ms(5)
    src.set_gain(150)
    assert src.info()["gain"] == 150
    slow = FakeSpotSource(CAL_LIKE, fps=2.0)
    slow.open()
    slow.latest(1.0)  # first frame is immediate
    assert slow.latest(0.05) is None  # next one is 0.5 s away


def test_fake_is_seeded_and_picklable():
    import pickle

    spec = {"kind": "fake", "cal_like": CAL_LIKE, "seed": 3, "fps": 1000.0}
    pickle.loads(pickle.dumps(spec))
    a, b = make_source(spec), make_source(spec)
    a.open()
    b.open()
    assert np.array_equal(a.render(0.0), b.render(0.0))
    other = make_source(spec | {"seed": 4})
    other.open()
    assert not np.array_equal(a.render(0.0), other.render(0.0))
    # the whole source, rendered state included, survives pickling
    c = pickle.loads(pickle.dumps(make_source(spec | {"hot_pixels": 5})))
    assert c.render(0.0).shape == SHAPE


def test_fake_accepts_the_old_constructor_keys():
    src = FakeSpotSource(
        CAL_LIKE | {"junk_key_ignored": 1},
        drift_um_per_s=0.1,
        noise=1.0,
        amplitude=500.0,
        sigma_px=6.0,
        shape=(480, 640),
        lateral_px_per_s=0.5,
        seed=1,
        fps=50.0,
        bits=16,
    )
    src.open()
    assert src.render(1.0).shape == SHAPE


def test_make_source_and_missing_sdk(monkeypatch, tmp_path):
    assert isinstance(make_source({"kind": "toupcam", "serial": "x"}), ToupcamSource)
    assert isinstance(
        make_source({"kind": "fake", "cal_like": CAL_LIKE}), FakeSpotSource
    )
    with pytest.raises(ValueError):
        make_source({"kind": "nope"})
    monkeypatch.delenv("TOUPCAM_SDK_DIR", raising=False)
    with pytest.raises(RuntimeError, match="TOUPCAM_SDK_DIR"):
        ToupcamSource().open()
    with pytest.raises(RuntimeError, match="toupcam.py"):
        ToupcamSource(sdk_dir=str(tmp_path)).open()


# ---------------------------------------------------------------------------
# Selectable edge detector: gaussderiv (default) and halfmax (the old one)
# ---------------------------------------------------------------------------
#
# The accuracy tests above run with the default detector (gaussderiv); the
# halfmax ones are pinned here, so both are covered.


def _phi(z):
    return 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))


def _asym_edge(e=100.0, shape=(200, 400)):
    """Soft, asymmetric edge like the rig's left edge: a ramp to a grey shoulder
    (75 % of the level) and a second, slower rise 22 px further to the plateau."""
    x = np.arange(shape[1], dtype=np.float64)
    f = 2000.0 * (0.75 * _phi((x - e) / 5.0) + 0.25 * _phi((x - e - 22.0) / 9.0))
    return np.tile(f, (shape[0], 1))


def _asym_box(width, e=100.0):
    bx = int(e) - width // 2
    return (bx - bx % 2, 60, width, 60)


def test_detector_constants():
    assert EDGE_DETECTORS == ("gaussderiv", "halfmax")
    assert DEFAULT_DETECTOR == EDGE_DETECTORS[0] == "gaussderiv"
    assert set(EDGE_DETECTOR_LABELS) == set(EDGE_DETECTORS)


def test_default_detector_is_gaussderiv_and_the_fix_names_it():
    img = clean(CX + 0.3)
    fix = measure_edge(img, BOX_LEFT)
    assert (fix.detector, fix.sigma_px) == ("gaussderiv", 6.0)
    assert fix.position == measure_edge(img, BOX_LEFT, detector="gaussderiv").position
    assert measure_edge(img, BOX_LEFT, detector="halfmax").detector == "halfmax"
    # a failed fix still names the detector
    off = measure_edge(img, (0, 0, 20, 20), roi_offset=(100, 100), detector="halfmax")
    assert not off.ok and off.detector == "halfmax"
    assert math.isnan(off.edge_width_px)


def test_unknown_detector_and_bad_sigma_raise():
    with pytest.raises(ValueError, match="detector"):
        measure_edge(clean(), BOX_LEFT, detector="sobel")
    with pytest.raises(ValueError, match="sigma_px"):
        measure_edge(clean(), BOX_LEFT, sigma_px=0.0)
    with pytest.raises(ValueError, match="sigma_px"):
        measure_edge(clean(), BOX_LEFT, sigma_px=math.nan)


@pytest.mark.parametrize("shift", [0.0, 0.25, 0.5, 0.9, -0.6])
def test_halfmax_pinned_subpixel_bias(shift):
    img = clean(CX + shift, CY + shift)
    truth = edge_positions(shift, shift)
    for box, key, axis, pol in [
        (BOX_LEFT, "left", "x", +1),
        (BOX_RIGHT, "right", "x", -1),
        (BOX_TOP, "top", "y", +1),
        (BOX_BOTTOM, "bottom", "y", -1),
    ]:
        fix = measure_edge(img, box, detector="halfmax")
        assert fix.ok, fix.reason
        assert (fix.axis, fix.polarity) == (axis, pol)
        assert fix.position == pytest.approx(truth[key], abs=0.05), key


def test_halfmax_is_unchanged_from_the_released_method():
    # Reference values taken from the code before the detector was selectable
    # (HEAD c2d34da): the asymmetric soft edge, whose half-max depends on the box.
    got = [
        measure_edge(_asym_edge(), _asym_box(w), detector="halfmax").position
        for w in (48, 64, 100, 200)
    ]
    assert got == pytest.approx([100.827, 101.494, 102.156, 102.194], abs=1e-3)


@pytest.mark.parametrize("edge_sigma", [0.3, 2.0, 8.0])
@pytest.mark.parametrize("shift", [0.0, 0.3, 0.7, -0.45])
def test_gaussderiv_locates_sharp_and_soft_steps(edge_sigma, shift):
    img = clean(CX + shift, CY + shift, edge_sigma=edge_sigma)
    truth = edge_positions(shift, shift)
    for box, key in [(BOX_LEFT, "left"), (BOX_RIGHT, "right"), (BOX_TOP, "top")]:
        fix = measure_edge(img, box, detector="gaussderiv")
        assert fix.ok, (key, fix.reason)
        assert fix.position == pytest.approx(truth[key], abs=0.1), key


def test_gaussderiv_sweep_is_monotonic_and_linear():
    shifts = np.linspace(-1.0, 1.0, 9)
    pos = np.array(
        [measure_edge(clean(CX + s, edge_sigma=4.0), BOX_LEFT).position for s in shifts]
    )
    assert np.all(np.diff(pos) > 0)
    slope, icpt = np.polyfit(shifts, pos, 1)
    assert slope == pytest.approx(1.0, abs=0.02)
    assert np.max(np.abs(pos - (slope * shifts + icpt))) < 0.02


def test_gaussderiv_is_less_sensitive_to_the_box_width_than_halfmax():
    pos = {
        det: [
            measure_edge(_asym_edge(), _asym_box(w), detector=det).position
            for w in (48, 64, 100, 200)
        ]
        for det in EDGE_DETECTORS
    }
    spread = {det: max(v) - min(v) for det, v in pos.items()}
    assert spread["halfmax"] > 1.0  # the levels sit on the ramp of a narrow box
    assert spread["gaussderiv"] < 0.1
    assert spread["gaussderiv"] < spread["halfmax"] / 10
    # and it follows a shift of the edge 1:1 in a narrow box
    a = measure_edge(_asym_edge(100.0), _asym_box(48), detector="gaussderiv")
    b = measure_edge(_asym_edge(100.4), _asym_box(48), detector="gaussderiv")
    assert b.position - a.position == pytest.approx(0.4, abs=0.03)


def test_gaussderiv_sigma_changes_the_filter_not_the_edge():
    img = clean(CX + 0.2, edge_sigma=3.0)
    pos = [measure_edge(img, BOX_LEFT, sigma_px=s).position for s in (3.0, 6.0, 9.0)]
    assert max(pos) - min(pos) < 0.1


def test_both_detectors_report_a_lost_edge_and_an_edge_at_the_border():
    dark = np.zeros(SHAPE)
    for det in EDGE_DETECTORS:
        fix = measure_edge(dark, BOX_LEFT, detector=det)
        assert not fix.ok and "edge lost" in fix.reason, det
        # an edge right at the box border is "edge left the box"
        fix = measure_edge(clean(), (264, 210, 80, 60), detector=det)
        assert fix.reason == "edge left the box", det


def test_edge_width_is_the_10_90_width_of_the_edge():
    # 10-90 % width of an erf edge is 2.563 sigma; the tilt is corrected first.
    for sigma in (2.0, 8.0):
        fix = measure_edge(clean(edge_sigma=sigma), BOX_LEFT)
        assert fix.edge_width_px == pytest.approx(2.563 * sigma, rel=0.15, abs=1.0)
    tilted = measure_edge(clean(angle_deg=8.0, edge_sigma=4.0), BOX_LEFT)
    flat = measure_edge(clean(edge_sigma=4.0), BOX_LEFT)
    assert tilted.edge_width_px == pytest.approx(flat.edge_width_px, rel=0.15)


def test_gaussderiv_is_cheap_per_box():
    img = clean()
    measure_edge(img, BOX_LEFT)  # warm the kernel cache
    t0 = time.perf_counter()
    for _ in range(50):
        measure_edge(img, BOX_LEFT)
    assert (time.perf_counter() - t0) / 50 < 0.02  # ~0.5 ms; generous under load


# -- a calibration carries its detector -------------------------------------


def test_edge_calibration_stores_and_round_trips_the_detector():
    p, e = _sweep(noise=0.02)
    cal = _fit(
        p,
        e,
        noise_px=0.02,
        detector="gaussderiv",
        detector_sigma_px=7.5,
        edge_width_px=18.5,
    )
    assert (cal.detector, cal.detector_sigma_px) == ("gaussderiv", 7.5)
    assert cal.edge_width_px == 18.5 and cal.box_normal_px == BOX_LEFT[2]
    d = cal.to_dict()
    assert d["detector"] == "gaussderiv" and d["detector_sigma_px"] == 7.5
    assert d["edge_width_px"] == 18.5 and d["box_normal_px"] == BOX_LEFT[2]
    again = EdgeCalibration.from_dict(json.loads(json.dumps(d)))
    assert again == cal and again.edge_width_px == 18.5
    # the normal of a y-axis edge is the box height
    top = _fit(p, e, box=BOX_TOP, axis="y")
    assert top.box_normal_px == BOX_TOP[3]


def test_old_calibration_without_the_new_keys_loads_as_halfmax():
    cal = _fit(*_sweep())
    old = cal.to_dict()
    for k in ("detector", "detector_sigma_px", "edge_width_px", "box_normal_px"):
        del old[k]
    loaded = EdgeCalibration.from_dict(old)
    assert loaded.detector == "halfmax" and loaded.detector_sigma_px == 6.0
    assert math.isnan(loaded.edge_width_px) and loaded.box_normal_px == 0
    assert loaded.slope_px_per_um == cal.slope_px_per_um
    # through the persisted forms: the two-level one and the old single-edge dict
    focus = FocusCalibration(edges=[cal], p0_um=50.0).to_dict()
    for e in focus["edges"]:
        for k in ("detector", "detector_sigma_px", "edge_width_px", "box_normal_px"):
            e.pop(k)
    got, msg = load_calibration(focus)
    assert msg == "" and got is not None and got.edges[0].detector == "halfmax"
    got, msg = load_calibration(old)
    assert msg == "" and got is not None and got.edges[0].detector == "halfmax"


def test_calibration_nan_width_is_json_clean_and_unknown_detector_is_rejected():
    cal = _fit(*_sweep())
    assert math.isnan(cal.edge_width_px)
    text = json.dumps(cal.to_dict(), allow_nan=False)  # no NaN literal
    assert math.isnan(EdgeCalibration.from_dict(json.loads(text)).edge_width_px)
    with pytest.raises(ValueError):
        EdgeCalibration.from_dict(cal.to_dict() | {"detector": "sobel"})
    assert load_calibration(cal.to_dict() | {"detector": "sobel"})[0] is None


def test_rezero_keeps_the_detector():
    cal = _fit(*_sweep(), detector="gaussderiv", detector_sigma_px=5.0)
    new = rezero(cal, e0_px=280.5, p0_um=51.25)
    assert (new.detector, new.detector_sigma_px) == ("gaussderiv", 5.0)
    focus = FocusCalibration(edges=[cal], p0_um=50.0).rezeroed([281.0], 51.0)
    assert (focus.edges[0].detector, focus.edges[0].detector_sigma_px) == (
        "gaussderiv",
        5.0,
    )


def test_detector_mismatch():
    hm = FocusCalibration(edges=[_edge(BOX_LEFT, "x", 2.0, 272.0)], p0_um=50.0)
    gd = FocusCalibration(
        edges=[_edge(BOX_LEFT, "x", 2.0, 272.0, detector="gaussderiv")], p0_um=50.0
    )
    assert detector_mismatch(hm, "halfmax") == ""
    assert detector_mismatch(hm, "halfmax", 9.0) == ""  # sigma is ignored by halfmax
    assert detector_mismatch(gd, "gaussderiv", 6.0) == ""
    assert detector_mismatch(hm, "gaussderiv") == (
        "calibrated with halfmax; the detector setting is gaussderiv: recalibrate"
    )
    assert detector_mismatch(gd, "halfmax") == (
        "calibrated with gaussderiv; the detector setting is halfmax: recalibrate"
    )
    why = detector_mismatch(gd, "gaussderiv", 8.0)
    assert "sigma 6 px" in why and "sigma 8 px" in why and "recalibrate" in why
    # two edges: one stale edge is enough
    mixed = FocusCalibration(
        edges=[
            _edge(BOX_LEFT, "x", 2.0, 272.0, detector="gaussderiv"),
            _edge(BOX_TOP, "y", 1.9, 194.0),
        ],
        p0_um=50.0,
    )
    assert "calibrated with halfmax" in detector_mismatch(mixed, "gaussderiv")
