from __future__ import annotations

import numpy as np
import pytest

from pymmcore_gui._spot_tracking import TrackedSpot, centroid_track, displacement


def _gaussian_frame(
    shape: tuple[int, int],
    center: tuple[float, float],
    *,
    sigma: float = 3.0,
    amplitude: float = 1000.0,
    background: float = 50.0,
) -> np.ndarray:
    """Build a synthetic frame with a single 2-D Gaussian blob at `center` (x, y)."""
    h, w = shape
    ys, xs = np.indices((h, w))
    cx, cy = center
    blob = amplitude * np.exp(-(((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * sigma**2)))
    return background + blob


def test_centroid_track_converges_to_synthetic_blob_center() -> None:
    frame = _gaussian_frame((100, 100), (50.3, 60.7))
    seed = TrackedSpot(x=50.0, y=61.0, box_radius=15)

    result = centroid_track(frame, seed)

    assert result is not None
    assert result.x == pytest.approx(50.3, abs=0.1)
    assert result.y == pytest.approx(60.7, abs=0.1)
    assert result.box_radius == seed.box_radius


def test_centroid_track_follows_a_drifting_blob_across_frames() -> None:
    true_center = np.array([40.0, 40.0])
    step = np.array([0.5, 0.3])
    spot = TrackedSpot(x=40.0, y=40.0, box_radius=15)

    for _ in range(10):
        true_center = true_center + step
        frame = _gaussian_frame((100, 100), (true_center[0], true_center[1]))
        result = centroid_track(frame, spot)
        assert result is not None
        assert result.x == pytest.approx(true_center[0], abs=0.2)
        assert result.y == pytest.approx(true_center[1], abs=0.2)
        spot = result


def test_centroid_track_robust_to_realistic_sensor_noise() -> None:
    """Regression test for the original weak-background-subtraction bug.

    The pre-fix implementation (fixed 10th-percentile background) left ~90%
    of a noisy box's pixels contributing nonzero weight, letting sensor
    noise drag the centroid off the true peak -- this wouldn't have shown up
    on the noise-free tests above. Real camera read noise is simulated here
    with a seeded RNG for determinism.
    """
    rng = np.random.default_rng(42)
    true_center = (55.0, 45.0)
    frame = _gaussian_frame(
        (100, 100), true_center, sigma=3.0, amplitude=800.0, background=100.0
    )
    frame = frame + rng.normal(0.0, 15.0, frame.shape)
    seed = TrackedSpot(x=53.0, y=47.0, box_radius=15)

    result = centroid_track(frame, seed)

    assert result is not None
    assert result.x == pytest.approx(true_center[0], abs=0.5)
    assert result.y == pytest.approx(true_center[1], abs=0.5)


def test_centroid_track_converges_from_a_significantly_off_center_seed() -> None:
    """A single call must converge onto the true peak even from a poor seed.

    Simulates a fresh user click that landed several pixels off the true
    feature center -- the intra-call iterative re-centering (not just
    cross-frame convergence) is what makes this work in one call.
    """
    frame = _gaussian_frame((100, 100), (50.0, 50.0), sigma=3.0)
    seed = TrackedSpot(x=40.0, y=40.0, box_radius=15)  # ~14 px off

    result = centroid_track(frame, seed)

    assert result is not None
    assert result.x == pytest.approx(50.0, abs=0.3)
    assert result.y == pytest.approx(50.0, abs=0.3)


def test_centroid_track_returns_none_at_frame_edge() -> None:
    frame = _gaussian_frame((100, 100), (5.0, 5.0))
    spot = TrackedSpot(x=2.0, y=2.0, box_radius=15)

    assert centroid_track(frame, spot) is None


def test_centroid_track_returns_none_on_uniform_background() -> None:
    frame = np.full((100, 100), 50.0)
    spot = TrackedSpot(x=50.0, y=50.0, box_radius=15)

    assert centroid_track(frame, spot) is None


def test_displacement() -> None:
    cam1 = TrackedSpot(x=10.0, y=20.0)
    cam2 = TrackedSpot(x=13.0, y=24.0)

    dx, dy, mag = displacement(cam1, cam2)

    assert dx == pytest.approx(3.0)
    assert dy == pytest.approx(4.0)
    assert mag == pytest.approx(5.0)
