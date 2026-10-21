from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest
import tifffile

from pymmcore_gui.asi_z_stack.image_analysis import (
    phase_shift,
    row_profile_ratio,
    volume_drift,
)

if TYPE_CHECKING:
    from pathlib import Path


def _blobs(shape: tuple[int, ...], seed: int = 0) -> np.ndarray:
    """Smooth random texture (bead-like) for registration tests."""
    rng = np.random.default_rng(seed)
    data = rng.random(shape)
    kernel = np.ones(3) / 3
    for axis in range(data.ndim):
        data = np.apply_along_axis(np.convolve, axis, data, kernel, mode="same")
    return data * 1000 + 100


def test_row_profile_ratio_flat_when_identical() -> None:
    img = _blobs((64, 48))
    ratios = row_profile_ratio(img, img, bands=8)
    np.testing.assert_allclose(ratios, 1.0)


def test_row_profile_ratio_shows_rolling_shutter_falloff(tmp_path: Path) -> None:
    snap = np.full((80, 40), 1100.0)
    # later rows get less light: the rolling-shutter + short-pulse signature
    mda = 100 + 1000 * np.linspace(1.0, 0.25, 80)[:, None] * np.ones((1, 40))
    snap_path = tmp_path / "snap.ome.tif"
    tifffile.imwrite(snap_path, snap.astype(np.uint16))
    stack = np.stack([mda, mda, mda]).astype(np.uint16)

    ratios = row_profile_ratio(snap_path, stack, bands=4, offset=100)

    assert ratios[0] > 0.85
    assert ratios[-1] < 0.4
    assert np.all(np.diff(ratios) < 0)


def test_row_profile_ratio_rejects_shape_mismatch() -> None:
    with pytest.raises(ValueError, match="differ in shape"):
        row_profile_ratio(np.ones((4, 4)), np.ones((4, 5)))


@pytest.mark.parametrize("shift", [(3, -5), (0, 7), (-2, 0)])
def test_phase_shift_recovers_integer_roll_2d(shift: tuple[int, int]) -> None:
    ref = _blobs((64, 64))
    moving = np.roll(ref, shift, axis=(0, 1))
    np.testing.assert_allclose(phase_shift(ref, moving), shift, atol=0.2)


def test_volume_drift_tracks_3d_shifts() -> None:
    ref = _blobs((16, 64, 64), seed=1)
    vols = [ref, np.roll(ref, (1, 4, -8), axis=(0, 1, 2))]

    drift = volume_drift(vols, bin_yx=2)

    np.testing.assert_allclose(drift[0], 0)
    np.testing.assert_allclose(drift[1], (1, 4, -8), atol=0.6)
