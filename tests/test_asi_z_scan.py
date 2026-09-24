"""Unit tests for translating useq z-plans into ASI galvo sweep parameters."""

from __future__ import annotations

import pytest
import useq

from pymmcore_gui.asi_z_stack.z_scan import compute_galvo_scan

SLOPE = 100.0  # um per deg, HardwareConstants default


def test_range_around_is_centered_on_focus() -> None:
    scan = compute_galvo_scan(useq.ZRangeAround(range=10, step=1), 50.0, SLOPE)
    assert scan.num_slices == 11
    assert scan.center_offset_um == 0
    assert scan.offset_deg == 0
    assert scan.amplitude_deg == pytest.approx(10 / SLOPE)


def test_above_below_only_above_starts_at_focus() -> None:
    scan = compute_galvo_scan(useq.ZAboveBelow(above=10, below=0, step=1), 0, SLOPE)
    assert scan.z_positions[0] == 0
    assert scan.center_offset_um == pytest.approx(5)
    assert scan.offset_deg == pytest.approx(5 / SLOPE)
    assert scan.amplitude_deg == pytest.approx(10 / SLOPE)


def test_above_below_only_below() -> None:
    scan = compute_galvo_scan(useq.ZAboveBelow(above=0, below=10, step=1), 0, SLOPE)
    assert scan.center_offset_um == pytest.approx(-5)


def test_above_below_asymmetric() -> None:
    scan = compute_galvo_scan(useq.ZAboveBelow(above=6, below=2, step=1), 0, SLOPE)
    assert scan.center_offset_um == pytest.approx(2)
    assert scan.amplitude_deg == pytest.approx(8 / SLOPE)


@pytest.mark.parametrize(("focus", "center"), [(100.0, 5.0), (105.0, 0.0), (0, 105)])
def test_top_bottom_is_measured_from_current_focus(focus: float, center: float) -> None:
    plan = useq.ZTopBottom(top=110, bottom=100, step=1)
    scan = compute_galvo_scan(plan, focus, SLOPE)
    assert scan.center_offset_um == pytest.approx(center)
    assert scan.amplitude_deg == pytest.approx(10 / SLOPE)


def test_go_down_reverses_sweep_direction() -> None:
    plan = useq.ZRangeAround(range=4, step=1, go_up=False)
    scan = compute_galvo_scan(plan, 0, SLOPE)
    assert scan.amplitude_deg == pytest.approx(-4 / SLOPE)
    assert scan.step_um == pytest.approx(-1)
    assert scan.center_offset_um == 0


def test_single_slice_has_zero_amplitude() -> None:
    scan = compute_galvo_scan(useq.ZRelativePositions(relative=[3.0]), 0, SLOPE)
    assert scan.num_slices == 1
    assert scan.amplitude_deg == 0
    assert scan.center_offset_um == pytest.approx(3)


def test_non_uniform_steps_are_rejected() -> None:
    with pytest.raises(ValueError, match="uniformly spaced"):
        compute_galvo_scan(useq.ZRelativePositions(relative=[0, 1, 3]), 0, SLOPE)
