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


LIVE = 0.1234  # deg, where the galvo sits in Live before the MDA


@pytest.mark.parametrize(
    ("z_plan", "focus", "first", "last", "live_slice"),
    [
        (useq.ZRangeAround(range=4, step=1), 0.0, -2.0, 2.0, "slice 2"),
        (useq.ZRangeAround(range=4, step=1, go_up=False), 0.0, 2.0, -2.0, "slice 2"),
        (useq.ZRangeAround(range=3, step=1), 0.0, -1.5, 1.5, "between slices 1 and 2"),
        (useq.ZAboveBelow(above=40, below=0, step=0.5), 0.0, 0.0, 40.0, "slice 0"),
        (useq.ZAboveBelow(above=2, below=1, step=1), 0.0, -1.0, 2.0, "slice 1"),
        (
            useq.ZAboveBelow(above=2, below=1, step=1, go_up=False),
            0,
            2.0,
            -1.0,
            "slice 2",
        ),
        (useq.ZTopBottom(top=103, bottom=100, step=1), 101.0, -1.0, 2.0, "slice 1"),
        (
            useq.ZTopBottom(top=103, bottom=100, step=1),
            90.0,
            10.0,
            13.0,
            "outside the stack",
        ),
        (useq.ZRelativePositions(relative=[0, 1, 2]), 0.0, 0.0, 2.0, "slice 0"),
        (useq.ZAbsolutePositions(absolute=[5, 6, 7]), 6.0, -1.0, 1.0, "slice 1"),
    ],
)
def test_every_mode_is_placed_relative_to_the_live_plane(
    z_plan: useq.ZPlan, focus: float, first: float, last: float, live_slice: str
) -> None:
    """Slice i images the Live plane + (z[i] - reference) in every mode."""
    scan = compute_galvo_scan(z_plan, focus, SLOPE, live_offset_deg=LIVE)

    assert scan.amplitude_deg == pytest.approx((last - first) / SLOPE)
    # the sweep center sits at the plan's center measured from the Live plane
    assert scan.offset_deg == pytest.approx(LIVE + (first + last) / 2 / SLOPE)
    assert scan.live_plane_slice() == live_slice
    assert f"{first:+.3f} .. {last:+.3f} um from the Live plane" in scan.describe()


def test_non_uniform_relative_positions_rejected() -> None:
    with pytest.raises(ValueError, match="uniformly spaced"):
        compute_galvo_scan(useq.ZRelativePositions(relative=[0, 1, 3]), 0, SLOPE)
