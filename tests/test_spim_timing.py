from __future__ import annotations

import pytest

from pymmcore_gui._settings import SpimTimingSettingsV1
from pymmcore_gui.asi_z_stack.spim_timing import (
    LEGACY_SETTLE_MS,
    ceil_tick,
    compute_slice_timing,
    readout_ms_from_camera_timing,
    timing_from_settings,
)


def test_legacy_reproduces_original_timing() -> None:
    """Today's rig: 25.75 ms stored settle + 10 ms pulse = 35.75 ms per slice."""
    t = compute_slice_timing("legacy", 10.0, 13.5, 25.75)

    assert t.camera_pulse_ms == 10.0
    assert t.hold_ms == 10.0
    assert t.slice_period_ms == 35.75
    assert t.volume_ms(81) == pytest.approx(81 * 35.75)
    # rolling shutter: later rows get less of the 10 ms pulse
    assert t.last_row_light_ms == 0.0
    assert t.mean_row_light_fraction == pytest.approx(10 / (2 * 13.5))
    assert not t.galvo_moves_during_exposure
    assert "mean 37% of Live" in t.describe()


def test_legacy_waits_for_readout_when_settle_is_short() -> None:
    t = compute_slice_timing("legacy", 10.0, 6.0, 2.0)
    assert t.hold_ms == 14.0  # S + E - s: camera must finish reading out
    assert t.slice_period_ms == 16.0


def test_all_rows_full_frame_costs_one_extra_readout() -> None:
    """S + E + max(s, S): with the settle under S, about 2S + E."""
    t = compute_slice_timing("all_rows", 10.0, 13.5, 2.0, margin_ms=0.5)

    assert t.camera_pulse_ms == 1.0  # Edge Trigger
    assert t.hold_ms == ceil_tick(2 * 13.5 + 10 - 2 + 0.5)
    assert t.slice_period_ms == pytest.approx(37.5)
    assert t.mean_row_light_fraction == 1.0
    assert t.last_row_light_ms == 10.0
    assert not t.galvo_moves_during_exposure


def test_all_rows_cropped_roi_is_faster() -> None:
    full = compute_slice_timing("all_rows", 10.0, 13.5, 2.0)
    cropped = compute_slice_timing("all_rows", 10.0, 13.5 * 1143 / 2400, 2.0)
    assert cropped.slice_period_ms < 0.65 * full.slice_period_ms


def test_all_rows_long_settle_dominates() -> None:
    t = compute_slice_timing("all_rows", 10.0, 6.0, 25.75, margin_ms=0.5)
    assert t.hold_ms == 16.5  # S + E + margin: readout already done in the settle
    assert t.slice_period_ms == 42.25


def test_all_rows_held_on_laser_waits_for_last_row() -> None:
    t = compute_slice_timing("all_rows", 10.0, 6.0, 25.75, laser_held_on=True)
    assert t.hold_ms == 16.5
    assert not t.galvo_moves_during_exposure


def test_legacy_held_on_laser_flags_blur() -> None:
    t = compute_slice_timing("legacy", 10.0, 6.0, 25.75, laser_held_on=True)
    assert t.galvo_moves_during_exposure
    assert "WARNING" in t.describe()


def test_ceil_tick_rounds_up_to_quarter_ms() -> None:
    assert ceil_tick(10.0) == 10.0
    assert ceil_tick(10.01) == 10.25
    assert ceil_tick(6.4125) == 6.5
    assert ceil_tick(0.1 + 0.2) == 0.5


@pytest.mark.parametrize(
    ("timing", "expected"),
    [
        ({"Timing-ReadoutTimeNs": "13500000"}, 13.5),
        ({"Timing-ReadoutTimeUs": "6400"}, 6.4),
        ({"ReadoutTime": "7.25"}, 7.25),
        ({"TriggerMode": "Edge Trigger"}, None),
        ({"Timing-ReadoutTimeNs": "n/a"}, None),
    ],
)
def test_readout_from_camera_timing(
    timing: dict[str, str], expected: float | None
) -> None:
    assert readout_ms_from_camera_timing(timing) == expected


def test_timing_from_settings_pins_legacy_to_original() -> None:
    cfg = SpimTimingSettingsV1(
        mode="legacy", galvo_settle_ms=0.25, volume_start_settle_ms=3.0, margin_ms=0
    )
    t = timing_from_settings(cfg, 10.0, 4.04)
    assert t.settle_ms == LEGACY_SETTLE_MS
    assert t.volume_start_settle_ms == 0.0
    assert t.slice_period_ms == 35.75

    cfg.mode = "all_rows"
    t = timing_from_settings(cfg, 10.0, 4.04)
    assert t.settle_ms == 0.25
    assert t.volume_start_settle_ms == 3.0
