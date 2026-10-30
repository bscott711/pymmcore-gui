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


def test_all_rows_delays_the_laser_into_the_all_rows_window() -> None:
    """Cropped rig: S = 4.04 ms. Camera pulse covers S + E; laser waits for S."""
    t = compute_slice_timing("all_rows", 10.0, 4.04, 25.75, margin_ms=0.5)

    assert t.laser_delay_ms == 4.75  # S + margin, on the 0.25 ms clock
    assert t.laser_pulse_ms == 10.0
    assert t.camera_pulse_ms == 15.25  # delay + E + margin
    # every row is integrating while the laser is on: it gets exactly E
    assert t.laser_delay_ms >= t.readout_ms
    assert t.laser_delay_ms + t.laser_pulse_ms <= t.camera_pulse_ms
    assert t.mean_row_light_fraction == 1.0
    assert t.last_row_light_ms == 10.0
    # proven settle: galvo holds for the camera pulse
    assert t.hold_ms == 15.25
    assert t.slice_period_ms == 41.0
    assert not t.galvo_moves_during_exposure
    assert "laser 10 ms after 4.75" in t.describe()


def test_all_rows_level_trigger_waits_for_readout_with_short_settle() -> None:
    t = compute_slice_timing("all_rows", 10.0, 4.04, 0.25, margin_ms=0.5)
    # next trigger only once the frame has read out: hold + settle >= C + S
    assert t.hold_ms + t.settle_ms >= t.camera_pulse_ms + t.readout_ms
    assert t.slice_period_ms == 19.5


def test_all_rows_overlap_steps_right_after_the_camera_pulse() -> None:
    t = compute_slice_timing("all_rows", 10.0, 4.04, 0.25, margin_ms=0.5, overlap=True)
    assert t.overlap
    assert t.hold_ms == t.camera_pulse_ms == 15.25
    assert t.slice_period_ms == 15.5  # about S + E + settle
    assert "(overlap)" in t.describe()


def test_all_rows_full_frame_vs_cropped() -> None:
    full = compute_slice_timing("all_rows", 10.0, 8.48, 0.25, overlap=True)
    cropped = compute_slice_timing("all_rows", 10.0, 4.04, 0.25, overlap=True)
    assert full.slice_period_ms == 19.75
    assert cropped.slice_period_ms < full.slice_period_ms


def test_all_rows_held_on_laser_waits_for_last_row() -> None:
    t = compute_slice_timing(
        "all_rows", 10.0, 4.04, 25.75, laser_held_on=True, overlap=True
    )
    assert not t.overlap  # constant light can't use overlap (uneven rows)
    assert t.camera_pulse_ms == 10.0
    assert t.laser_delay_ms == 0.0
    assert t.hold_ms == 14.75  # S + E + margin
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
