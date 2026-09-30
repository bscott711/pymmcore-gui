"""The replay's pacing and read-ahead (pymmcore_gui._argus_stream.replay)."""

from __future__ import annotations

import hashlib

import numpy as np
import pytest
import zarr

from pymmcore_gui._argus_stream import replay


def _meta(t: int, c: int, z: int, ms: float) -> dict:
    return {"runner_time_ms": ms, "mda_event": {"index": {"t": t, "c": c, "z": z}}}


def _recorded_run(n_t=3, nz=4, plane_ms=36.0, gap_ms=830.0, every_ms=23_000.0):
    """Frame times as the rig writes them: the whole 488 stack, a gap, the
    whole 561 stack; the first plane 5.2 s after the run starts."""
    metas = []
    for t in range(n_t):
        t0 = 5200.0 + t * every_ms
        for c in range(2):
            c0 = t0 + c * ((nz - 1) * plane_ms + gap_ms)
            metas += [_meta(t, c, z, c0 + z * plane_ms) for z in range(nz)]
    return metas


def test_recorded_schedule_replays_channels_one_after_the_other():
    s = replay.recorded_schedule(_recorded_run(), n_c=2, nz=4)
    assert s is not None and s.source == "recorded"
    assert s.setup_s == pytest.approx(5.2)
    assert s.interval_s == pytest.approx(23.0)
    info = s.summary()
    assert info["channel_stack_s"] == {0: pytest.approx(0.108), 1: pytest.approx(0.108)}
    assert info["channel_start_s"][1] == pytest.approx(0.108 + 0.83)
    order = s.order(2)
    # every 488 plane of a timepoint before its first 561 plane
    assert [(t, c) for _s, t, c, _z in order[:8]] == [(0, 0)] * 4 + [(0, 1)] * 4
    assert order[8][0] == pytest.approx(5.2 + 23.0)


def test_recorded_schedule_uses_only_complete_timepoints():
    metas = _recorded_run(n_t=3)
    metas = [
        m
        for m in metas
        if m["mda_event"]["index"]["t"] != 2 or m["runner_time_ms"] < 51_300
    ]
    s = replay.recorded_schedule(metas, n_c=2, nz=4)
    assert s is not None and s.interval_s == pytest.approx(23.0)
    assert replay.recorded_schedule([], n_c=2, nz=4) is None
    # a plane missing from every timepoint: nothing complete to go on
    assert replay.recorded_schedule(_recorded_run(), n_c=2, nz=5) is None


def test_recorded_schedule_bridges_a_gap_in_the_recorded_timepoints():
    metas = [m for m in _recorded_run(n_t=4) if m["mda_event"]["index"]["t"] != 1]
    s = replay.recorded_schedule(metas, n_c=2, nz=4)
    assert s is not None and s.interval_s == pytest.approx(23.0)


def test_fixed_schedule_paces_channels_in_turn():
    s = replay.fixed_schedule(2, nz=10, stack_s=1.0, interval_s=5.0)
    assert s.at(0, 0, 0) == pytest.approx(1.0)
    assert s.at(0, 1, 0) == pytest.approx(2.0)  # after the 488 stack
    assert s.at(1, 0, 9) == pytest.approx(1.0 + 5.0 + 0.9)


def test_recorded_z_step_comes_from_the_saved_sequence():
    arr = zarr.zeros((1, 2, 3, 4), dtype="uint16")
    assert replay.recorded_z_step([arr]) is None
    arr.attrs["useq_MDASequence"] = {"z_plan": {"range": 30.0, "step": 0.1}}
    assert replay.recorded_z_step([arr]) == pytest.approx(0.1)


def test_volume_reader_reads_ahead_in_send_order_and_hashes():
    rng = np.random.default_rng(0)
    arrays = [rng.integers(0, 1000, (3, 2, 4, 5), dtype=np.uint16) for _ in range(2)]
    order = [(t, c) for t in range(3) for c in range(2)]
    reader = replay.VolumeReader(arrays, order, depth=2)
    reader.start()
    for t, c in order:
        np.testing.assert_array_equal(reader.get(t, c), arrays[c][t])
    reader.join(5)
    assert reader.sha1["2,1"] == hashlib.sha1(arrays[1][2].tobytes()).hexdigest()
    assert len(reader.stalls) == len(order)


def test_volume_reader_reports_a_failed_read():
    class Broken:
        def __getitem__(self, t):
            raise OSError("disk gone")

    reader = replay.VolumeReader([Broken()], [(0, 0)], depth=1)
    reader.start()
    with pytest.raises(RuntimeError, match="t=0 c=0"):
        reader.get(0, 0)
