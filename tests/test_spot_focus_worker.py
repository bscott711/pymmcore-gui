"""The spot focus lock's subprocess and service, with a real spawned worker.

The metric is the position of one straight edge of the LED patch inside an edge
box. Covers the live view too: preview frames through shared memory, the edge
overlay, live camera settings, ROI changes in sensor coordinates and the
time-based settle.

A ``FakeSpotSource`` (high drift rate, short runs) stands in for the camera and
a fake mmc records every ``setPosition`` and never waits. The fake patch is
centred at (320, 240) and is 96 px wide, so its left edge is at x = 272 and
moves along x by ``SLOPE`` px per um of P. No hardware, Qt, pymmcore or
Micro-Manager install needed; run with ``--noconftest``.
"""

from __future__ import annotations

import functools
import itertools
import os
import threading
import time
import types
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.spot_focus import (
    EdgeCalibration,
    SpotLoopParams,
    measure_edge,
)
from pymmcore_gui.asi_z_stack.spot_focus_service import SpotFocusService
from pymmcore_gui.asi_z_stack.spot_focus_worker import (
    Link,
    Settle,
    _Worker,
    launch_worker,
    settle_time_s,
)
from pymmcore_gui.asi_z_stack.spot_frame_shm import FrameBuffer
from pymmcore_gui.asi_z_stack.toupcam_source import make_source

P_LABEL = "PiezoStage:P:34"
CRISP = "CRISP"
P0 = 50.0
SLOPE = 5.0  # the fake's true image motion along x, px per um of P
EDGE_X = 272.0  # the patch's left edge at P0 (patch centre 320, width 96)
CAL = {
    "x0": 320.0,
    "y0": 240.0,
    "ux": 1.0,
    "uy": 0.0,
    "slope_px_per_um": SLOPE,
    "p0_um": P0,
}
#: Across the left edge; clear of the rounded corners and of the ghost.
BOX = (232, 210, 80, 60)
BOX_CENTER_Y = 240.0


class FakeMMC:
    """Records setPosition; never waits; remembers properties."""

    def __init__(self) -> None:
        self.pos = {P_LABEL: P0}
        self.props = {(CRISP, "CRISP State"): "Idle", (CRISP, "LED Intensity"): "0"}
        self.sets: list[tuple[float, float]] = []  # (wall, target)
        self.waits = 0
        self.active = 0
        self.max_active = 0
        self._lock = threading.Lock()

    def setPosition(self, label: str, value: float) -> None:
        with self._lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        time.sleep(0.002)  # a no-wait serial write still takes a moment
        self.pos[label] = float(value)
        self.sets.append((time.monotonic(), float(value)))
        with self._lock:
            self.active -= 1

    def getPosition(self, label: str) -> float:
        return self.pos[label]

    def waitForDevice(self, label: str) -> None:
        self.waits += 1

    def getProperty(self, label: str, prop: str) -> str:
        return str(self.props[(label, prop)])

    def setProperty(self, label: str, prop: str, value: object) -> None:
        self.props[(label, prop)] = str(value)


def _spec(**kw: object) -> dict:
    spec = {"kind": "fake", "cal_like": dict(CAL), "noise": 2.0, "fps": 50.0}
    spec.update(kw)
    return spec


@functools.cache
def _e0() -> float:
    """The edge position the fake shows at P0, as the metric measures it."""
    src = make_source(_spec())
    fixes = [measure_edge(src.render(0.0), BOX) for _ in range(5)]  # type: ignore[attr-defined]
    assert all(f.ok for f in fixes)
    return float(np.mean([f.position for f in fixes]))


def _cal(**kw: object) -> EdgeCalibration:
    base: dict[str, Any] = {
        "box": BOX,
        "axis": "x",
        "polarity": 1,
        "slope_px_per_um": SLOPE,
        "e0_px": _e0(),
        "p0_um": P0,
        "linear_range_um": 1.0,
        "noise_px": 0.02,
        "angle_deg": 0.0,
        "contrast": 700.0,
        "n_points": 9,
        "created": "2026-10-28T00:00:00",
    }
    base.update(kw)
    return EdgeCalibration(**base)


def _params(**kw: object) -> SpotLoopParams:
    base = {"avg_frames": 4, "max_rate_hz": 5.0, "strikes": 3}
    base.update(kw)
    return SpotLoopParams(**base)  # type: ignore[arg-type]


def _wait(cond, timeout: float = 10.0, step: float = 0.02) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def _gone(proc: Any) -> bool:
    """True once *proc* has exited. Not ``os.kill(pid, 0)``: on Windows signal 0
    is CTRL_C_EVENT, which sends a console Ctrl+C and never raises."""
    return not proc.is_alive() and proc.exitcode is not None


@pytest.fixture(autouse=True)
def _child_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make sure the spawned child can import the package under test."""
    import pymmcore_gui

    src = str(Path(pymmcore_gui.__file__).resolve().parent.parent)
    old = os.environ.get("PYTHONPATH", "")
    monkeypatch.setenv("PYTHONPATH", src + (os.pathsep + old if old else ""))


def _service(mmc: FakeMMC, **spec_kw: object) -> SpotFocusService:
    return SpotFocusService(mmc, P_LABEL, CRISP, _spec(**spec_kw), ready_timeout_s=20.0)


# One shared closed-loop run feeds the convergence and the rate tests.
RATE_CAP_HZ = 5.0
RUN_S = 3.5
DRIFT_UM_S = 0.3


@pytest.fixture(scope="module")
def closed_loop() -> dict:
    mp = pytest.MonkeyPatch()
    import pymmcore_gui

    mp.setenv("PYTHONPATH", str(Path(pymmcore_gui.__file__).resolve().parent.parent))
    mmc = FakeMMC()
    tele: list[dict] = []
    moved: list[dict] = []
    try:
        with _service(mmc, drift_um_per_s=DRIFT_UM_S) as svc:
            svc.telemetry.connect(tele.append)
            svc.moved.connect(moved.append)
            assert svc.wait_ready()
            info = svc.info
            proc = svc._proc
            svc.start(_params(max_rate_hz=RATE_CAP_HZ), _cal())
            t0 = time.monotonic()
            time.sleep(RUN_S)
            elapsed = time.monotonic() - t0
            stats = svc.stats()
            svc.stop()
            final_state = svc.current_state
        return {
            "mmc": mmc,
            "tele": list(tele),
            "moved": list(moved),
            "stats": stats,
            "elapsed": elapsed,
            "info": info,
            "proc": proc,
            "state": final_state,
        }
    finally:
        mp.undo()


def test_closed_loop_holds_error_within_band(closed_loop: dict) -> None:
    tele = closed_loop["tele"]
    assert len(tele) > 10
    late = [
        abs(t["along_um"]) for t in tele[len(tele) // 2 :] if t["along_um"] is not None
    ]
    assert late
    late.sort()
    median = late[len(late) // 2]
    # Uncorrected, the drift alone would be ~RUN_S*DRIFT_UM_S = 1 um by the end.
    assert median < 0.12, f"median |error| {median:.3f} um"
    # P was pulled against the drift (target ~ start - drift*t).
    final_p = closed_loop["mmc"].sets[-1][1]
    assert final_p < P0 - 0.5
    assert closed_loop["mmc"].sets[-1][1] == pytest.approx(
        P0 - DRIFT_UM_S * RUN_S, abs=0.35
    )


def test_telemetry_carries_the_edge_fields(closed_loop: dict) -> None:
    t = closed_loop["tele"][-1]
    for key in (
        "t edge_px x y along_um contrast angle_deg residual_px valid_fraction fps "
        "p_cmd_um n_moves ok reason max_value saturated_frac"
    ).split():
        assert key in t, key
    assert "lateral_px" not in t and "amplitude" not in t
    # The edge is held near its zero; the overlay point sits on the edge.
    assert t["edge_px"] == pytest.approx(_e0(), abs=1.5)
    assert t["x"] == t["edge_px"]
    assert t["y"] == pytest.approx(BOX_CENTER_Y)


def test_move_rate_respects_cap_and_never_waits(closed_loop: dict) -> None:
    stats = closed_loop["stats"]
    mmc = closed_loop["mmc"]
    cap_per_min = RATE_CAP_HZ * 60.0
    print(
        f"\nmoves={stats['n_moves']} moves/min={stats['moves_per_min']:.1f} "
        f"(cap {cap_per_min:.0f}) mean_dt={stats['mean_dt_ms']:.2f} ms "
        f"over {closed_loop['elapsed']:.1f} s"
    )
    assert stats["n_moves"] >= 3
    assert stats["moves_per_min"] <= cap_per_min * 1.1
    assert stats["n_moves"] / closed_loop["elapsed"] * 60.0 <= cap_per_min * 1.1
    # Fire-and-forget: no waitForDevice, never two setPositions at once.
    assert mmc.waits == 0
    assert mmc.max_active == 1
    assert all(m["ok"] for m in closed_loop["moved"])
    # Moves are at least 1/cap apart (within scheduling jitter).
    times = [t for t, _ in mmc.sets]
    gaps = [b - a for a, b in itertools.pairwise(times)]
    assert min(gaps) > 0.8 / RATE_CAP_HZ


def test_run_ends_stopped_and_worker_exits(closed_loop: dict) -> None:
    assert closed_loop["state"] == "stopped"
    assert _wait(lambda: _gone(closed_loop["proc"]), 5.0)


def test_killing_the_worker_faults_and_stops_moves() -> None:
    mmc = FakeMMC()
    faults: list[str] = []
    with _service(mmc, drift_um_per_s=0.5) as svc:
        svc.fault.connect(faults.append)
        svc.start(_params(), _cal())
        assert _wait(lambda: len(mmc.sets) >= 2, 8.0)
        proc = svc._proc
        proc.kill()
        assert _wait(lambda: faults, 5.0)
        assert faults == ["worker died"]
        assert svc.current_state == "fault"
        n = len(mmc.sets)
        time.sleep(0.6)
        assert len(mmc.sets) == n
        with pytest.raises(RuntimeError):
            svc.start(_params(), _cal())
    assert _wait(lambda: _gone(proc), 5.0)


def test_a_dark_patch_is_a_fault_and_not_a_move() -> None:
    """The LED effectively off (exposure at its minimum): edge lost, P held."""
    mmc = FakeMMC()
    faults: list[str] = []
    with _service(mmc) as svc:
        svc.fault.connect(faults.append)
        svc.start(_params(min_contrast=100.0), _cal())
        assert _wait(lambda: svc.current_state == "running", 5.0)
        time.sleep(0.5)
        assert svc.current_state == "running"  # healthy at the normal exposure
        svc.set_camera(exposure_ms=0.105)  # the patch drops to ~1 % brightness
        assert _wait(lambda: faults, 8.0)
        assert "edge lost" in faults[0]
        assert svc.current_state == "fault"
        time.sleep(0.5)
        assert mmc.sets == []  # no move, P is held
        assert svc.stats()["n_moves"] == 0


def test_exposure_change_does_not_move_p() -> None:
    """Contrast scales with exposure, the 50 % crossing does not."""
    mmc = FakeMMC()
    with _service(mmc) as svc:
        svc.start(_params(), _cal())
        assert _wait(lambda: svc.current_state == "running", 5.0)
        time.sleep(0.8)
        svc.set_camera(exposure_ms=20.0)
        assert _wait(lambda: svc.camera_info["exposure_ms"] == 20.0, 5.0)
        time.sleep(1.5)
        assert svc.current_state == "running"
        assert mmc.sets == []
        assert abs(svc.last_telemetry["along_um"]) < 0.03


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_calibration_recovers_slope_sign_and_zero(sign: float, tmp_path: Path) -> None:
    mmc = FakeMMC()
    cal_like = {**CAL, "slope_px_per_um": sign * SLOPE}
    rec = tmp_path / "sweep"
    with _service(mmc, cal_like=cal_like) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        first = _next_frame(svc, None)
        cal = svc.calibrate(
            BOX, span_um=2.0, steps=7, n_frames=3, settle_s=0.0, record_path=rec
        )
        assert cal.slope_px_per_um == pytest.approx(sign * SLOPE, rel=0.1)
        assert cal.r2 > 0.98
        assert cal.e0_px == pytest.approx(_e0(), abs=0.3)
        assert cal.p0_um == pytest.approx(P0)
        assert (cal.axis, cal.polarity, cal.box) == ("x", 1, BOX)
        assert cal.linear_range_um == pytest.approx(1.0)
        assert cal.n_points == 7
        assert cal.noise_px < 0.2
        # P is back where it started, the LED claim is gone, Live is still on.
        assert mmc.pos[P_LABEL] == pytest.approx(P0)
        assert mmc.waits >= 8  # a calibration (lock stopped) may wait
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
        assert svc.preview_active and svc.current_state == "preview"
        assert _next_frame(svc, first.seq).seq > first.seq
    # The record holds one raw frame per step, in sweep order.
    assert cal.record_path == str(tmp_path / "sweep.npz")
    with np.load(cal.record_path) as z:
        assert z["p_um"].shape == (7,) and np.all(np.diff(z["p_um"]) > 0)
        assert z["p_um"][3] == pytest.approx(P0)
        assert z["edge_px"].shape == (7,)
        assert z["frames"].shape == (7, 480, 640)
        assert tuple(z["box"]) == BOX
        # Each frame shows its own step: the patch moved along x.
        e = [measure_edge(f, BOX).position for f in z["frames"]]
        assert e == pytest.approx(z["edge_px"], abs=0.5)
    # The calibration survives the settings round trip.
    assert EdgeCalibration.from_dict(cal.to_dict()) == cal


def test_calibration_works_while_previewing_in_an_roi() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        first = _next_frame(svc, None)
        svc.set_camera(roi=(100, 80, 400, 320))  # the sweep also works in an ROI
        cal = svc.calibrate(BOX, span_um=2.0, steps=7, n_frames=3, settle_s=0.0)
        assert cal.slope_px_per_um == pytest.approx(SLOPE, rel=0.1)
        # Positions are in sensor pixels, not ROI pixels.
        assert cal.e0_px == pytest.approx(_e0(), abs=0.3)
        assert cal.record_path == ""
        assert svc.preview_active and svc.current_state == "preview"
        assert _next_frame(svc, first.seq).seq > first.seq


def test_calibration_that_loses_the_edge_raises_and_cleans_up() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        real = svc._snap
        calls: list[float] = []

        def snap(n, p, box, axis, polarity, return_frame):
            calls.append(p)
            if len(calls) >= 4:  # the zero and two steps are fine
                return {"n": 0, "reason": "edge left the box", "p_um": p}
            return real(n, p, box, axis, polarity, return_frame)

        svc._snap = snap  # type: ignore[method-assign]
        with pytest.raises(ValueError, match="edge left the box: widen the box"):
            svc.calibrate(BOX, span_um=2.0, steps=7, n_frames=3, settle_s=0.0)
        assert mmc.pos[P_LABEL] == pytest.approx(P0)  # P is back
        assert not svc.led_is_on
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
        assert svc.preview_active and svc.current_state == "preview"


def test_calibration_with_no_edge_in_the_box_says_so() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        with pytest.raises(ValueError, match="edge not measurable"):
            svc.calibrate((20, 20, 80, 60), steps=7, n_frames=2, settle_s=0.0)
        assert mmc.pos[P_LABEL] == pytest.approx(P0)
        assert mmc.sets[-1][1] == pytest.approx(P0)


def test_calibration_and_zero_refuse_while_running_or_during_an_mda() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        svc.start(_params(), _cal())
        assert _wait(lambda: svc.current_state == "running", 5.0)
        with pytest.raises(RuntimeError, match="stop the loop"):
            svc.calibrate(BOX)
        with pytest.raises(RuntimeError, match="stop the loop"):
            svc.zero(_cal())
        with pytest.raises(RuntimeError, match="stop the loop"):
            svc.measure(3, box=BOX)
        svc.stop()
        n = len(mmc.sets)
        mmc.mda = types.SimpleNamespace(is_running=lambda: True)  # type: ignore[attr-defined]
        with pytest.raises(RuntimeError, match="acquisition"):
            svc.calibrate(BOX)
        assert len(mmc.sets) == n and mmc.waits == 0  # nothing moved


def test_zero_makes_the_present_edge_the_setpoint() -> None:
    mmc = FakeMMC()
    cal = _cal()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        mmc.pos[P_LABEL] = P0 + 0.4  # the user refocused by 0.4 um
        z = svc.zero(cal, n_frames=5)
        assert z.p0_um == pytest.approx(P0 + 0.4)
        assert z.e0_px == pytest.approx(cal.e0_px + SLOPE * 0.4, abs=0.3)
        assert z.slope_px_per_um == cal.slope_px_per_um
        assert (z.box, z.axis, z.polarity) == (cal.box, cal.axis, cal.polarity)
        assert z.linear_range_um == cal.linear_range_um
        assert mmc.sets == [] and mmc.waits == 0  # reads P once, moves nothing
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
        # The lock on the new zero holds the new P.
        svc.start(_params(), z)
        time.sleep(1.0)
        svc.stop()
        assert mmc.sets == []


def test_offset_via_update_params_moves_the_held_p() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        svc.start(_params(), _cal())
        assert _wait(lambda: svc.current_state == "running", 5.0)
        time.sleep(0.6)
        assert mmc.sets == []  # on the zero, nothing to do
        svc.update_params(_params(offset_um=0.4))
        assert _wait(
            lambda: bool(mmc.sets) and abs(mmc.sets[-1][1] - (P0 + 0.4)) < 0.08, 8.0
        ), mmc.sets
        time.sleep(0.8)
        assert mmc.sets[-1][1] == pytest.approx(P0 + 0.4, abs=0.08)  # and it holds
        assert len(mmc.sets) <= 10
        svc.update_params(_params(offset_um=0.0))
        assert _wait(lambda: abs(mmc.sets[-1][1] - P0) < 0.08, 8.0), mmc.sets
        svc.stop()


def test_stop_restores_crisp_and_never_moves_p() -> None:
    mmc = FakeMMC()
    with _service(mmc, drift_um_per_s=0.5) as svc:
        svc.start(_params(), _cal(), led_intensity=30)
        assert mmc.props[(CRISP, "CRISP State")] == "Ready"
        assert mmc.props[(CRISP, "LED Intensity")] == "30"
        assert _wait(lambda: len(mmc.sets) >= 1, 8.0)
        svc.stop()
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
        assert svc.current_state == "stopped"
        time.sleep(0.3)  # let any in-flight request drain
        n = len(mmc.sets)
        time.sleep(0.5)
        assert len(mmc.sets) == n  # stop sends no move and none follow


def test_a_bad_calibration_is_a_fault_not_a_crash() -> None:
    mmc = FakeMMC()
    faults: list[str] = []
    with _service(mmc) as svc:
        svc.fault.connect(faults.append)
        assert svc.wait_ready()
        svc._send(
            {
                "type": "start",
                "params": _params().to_dict(),
                "calibration": {"x0": 1.0, "slope_px_per_um": 2.0},  # old schema
                "p_start_um": P0,
            }
        )
        assert _wait(lambda: faults, 5.0)
        assert "calibration" in faults[0]
        assert svc._proc.is_alive()
        assert mmc.sets == []


def test_shutdown_exits_cleanly_without_orphans() -> None:
    mmc = FakeMMC()
    svc = _service(mmc)
    assert svc.wait_ready()
    proc = svc._proc
    svc.shutdown()
    svc.shutdown()  # idempotent
    assert not proc.is_alive()
    assert proc.exitcode == 0
    assert _gone(proc)
    assert not svc._reader.is_alive()


def test_worker_exits_when_the_pipe_closes() -> None:
    """The parent vanishing (pipe EOF) must not leave an orphan worker."""
    proc, conn = launch_worker(_spec())
    try:
        assert _wait(lambda: conn.poll(0.1), 15.0)
        assert conn.recv()["type"] == "ready"
        conn.close()
        proc.join(8.0)
        assert not proc.is_alive()
        assert proc.exitcode == 0
    finally:
        if proc.is_alive():
            proc.kill()
            proc.join(2.0)


def test_shutdown_reaches_a_worker_whose_reader_is_late() -> None:
    """Regression (rig PC, 2026-10-27): with nobody reading, the worker's
    telemetry fills the pipe. A synchronous send then blocked the
    control loop, so ``shutdown`` was never read and the worker never exited."""
    proc, conn = launch_worker(_spec())
    try:
        assert _wait(lambda: conn.poll(0.1), 15.0)
        assert conn.recv()["type"] == "ready"
        conn.send({"type": "set_edge_box", "box": list(BOX)})
        conn.send({"type": "preview_start"})
        time.sleep(3.0)  # read nothing while the worker publishes
        conn.send({"type": "shutdown"})
        proc.join(8.0)
        assert not proc.is_alive()
        assert proc.exitcode == 0
    finally:
        if proc.is_alive():
            proc.kill()
            proc.join(2.0)
        conn.close()


def test_shutdown_right_after_start_restores_crisp() -> None:
    """Regression (rig PC, 2026-10-27): shutdown before the worker reported
    "running" skipped the CRISP restore and left CRISP in Ready."""
    mmc = FakeMMC()
    svc = _service(mmc)
    assert svc.wait_ready()
    svc.start(_params(), _cal())
    assert mmc.props[(CRISP, "CRISP State")] == "Ready"
    svc.shutdown()  # no wait for the worker's "running"
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"


# --------------------------------------------------------------------------
# CRISP LED: lit only in Ready (or Lock...), off in Idle
# --------------------------------------------------------------------------


def test_led_on_sets_ready_and_intensity_and_led_off_restores_idle() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert not svc.led_is_on
        svc.led_on(40)
        assert svc.led_is_on
        assert mmc.props[(CRISP, "CRISP State")] == "Ready"
        assert mmc.props[(CRISP, "LED Intensity")] == "40"
        svc.set_led_intensity(70)  # live
        assert mmc.props[(CRISP, "LED Intensity")] == "70"
        svc.led_off()
        assert not svc.led_is_on
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"


def test_led_on_leaves_a_lit_state_alone() -> None:
    mmc = FakeMMC()
    mmc.props[(CRISP, "CRISP State")] = "Lock"
    with _service(mmc) as svc:
        svc.led_on(20)
        assert mmc.props[(CRISP, "CRISP State")] == "Lock"
        svc.led_off()
        assert mmc.props[(CRISP, "CRISP State")] == "Lock"


def test_preview_and_lock_overlap_keep_the_led_on_until_both_are_gone() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        svc.led_on(30, owner="preview")
        svc.start(_params(), _cal(), led_intensity=30)  # the lock joins
        assert mmc.props[(CRISP, "CRISP State")] == "Ready"
        svc.stop()  # lock gone, preview still on
        assert svc.led_is_on
        assert mmc.props[(CRISP, "CRISP State")] == "Ready"
        svc.led_off("preview")
        assert not svc.led_is_on
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
    # And the other order: preview leaves first, the lock's stop restores.
    mmc = FakeMMC()
    with _service(mmc) as svc:
        svc.led_on(30, owner="preview")
        svc.start(_params(), _cal())
        svc.led_off("preview")
        assert mmc.props[(CRISP, "CRISP State")] == "Ready"
        svc.stop()
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"


def test_shutdown_restores_a_lit_led() -> None:
    mmc = FakeMMC()
    svc = _service(mmc)
    svc.led_on(30, owner="preview")
    svc.led_on(30, owner="manual")
    assert mmc.props[(CRISP, "CRISP State")] == "Ready"
    svc.shutdown()
    assert mmc.props[(CRISP, "CRISP State")] == "Idle"


def test_led_calls_are_a_no_op_without_a_crisp_device() -> None:
    mmc = FakeMMC()
    svc = SpotFocusService(mmc, P_LABEL, None, _spec(), ready_timeout_s=20.0)
    with svc:
        svc.led_on(50)
        assert not svc.led_is_on
        svc.set_led_intensity(60)
        svc.led_off()
        assert svc.crisp_label is None
        assert svc.crisp_state() is None
        assert mmc.props[(CRISP, "CRISP State")] == "Idle"
        assert mmc.props[(CRISP, "LED Intensity")] == "0"


# --------------------------------------------------------------------------
# Live view: preview, shared memory, camera controls
# --------------------------------------------------------------------------


def _next_frame(svc: SpotFocusService, last: int | None, timeout: float = 8.0):
    """Poll ``latest_frame`` until a frame newer than *last* arrives."""
    out: list = []

    def got() -> bool:
        f = svc.latest_frame(last)
        if f is not None:
            out.append(f)
        return bool(out)

    assert _wait(got, timeout, 0.005), "no new frame"
    return out[0]


def test_ready_info_carries_the_camera_contract() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        info = svc.camera_info
        for key in (
            "shm sensor bits model serial exposure_range_ms gain_range roi "
            "exposure_ms gain"
        ).split():
            assert key in info, key
        assert info["sensor"] == (640, 480)
        assert info["roi"] == (0, 0, 640, 480)
        assert info["bits"] == 16
        assert info["exposure_range_ms"] == pytest.approx((0.105, 15000.0))
        assert info["gain_range"] == (100, 5000)
        assert isinstance(info["shm"], str) and info["shm"]
        assert svc.preview_active is False
        assert svc.latest_frame() is None  # nothing written while idle
        name = info["shm"]
    # The worker unlinked the segment on exit (POSIX; a no-op on Windows).
    if os.name != "nt":
        with pytest.raises(FileNotFoundError):
            FrameBuffer.attach(name)


def test_preview_delivers_frames_and_the_overlay_sits_on_the_edge() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        states: list[str] = []
        svc.state.connect(states.append)
        assert svc.wait_ready()
        svc.start_preview()
        assert svc.preview_active
        seqs: list[int] = []
        last = None
        frames = []
        for _ in range(12):
            f = _next_frame(svc, last)
            seqs.append(f.seq)
            frames.append(f)
            last = f.seq
        assert seqs == sorted(set(seqs)) and len(seqs) == 12
        f = frames[-1]
        assert f.image.shape == (480, 640) and f.image.dtype == np.uint16
        assert f.roi == (0, 0, 640, 480) and f.sensor == (640, 480)
        assert f.bits == 16
        assert f.max_value == int(f.image.max()) > 800
        # No edge box, no overlay.
        assert all(fr.spot is None for fr in frames)
        # With a box the overlay is the edge point, in sensor pixels.
        svc.set_edge_box(BOX)
        assert _wait(lambda: (svc.latest_frame().spot is not None), 5.0)
        x, y = svc.latest_frame().spot
        assert x == pytest.approx(EDGE_X, abs=1.0)
        assert y == pytest.approx(BOX_CENTER_Y, abs=1.0)
        assert svc.latest_frame().spot_ok
        assert svc.current_state == "preview"
        assert "preview" in states
        assert _wait(lambda: svc.last_telemetry is not None, 5.0)
        assert svc.last_telemetry["edge_px"] == pytest.approx(EDGE_X, abs=1.0)
        # Clearing the box removes the overlay again.
        svc.set_edge_box(None)
        assert _wait(lambda: svc.latest_frame().spot is None, 5.0)
        # Off: the worker stops writing.
        svc.stop_preview()
        assert svc.preview_active is False
        assert _wait(lambda: svc.current_state == "idle", 5.0)
        time.sleep(0.3)
        last = svc.latest_frame().seq
        time.sleep(0.4)
        assert svc.latest_frame(last) is None


def test_roi_change_moves_the_frame_roi_and_keeps_sensor_coordinates() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        changed: list[dict] = []
        svc.camera_changed.connect(changed.append)
        full = svc.measure(5, box=BOX)  # the reference, at the full frame
        assert full["n"] == 5
        svc.start_preview()
        svc.set_edge_box(BOX)
        svc.set_camera(roi=(200, 150, 200, 160))
        assert _wait(
            lambda: (f := svc.latest_frame()) is not None
            and f.roi == (200, 150, 200, 160),
            8.0,
        )
        # After the change every frame has the new shape; the edge is where it
        # was on the sensor, not where it is in the cropped image.
        assert _wait(
            lambda: (f := svc.latest_frame()) is not None
            and f.spot is not None
            and f.roi == (200, 150, 200, 160),
            8.0,
        )
        f = svc.latest_frame()
        assert f.image.shape == (160, 200)
        assert f.sensor == (640, 480)
        assert f.spot == pytest.approx((EDGE_X, BOX_CENTER_Y), abs=1.0)
        assert svc.camera_info["roi"] == (200, 150, 200, 160)
        assert changed and changed[-1]["roi"] == (200, 150, 200, 160)
        # Telemetry and measurements are in sensor pixels as well.
        assert _wait(
            lambda: (t := svc.last_telemetry) is not None
            and abs(t["x"] - EDGE_X) < 1.0
            and abs(t["y"] - BOX_CENTER_Y) < 1.0,
            5.0,
        ), svc.last_telemetry
        m = svc.measure(n_frames=5, box=BOX)
        assert m["n"] == 5
        assert m["edge_mean"] == pytest.approx(full["edge_mean"], abs=0.3)
        assert m["x_mean"] == pytest.approx(m["edge_mean"])
        assert m["y_mean"] == pytest.approx(BOX_CENTER_Y)
        assert m["roi"] == (200, 150, 200, 160)
        # A box outside the ROI is reported, not misplaced.
        out = svc.measure(n_frames=2, box=(20, 20, 80, 60))
        assert out["n"] == 0 and "outside" in out["reason"]
        # (0, 0, 0, 0) is the full frame again.
        svc.set_camera(roi=(0, 0, 0, 0))
        assert _wait(
            lambda: (f := svc.latest_frame()) is not None and f.roi == (0, 0, 640, 480),
            8.0,
        )
        assert svc.latest_frame().image.shape == (480, 640)
        assert svc.camera_info["roi"] == (0, 0, 640, 480)


def test_measure_reply_has_the_documented_fields() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        m = svc.measure(4, P0, box=BOX, return_frame=True)
        for key in (
            "p_um n edge_mean edge_sd x_mean y_mean contrast_mean angle_mean "
            "residual_mean axis polarity reason frame roi"
        ).split():
            assert key in m, key
        assert m["n"] == 4 and m["p_um"] == P0
        assert (m["axis"], m["polarity"], m["reason"]) == ("x", 1, "")
        assert m["edge_mean"] == pytest.approx(_e0(), abs=0.3)
        assert 0.0 <= m["edge_sd"] < 0.3
        assert m["contrast_mean"] > 300
        assert abs(m["angle_mean"]) < 1.0
        assert m["frame"].shape == (480, 640) and m["roi"] == (0, 0, 640, 480)
        # Without return_frame the raw frame is not sent.
        assert "frame" not in svc.measure(2, box=BOX)
        # Given the axis and polarity it measures with those (not detected).
        assert svc.measure(2, box=BOX, axis="x", polarity=1)["n"] == 2
        wrong = svc.measure(2, box=BOX, axis="x", polarity=-1)
        assert wrong["n"] == 0 and "edge lost" in wrong["reason"]


def test_exposure_gain_and_channel_apply_live() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        before = _next_frame(svc, None)
        # The fake scales the patch with exposure x gain, like a camera.
        svc.set_camera(exposure_ms=40.0, gain=200, channel="red")
        assert _wait(lambda: svc.camera_info["exposure_ms"] == 40.0, 5.0)
        assert svc.camera_info["gain"] == 200
        assert svc.camera_info["channel"] == "red"
        assert _wait(lambda: svc.latest_frame().max_value > 3 * before.max_value, 5.0)
        # Limits are the camera's: out-of-range requests are clamped.
        svc.set_camera(exposure_ms=1e9, gain=1)
        assert _wait(lambda: svc.camera_info["exposure_ms"] == 15000.0, 5.0)
        assert svc.camera_info["gain"] == 100


def test_a_bad_channel_is_reported_as_a_fault() -> None:
    mmc = FakeMMC()
    faults: list[str] = []
    with _service(mmc) as svc:
        svc.fault.connect(faults.append)
        assert svc.wait_ready()
        svc.set_camera(channel="blue")
        assert _wait(lambda: faults, 5.0)
        assert "channel" in faults[0]


def test_stop_returns_to_preview_and_frames_keep_flowing_while_running() -> None:
    mmc = FakeMMC()
    with _service(mmc, drift_um_per_s=0.3) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        f = _next_frame(svc, None)
        svc.start(_params(), _cal())
        assert _wait(lambda: svc.current_state == "running", 5.0)
        f2 = _next_frame(svc, f.seq)  # frames keep coming during the run
        f3 = _next_frame(svc, f2.seq)
        # While running the calibration's box is measured: the overlay is there
        # even though no preview box was set.
        assert f3.spot is not None and f3.spot_ok
        svc.stop()
        assert svc.current_state == "preview"
        assert svc.preview_active
        g = _next_frame(svc, f3.seq)  # and after it
        assert g.seq > f3.seq
        assert _wait(lambda: svc.current_state == "preview", 3.0)
        # With the preview off, stop() lands in "stopped" instead.
        svc.stop_preview()
        svc.start(_params(), _cal())
        svc.stop()
        assert svc.current_state == "stopped"


def test_closed_loop_works_inside_an_roi() -> None:
    """A calibration made at full frame still holds after the ROI changes."""
    mmc = FakeMMC()
    tele: list[dict] = []
    with _service(mmc, drift_um_per_s=0.3) as svc:
        svc.telemetry.connect(tele.append)
        assert svc.wait_ready()
        svc.set_camera(roi=(200, 150, 300, 200))
        svc.start(_params(max_rate_hz=RATE_CAP_HZ), _cal())
        time.sleep(3.0)
        svc.stop()
    late = sorted(
        abs(t["along_um"]) for t in tele[len(tele) // 2 :] if t["along_um"] is not None
    )
    assert late and late[len(late) // 2] < 0.12
    assert mmc.sets[-1][1] == pytest.approx(P0 - 0.3 * 3.0, abs=0.5)


# -- the settle ---------------------------------------------------------------


def test_settle_time_formula() -> None:
    assert settle_time_s(10.0, 50.0) == pytest.approx(0.05)  # 0.04 -> floor
    assert settle_time_s(100.0, 10.0) == pytest.approx(0.3)  # 2*0.1 + 0.1
    assert settle_time_s(0.105, 0.0) == pytest.approx(0.05)  # no fps estimate yet
    assert settle_time_s(2000.0, 0.5) == pytest.approx(6.0)


def test_settle_needs_both_four_frames_and_the_time() -> None:
    st = Settle()
    assert st.frame(0.0)  # settled from the start
    st.arm(10.0, exposure_ms=100.0, fps=10.0)  # 0.3 s
    # Plenty of time but too few frames: the first four are ignored.
    assert [st.frame(20.0) for _ in range(4)] == [False] * 4
    assert st.frame(20.0)
    # Plenty of frames but too little time.
    st.arm(30.0, exposure_ms=100.0, fps=10.0)
    assert [st.frame(30.1) for _ in range(10)] == [False] * 10
    assert not st.frame(30.29)
    assert st.frame(30.31)


def _inproc_worker(**spec_kw: object):
    """A ``_Worker`` driven by hand (no process, no loop) with a fake camera."""
    import multiprocessing as mp

    r1, s1 = mp.Pipe(duplex=False)
    r2, s2 = mp.Pipe(duplex=False)
    w = _Worker(Link(r1, s2), _spec(**spec_kw))
    w.source = make_source(w.spec)
    w.source.open()
    w.refresh_camera()
    return w, (r1, s1, r2, s2)


def _close(w: _Worker, pipes: tuple) -> None:
    w.outbox.close()
    for c in pipes:
        c.close()


def _messages(pipes: tuple, wait: float = 0.5) -> list[dict]:
    """Everything the in-process worker has sent so far."""
    out: list[dict] = []
    while pipes[2].poll(wait):
        out.append(pipes[2].recv())
        wait = 0.05
    return out


def _start_msg(**params: object) -> dict:
    return {
        "params": _params(**params).to_dict(),
        "calibration": _cal().to_dict(),
        "p_start_um": P0,
    }


def test_settle_ignores_early_frames_for_control() -> None:
    """After a camera change the stale frames must not count as 'edge lost'."""
    w, pipes = _inproc_worker()
    try:
        w.on_start(_start_msg(strikes=1))
        blank = np.full((480, 640), 100, dtype=np.uint16)  # the camera's stale frame
        good = w.source.render(0.0)
        w.arm_settle(10.0)  # 10 ms exposure, so the time criterion is 0.05 s
        # Four stale frames, even long after the time criterion, are ignored:
        # with strikes=1 a single counted blank frame would fault the loop.
        for k in range(4):
            w.on_frame(blank, float(k), 11.0 + k)
        assert w.state == "running" and not w.ctl.fault
        # Time not yet up: still ignored, however many frames arrived.
        w.arm_settle(20.0)
        for k in range(10):
            w.on_frame(good, 20.0 + k, 20.01)
        w.on_frame(blank, 31.0, 20.02)
        assert w.state == "running" and not w.ctl.fault
        # Once both hold, a blank frame is a real strike.
        w.on_frame(blank, 32.0, 20.2)
        assert w.state == "fault"
    finally:
        _close(w, pipes)


def test_led_off_faults_after_the_strikes_and_p_is_held() -> None:
    w, pipes = _inproc_worker()
    try:
        w.on_start(_start_msg(strikes=3))
        good = w.source.render(0.0)
        w.source.set_level(0.0)  # LED off: only the pedestal and noise are left
        dark = w.source.render(0.0)
        # Bad frames below the strike count are skipped; a good one resets them.
        for k, img in enumerate([dark, dark, good, dark, dark]):
            w.on_frame(img, float(k), 1.0 + k)
        assert w.state == "running" and not w.ctl.fault
        w.on_frame(dark, 10.0, 11.0)  # the third in a row
        assert w.state == "fault"
        w.on_frame(dark, 11.0, 12.0)  # a faulted loop ignores later frames
        msgs = _messages(pipes)
        faults = [m for m in msgs if m["type"] == "fault"]
        assert len(faults) == 1 and "edge lost" in faults[0]["reason"]
        assert not [m for m in msgs if m["type"] == "move"]
        assert w.n_moves == 0 and w.p_cmd == P0 and w.inflight is None
    finally:
        _close(w, pipes)


def test_an_offset_change_clears_the_averaging_window() -> None:
    w, pipes = _inproc_worker()
    try:
        w.on_start(_start_msg(avg_frames=50))
        good = w.source.render(0.0)
        for k in range(3):
            w.on_frame(good, float(k), 1.0 + k)
        assert len(w.window) == 3
        w.on_update_params({"params": _params(avg_frames=50, offset_um=0.3).to_dict()})
        assert len(w.window) == 0 and w.params.offset_um == 0.3
        w.on_frame(good, 5.0, 6.0)
        # The error now includes the offset: the edge is on the zero.
        assert w.window[-1] == pytest.approx(-0.3, abs=0.05)
    finally:
        _close(w, pipes)


def test_a_move_ack_arms_the_settle() -> None:
    w, pipes = _inproc_worker()
    try:
        w.on_start(_start_msg())
        w.inflight = 7
        t0 = time.monotonic()
        w.on_moved({"seq": 7, "ok": True})
        assert w.inflight is None and w.settle.seen == 0
        assert w.settle.until >= t0 + 0.05
    finally:
        _close(w, pipes)


def test_frames_of_the_wrong_shape_are_dropped_not_misplaced() -> None:
    """A frame pulled just before an ROI change has the old shape."""
    w, pipes = _inproc_worker()
    try:
        w.on_set_edge_box({"box": list(BOX)})
        w.on_set_camera({"roi": (200, 150, 200, 160)})
        assert w.roi == (200, 150, 200, 160)
        w.preview = True
        w.on_frame(np.zeros((480, 640), np.uint16), 1.0, 1.0)  # the old full frame
        assert w.last_fix is None  # not measured, not drawn at the new offset
        w.arm_settle(0.0)
        w.on_frame(w.source.render(0.0), 2.0, 2.0)
        assert w.last_fix is not None and w.last_fix.ok
        assert (w.last_fix.x, w.last_fix.y) == pytest.approx(
            (EDGE_X, BOX_CENTER_Y), abs=1.0
        )
    finally:
        _close(w, pipes)


def test_no_box_means_no_overlay_and_a_box_outside_the_roi_is_not_ok() -> None:
    w, pipes = _inproc_worker()
    try:
        w.preview = True
        w.on_frame(w.source.render(0.0), 1.0, 1.0)
        assert w.last_fix is None  # nothing to measure
        w.on_set_edge_box({"box": list(BOX)})
        w.on_frame(w.source.render(0.0), 2.0, 2.0)
        assert w.last_fix.ok
        w.on_set_edge_box({"box": None})
        assert w.last_fix is None and w.edge_box is None
        w.on_set_edge_box({"box": [1, 2, 3]})  # malformed: reported, box unchanged
        assert w.edge_box is None
        assert [m for m in _messages(pipes) if m["type"] == "error"]
    finally:
        _close(w, pipes)


def test_measure_uses_only_settled_frames_and_returns_one() -> None:
    w, pipes = _inproc_worker()
    try:
        w.on_measure(
            {"n_frames": 3, "p_um": P0, "box": list(BOX), "return_frame": True}
        )
        (m,) = [m for m in _messages(pipes) if m["type"] == "measurement"]
        assert m["n"] == 3 and (m["axis"], m["polarity"]) == ("x", 1)
        assert m["edge_mean"] == pytest.approx(_e0(), abs=0.3)
        assert m["frame"].shape == (480, 640)
        assert m["reason"] == ""
        # The first four frames after the request were thrown away.
        assert w.settle.seen >= 3 + 4
        # No box: nothing to measure, and it says why.
        w.on_measure({"n_frames": 2})
        (m,) = [m for m in _messages(pipes) if m["type"] == "measurement"]
        assert m["n"] == 0 and m["reason"] == "no edge box"
        assert np.isnan(m["edge_mean"])
    finally:
        _close(w, pipes)
