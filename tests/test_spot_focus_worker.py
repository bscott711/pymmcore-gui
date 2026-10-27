"""The spot focus lock's subprocess and service, with a real spawned worker.

A ``FakeSpotSource`` (high drift rate, short runs) stands in for the camera and
a fake mmc records every ``setPosition`` and never waits. No hardware, Qt,
pymmcore or Micro-Manager install needed; run with ``--noconftest``.
"""

from __future__ import annotations

import itertools
import math
import os
import threading
import time
from pathlib import Path
from typing import Any

import pytest

from pymmcore_gui.asi_z_stack.spot_focus import SpotCalibration, SpotLoopParams
from pymmcore_gui.asi_z_stack.spot_focus_service import SpotFocusService
from pymmcore_gui.asi_z_stack.spot_focus_worker import launch_worker

P_LABEL = "PiezoStage:P:34"
CRISP = "CRISP"
P0 = 50.0
UX, UY = 0.8, 0.6
SLOPE = 10.0
CAL = {
    "x0": 320.0,
    "y0": 240.0,
    "ux": UX,
    "uy": UY,
    "slope_px_per_um": SLOPE,
    "p0_um": P0,
}


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


def _cal() -> SpotCalibration:
    return SpotCalibration(
        x0=320.0,
        y0=240.0,
        ux=UX,
        uy=UY,
        slope_px_per_um=SLOPE,
        p0_um=P0,
        residual_px=0.0,
        n_points=11,
        created="2026-10-26T00:00:00",
    )


def _spec(**kw: object) -> dict:
    spec = {"kind": "fake", "cal_like": dict(CAL), "noise": 2.0, "fps": 50.0}
    spec.update(kw)
    return spec


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


def test_lateral_drift_is_a_fault_and_not_a_move() -> None:
    """Negative control: a purely lateral shift is not focus."""
    mmc = FakeMMC()
    faults: list[str] = []
    with _service(mmc, lateral_px_per_s=40.0) as svc:
        svc.fault.connect(faults.append)
        svc.start(_params(lateral_limit_px=8.0), _cal())
        assert _wait(lambda: faults, 8.0)
        assert "lateral" in faults[0]
        assert svc.current_state == "fault"
        assert mmc.sets == []
        assert svc.stats()["n_moves"] == 0


def test_calibration_recovers_slope_and_direction() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        cal = svc.calibrate(span_um=2.0, steps=7, n_frames=6)
        assert cal.slope_px_per_um == pytest.approx(SLOPE, rel=0.02)
        assert cal.ux == pytest.approx(UX, abs=0.01)
        assert cal.uy == pytest.approx(UY, abs=0.01)
        assert cal.x0 == pytest.approx(CAL["x0"], abs=0.3)
        assert cal.y0 == pytest.approx(CAL["y0"], abs=0.3)
        assert cal.p0_um == pytest.approx(P0)
        assert math.hypot(cal.ux, cal.uy) == pytest.approx(1.0)
    assert mmc.waits >= 7  # calibration (stopped) may wait
    assert mmc.pos[P_LABEL] == pytest.approx(P0)  # returned to the start


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
    telemetry and thumbnails fill the pipe. A synchronous send then blocked the
    control loop, so ``shutdown`` was never read and the worker never exited."""
    proc, conn = launch_worker(_spec())
    try:
        assert _wait(lambda: conn.poll(0.1), 15.0)
        assert conn.recv()["type"] == "ready"
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
