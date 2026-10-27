"""The spot focus lock's subprocess and service, with a real spawned worker.

Covers the live view too: preview frames through shared memory, live camera
settings, ROI changes in sensor coordinates and the time-based settle.

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

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.spot_focus import SpotCalibration, SpotLoopParams
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
    telemetry fills the pipe. A synchronous send then blocked the
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


def test_preview_delivers_frames_with_increasing_seq() -> None:
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
        # The overlay spot is the fitted one, in sensor pixels.
        assert _wait(lambda: (svc.latest_frame().spot is not None), 5.0)
        spot = svc.latest_frame().spot
        assert spot == pytest.approx((320.0, 240.0), abs=0.5)
        assert svc.current_state == "preview"
        assert "preview" in states
        assert _wait(lambda: svc.last_telemetry is not None, 5.0)
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
        svc.start_preview()
        svc.set_camera(roi=(200, 150, 200, 160))
        assert _wait(
            lambda: (f := svc.latest_frame()) is not None
            and f.roi == (200, 150, 200, 160),
            8.0,
        )
        # After the change every frame has the new shape; the spot is where it
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
        assert f.spot == pytest.approx((320.0, 240.0), abs=0.5)
        assert svc.camera_info["roi"] == (200, 150, 200, 160)
        assert changed and changed[-1]["roi"] == (200, 150, 200, 160)
        # Telemetry and measurements are in sensor pixels as well.
        assert _wait(
            lambda: (t := svc.last_telemetry) is not None
            and abs(t["x"] - 320.0) < 0.5
            and abs(t["y"] - 240.0) < 0.5,
            5.0,
        ), svc.last_telemetry
        m = svc.measure(n_frames=5)
        assert m["n"] == 5
        assert m["x_mean"] == pytest.approx(320.0, abs=0.3)
        assert m["y_mean"] == pytest.approx(240.0, abs=0.3)
        # (0, 0, 0, 0) is the full frame again.
        svc.set_camera(roi=(0, 0, 0, 0))
        assert _wait(
            lambda: (f := svc.latest_frame()) is not None and f.roi == (0, 0, 640, 480),
            8.0,
        )
        assert svc.latest_frame().image.shape == (480, 640)
        assert svc.camera_info["roi"] == (0, 0, 640, 480)


def test_exposure_gain_and_channel_apply_live() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        before = _next_frame(svc, None)
        # The fake scales the spot with exposure x gain, like a camera.
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


def test_calibration_works_while_previewing() -> None:
    mmc = FakeMMC()
    with _service(mmc) as svc:
        assert svc.wait_ready()
        svc.start_preview()
        first = _next_frame(svc, None)
        svc.set_camera(roi=(100, 80, 400, 320))  # the sweep also works in an ROI
        cal = svc.calibrate(span_um=2.0, steps=7, n_frames=6)
        assert cal.slope_px_per_um == pytest.approx(SLOPE, rel=0.02)
        # The setpoint is in sensor pixels, not ROI pixels.
        assert cal.x0 == pytest.approx(CAL["x0"], abs=0.3)
        assert cal.y0 == pytest.approx(CAL["y0"], abs=0.3)
        assert svc.preview_active and svc.current_state == "preview"
        assert _next_frame(svc, first.seq).seq > first.seq


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


def test_settle_ignores_early_frames_for_control() -> None:
    """After a camera change the stale frames must not count as 'spot lost'."""
    w, pipes = _inproc_worker()
    try:
        w.on_start(
            {
                "params": _params(strikes=1).to_dict(),
                "calibration": _cal().to_dict(),
                "p_start_um": P0,
            }
        )
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
        w.outbox.close()
        for c in pipes:
            c.close()


def test_a_move_ack_arms_the_settle() -> None:
    w, pipes = _inproc_worker()
    try:
        w.on_start(
            {
                "params": _params().to_dict(),
                "calibration": _cal().to_dict(),
                "p_start_um": P0,
            }
        )
        w.inflight = 7
        t0 = time.monotonic()
        w.on_moved({"seq": 7, "ok": True})
        assert w.inflight is None and w.settle.seen == 0
        assert w.settle.until >= t0 + 0.05
    finally:
        w.outbox.close()
        for c in pipes:
            c.close()


def test_frames_of_the_wrong_shape_are_dropped_not_misplaced() -> None:
    """A frame pulled just before an ROI change has the old shape."""
    w, pipes = _inproc_worker()
    try:
        w.on_set_camera({"roi": (200, 150, 200, 160)})
        assert w.roi == (200, 150, 200, 160)
        w.preview = True
        w.on_frame(np.zeros((480, 640), np.uint16), 1.0, 1.0)  # the old full frame
        assert w.last_fix is None  # not fitted, not drawn at the new offset
        w.arm_settle(0.0)
        w.on_frame(w.source.render(0.0), 2.0, 2.0)
        assert w.last_fix is not None
        assert (w.last_fix.x, w.last_fix.y) == pytest.approx((320.0, 240.0), abs=0.5)
    finally:
        w.outbox.close()
        for c in pipes:
            c.close()
