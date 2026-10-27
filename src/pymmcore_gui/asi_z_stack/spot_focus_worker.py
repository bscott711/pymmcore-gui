"""The spot-focus-lock subprocess: camera, spot measurement and the controller.

Everything that touches the camera SDK runs here, in its own OS process, so a
vendor-driver crash or stall cannot take down the GUI and the Tiger serial
line stays quiet. This process never imports Qt or pymmcore and never touches
MMCore: it only *asks* the main process to move piezo P (``move`` messages,
answered by ``moved``). See ``docs/spot_focus_lock.md``.

Started like the camera workers: a plain ``spawn`` process with a module-level
:func:`worker_main` (importing the package in the child is accepted; the worker
code itself uses neither Qt nor pymmcore and creates no QApplication).

Wire protocol: a :func:`multiprocessing.Pipe`; every message is a ``dict`` with
a ``"type"``. Main to worker: ``start``, ``stop``, ``update_params``,
``set_camera``, ``measure``, ``moved``, ``shutdown``. Worker to main: ``ready``,
``move``, ``telemetry``, ``thumb``, ``fault``, ``status``, ``measurement``,
``error``.

The loop exits on ``shutdown`` or when the pipe closes (the parent died), so no
orphan process is left behind.
"""

from __future__ import annotations

import math
import time
import traceback
from collections import deque
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from multiprocessing.connection import Connection

#: Frames discarded right after a move is acknowledged (camera pipeline latency).
SETTLE_FRAMES = 2
#: A move the main process has not acknowledged for this long is a fault.
MOVE_ACK_TIMEOUT_S = 10.0
#: Running with no new frame for this long is a fault (camera stalled).
NO_FRAME_TIMEOUT_S = 3.0
#: Telemetry and thumbnail periods.
TELEMETRY_PERIOD_S = 0.1
THUMB_PERIOD_S = 0.2
THUMB_MAX = 128
#: Per-poll wait for a frame, which also paces the loop.
FRAME_WAIT_S = 0.05


# --------------------------------------------------------------------------
# Launching (Qt-free child)
# --------------------------------------------------------------------------


def launch_worker(source_spec: dict) -> tuple[Any, Connection]:
    """Start the worker in a spawned, daemon process.

    Returns ``(process, parent_conn)``. The child end of the pipe is closed in
    this process so a dead worker shows up as EOF on ``parent_conn``.
    """
    import multiprocessing as mp

    ctx = mp.get_context("spawn")
    parent_conn, child_conn = ctx.Pipe(duplex=True)
    proc = ctx.Process(
        target=worker_main,
        args=(child_conn, dict(source_spec)),
        name="spot-focus-worker",
        daemon=True,
    )
    proc.start()
    child_conn.close()
    return proc, parent_conn


# --------------------------------------------------------------------------
# The worker
# --------------------------------------------------------------------------


def _params_from(obj: Any) -> Any:
    from .spot_focus import SpotLoopParams

    return obj if isinstance(obj, SpotLoopParams) else SpotLoopParams.from_dict(obj)


def _cal_from(obj: Any) -> Any:
    from .spot_focus import SpotCalibration

    return obj if isinstance(obj, SpotCalibration) else SpotCalibration.from_dict(obj)


def _thumb(frame: Any, x: float, y: float) -> Any:
    """Return a uint8 crop (at most 128x128) of *frame* centred on (x, y)."""
    import numpy as np

    h, w = frame.shape[:2]
    half = THUMB_MAX // 2
    cx = round(min(max(x, 0.0), w - 1.0))
    cy = round(min(max(y, 0.0), h - 1.0))
    x0, y0 = (
        max(0, min(cx - half, w - THUMB_MAX)),
        max(0, min(cy - half, h - THUMB_MAX)),
    )
    crop = np.asarray(frame[y0 : y0 + THUMB_MAX, x0 : x0 + THUMB_MAX], dtype=np.float32)
    lo, hi = float(crop.min()), float(crop.max())
    if hi <= lo:
        return np.zeros(crop.shape, dtype=np.uint8)
    return ((crop - lo) * (255.0 / (hi - lo))).astype(np.uint8)


class _Worker:
    """The state of one worker process (single thread, one pipe)."""

    def __init__(self, conn: Connection, source_spec: dict) -> None:
        self.conn = conn
        spec = dict(source_spec)
        #: ``locate_spot`` options (roi_half, channel, bayer, saturation); not
        #: a source argument, so split off before ``make_source``.
        self.analysis: dict[str, Any] = dict(spec.pop("analysis", {}) or {})
        self.spec = spec
        self.source: Any = None
        self.running = False
        self.state = "idle"
        self.exit = False

        self.params: Any = None
        self.cal: Any = None
        self.ctl: Any = None
        self.window: deque[float] = deque()
        self.seq = 0
        self.inflight: int | None = None
        self.inflight_since = 0.0
        self.skip = 0

        self.last_ts: float | None = None
        self.last_frame_wall = time.monotonic()
        self.fps = 0.0
        self.last_fix: Any = None
        self._frame: Any = None
        self.last_along: float | None = None
        self.last_lateral: float | None = None
        self.t_telemetry = 0.0
        self.t_thumb = 0.0
        self.p_cmd: float | None = None
        self.n_moves = 0

    # -- io ---------------------------------------------------------------

    def send(self, msg: dict) -> None:
        try:
            self.conn.send(msg)
        except (OSError, ValueError, BrokenPipeError):
            self.exit = True

    def set_state(self, state: str) -> None:
        self.state = state
        self.send({"type": "status", "state": state})

    def fault(self, reason: str) -> None:
        if self.state == "fault":
            return
        self.running = False
        self.inflight = None
        self.set_state("fault")
        self.send({"type": "fault", "reason": reason})

    # -- measurement ------------------------------------------------------

    def locate(self, frame: Any, min_amplitude: float = 0.0) -> Any:
        from .spot_focus import locate_spot

        return locate_spot(
            frame,
            roi_half=int(self.analysis.get("roi_half", 50)),
            channel=self.analysis.get("channel", "sum"),
            bayer=self.analysis.get("bayer", "RGGB"),
            saturation=self.analysis.get("saturation"),
            min_amplitude=min_amplitude,
        )

    # -- messages ---------------------------------------------------------

    def handle(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "shutdown":
            self.exit = True
        elif kind == "start":
            self.on_start(msg)
        elif kind == "stop":
            self.running = False
            self.inflight = None
            if self.state != "fault":
                self.set_state("stopped")
        elif kind == "update_params":
            self.on_update_params(msg)
        elif kind == "set_camera":
            self.on_set_camera(msg)
        elif kind == "measure":
            self.on_measure(msg)
        elif kind == "moved":
            self.on_moved(msg)

    def on_start(self, msg: dict) -> None:
        from .spot_focus import SpotFocusController

        self.params = _params_from(msg["params"])
        self.cal = _cal_from(msg["calibration"])
        p_start = float(msg["p_start_um"])
        self.ctl = SpotFocusController(self.params, p_start)
        self.p_cmd = p_start
        self.n_moves = 0
        self.window.clear()
        self.inflight = None
        self.skip = 0
        self.last_along = self.last_lateral = None
        self.source.set_actuator_um(p_start)
        self.last_frame_wall = time.monotonic()
        self.running = True
        self.set_state("running")

    def on_update_params(self, msg: dict) -> None:
        self.params = _params_from(msg["params"])
        if self.ctl is not None:
            self.ctl.params = self.params

    def on_set_camera(self, msg: dict) -> None:
        try:
            if msg.get("exposure_ms") is not None:
                self.source.set_exposure_ms(float(msg["exposure_ms"]))
            if msg.get("gain") is not None:
                self.source.set_gain(int(msg["gain"]))
            if msg.get("roi") is not None:
                self.source.set_roi(*[int(v) for v in msg["roi"]])
        except Exception as exc:
            self.send(
                {
                    "type": "error",
                    "message": f"set_camera: {exc}",
                    "traceback": traceback.format_exc(),
                }
            )

    def on_moved(self, msg: dict) -> None:
        if msg.get("seq") != self.inflight:
            return
        self.inflight = None
        if not msg.get("ok", False) and not msg.get("dropped", False):
            self.fault("move failed")
            return
        self.window.clear()
        self.skip = SETTLE_FRAMES

    def on_measure(self, msg: dict) -> None:
        """Average *n_frames* fresh spot fixes at P = *p_um* (loop stopped)."""
        import numpy as np

        n = max(1, int(msg.get("n_frames", 10)))
        p_um = msg.get("p_um")
        if p_um is not None:
            self.source.set_actuator_um(float(p_um))
        t0 = time.monotonic()
        xs: list[float] = []
        ys: list[float] = []
        amps: list[float] = []
        seen: float | None = None
        skip = SETTLE_FRAMES
        deadline = t0 + float(msg.get("timeout_s", 5.0 + n / 5.0))
        while len(xs) < n and time.monotonic() < deadline and not self.exit:
            got = self.source.latest(FRAME_WAIT_S)
            if got is None:
                continue
            frame, ts = got
            if ts == seen:
                continue
            seen = ts
            if skip > 0:
                skip -= 1
                continue
            fix = self.locate(frame)
            if fix.ok:
                xs.append(fix.x)
                ys.append(fix.y)
                amps.append(fix.amplitude)
        a = lambda v: np.asarray(v, dtype=float)  # noqa: E731
        reply: dict[str, Any] = {"type": "measurement", "p_um": p_um, "n": len(xs)}
        for name, vals in (("x", xs), ("y", ys), ("amplitude", amps)):
            reply[f"{name}_mean"] = float(a(vals).mean()) if vals else math.nan
            reply[f"{name}_sd"] = float(a(vals).std()) if vals else math.nan
        self.send(reply)

    # -- per-frame --------------------------------------------------------

    def on_frame(self, frame: Any, ts: float, now: float) -> None:
        from .spot_focus import focus_error

        min_amp = self.params.min_amplitude if self.params is not None else 0.0
        fix = self.locate(frame, min_amplitude=min_amp)
        self.last_fix = fix
        self._frame = frame
        if self.cal is not None and fix.ok:
            self.last_along, self.last_lateral = focus_error(fix, self.cal)
        else:
            self.last_along = self.last_lateral = None
        if not self.running:
            return
        if self.skip > 0:
            self.skip -= 1
            return
        ctl, params = self.ctl, self.params
        if fix.saturated:
            ctl.bad("saturated")
        elif not fix.ok:
            ctl.bad(fix.reason or "spot lost")
        elif abs(self.last_lateral) > params.lateral_limit_px:
            ctl.bad("lateral drift")
        else:
            ctl.good()
            self.window.append(self.last_along)
        if ctl.fault:
            self.fault(str(ctl.fault))
            return
        if self.inflight is not None or len(self.window) < params.avg_frames:
            return
        mean_along = sum(self.window) / len(self.window)
        self.window.clear()
        target = ctl.update(mean_along, now)
        if ctl.fault:
            self.fault(str(ctl.fault))
            return
        if target is None:
            return
        self.seq += 1
        self.inflight = self.seq
        self.inflight_since = now
        self.p_cmd = float(target)
        self.n_moves = ctl.n_moves
        self.source.set_actuator_um(float(target))
        self.send(
            {
                "type": "move",
                "seq": self.seq,
                "target_um": float(target),
                "along_um": float(mean_along),
            }
        )

    def publish(self, now: float) -> None:
        fix = self.last_fix
        if fix is None:
            return
        if now - self.t_telemetry >= TELEMETRY_PERIOD_S:
            self.t_telemetry = now
            self.send(
                {
                    "type": "telemetry",
                    "t": now,
                    "x": fix.x,
                    "y": fix.y,
                    "along_um": self.last_along,
                    "lateral_px": self.last_lateral,
                    "amplitude": fix.amplitude,
                    "fps": self.fps,
                    "p_cmd_um": self.p_cmd if self.ctl is None else self.ctl.p_cmd_um,
                    "n_moves": 0 if self.ctl is None else self.ctl.n_moves,
                    "ok": bool(fix.ok),
                    "reason": fix.reason,
                }
            )
        if now - self.t_thumb >= THUMB_PERIOD_S and fix.ok:
            self.t_thumb = now
            self.send({"type": "thumb", "image": _thumb(self._frame, fix.x, fix.y)})

    # -- main loop --------------------------------------------------------

    def run(self) -> None:
        from .toupcam_source import make_source

        try:
            self.source = make_source(self.spec)
            self.source.open()
        except Exception as exc:
            self.send(
                {
                    "type": "error",
                    "message": f"camera open failed: {exc}",
                    "traceback": traceback.format_exc(),
                }
            )
            return
        try:
            self.send(
                {
                    "type": "ready",
                    "info": dict(self.source.info()),
                }
            )
            self.send({"type": "status", "state": "idle"})
            self.loop()
        except Exception as exc:
            self.send(
                {
                    "type": "error",
                    "message": str(exc),
                    "traceback": traceback.format_exc(),
                }
            )
        finally:
            try:
                self.source.close()
            except Exception:
                pass
            try:
                self.conn.close()
            except Exception:
                pass

    def loop(self) -> None:
        while not self.exit:
            try:
                while self.conn.poll(0):
                    self.handle(self.conn.recv())
                    if self.exit:
                        return
            except (EOFError, OSError):
                return  # parent gone or pipe closed
            now = time.monotonic()
            if (
                self.running
                and self.inflight is not None
                and now - self.inflight_since > MOVE_ACK_TIMEOUT_S
            ):
                self.fault("move not acknowledged")
            got = self.source.latest(FRAME_WAIT_S)
            now = time.monotonic()
            if got is None:
                if self.running and now - self.last_frame_wall > NO_FRAME_TIMEOUT_S:
                    self.fault("no frames from camera")
                continue
            frame, ts = got
            if ts == self.last_ts:
                continue
            dt = now - self.last_frame_wall
            self.last_ts = ts
            self.last_frame_wall = now
            if dt > 0:
                inst = 1.0 / dt
                self.fps = inst if self.fps == 0.0 else 0.9 * self.fps + 0.1 * inst
            self.on_frame(frame, ts, now)
            self.publish(now)


def worker_main(conn: Connection, source_spec: dict) -> None:
    """Subprocess entry point (module-level so ``spawn`` can pickle it)."""
    _Worker(conn, source_spec).run()
