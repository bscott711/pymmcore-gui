"""The spot-focus-lock subprocess: camera, edge measurement and the controller.

Everything that touches the camera SDK runs here, in its own OS process, so a
vendor-driver crash or stall cannot take down the GUI and the Tiger serial
line stays quiet. This process never imports Qt or pymmcore and never touches
MMCore: it only *asks* the main process to move piezo P (``move`` messages,
answered by ``moved``). See ``docs/spot_focus_lock.md``.

Started like the camera workers: a plain ``spawn`` process with a module-level
:func:`worker_main` (importing the package in the child is accepted; the worker
code itself uses neither Qt nor pymmcore and creates no QApplication).

Wire protocol: two one-way :func:`multiprocessing.Pipe` s (commands in, events
out; see :class:`Link`); every message is a ``dict`` with a ``"type"``.
Main to worker: ``start``, ``stop``, ``preview_start``, ``preview_stop``,
``update_params``, ``set_camera``, ``set_edge_boxes`` (``set_edge_box`` is the
one-box alias), ``set_calibration``, ``measure``, ``moved``, ``shutdown``.
Worker to main: ``ready``, ``move``, ``telemetry``, ``camera``, ``fault``,
``status``, ``measurement``, ``error``.

One or two edge boxes are measured on every frame and fused into one focus
error (:func:`~.spot_focus.fuse_edges`); a ``set_calibration`` lets the preview
show that error before the lock starts.

Frames never travel on the pipe (except the one raw frame a ``measure`` with
``return_frame`` returns): while previewing, running or measuring every frame
goes to a shared-memory slot (:mod:`.spot_frame_shm`) whose name is in the
``ready`` info. **All edge positions are in sensor pixels**: the ROI offset is
passed to the measurement, so a calibration survives ROI changes.

The loop exits on ``shutdown`` or when the pipe closes (the parent died), so no
orphan process is left behind.

The control loop never blocks on output: :class:`_Outbox` sends from its own
thread. A Windows pipe holds only a few kB, so a synchronous send of telemetry
stalls as soon as the main process reads late, and the loop then misses
``shutdown`` (seen on the rig PC, 2026-10-27).
"""

from __future__ import annotations

import logging
import math
import threading
import time
import traceback
from collections import deque
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from multiprocessing.connection import Connection

logger = logging.getLogger(__name__)

#: After a move acknowledgement or a camera change the camera delivers stale
#: frames. Ignore frames until at least this many have arrived ...
SETTLE_MIN_FRAMES = 4
#: ... and until max(2 * exposure + 1 / fps, SETTLE_MIN_S) seconds have passed.
SETTLE_MIN_S = 0.05
#: A move the main process has not acknowledged for this long is a fault.
MOVE_ACK_TIMEOUT_S = 10.0
#: Running with no new frame for this long is a fault (camera stalled); the
#: limit grows with the exposure (see ``_Worker.no_frame_limit_s``).
NO_FRAME_TIMEOUT_S = 3.0
#: Telemetry period.
TELEMETRY_PERIOD_S = 0.1
#: In preview the edge is measured for the overlay at most this often (5 Hz).
PREVIEW_FIT_PERIOD_S = 0.2
#: A fit older than this is not drawn on a newer frame.
SPOT_MAX_AGE_S = 0.5
#: Per-poll wait for a frame, which also paces the loop.
FRAME_WAIT_S = 0.05
#: Camera channels ``set_camera`` accepts.
CHANNELS = ("sum", "red")
#: At most this many edge boxes are measured at once.
MAX_EDGES = 2


def settle_time_s(exposure_ms: float, fps: float) -> float:
    """Seconds to wait after a change: ``max(2 * exposure + 1 / fps, 0.05)``."""
    frame_s = 1.0 / fps if fps > 0 else 0.0
    return max(2.0 * exposure_ms / 1000.0 + frame_s, SETTLE_MIN_S)


class Settle:
    """Decides which frames to ignore after a move ack or a camera change.

    A frame is usable only when more than :data:`SETTLE_MIN_FRAMES` frames have
    arrived since :meth:`arm` (so the first four are ignored) **and**
    :func:`settle_time_s` has elapsed since :meth:`arm`. Replaces the old
    fixed count of two frames, which was shorter than the camera's pipeline.
    """

    def __init__(self) -> None:
        self.seen = SETTLE_MIN_FRAMES + 1
        self.until = 0.0

    def arm(self, now: float, exposure_ms: float, fps: float) -> None:
        """Start a settle period at *now*."""
        self.seen = 0
        self.until = now + settle_time_s(exposure_ms, fps)

    def frame(self, now: float) -> bool:
        """Count a frame that arrived at *now*; True if it may be used."""
        self.seen += 1
        return self.seen > SETTLE_MIN_FRAMES and now >= self.until

    def remaining_s(self, now: float) -> float:
        """Seconds of the time criterion left (0 once it has passed)."""
        return max(self.until - now, 0.0)


# --------------------------------------------------------------------------
# Launching (Qt-free child)
# --------------------------------------------------------------------------


class Link:
    """One end of the worker link: sends on one one-way pipe, receives on another.

    Two one-way pipes instead of one duplex pipe, so the thread that receives
    and the thread that sends never share a :class:`Connection` object.
    """

    def __init__(self, rx: Connection, tx: Connection) -> None:
        self.rx = rx
        self.tx = tx

    def send(self, msg: dict) -> None:
        """Send *msg* (may block while the pipe is full)."""
        self.tx.send(msg)

    def poll(self, timeout: float = 0.0) -> bool:
        """True if a message is waiting."""
        return self.rx.poll(timeout)

    def recv(self) -> Any:
        """Receive one message."""
        return self.rx.recv()

    def close(self) -> None:
        """Close both pipes."""
        for c in (self.tx, self.rx):
            try:
                c.close()
            except OSError:
                pass


def launch_worker(source_spec: dict) -> tuple[Any, Link]:
    """Start the worker in a spawned, daemon process.

    Returns ``(process, link)``. The child's pipe ends are closed in this
    process so a dead worker shows up as EOF on ``link.recv``.
    """
    import multiprocessing as mp

    ctx = mp.get_context("spawn")
    cmd_rx, cmd_tx = ctx.Pipe(duplex=False)
    evt_rx, evt_tx = ctx.Pipe(duplex=False)
    proc = ctx.Process(
        target=worker_main,
        args=(Link(cmd_rx, evt_tx), dict(source_spec)),
        name="spot-focus-worker",
        daemon=True,
    )
    proc.start()
    cmd_rx.close()
    evt_tx.close()
    return proc, Link(evt_rx, cmd_tx)


class _Outbox:
    """Sends the worker's messages from its own thread.

    Control messages (``move``, ``fault``, ``status`` ...) queue in order and are
    never dropped. ``telemetry`` keeps only the newest, so a slow reader costs
    stale telemetry, never a stalled control loop.
    """

    _LATEST = ("telemetry",)

    def __init__(self, link: Link, on_broken: Any) -> None:
        self._link = link
        self._on_broken = on_broken
        self._cv = threading.Condition()
        self._control: deque[dict] = deque()
        self._latest: dict[str, dict] = {}
        self._closing = False
        self._thread = threading.Thread(
            target=self._run, name="spot-focus-outbox", daemon=True
        )
        self._thread.start()

    def put(self, msg: dict) -> None:
        """Queue *msg*; never blocks."""
        with self._cv:
            if msg.get("type") in self._LATEST:
                self._latest[msg["type"]] = msg
            else:
                self._control.append(msg)
            self._cv.notify()

    def close(self, timeout_s: float = 2.0) -> None:
        """Send the queued control messages (bounded wait), then stop."""
        with self._cv:
            self._closing = True
            self._latest.clear()
            self._cv.notify()
        self._thread.join(timeout_s)

    def _next(self) -> dict | None:
        with self._cv:
            while not (self._control or self._latest or self._closing):
                self._cv.wait()
            if self._control:
                return self._control.popleft()
            for kind in self._LATEST:
                if kind in self._latest:
                    return self._latest.pop(kind)
            return None  # closing and drained

    def _run(self) -> None:
        while (msg := self._next()) is not None:
            try:
                self._link.send(msg)
            except (OSError, ValueError, BrokenPipeError):
                self._on_broken()
                return


# --------------------------------------------------------------------------
# The worker
# --------------------------------------------------------------------------


def _params_from(obj: Any) -> Any:
    from .spot_focus import SpotLoopParams

    return obj if isinstance(obj, SpotLoopParams) else SpotLoopParams.from_dict(obj)


def _cal_from(obj: Any) -> Any:
    """A ``FocusCalibration`` from one, or from its dict (also a single-edge dict)."""
    from .spot_focus import FocusCalibration

    return obj if isinstance(obj, FocusCalibration) else FocusCalibration.from_dict(obj)


def _edge_summary(
    rows: list[tuple[float, ...]],
    axis: Any,
    polarity: Any,
    reason: str,
    detector: str = "",
    sigma_px: float = math.nan,
) -> dict[str, Any]:
    """Mean and SD of the good fixes of one box, in the ``measurement`` layout.

    *rows* hold, per good frame: edge, x, y, contrast, angle, residual, edge
    width. *detector* and *sigma_px* are what the box was measured with.
    """
    import numpy as np

    data = np.asarray(rows, dtype=float).reshape(len(rows), 7)
    mean = data.mean(axis=0) if rows else np.full(7, math.nan)
    sd = float(data[:, 0].std(ddof=1)) if len(rows) > 1 else 0.0
    widths = data[np.isfinite(data[:, 6]), 6]
    return {
        "n": len(rows),
        "edge_mean": float(mean[0]),
        "edge_sd": sd if rows else math.nan,
        "x_mean": float(mean[1]),
        "y_mean": float(mean[2]),
        "contrast_mean": float(mean[3]),
        "angle_mean": float(mean[4]),
        "residual_mean": float(mean[5]),
        "edge_width_mean": float(widths.mean()) if widths.size else math.nan,
        "detector": detector,
        "detector_sigma_px": sigma_px,
        "axis": axis,
        "polarity": polarity,
        "reason": "" if rows else (reason or "no frames from camera"),
    }


class _Worker:
    """The state of one worker process (single thread, one pipe)."""

    def __init__(self, conn: Link, source_spec: dict) -> None:
        self.conn = conn
        self.outbox = _Outbox(conn, self._broken)
        spec = dict(source_spec)
        #: ``measure_edge`` options (channel, bayer, saturation, detector,
        #: detector_sigma_px); not a source argument, so split off before
        #: ``make_source``.
        self.analysis: dict[str, Any] = dict(spec.pop("analysis", {}) or {})
        #: Saturation ceiling from the camera's bit depth (12-bit data arrives in
        #: uint16); None falls back to the dtype maximum in ``measure_edge``.
        self.saturation: int | None = None
        self.spec = spec
        self.source: Any = None
        self.running = False
        self.preview = False
        self.state = "idle"
        self.exit = False

        self.params: Any = None
        self.cal: Any = None
        self.ctl: Any = None
        self.window: deque[float] = deque()
        self.seq = 0
        self.inflight: int | None = None
        self.inflight_since = 0.0
        self.settle = Settle()

        # Camera state, mirrored from ``source.info()`` after every change.
        self.sensor: tuple[int, int] = (0, 0)
        self.roi: tuple[int, int, int, int] = (0, 0, 0, 0)
        self.bits = 8
        self.exposure_ms = 10.0
        self.gain = 100
        self.shm: Any = None

        self.last_ts: float | None = None
        self.last_frame_wall = time.monotonic()
        self.fps = 0.0
        #: The edge boxes of the preview overlay (sensor px, 0-2), set by the panel.
        self.edge_boxes: list[tuple[int, ...]] = []
        #: The newest fix of every measured box; ``last_fix`` is the first.
        self.fixes: list[Any] = []
        self.last_fix: Any = None
        #: The fusion of ``fixes`` (None when no calibration applies to them).
        self.fused: Any = None
        self.fix_time = -math.inf
        self.t_fit = -math.inf
        #: The fused error in um (offset applied); None when unusable.
        self.last_along: float | None = None
        #: Why the newest measurement is unusable for control ("" when fine).
        self.last_reason = ""
        #: Informational (not a strike), e.g. "edge 2 lost (...); using edge 1".
        self.last_note = ""
        #: Per box: why that edge is not used ("" when it is).
        self.edge_reasons: list[str] = []
        self.last_max = 0
        self.last_sat_frac = 0.0
        self.t_telemetry = 0.0
        self.p_cmd: float | None = None
        self.n_moves = 0
        self._shm_warned = False

    # -- io ---------------------------------------------------------------

    def send(self, msg: dict) -> None:
        """Queue *msg* for the outbox thread; never blocks the loop."""
        self.outbox.put(msg)

    def _broken(self) -> None:
        self.exit = True  # the parent is gone

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

    def error(self, message: str) -> None:
        self.send(
            {"type": "error", "message": message, "traceback": traceback.format_exc()}
        )

    # -- camera -----------------------------------------------------------

    def refresh_camera(self) -> dict[str, Any]:
        """Re-read the source's state; return the full camera info dict."""
        info = dict(self.source.info())
        size = info.get("size") or (0, 0)
        sensor = info.get("sensor") or size
        self.sensor = (int(sensor[0]), int(sensor[1]))
        roi = info.get("roi") or (0, 0, *self.sensor)
        self.roi = tuple(int(v) for v in roi)  # type: ignore[assignment]
        bits = info.get("bits")
        self.bits = int(bits) if bits else 16
        self.saturation = (1 << self.bits) - 1
        self.exposure_ms = float(info.get("exposure_ms", self.exposure_ms))
        self.gain = int(info.get("gain", self.gain))
        exp_range = getattr(self.source, "exposure_range_ms", None)
        gain_range = getattr(self.source, "gain_range", None)
        info.update(
            sensor=self.sensor,
            roi=self.roi,
            bits=self.bits,
            exposure_ms=self.exposure_ms,
            gain=self.gain,
            exposure_range_ms=tuple(exp_range()) if exp_range else None,
            gain_range=tuple(gain_range()) if gain_range else None,
            channel=self.analysis.get("channel", "sum"),
            detector=self.detector_setting[0],
            detector_sigma_px=self.detector_setting[1],
        )
        return info

    @property
    def detector_setting(self) -> tuple[str, float]:
        """The detector and ``gaussderiv`` width of the settings (not a calibration)."""
        from .spot_focus import DEFAULT_DETECTOR, DEFAULT_SIGMA_PX

        return (
            str(self.analysis.get("detector", DEFAULT_DETECTOR)),
            float(self.analysis.get("detector_sigma_px", DEFAULT_SIGMA_PX)),
        )

    def arm_settle(self, now: float | None = None) -> None:
        """Ignore frames until the camera has caught up with a change."""
        self.settle.arm(
            time.monotonic() if now is None else now, self.exposure_ms, self.fps
        )

    def no_frame_limit_s(self) -> float:
        """How long running may go without a frame before it is a fault."""
        return max(NO_FRAME_TIMEOUT_S, 3.0 * self.exposure_ms / 1000.0 + 2.0)

    # -- measurement ------------------------------------------------------

    def edge(
        self,
        frame: Any,
        roi: Any,
        box: Any,
        axis: str | None = None,
        polarity: int | None = None,
        detector: tuple[str, float] | None = None,
    ) -> Any:
        """Measure the edge in *box*; the result is in sensor pixels.

        *detector* is ``(name, sigma_px)``: the one of the calibration the box
        belongs to. None uses the detector setting (no calibration applies).
        """
        from .spot_focus import measure_edge

        det, sigma = detector if detector is not None else self.detector_setting
        limits: dict[str, Any] = {}
        if self.params is not None:
            limits = {
                "max_saturated_frac": self.params.max_saturated_frac,
                "min_contrast": self.params.min_contrast,
                "min_valid_fraction": self.params.min_valid_fraction,
            }
        return measure_edge(
            frame,
            box,
            roi_offset=(int(roi[0]), int(roi[1])),
            channel=self.analysis.get("channel", "sum"),
            bayer=self.analysis.get("bayer", "RGGB"),
            axis=axis,
            polarity=polarity,
            saturation=self.analysis.get("saturation", self.saturation),
            detector=det,
            sigma_px=sigma,
            **limits,
        )

    def clear_fix(self) -> None:
        """Forget the newest measurement (no overlay, no telemetry)."""
        self.fixes = []
        self.last_fix = None
        self.fused = None
        self.last_along = None
        self.last_reason = ""
        self.last_note = ""
        self.edge_reasons = []

    def set_fixes(
        self,
        fixes: list[Any],
        now: float,
        fused: Any = None,
        params: Any = None,
    ) -> None:
        """Record the newest fixes (one per box) and, if any, their fusion.

        Parameters
        ----------
        fixes : list
            The :class:`~.spot_focus.EdgeFix` of each measured box, box 1 first.
        now : float
            When they were measured.
        fused : FusedError or None
            ``fuse_edges`` of *fixes*; None when no calibration applies, so
            there is no focus error.
        params : SpotLoopParams or None
            The parameters *fused* was computed with.
        """
        self.fixes = fixes
        self.last_fix = fixes[0] if fixes else None
        self.fix_time = now
        self.fused = fused
        if fused is None:
            self.edge_reasons = [
                ("" if f.ok else (f.reason or "edge lost")) for f in fixes
            ]
            self.last_along = None
            self.last_reason = self.edge_reasons[0] if fixes else ""
            self.last_note = ""
            return
        from .spot_focus import check_fix

        self.edge_reasons = [
            check_fix(f, e, params) for f, e in zip(fixes, self.cal.edges, strict=True)
        ]
        self.last_along = fused.along_um
        self.last_reason = fused.reason
        self.last_note = fused.note

    def measure_current(self, frame: Any, roi: Any, now: float) -> None:
        """Measure the live edges and fuse them into the focus error.

        While running these are the calibration's boxes. In preview they are the
        preview boxes, and only when those are the calibration's boxes (after
        ``snap_box``) is the error computed, from the calibration held by the
        worker (``SpotLoopParams()`` until params have been sent). With no box
        there is nothing to measure and no overlay.
        """
        from .spot_focus import SpotLoopParams, fuse_edges, snap_box

        cal = self.cal
        if self.running and cal is not None:
            boxes: list[Any] = list(cal.boxes)
            fuse = True
        else:
            boxes = list(self.edge_boxes)
            fuse = (
                cal is not None
                and len(boxes) == len(cal.edges)
                and [snap_box(b) for b in boxes] == list(cal.boxes)
            )
        if not boxes:
            self.clear_fix()
            return
        # Axis and polarity are the calibration's when these are its boxes.
        specs = (
            [(e.axis, e.polarity) for e in cal.edges]
            if fuse
            else [(None, None)] * len(boxes)
        )
        # A calibration is only valid for the detector it was made with, so its
        # boxes are measured with that one, whatever the setting says now.
        dets: list[tuple[str, float] | None] = (
            [(e.detector, e.detector_sigma_px) for e in cal.edges]
            if fuse
            else [None] * len(boxes)
        )
        fixes = [
            self.edge(frame, roi, box, axis, polarity, det)
            for box, (axis, polarity), det in zip(boxes, specs, dets, strict=True)
        ]
        fused = None
        params = self.params if self.params is not None else SpotLoopParams()
        if fuse:
            fused = fuse_edges(fixes, cal, params)
        self.set_fixes(fixes, now, fused, params)

    def frame_roi(self, frame: Any) -> tuple[int, int, int, int] | None:
        """The sensor ROI of *frame*, or None if it is stale.

        A frame pulled just before an ROI change has the old shape; it is
        dropped rather than drawn at the new offset.
        """
        h, w = frame.shape[:2]
        if (self.roi[3], self.roi[2]) != (h, w):
            try:
                self.refresh_camera()
            except Exception:
                logger.exception("could not read the camera state")
            if (self.roi[3], self.roi[2]) != (h, w):
                return None
        return self.roi

    def write_frame(self, frame: Any, ts: float, roi: Any, now: float) -> None:
        """Put *frame* in the shared slot with the newest fit as its overlay."""
        import numpy as np

        if self.shm is None:
            return
        top = self.saturation if self.saturation else int(np.iinfo(frame.dtype).max)
        self.last_max = int(frame.max())
        self.last_sat_frac = float(np.count_nonzero(frame >= top)) / frame.size
        fix = self.last_fix
        spot = None
        ok = False
        if fix is not None and now - self.fix_time <= SPOT_MAX_AGE_S:
            if math.isfinite(fix.x) and math.isfinite(fix.y):
                spot = (fix.x, fix.y)
                ok = bool(fix.ok)
        try:
            self.shm.write(
                frame,
                t=ts,
                roi=roi,
                sensor=self.sensor,
                bits=self.bits,
                spot=spot,
                spot_ok=ok,
                saturated_frac=self.last_sat_frac,
                max_value=self.last_max,
            )
        except Exception:
            if not self._shm_warned:
                self._shm_warned = True
                logger.exception("could not write a frame to the shared buffer")

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
                self.set_state("preview" if self.preview else "stopped")
        elif kind == "preview_start":
            self.preview = True
            self.t_fit = -math.inf
            if not self.running and self.state != "fault":
                self.set_state("preview")
        elif kind == "preview_stop":
            self.preview = False
            if not self.running and self.state == "preview":
                self.set_state("idle")
        elif kind == "update_params":
            self.on_update_params(msg)
        elif kind == "set_camera":
            self.on_set_camera(msg)
        elif kind == "set_edge_box":
            self.on_set_edge_box(msg)
        elif kind == "set_edge_boxes":
            self.on_set_edge_boxes(msg)
        elif kind == "set_calibration":
            self.on_set_calibration(msg)
        elif kind == "measure":
            self.on_measure(msg)
        elif kind == "moved":
            self.on_moved(msg)

    def on_start(self, msg: dict) -> None:
        from .spot_focus import SpotFocusController

        self.params = _params_from(msg["params"])
        try:
            self.cal = _cal_from(msg["calibration"])
        except ValueError as exc:
            self.fault(f"bad calibration: {exc}")
            return
        from .spot_focus import detector_mismatch

        why = detector_mismatch(self.cal, *self.detector_setting)
        if why:
            self.fault(why)
            return
        p_start = float(msg["p_start_um"])
        self.ctl = SpotFocusController(
            self.params, p_start, linear_range_um=self.cal.linear_range_um
        )
        self.p_cmd = p_start
        self.n_moves = 0
        self.window.clear()
        self.inflight = None
        self.last_along = None
        self.source.set_actuator_um(p_start)
        self.last_frame_wall = time.monotonic()
        self.running = True
        self.set_state("running")

    def on_update_params(self, msg: dict) -> None:
        old = self.params
        self.params = _params_from(msg["params"])
        if self.ctl is not None:
            self.ctl.params = self.params
        if old is not None and old.offset_um != self.params.offset_um:
            self.window.clear()  # errors averaged so far used the old offset

    def on_set_edge_box(self, msg: dict) -> None:
        """Set (or clear) one preview edge box; alias of ``set_edge_boxes``."""
        box = msg.get("box")
        self.apply_edge_boxes([] if box is None else [box], "set_edge_box")

    def on_set_edge_boxes(self, msg: dict) -> None:
        """Set the preview edge boxes (0-2); an empty list removes the overlay."""
        self.apply_edge_boxes(msg.get("boxes") or [], "set_edge_boxes")

    def apply_edge_boxes(self, boxes: Any, who: str) -> None:
        """Validate and take the preview *boxes*; a bad list is reported, not set."""
        try:
            new = [tuple(int(v) for v in box) for box in boxes]
            if any(len(box) != 4 for box in new):
                raise ValueError(f"a box must be (x, y, w, h), not {boxes!r}")
            if len(new) > MAX_EDGES:
                raise ValueError(f"at most {MAX_EDGES} edge boxes, got {len(new)}")
        except (TypeError, ValueError) as exc:
            self.error(f"{who}: {exc}")
            return
        self.edge_boxes = new
        if not self.running:
            self.clear_fix()
            self.t_fit = -math.inf

    def on_set_calibration(self, msg: dict) -> None:
        """Hold a calibration (or None) so the preview can compute the error.

        Ignored while the lock runs: the controller was built from the one it
        started with. A bad dict is reported and the old calibration kept.
        """
        raw = msg.get("calibration")
        if self.running:
            logger.warning("set_calibration ignored while the lock is running")
            return
        if raw is None:
            self.cal = None
        else:
            try:
                self.cal = _cal_from(raw)
            except ValueError as exc:
                self.error(f"set_calibration: {exc}")
                return
        self.clear_fix()
        self.t_fit = -math.inf

    def on_set_camera(self, msg: dict) -> None:
        """Apply exposure, gain, channel, detector and ROI now.

        An ROI of zeros is full frame. ``detector`` (with ``detector_sigma_px``)
        is the setting used wherever no calibration applies; a running lock and
        a held calibration keep measuring with the calibration's own.
        """
        changed = False
        try:
            if msg.get("channel") is not None:
                channel = str(msg["channel"])
                if channel not in CHANNELS:
                    raise ValueError(
                        f"channel must be one of {CHANNELS}, not {channel!r}"
                    )
                if self.analysis.get("channel", "sum") != channel:
                    self.analysis["channel"] = channel
                    changed = True
            if msg.get("detector") is not None:
                from .spot_focus import EDGE_DETECTORS

                detector = str(msg["detector"])
                if detector not in EDGE_DETECTORS:
                    raise ValueError(
                        f"detector must be one of {EDGE_DETECTORS}, not {detector!r}"
                    )
                sigma = float(msg.get("detector_sigma_px") or self.detector_setting[1])
                if not (math.isfinite(sigma) and sigma > 0):
                    raise ValueError(f"detector_sigma_px must be positive, not {sigma}")
                if (detector, sigma) != self.detector_setting:
                    self.analysis["detector"] = detector
                    self.analysis["detector_sigma_px"] = sigma
                    changed = True
            if msg.get("exposure_ms") is not None:
                self.source.set_exposure_ms(float(msg["exposure_ms"]))
                changed = True
            if msg.get("gain") is not None:
                self.source.set_gain(int(msg["gain"]))
                changed = True
            if msg.get("roi") is not None:
                roi = [int(v) for v in msg["roi"]]
                if len(roi) != 4:
                    raise ValueError(f"roi must be (x, y, w, h), not {msg['roi']!r}")
                self.source.set_roi(*roi)
                changed = True
        except Exception as exc:
            self.error(f"set_camera: {exc}")
        if not changed:
            return
        try:
            info = self.refresh_camera()
        except Exception as exc:
            self.error(f"camera state: {exc}")
            return
        # The camera now delivers stale frames for a few frames, and any fit or
        # averaging window from before the change describes the old image.
        self.arm_settle()
        self.window.clear()
        self.clear_fix()
        self.t_fit = -math.inf
        info.pop("shm", None)
        self.send({"type": "camera", "info": info})

    def on_moved(self, msg: dict) -> None:
        if msg.get("seq") != self.inflight:
            return
        self.inflight = None
        if not msg.get("ok", False) and not msg.get("dropped", False):
            self.fault("move failed")
            return
        self.window.clear()
        self.arm_settle()

    def on_measure(self, msg: dict) -> None:
        """Average *n_frames* fresh edge fixes per box at P = *p_um* (loop stopped).

        ``boxes`` is a list of ``{"box", "axis", "polarity"}`` (the old single
        ``box`` / ``axis`` / ``polarity`` keys make one entry; neither means the
        preview boxes), each optionally with ``"detector"`` and
        ``"detector_sigma_px"`` (a calibration's own; the detector setting
        otherwise). All boxes are measured on the same frames. Only frames
        that pass the settle are used (they were exposed after the request, a
        "snap") and only those go to the shared buffer, so the live view shows
        what was measured. A box's *axis* / *polarity* (None = detect on its
        first good frame) are then held for the rest of the measurement.

        The reply has ``edges``, one summary per box, and the first summary's
        fields at the top level. It ends when every box has *n_frames* good
        frames, has found no good frame in ``n_frames + 6`` settled frames, or at
        the deadline.
        """
        import numpy as np

        n = max(1, int(msg.get("n_frames", 10)))
        p_um = msg.get("p_um")
        raw = msg.get("boxes")
        if raw is None:
            if msg.get("box") is not None:
                raw = [
                    {
                        "box": msg["box"],
                        "axis": msg.get("axis"),
                        "polarity": msg.get("polarity"),
                    }
                ]
            else:  # the preview boxes; axis / polarity, if any, are for the first
                raw = [
                    {"box": b, "axis": msg.get("axis"), "polarity": msg.get("polarity")}
                    if i == 0
                    else {"box": b}
                    for i, b in enumerate(self.edge_boxes)
                ]
        boxes = [d["box"] for d in raw]
        axes = [d.get("axis") for d in raw]
        pols = [d.get("polarity") for d in raw]
        dets = [
            (str(d["detector"]), float(d["detector_sigma_px"]))
            if d.get("detector")
            else self.detector_setting
            for d in raw
        ]
        want_frame = bool(msg.get("return_frame", False))
        if p_um is not None:
            self.source.set_actuator_um(float(p_um))
        t0 = time.monotonic()
        self.arm_settle(t0)
        # Per box and good frame: edge, x, y, contrast, angle, residual, width.
        rows: list[list[tuple[float, ...]]] = [[] for _ in boxes]
        reasons = ["" for _ in boxes]
        kept: Any = None
        seen: float | None = None
        bad_run = [0] * len(boxes)  # settled frames in a row with no edge
        give_up = n + SETTLE_MIN_FRAMES + 2
        frame_s = max(
            1.0 / self.fps if self.fps > 0 else 0.1, self.exposure_ms / 1000.0
        )
        default_timeout = (
            self.settle.remaining_s(t0)
            + (n + SETTLE_MIN_FRAMES + 2) * frame_s * 1.5
            + 5.0
        )
        deadline = t0 + float(msg.get("timeout_s", default_timeout))

        def finished(i: int) -> bool:
            # Complete, or no edge at all in this box: give up on it.
            return len(rows[i]) >= n or (not rows[i] and bad_run[i] >= give_up)

        while (
            boxes
            and not all(finished(i) for i in range(len(boxes)))
            and time.monotonic() < deadline
            and not self.exit
        ):
            got = self.source.latest(FRAME_WAIT_S)
            if got is None:
                continue
            frame, ts = got
            if ts == seen:
                continue
            seen = ts
            now = time.monotonic()
            roi = self.frame_roi(frame)
            if roi is None:
                continue
            if not self.settle.frame(now):
                continue
            fixes = [
                self.edge(frame, roi, box, axes[i], pols[i], dets[i])
                for i, box in enumerate(boxes)
            ]
            self.set_fixes(fixes, now)
            self.write_frame(frame, ts, roi, now)
            for i, fix in enumerate(fixes):
                if len(rows[i]) >= n:
                    continue  # complete: later frames do not change its mean
                if not fix.ok:
                    reasons[i] = fix.reason
                    bad_run[i] += 1
                    continue
                bad_run[i] = 0
                if axes[i] is None:
                    axes[i], pols[i] = fix.axis, fix.polarity
                rows[i].append(
                    (
                        fix.position,
                        fix.x,
                        fix.y,
                        fix.contrast,
                        fix.angle_deg,
                        fix.residual_px,
                        fix.edge_width_px,
                    )
                )
                if want_frame:
                    kept = np.array(frame)
        entries = [
            _edge_summary(rows[i], axes[i], pols[i], reasons[i], *dets[i])
            for i in range(len(boxes))
        ]
        head = entries[0] if entries else _edge_summary([], None, None, "no edge box")
        reply: dict[str, Any] = {
            "type": "measurement",
            "p_um": p_um,
            **head,
            "edges": entries,
            "roi": tuple(self.roi),
        }
        if kept is not None:
            reply["frame"] = kept
        self.send(reply)

    # -- per-frame --------------------------------------------------------

    def on_frame(self, frame: Any, ts: float, now: float) -> None:
        """Handle one new frame: overlay, shared buffer, then control."""
        if not (self.running or self.preview):
            return  # idle: frames are drained, not processed
        roi = self.frame_roi(frame)
        if roi is None:
            return  # stale frame from before an ROI change
        usable = self.settle.frame(now)
        # While running the edge is measured every frame; in preview at <= 5 Hz.
        if self.running or now - self.t_fit >= PREVIEW_FIT_PERIOD_S:
            self.t_fit = now
            self.measure_current(frame, roi, now)
        self.write_frame(frame, ts, roi, now)
        if not self.running:
            return
        if not usable or self.last_fix is None:
            return
        ctl, params = self.ctl, self.params
        if self.last_reason or self.last_along is None:
            ctl.bad(self.last_reason or "edge lost")
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
        if fix is None or not (self.running or self.preview):
            return
        if now - self.t_telemetry >= TELEMETRY_PERIOD_S:
            self.t_telemetry = now
            fused = self.fused
            per_edge = fused.per_edge_um if fused is not None else ()
            edges = [
                {
                    "edge_px": f.position,
                    "x": f.x,
                    "y": f.y,
                    "ok": bool(f.ok and not why),
                    "reason": why,
                    "along_um": per_edge[i] if i < len(per_edge) else None,
                }
                for i, (f, why) in enumerate(
                    zip(self.fixes, self.edge_reasons, strict=True)
                )
            ]
            sigma = None if fused is None else fused.sigma_um
            self.send(
                {
                    "type": "telemetry",
                    "t": now,
                    "edge_px": fix.position,
                    "x": fix.x,
                    "y": fix.y,
                    "along_um": self.last_along,
                    "contrast": fix.contrast,
                    "angle_deg": fix.angle_deg,
                    "residual_px": fix.residual_px,
                    "valid_fraction": fix.valid_fraction,
                    "edges": edges,
                    "sigma_um": sigma
                    if sigma is not None and math.isfinite(sigma)
                    else None,
                    "disagreement_um": None if fused is None else fused.disagreement_um,
                    # 0-based index into "edges" (panel "Edge 1" is 0); None
                    # unless the two edges disagreed and one was held alone.
                    "held_edge": None if fused is None else fused.held_edge,
                    "note": self.last_note,
                    "fps": self.fps,
                    "p_cmd_um": self.p_cmd if self.ctl is None else self.ctl.p_cmd_um,
                    "n_moves": 0 if self.ctl is None else self.ctl.n_moves,
                    "ok": (
                        fused.along_um is not None
                        if fused is not None
                        else bool(fix.ok and not self.last_reason)
                    ),
                    "reason": self.last_reason,
                    "max_value": self.last_max,
                    "saturated_frac": self.last_sat_frac,
                }
            )

    # -- main loop --------------------------------------------------------

    def open_shm(self) -> str | None:
        """Create the shared frame buffer; return an error text on failure."""
        from .spot_frame_shm import FrameBuffer

        w, h = self.sensor
        try:
            self.shm = FrameBuffer.create(max(w * h, 1) * 2)
        except Exception as exc:
            logger.exception("could not create the shared frame buffer")
            self.shm = None
            return str(exc)
        return None

    def close_shm(self) -> None:
        """Unlink the buffer's name and drop our handle (at exit)."""
        shm, self.shm = self.shm, None
        if shm is None:
            return
        try:
            shm.unlink()
        except Exception:
            logger.exception("could not unlink the shared frame buffer")
        shm.close()

    def run(self) -> None:
        from .toupcam_source import make_source

        try:
            self.source = make_source(self.spec)
            self.source.open()
            info = self.refresh_camera()
        except Exception as exc:
            self.send(
                {
                    "type": "error",
                    "message": f"camera open failed: {exc}",
                    "traceback": traceback.format_exc(),
                }
            )
            try:
                if self.source is not None:
                    self.source.close()
            except Exception:
                pass
            self.outbox.close()  # deliver the error before the process exits
            self.conn.close()
            return
        try:
            shm_error = self.open_shm()
            info["shm"] = None if self.shm is None else self.shm.name
            if shm_error:
                info["shm_error"] = shm_error
            self.send({"type": "ready", "info": info})
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
            self.outbox.close()
            # Last: on Windows the segment lives while any handle is open, and
            # the main process may still be reading its final frame.
            self.close_shm()
            self.conn.close()

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
                if (
                    self.running
                    and now - self.last_frame_wall > self.no_frame_limit_s()
                ):
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


def worker_main(conn: Link, source_spec: dict) -> None:
    """Subprocess entry point (module-level so ``spawn`` can pickle it)."""
    _Worker(conn, source_spec).run()
