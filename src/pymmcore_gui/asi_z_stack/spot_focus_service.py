"""Main-process side of the spot focus lock (Qt-free, psygnal signals).

Owns the Tiger port: it starts the camera subprocess
(:mod:`~pymmcore_gui.asi_z_stack.spot_focus_worker`), and when the worker asks
for a move it calls ``mmc.setPosition(piezo_label, target)`` with **no
``waitForDevice`` and no readback**. It never polls the Tiger while the loop
runs, never queues moves (one in flight at most), and sends no further moves
once the worker is dead or the loop has faulted. See
``docs/spot_focus_lock.md``.

**Threading:** every signal is emitted from the service's reader thread, never
from the GUI thread; a Qt slot must hop threads itself (as for
:mod:`~pymmcore_gui.asi_z_stack.image_focus_events`). A slot that raises is
logged and swallowed so it cannot stall the move path.
"""

from __future__ import annotations

import logging
import threading
import time
from queue import Empty, Queue
from typing import TYPE_CHECKING, Any

from psygnal import Signal

from .spot_focus_worker import launch_worker
from .spot_frame_shm import FrameBuffer, unlink_by_name

if TYPE_CHECKING:
    from .spot_focus import SpotCalibration, SpotLoopParams
    from .spot_frame_shm import LiveFrame

logger = logging.getLogger(__name__)

PIEZO_P_LABEL = "PiezoStage:P:34"
CRISP_STATE_PROP = "CRISP State"
CRISP_LED_PROP = "LED Intensity"
#: Settable CRISP state that stands in for a reported "locked" state on restore.
_LOCKED_STATES = frozenset({"Lock", "In Focus", "Focusing"})
DEFAULT_SOURCE_SPEC: dict[str, Any] = {"kind": "toupcam"}


class SpotFocusService:
    """Run the spot focus lock: worker process, move relay, calibration.

    Parameters
    ----------
    mmc : CMMCorePlus-like
        Needs ``setPosition``, ``getPosition``, ``waitForDevice``,
        ``getProperty`` and ``setProperty``.
    piezo_label : str
        The piezo the loop drives (default ``"PiezoStage:P:34"``).
    crisp_label : str or None
        The CRISP device. When set, :meth:`start` puts it in ``Ready`` and
        :meth:`stop` restores the previous state.
    source_spec : dict
        Passed to ``make_source`` in the worker (``{"kind": "toupcam"}`` or
        ``{"kind": "fake", ...}``).
    led_intensity : float or None
        CRISP LED intensity to set on :meth:`start` (``None`` leaves it).
    """

    telemetry = Signal(dict)
    camera_changed = Signal(dict)
    """The camera's info after a ``set_camera`` (actual exposure, gain, ROI)."""
    fault = Signal(str)
    state = Signal(str)
    moved = Signal(dict)

    def __init__(
        self,
        mmc: Any,
        piezo_label: str = PIEZO_P_LABEL,
        crisp_label: str | None = None,
        source_spec: dict | None = None,
        *,
        led_intensity: float | None = None,
        ready_timeout_s: float = 30.0,
    ) -> None:
        self._mmc = mmc
        self._piezo = piezo_label
        self._crisp = crisp_label
        self._led = led_intensity
        self._ready_timeout = ready_timeout_s
        self._spec = dict(source_spec or DEFAULT_SOURCE_SPEC)

        self._lock = threading.Lock()
        #: The GUI thread and the reader thread (``moved`` replies) both send.
        self._send_lock = threading.Lock()
        self._state = "idle"
        self._shutting_down = False
        self._move_busy = False
        self._ready = threading.Event()
        self._info: dict = {}
        self._preview = False
        #: The reader's view of the worker's shared frame buffer.
        self._fb: FrameBuffer | None = None
        self._fb_name: str | None = None
        self._fb_lock = threading.Lock()
        self._meas: Queue[dict] = Queue()
        self._prev_crisp: str | None = None
        self._last_fault: str | None = None
        self._moves: list[float] = []  # setPosition durations (ms)
        self._move_times: list[float] = []  # wall times of moves
        self._last_telemetry: dict | None = None

        self._proc, self._conn = launch_worker(self._spec)
        self._reader = threading.Thread(
            target=self._read_loop, name="spot-focus-reader", daemon=True
        )
        self._reader.start()

    # -- context manager --------------------------------------------------

    def __enter__(self) -> SpotFocusService:
        """Return self."""
        return self

    def __exit__(self, *exc: object) -> None:
        """Shut the worker down."""
        self.shutdown()

    # -- properties -------------------------------------------------------

    @property
    def current_state(self) -> str:
        """The service state: idle, preview, running, stopped or fault."""
        return self._state

    @property
    def preview_active(self) -> bool:
        """True while the live camera view is on (also during a run)."""
        return self._preview

    @property
    def camera_info(self) -> dict:
        """The camera's ``ready`` info, kept current by ``set_camera``.

        Keys: ``shm, sensor, bits, model, serial, exposure_range_ms,
        gain_range, roi, exposure_ms, gain`` (and ``channel``, ``size``).
        """
        return dict(self._info)

    @property
    def worker_pid(self) -> int | None:
        """The worker process id."""
        return self._proc.pid

    @property
    def info(self) -> dict:
        """The worker's ``ready`` payload (camera info); see :attr:`camera_info`."""
        return dict(self._info)

    @property
    def last_telemetry(self) -> dict | None:
        """The newest telemetry message, if any."""
        return self._last_telemetry

    def wait_ready(self, timeout_s: float | None = None) -> bool:
        """Block until the worker reports ``ready`` (or *timeout_s*)."""
        return self._ready.wait(self._ready_timeout if timeout_s is None else timeout_s)

    # -- control ----------------------------------------------------------

    def start(
        self,
        params: SpotLoopParams,
        calibration: SpotCalibration,
        led_intensity: float | None = None,
    ) -> None:
        """Set CRISP to Ready, read P once, and start the loop."""
        if self._state == "fault" and not self._proc.is_alive():
            raise RuntimeError("spot focus worker is dead; create a new service")
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        p_start = float(self._mmc.getPosition(self._piezo))
        self._prev_crisp = None
        if self._crisp:
            self._prev_crisp = str(self._mmc.getProperty(self._crisp, CRISP_STATE_PROP))
            if self._prev_crisp != "Ready":
                self._mmc.setProperty(self._crisp, CRISP_STATE_PROP, "Ready")
            led = self._led if led_intensity is None else led_intensity
            if led is not None:
                self._mmc.setProperty(self._crisp, CRISP_LED_PROP, led)
        self._last_fault = None
        self._moves.clear()
        self._move_times.clear()
        self._send(
            {
                "type": "start",
                "params": params.to_dict(),
                "calibration": calibration.to_dict(),
                "p_start_um": p_start,
            }
        )

    def stop(self) -> None:
        """Stop the loop and restore CRISP. Never moves P.

        Back to ``preview`` if the live view was on, else ``stopped``.
        """
        self._send({"type": "stop"})
        if self._state != "fault":
            self._set_state("preview" if self._preview else "stopped")
        self._restore_crisp()

    def start_preview(self) -> None:
        """Turn the live camera view on (frames into the shared buffer)."""
        self._preview = True
        self._send({"type": "preview_start"})
        if self._state in ("idle", "stopped"):
            self._set_state("preview")

    def stop_preview(self) -> None:
        """Turn the live camera view off. A running lock keeps running."""
        self._preview = False
        self._send({"type": "preview_stop"})
        if self._state == "preview":
            self._set_state("idle")

    def latest_frame(self, last_seq: int | None = None) -> LiveFrame | None:
        """The newest frame as a :class:`LiveFrame` copy, or None.

        None if the buffer is not attached, or nothing is newer than
        *last_seq*. Safe to call from the GUI thread.
        """
        with self._fb_lock:
            fb = self._fb
            return None if fb is None else fb.read(last_seq)

    def update_params(self, params: SpotLoopParams) -> None:
        """Change the loop parameters while running."""
        self._send({"type": "update_params", "params": params.to_dict()})

    def set_camera(
        self,
        exposure_ms: float | None = None,
        gain: int | None = None,
        roi: tuple[int, int, int, int] | None = None,
        channel: str | None = None,
    ) -> None:
        """Set exposure, gain, ROI and channel on the worker's camera, live.

        *roi* is ``(x, y, w, h)`` in sensor pixels; ``(0, 0, 0, 0)`` is full
        frame. *channel* is ``"sum"`` or ``"red"``. ``None`` leaves a value.
        """
        self._send(
            {
                "type": "set_camera",
                "exposure_ms": exposure_ms,
                "gain": gain,
                "roi": None if roi is None else tuple(int(v) for v in roi),
                "channel": channel,
            }
        )

    def shutdown(self, timeout_s: float = 5.0) -> None:
        """Stop the worker process and the reader thread. Idempotent."""
        if self._shutting_down:
            return
        self._shutting_down = True
        # Restore whenever start() changed CRISP, not only once the worker has
        # reported "running": Stop or close right after Start used to leave
        # CRISP in Ready (seen on the rig PC, 2026-10-27).
        self._restore_crisp()
        self._send({"type": "shutdown"})
        # The reader keeps draining meanwhile: a worker blocked on a full pipe
        # would never read ``shutdown``.
        self._proc.join(timeout_s)
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout_s)
        if self._proc.is_alive():
            self._proc.kill()
            self._proc.join(timeout_s)
        try:
            self._conn.close()
        except OSError:
            pass
        if threading.current_thread() is not self._reader:
            self._reader.join(timeout_s)
        self._close_frames()

    # -- calibration ------------------------------------------------------

    def calibrate(
        self, span_um: float = 2.5, steps: int = 11, n_frames: int = 20
    ) -> SpotCalibration:
        """Sweep P over *span_um* centred on the current position and fit.

        Only while the loop is stopped. Uses ``waitForDevice`` (fine here, the
        loop is not running) and returns P to where it started.
        """
        import numpy as np

        from .spot_focus import fit_spot_calibration

        if self._state == "running":
            raise RuntimeError("stop the loop before calibrating")
        # Calibrating while previewing is fine: the sweep's frames also go to
        # the live view.
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        mmc = self._mmc
        p0 = float(mmc.getPosition(self._piezo))
        was_ready = False
        if self._crisp:
            prev = str(mmc.getProperty(self._crisp, CRISP_STATE_PROP))
            was_ready = prev == "Ready"
            if not was_ready:
                mmc.setProperty(self._crisp, CRISP_STATE_PROP, "Ready")
            if self._led is not None:
                mmc.setProperty(self._crisp, CRISP_LED_PROP, self._led)
        ps = p0 + np.linspace(-span_um / 2.0, span_um / 2.0, steps)
        xs: list[float] = []
        ys: list[float] = []
        good_p: list[float] = []
        try:
            for p in ps:
                mmc.setPosition(self._piezo, float(p))
                mmc.waitForDevice(self._piezo)
                self._flush_measurements()
                self._send({"type": "measure", "n_frames": n_frames, "p_um": float(p)})
                m = self._meas.get(timeout=self._measure_timeout(n_frames))
                if m.get("n", 0) > 0:
                    good_p.append(float(p))
                    xs.append(m["x_mean"])
                    ys.append(m["y_mean"])
            return fit_spot_calibration(good_p, xs, ys, p0_um=p0)
        finally:
            mmc.setPosition(self._piezo, p0)
            mmc.waitForDevice(self._piezo)
            self._send({"type": "measure", "n_frames": 1, "p_um": p0, "timeout_s": 0.5})
            if self._crisp and not was_ready:
                self._restore_crisp(prev)

    def measure(self, n_frames: int = 20, p_um: float | None = None) -> dict:
        """One averaged measurement from the worker (loop stopped)."""
        if self._state == "running":
            raise RuntimeError("stop the loop before measuring")
        self._flush_measurements()
        self._send({"type": "measure", "n_frames": n_frames, "p_um": p_um})
        return self._meas.get(timeout=self._measure_timeout(n_frames))

    # -- stats ------------------------------------------------------------

    def stats(self) -> dict:
        """``n_moves``, ``mean_dt_ms``, ``max_dt_ms`` and ``moves_per_min``."""
        dts = list(self._moves)
        times = list(self._move_times)
        n = len(dts)
        rate = 0.0
        if n >= 2 and times[-1] > times[0]:
            rate = (n - 1) * 60.0 / (times[-1] - times[0])
        elif n == 1 and self._state == "running":
            rate = 0.0
        return {
            "n_moves": n,
            "mean_dt_ms": (sum(dts) / n) if n else 0.0,
            "max_dt_ms": max(dts) if n else 0.0,
            "moves_per_min": rate,
        }

    # -- internals --------------------------------------------------------

    def _measure_timeout(self, n_frames: int) -> float:
        """Generous wait for a ``measurement`` reply (slow at long exposures)."""
        exposure_s = float(self._info.get("exposure_ms") or 0.0) / 1000.0
        return 30.0 + n_frames * max(1.0, 3.0 * exposure_s) + 10.0 * exposure_s

    def _attach_frames(self, name: str | None) -> None:
        """Attach to the worker's shared frame buffer (reader thread)."""
        if not name:
            return
        try:
            fb = FrameBuffer.attach(name)
        except Exception:
            logger.exception("could not attach to the shared frame buffer %r", name)
            self._info["shm_error"] = f"could not attach to {name!r}"
            return
        with self._fb_lock:
            self._fb, self._fb_name = fb, name

    def _close_frames(self) -> None:
        """Close the reader's handle and remove the name if the worker could not."""
        with self._fb_lock:
            fb, self._fb = self._fb, None
        if fb is not None:
            fb.close()
        if self._fb_name:
            # Normally the worker unlinked it at exit; after a kill it did not.
            unlink_by_name(self._fb_name)
            self._fb_name = None

    def _flush_measurements(self) -> None:
        while True:
            try:
                self._meas.get_nowait()
            except Empty:
                return

    def _send(self, msg: dict) -> bool:
        try:
            with self._send_lock:
                self._conn.send(msg)
            return True
        except (OSError, ValueError):
            if not self._shutting_down:
                self._worker_died()
            return False

    def _restore_crisp(self, prev: str | None = None) -> None:
        prev = self._prev_crisp if prev is None else prev
        self._prev_crisp = None
        if not self._crisp or prev is None or prev == "Ready":
            return
        target = "Lock" if prev in _LOCKED_STATES else prev
        try:
            self._mmc.setProperty(self._crisp, CRISP_STATE_PROP, target)
        except Exception:
            logger.warning("could not restore CRISP state %r", target, exc_info=True)

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        self._emit(self.state, state)

    def _emit(self, signal: Any, *args: Any) -> None:
        try:
            signal.emit(*args)
        except Exception:
            logger.exception("spot focus slot raised")

    def _fault(self, reason: str) -> None:
        self._set_state("fault")
        if reason != self._last_fault:
            self._last_fault = reason
            logger.warning("spot focus fault: %s", reason)
            self._emit(self.fault, reason)

    def _worker_died(self) -> None:
        if self._shutting_down or self._last_fault == "worker died":
            return
        self._fault("worker died")

    def _read_loop(self) -> None:
        """Drain the worker until its pipe closes, also while shutting down."""
        while True:
            try:
                if not self._conn.poll(0.2):
                    if not self._proc.is_alive():
                        self._worker_died()
                        return
                    continue
                msg = self._conn.recv()
            except (EOFError, OSError, ValueError):
                self._worker_died()
                return
            try:
                self._dispatch(msg)
            except Exception:
                logger.exception("spot focus message %r failed", msg.get("type"))

    def _dispatch(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "ready":
            self._info = dict(msg.get("info", {}))
            self._attach_frames(self._info.get("shm"))
            self._ready.set()
        elif kind == "camera":
            self._info.update(msg.get("info", {}))
            self._emit(self.camera_changed, dict(self._info))
        elif kind == "move":
            self._on_move(msg)
        elif kind == "telemetry":
            self._last_telemetry = msg
            self._emit(self.telemetry, msg)
        elif kind == "fault":
            self._fault(str(msg.get("reason", "fault")))
        elif kind == "status":
            st = str(msg.get("state", ""))
            if st == "idle" and self._preview:
                return  # the worker's start-up "idle" must not hide a preview
            if self._state != "fault" or st == "running":
                self._set_state(st)
        elif kind == "measurement":
            self._meas.put(msg)
        elif kind == "error":
            logger.error(
                "spot focus worker: %s\n%s",
                msg.get("message"),
                msg.get("traceback", ""),
            )
            self._fault(f"worker error: {msg.get('message')}")

    def _on_move(self, msg: dict) -> None:
        seq = msg["seq"]
        # Moves only while running: a late request after stop/fault is dropped.
        if self._state != "running" or self._move_busy or not self._proc.is_alive():
            self._send(
                {
                    "type": "moved",
                    "seq": seq,
                    "ok": False,
                    "dt_ms": 0.0,
                    "dropped": True,
                }
            )
            return
        self._move_busy = True
        ok = True
        t0 = time.perf_counter()
        try:
            self._mmc.setPosition(self._piezo, float(msg["target_um"]))  # no wait
        except Exception:
            ok = False
            logger.exception("spot focus setPosition failed")
        finally:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._move_busy = False
        if ok:
            self._moves.append(dt_ms)
            self._move_times.append(time.monotonic())
        info = {
            "seq": seq,
            "ok": ok,
            "dt_ms": dt_ms,
            "target_um": msg["target_um"],
            "along_um": msg.get("along_um"),
        }
        self._send({"type": "moved", "seq": seq, "ok": ok, "dt_ms": dt_ms})
        self._emit(self.moved, info)
