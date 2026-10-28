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

import contextlib
import logging
import math
import threading
import time
from dataclasses import replace
from pathlib import Path
from queue import Empty, Queue
from typing import TYPE_CHECKING, Any

from psygnal import Signal

from .spot_focus_worker import launch_worker
from .spot_frame_shm import FrameBuffer, unlink_by_name

if TYPE_CHECKING:
    from collections.abc import Sequence

    from .spot_focus import EdgeCalibration, SpotLoopParams
    from .spot_frame_shm import LiveFrame

logger = logging.getLogger(__name__)

PIEZO_P_LABEL = "PiezoStage:P:34"
CRISP_STATE_PROP = "CRISP State"
CRISP_LED_PROP = "LED Intensity"
#: Settable CRISP state that stands in for a reported "locked" state on restore.
_LOCKED_STATES = frozenset({"Lock", "In Focus", "Focusing"})
#: CRISP states in which the LED is already lit; ``led_on`` leaves them alone.
_LIT_STATES = _LOCKED_STATES | {
    "Ready",
    "Dither",
    "loG_cal",
    "gain_Cal",
    "Calibrating",
}
DEFAULT_SOURCE_SPEC: dict[str, Any] = {"kind": "toupcam"}


def _edge_text(reason: str) -> str:
    """A plain message for a measurement that found no edge."""
    if "left the box" in reason:
        return (
            "edge left the box: widen the box along the edge normal or reduce the span"
        )
    return f"edge not measurable: {reason or 'no usable frames'}"


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
        The CRISP device. When set, :meth:`led_on` (and :meth:`start`) put it
        in ``Ready``, which lights the LED; the previous state is restored
        when the last user (preview, lock, manual) lets go.
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
        #: Who needs the LED lit ("preview", "lock", "manual", ...). CRISP is
        #: restored when the last one lets go.
        self._led_owners: set[str] = set()
        self._led_lock = threading.RLock()
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
        calibration: EdgeCalibration,
        led_intensity: float | None = None,
    ) -> None:
        """Set CRISP to Ready, read P once, and start the loop."""
        if self._state == "fault" and not self._proc.is_alive():
            raise RuntimeError("spot focus worker is dead; create a new service")
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        p_start = float(self._mmc.getPosition(self._piezo))
        params = self._with_travel_limits(params, p_start)
        self.led_on(led_intensity, owner="lock", force_ready=True)
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

    def _with_travel_limits(
        self, params: SpotLoopParams, p_start: float
    ) -> SpotLoopParams:
        """Fill missing absolute P limits from the device, once, at start.

        Only when *params* has neither bound. Reads the ``Position`` property
        limits if the piezo exposes them (a one-time read, like ``getPosition``;
        nothing is polled while locked). Any failure, a missing property or a
        range that does not contain *p_start* leaves the bounds at None: a
        missing limit source never faults the lock, and the excursion limit
        from the start P remains the guard.
        """
        if params.p_min_um is not None or params.p_max_um is not None:
            return params
        mmc, label = self._mmc, self._piezo
        try:
            for prop in ("Position", "Position(um)"):
                if not (
                    mmc.hasProperty(label, prop) and mmc.hasPropertyLimits(label, prop)
                ):
                    continue
                lo = float(mmc.getPropertyLowerLimit(label, prop))
                hi = float(mmc.getPropertyUpperLimit(label, prop))
                if math.isfinite(lo) and math.isfinite(hi) and lo <= p_start <= hi:
                    logger.info(
                        "Spot focus: using %s.%s limits [%g, %g] um (start P %.2f um).",
                        label,
                        prop,
                        lo,
                        hi,
                        p_start,
                    )
                    return replace(params, p_min_um=lo, p_max_um=hi)
        except Exception:
            logger.debug("Could not read the piezo limits.", exc_info=True)
        logger.info(
            "Spot focus: no piezo travel limits found; guarding by the "
            "excursion limit from the start P %.2f um.",
            p_start,
        )
        return params

    def stop(self) -> None:
        """Stop the loop and restore CRISP. Never moves P.

        Back to ``preview`` if the live view was on, else ``stopped``.
        """
        self._send({"type": "stop"})
        if self._state != "fault":
            self._set_state("preview" if self._preview else "stopped")
        self.led_off("lock")

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
        """Change the loop parameters (including ``offset_um``) while running."""
        self._send({"type": "update_params", "params": params.to_dict()})

    def set_edge_box(self, box: Sequence[float] | None) -> None:
        """Set the edge box ``(x, y, w, h)`` the preview measures; None removes it.

        While the lock runs the calibration's own box is measured instead.
        """
        self._send(
            {
                "type": "set_edge_box",
                "box": None if box is None else [int(v) for v in box],
            }
        )

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
        with self._led_lock:
            self._led_owners.clear()
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

    # -- CRISP LED --------------------------------------------------------

    @property
    def crisp_label(self) -> str | None:
        """The CRISP device label, ``None`` when there is no CRISP device."""
        return self._crisp or None

    @property
    def led_is_on(self) -> bool:
        """True while some user holds the LED lit (always False with no CRISP)."""
        return bool(self._crisp) and bool(self._led_owners)

    def crisp_state(self) -> str | None:
        """The CRISP device's current ``CRISP State`` (``None`` if unreadable)."""
        if not self._crisp:
            return None
        try:
            return str(self._mmc.getProperty(self._crisp, CRISP_STATE_PROP))
        except Exception:
            logger.warning("could not read the CRISP state", exc_info=True)
            return None

    def led_on(
        self,
        intensity: float | None = None,
        *,
        owner: str = "manual",
        force_ready: bool = False,
    ) -> None:
        """Light the CRISP LED: put CRISP in ``Ready`` and set the intensity.

        The state CRISP was in is remembered once, when the first *owner*
        arrives, and restored when the last one calls :meth:`led_off`. A state
        in which the LED is already lit (Ready, Lock, Dither, ...) is left
        alone unless *force_ready*. *intensity* (else the one given at
        construction) is set every call. Does nothing without a CRISP device.
        """
        if not self._crisp:
            return
        with self._led_lock:
            first = not self._led_owners
            self._led_owners.add(owner)
            try:
                if first:
                    cur = str(self._mmc.getProperty(self._crisp, CRISP_STATE_PROP))
                    self._prev_crisp = None
                    if cur != "Ready" and (force_ready or cur not in _LIT_STATES):
                        self._prev_crisp = cur
                        self._mmc.setProperty(self._crisp, CRISP_STATE_PROP, "Ready")
                if intensity is not None:
                    self._led = intensity
                if self._led is not None:
                    self._mmc.setProperty(self._crisp, CRISP_LED_PROP, self._led)
            except Exception:
                self._led_owners.discard(owner)
                if not self._led_owners:
                    self._restore_crisp()
                raise

    def led_off(self, owner: str = "manual") -> None:
        """Let go of the LED; restores CRISP once no owner is left."""
        with self._led_lock:
            self._led_owners.discard(owner)
            if not self._led_owners:
                self._restore_crisp()

    def set_led_intensity(self, value: float) -> None:
        """Change the LED intensity; applied at once if the LED is lit."""
        self._led = value
        if self.led_is_on:
            with self._led_lock:
                self._mmc.setProperty(self._crisp, CRISP_LED_PROP, self._led)

    # -- calibration ------------------------------------------------------

    def calibrate(
        self,
        box: Sequence[float],
        *,
        span_um: float = 2.0,
        steps: int = 9,
        n_frames: int = 3,
        settle_s: float = 0.1,
        record_path: str | Path | None = None,
    ) -> EdgeCalibration:
        """Sweep P around its current value, measure the edge and fit a line.

        The user has focused on the cells, so the current P is the zero (best
        focus). The sweep goes in increasing P over ``+-span_um / 2``; at each
        step the worker snaps *n_frames* fresh frames (it drops stale ones) and
        measures the edge in *box*. P goes back to the start, the edge is
        measured once more (a drift / hysteresis check) and the line is fitted.

        Only while the loop is stopped and no acquisition runs. Uses
        ``waitForDevice`` (this polls the Tiger, which is fine for a short,
        user-started procedure with the serial line otherwise idle; the "never
        poll" rule is for the running lock and for volumes). Live streaming is
        paused meanwhile and restored; P, the LED claim and the preview are put
        back even on failure. Run it off the GUI thread.

        Parameters
        ----------
        box : Sequence[float]
            Edge box ``(x, y, w, h)`` in sensor pixels.
        span_um : float
            Total sweep span in um of P.
        steps : int
            Number of sweep points.
        n_frames : int
            Good frames averaged per point (the zero measurements use twice as
            many).
        settle_s : float
            Extra pause after each piezo move, before the worker's own settle.
        record_path : str | Path | None
            If given, save the sweep (P, edge positions, one raw frame per
            step) there with ``numpy.savez_compressed`` (``.npz`` is appended).

        Returns
        -------
        EdgeCalibration
            Zero at the start P, signed slope, linear range ``span_um / 2``.

        Raises
        ------
        RuntimeError
            If the loop or an acquisition is running, or the worker is dead.
        ValueError
            With a plain message if the edge cannot be measured, leaves the
            box, or the response is not a usable line.
        """
        import numpy as np

        from .spot_focus import fit_edge_calibration, snap_box

        if self._state == "running":
            raise RuntimeError("stop the loop before calibrating")
        mda = getattr(self._mmc, "mda", None)
        if mda is not None and mda.is_running():
            raise RuntimeError("an acquisition is running; calibrate after it ends")
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        mmc = self._mmc
        sbox = snap_box(box)
        p_c = float(mmc.getPosition(self._piezo))
        ps = p_c + np.linspace(-span_um / 2.0, span_um / 2.0, steps)
        self.led_on(owner="calibrate", force_ready=True)
        resume_preview = self._preview
        if resume_preview:  # pause Live without touching the service state
            self._send({"type": "preview_stop"})
        centred = False
        try:
            m0 = self._snap(2 * n_frames, p_c, sbox, None, None, False)
            if m0["n"] == 0:
                raise ValueError(_edge_text(m0.get("reason", "")))
            axis, polarity = str(m0["axis"]), int(m0["polarity"])
            e_start = float(m0["edge_mean"])
            noise_px = float(m0["edge_sd"])
            edges: list[float] = []
            frames: list[Any] = []
            for p in ps:
                mmc.setPosition(self._piezo, float(p))
                mmc.waitForDevice(self._piezo)
                time.sleep(settle_s)
                m = self._snap(n_frames, float(p), sbox, axis, polarity, True)
                if m["n"] == 0 and "left the box" in str(m.get("reason", "")):
                    raise ValueError(_edge_text(m["reason"]))
                edges.append(float(m["edge_mean"]) if m["n"] else float("nan"))
                frames.append(m.get("frame"))
            mmc.setPosition(self._piezo, p_c)
            mmc.waitForDevice(self._piezo)
            time.sleep(settle_s)
            m1 = self._snap(2 * n_frames, p_c, sbox, axis, polarity, False)
            centred = True
            e_end = float(m1["edge_mean"]) if m1["n"] else float("nan")
            warns: list[str] = []
            if math.isfinite(e_end):
                e0 = 0.5 * (e_start + e_end)
            else:
                e0 = e_start
                warns.append("could not re-measure the edge at the start position")
            saved = self._save_record(
                record_path,
                p_um=ps,
                edge_px=np.asarray(edges, dtype=float),
                frames=frames,
                box=np.asarray(sbox),
                axis=axis,
                polarity=polarity,
                p_c=p_c,
                e_start=e_start,
                e_end=e_end,
                noise_px=noise_px,
            )
            if record_path is not None and saved is None:
                warns.append("could not save the calibration record")
            cal = fit_edge_calibration(
                ps.tolist(),
                edges,
                box=sbox,
                axis=axis,
                polarity=polarity,
                p0_um=p_c,
                e0_px=e0,
                noise_px=noise_px,
                angle_deg=float(m0["angle_mean"]),
                contrast=float(m0["contrast_mean"]),
                linear_range_um=span_um / 2.0,
            )
            drift = abs(e_end - e_start)
            if math.isfinite(drift) and drift > max(
                3.0 * noise_px, 0.05 * abs(cal.slope_px_per_um)
            ):
                warns.append(
                    f"the edge moved {drift:.2f} px between the start and the end "
                    "of the sweep (drift or hysteresis)"
                )
            cal.warnings.extend(warns)
            cal.record_path = "" if saved is None else str(saved)
            return cal
        finally:
            try:
                mmc.setPosition(self._piezo, p_c)
                mmc.waitForDevice(self._piezo)
                if not centred:  # the fake camera follows P only through a measure
                    with contextlib.suppress(RuntimeError):
                        self._request_measurement(
                            {
                                "type": "measure",
                                "n_frames": 1,
                                "p_um": p_c,
                                "box": list(sbox),
                                "timeout_s": 0.5,
                            },
                            1,
                        )
            finally:
                self.led_off("calibrate")
                if resume_preview:
                    self._send({"type": "preview_start"})

    def zero(self, cal: EdgeCalibration, *, n_frames: int = 8) -> EdgeCalibration:
        """Make the present edge position the zero (best focus) of *cal*.

        Reads P once, measures the edge in the calibration's box with its axis
        and polarity, and returns :func:`~.spot_focus.rezero` of it. The slope
        and the range are kept. Only while the loop is stopped; sends no move.

        Parameters
        ----------
        cal : EdgeCalibration
            The calibration to re-zero.
        n_frames : int
            Good frames to average.

        Returns
        -------
        EdgeCalibration
            *cal* with ``e0_px`` / ``p0_um`` at the present edge and P.

        Raises
        ------
        RuntimeError
            If the loop is running or the worker is dead.
        ValueError
            If the edge cannot be measured.
        """
        from .spot_focus import rezero

        if self._state == "running":
            raise RuntimeError("stop the loop before zeroing")
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        p = float(self._mmc.getPosition(self._piezo))
        self.led_on(owner="zero")
        try:
            m = self._snap(n_frames, p, cal.box, cal.axis, cal.polarity, False)
        finally:
            self.led_off("zero")
        if m["n"] == 0:
            raise ValueError(_edge_text(m.get("reason", "")))
        return rezero(cal, e0_px=float(m["edge_mean"]), p0_um=p)

    def measure(
        self,
        n_frames: int = 20,
        p_um: float | None = None,
        *,
        box: Sequence[float],
        axis: str | None = None,
        polarity: int | None = None,
        return_frame: bool = False,
    ) -> dict:
        """One averaged edge measurement from the worker (loop stopped).

        Parameters
        ----------
        n_frames : int
            Good frames to average; all were exposed after the request.
        p_um : float | None
            P the fake camera should follow (a real camera ignores it).
        box : Sequence[float]
            Edge box ``(x, y, w, h)`` in sensor pixels.
        axis : str | None
            ``"x"`` / ``"y"`` edge normal; detected on the first good frame
            when None.
        polarity : int | None
            +1 / -1 bright side; detected when None.
        return_frame : bool
            Also return one raw full-ROI frame under ``"frame"``.

        Returns
        -------
        dict
            ``p_um, n, edge_mean, edge_sd, x_mean, y_mean, contrast_mean,
            angle_mean, residual_mean, axis, polarity, reason, roi`` (and
            ``frame``). ``reason`` is why the last frame was bad when ``n`` is 0.
        """
        if self._state == "running":
            raise RuntimeError("stop the loop before measuring")
        return self._snap(n_frames, p_um, box, axis, polarity, return_frame)

    def _snap(
        self,
        n_frames: int,
        p_um: float | None,
        box: Sequence[float],
        axis: str | None,
        polarity: int | None,
        return_frame: bool,
    ) -> dict:
        """Ask the worker for *n_frames* fresh edge fixes and wait for the reply."""
        return self._request_measurement(
            {
                "type": "measure",
                "n_frames": int(n_frames),
                "p_um": p_um,
                "box": [int(v) for v in box],
                "axis": axis,
                "polarity": polarity,
                "return_frame": return_frame,
            },
            int(n_frames),
        )

    def _request_measurement(self, msg: dict, n_frames: int) -> dict:
        self._flush_measurements()
        if not self._send(msg):
            raise RuntimeError("spot focus worker is dead")
        try:
            return self._meas.get(timeout=self._measure_timeout(n_frames))
        except Empty:
            raise RuntimeError("spot focus worker did not answer") from None

    @staticmethod
    def _save_record(path: str | Path | None, **arrays: Any) -> Path | None:
        """Save the sweep as ``.npz``; None if not wanted or not possible."""
        import numpy as np

        if path is None:
            return None
        out = Path(path)
        if out.suffix != ".npz":
            out = out.with_name(out.name + ".npz")
        frames = [f for f in arrays["frames"] if f is not None]
        if frames:
            arrays["frames"] = np.stack(
                [np.zeros_like(frames[0]) if f is None else f for f in arrays["frames"]]
            )
        else:
            arrays["frames"] = np.zeros((0,), dtype=np.uint8)
        try:
            out.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(out, **arrays)
        except OSError:
            logger.warning(
                "could not save the calibration record %s", out, exc_info=True
            )
            return None
        return out

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

    def _restore_crisp(self) -> None:
        prev, self._prev_crisp = self._prev_crisp, None
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
