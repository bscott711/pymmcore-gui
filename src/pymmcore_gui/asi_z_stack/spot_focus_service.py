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
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from typing import TYPE_CHECKING, Any

from psygnal import Signal

from .spot_focus_worker import launch_worker
from .spot_frame_shm import FrameBuffer, unlink_by_name

if TYPE_CHECKING:
    from collections.abc import Sequence

    from .spot_focus import EdgeCalibration, FocusCalibration, SpotLoopParams
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


def _as_boxes(boxes: Sequence[Any]) -> list[tuple[int, int, int, int]]:
    """One box ``(x, y, w, h)`` or a list of 1-2 boxes, as a list of boxes."""
    seq = list(boxes)
    if seq and not hasattr(seq[0], "__len__"):  # numbers: a single box
        seq = [seq]
    if not 1 <= len(seq) <= 2:
        raise ValueError(f"need one or two edge boxes, got {len(seq)}")
    out = []
    for box in seq:
        if len(box) != 4:
            raise ValueError(f"an edge box is (x, y, w, h), not {box!r}")
        out.append(tuple(int(v) for v in box))
    return out  # type: ignore[return-value]


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
        calibration: FocusCalibration | EdgeCalibration,
        led_intensity: float | None = None,
    ) -> None:
        """Set CRISP to Ready, read P once, and start the loop.

        *calibration* is a :class:`~.spot_focus.FocusCalibration`; a single
        :class:`~.spot_focus.EdgeCalibration` is wrapped into a one-edge one.
        """
        from .spot_focus import EdgeCalibration, FocusCalibration

        if isinstance(calibration, EdgeCalibration):
            calibration = FocusCalibration.from_dict(calibration.to_dict())
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

    def set_edge_boxes(self, boxes: Sequence[Sequence[float]]) -> None:
        """Set the 0-2 edge boxes the preview measures; empty removes them.

        When they are the held calibration's boxes (see :meth:`set_calibration`)
        the preview telemetry also carries the fused error ``along_um``.
        """
        self._send(
            {
                "type": "set_edge_boxes",
                "boxes": [[int(v) for v in box] for box in boxes],
            }
        )

    def set_calibration(self, cal: FocusCalibration | None) -> None:
        """Hand the worker a calibration for the preview error (None clears it).

        The worker also keeps the one it was started with. Ignored by the
        worker while the lock runs.
        """
        self._send(
            {
                "type": "set_calibration",
                "calibration": None if cal is None else cal.to_dict(),
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
        boxes: Sequence[Any],
        *,
        span_um: float = 2.0,
        steps: int = 9,
        n_frames: int = 3,
        settle_s: float = 0.25,
        record_path: str | Path | None = None,
    ) -> FocusCalibration:
        """Sweep P around its current value, measure one or two edges and fit them.

        The user has focused on the cells, so the current P is the zero (best
        focus). The sweep goes in increasing P over ``+-span_um / 2``; at each
        step the worker snaps *n_frames* fresh frames (it drops stale ones) and
        measures the edge in every box **on the same frames**. P goes back to
        the start, the edges are measured once more (a drift / hysteresis
        check) and each edge is fitted on its own: its own slope and sign, zero
        (mean of the start and end measurement), noise, angle and contrast.

        An edge that cannot be measured, leaves its box or does not fit is
        dropped with a warning ``"edge K dropped: <message>"`` (K counts the
        boxes from 1); the calibration fails only if no edge survives.

        Only while the loop is stopped and no acquisition runs. Uses
        ``waitForDevice`` (this polls the Tiger, which is fine for a short,
        user-started procedure with the serial line otherwise idle; the "never
        poll" rule is for the running lock and for volumes). Live streaming is
        paused meanwhile and restored; P, the LED claim and the preview are put
        back even on failure. Run it off the GUI thread.

        Parameters
        ----------
        boxes : Sequence
            One edge box ``(x, y, w, h)`` or a list of one or two boxes, in
            sensor pixels. Box 1 is the main edge.
        span_um : float
            Total sweep span in um of P.
        steps : int
            Number of sweep points.
        n_frames : int
            Good frames averaged per point (the zero measurements use
            ``max(2 * n_frames, 10)``, which also detects axis and polarity).
        settle_s : float
            Extra pause after each piezo move, before the worker's own settle.
        record_path : str | Path | None
            If given, save the sweep there with ``numpy.savez_compressed``
            (``.npz`` is appended): ``p_um``, ``edge_px`` of shape
            ``(steps, n_boxes)`` (nan where an edge was not measured), ``boxes``,
            ``axes``, ``polarities``, ``frames`` (one raw frame per step) and
            ``p_c``, ``e_start``, ``e_end``, ``noise_px`` (one value per box).

        Returns
        -------
        FocusCalibration
            One or two edges with a common zero P (the start P). Its
            ``warnings`` list the dropped edges and the edges' own warnings
            (prefixed with the edge number when there are two boxes).

        Raises
        ------
        RuntimeError
            If the loop or an acquisition is running, or the worker is dead.
        ValueError
            With a plain message (edge 1's first) if no edge can be measured,
            stays in its box and gives a usable line.
        """
        import numpy as np

        from .spot_focus import FocusCalibration, fit_edge_calibration, snap_box

        if self._state == "running":
            raise RuntimeError("stop the loop before calibrating")
        mda = getattr(self._mmc, "mda", None)
        if mda is not None and mda.is_running():
            raise RuntimeError("an acquisition is running; calibrate after it ends")
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        mmc = self._mmc
        sboxes = [snap_box(b) for b in _as_boxes(boxes)]
        nb = len(sboxes)
        p_c = float(mmc.getPosition(self._piezo))
        ps = p_c + np.linspace(-span_um / 2.0, span_um / 2.0, steps)
        n_zero = max(2 * n_frames, 10)
        self.led_on(owner="calibrate", force_ready=True)
        resume_preview = self._preview
        if resume_preview:  # pause Live without touching the service state
            self._send({"type": "preview_stop"})
        centred = False
        try:
            m0 = self._snap_edges(n_zero, p_c, [(b, None, None) for b in sboxes], False)
            ent0 = m0["edges"]
            errors: dict[int, str] = {}  # box index -> why it is dropped
            for k, ent in enumerate(ent0):
                if ent["n"] == 0:
                    errors[k] = _edge_text(ent.get("reason", ""))
            axes = [e.get("axis") for e in ent0]
            pols = [e.get("polarity") for e in ent0]
            alive = [k for k in range(nb) if k not in errors]
            if not alive:
                raise ValueError(errors[min(errors)])
            specs = [(sboxes[k], axes[k], pols[k]) for k in alive]
            edge_px = np.full((len(ps), nb), np.nan)
            frames: list[Any] = []
            for s, p in enumerate(ps):
                mmc.setPosition(self._piezo, float(p))
                mmc.waitForDevice(self._piezo)
                time.sleep(settle_s)
                m = self._snap_edges(n_frames, float(p), specs, True)
                for k, ent in zip(alive, m["edges"], strict=True):
                    if ent["n"]:
                        edge_px[s, k] = float(ent["edge_mean"])
                    elif "left the box" in str(ent.get("reason", "")):
                        errors.setdefault(k, _edge_text(ent["reason"]))
                frames.append(m.get("frame"))
                if all(k in errors for k in alive):
                    raise ValueError(errors[min(errors)])
            mmc.setPosition(self._piezo, p_c)
            mmc.waitForDevice(self._piezo)
            time.sleep(settle_s)
            m1 = self._snap_edges(n_zero, p_c, specs, False)
            centred = True
            e_start = np.full(nb, np.nan)
            e_end = np.full(nb, np.nan)
            noise = np.zeros(nb)
            for k in alive:
                e_start[k] = float(ent0[k]["edge_mean"])
                noise[k] = float(ent0[k]["edge_sd"])
            for k, ent in zip(alive, m1["edges"], strict=True):
                if ent["n"]:
                    e_end[k] = float(ent["edge_mean"])
            warns: list[str] = []
            saved = self._save_record(
                record_path,
                p_um=ps,
                edge_px=edge_px,
                frames=frames,
                boxes=np.asarray(sboxes),
                axes=np.asarray([a or "" for a in axes]),
                polarities=np.asarray([int(p or 0) for p in pols]),
                p_c=p_c,
                e_start=e_start,
                e_end=e_end,
                noise_px=noise,
            )
            if record_path is not None and saved is None:
                warns.append("could not save the calibration record")
            fitted: dict[int, Any] = {}
            for k in alive:
                if k in errors:
                    continue
                own: list[str] = []
                if math.isfinite(e_end[k]):
                    e0 = 0.5 * (e_start[k] + e_end[k])
                else:
                    e0 = float(e_start[k])
                    own.append("could not re-measure the edge at the start position")
                try:
                    edge = fit_edge_calibration(
                        ps.tolist(),
                        edge_px[:, k].tolist(),
                        box=sboxes[k],
                        axis=str(axes[k]),
                        polarity=int(pols[k]),
                        p0_um=p_c,
                        e0_px=e0,
                        noise_px=float(noise[k]),
                        angle_deg=float(ent0[k]["angle_mean"]),
                        contrast=float(ent0[k]["contrast_mean"]),
                        linear_range_um=span_um / 2.0,
                    )
                except ValueError as exc:
                    errors[k] = str(exc)
                    continue
                drift = abs(e_end[k] - e_start[k])
                if math.isfinite(drift) and drift > max(
                    3.0 * noise[k], 0.05 * abs(edge.slope_px_per_um)
                ):
                    own.append(
                        f"the edge moved {drift:.2f} px between the start and the "
                        "end of the sweep (drift or hysteresis)"
                    )
                edge.warnings.extend(own)
                fitted[k] = edge
            if not fitted:
                raise ValueError(errors[min(errors)])
            for k in sorted(errors):
                warns.append(f"edge {k + 1} dropped: {errors[k]}")
            for k, edge in fitted.items():
                prefix = f"edge {k + 1}: " if nb > 1 else ""
                warns.extend(prefix + w for w in edge.warnings)
            return FocusCalibration(
                edges=list(fitted.values()),
                p0_um=p_c,
                created=datetime.now().isoformat(timespec="seconds"),
                record_path="" if saved is None else str(saved),
                warnings=warns,
            )
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
                                "boxes": [{"box": list(sboxes[0])}],
                                "timeout_s": 0.5,
                            },
                            1,
                        )
            finally:
                self.led_off("calibrate")
                if resume_preview:
                    self._send({"type": "preview_start"})

    def zero(self, cal: FocusCalibration, *, n_frames: int = 8) -> FocusCalibration:
        """Make the present edge positions the zero (best focus) of *cal*.

        Reads P once, measures every edge in its box with its axis and
        polarity on the same frames, and returns :meth:`FocusCalibration.rezeroed`
        of them. The slopes and the range are kept. Only while the loop is
        stopped; sends no move.

        Parameters
        ----------
        cal : FocusCalibration
            The calibration to re-zero (one or two edges).
        n_frames : int
            Good frames to average.

        Returns
        -------
        FocusCalibration
            *cal* with every ``e0_px`` and ``p0_um`` at the present edges and P.

        Raises
        ------
        RuntimeError
            If the loop is running or the worker is dead.
        ValueError
            If any edge cannot be measured (the reason is in the message).
        """
        if self._state == "running":
            raise RuntimeError("stop the loop before zeroing")
        if not self.wait_ready():
            raise RuntimeError("spot focus worker did not become ready")
        p = float(self._mmc.getPosition(self._piezo))
        self.led_on(owner="zero")
        try:
            m = self._snap_edges(
                n_frames, p, [(e.box, e.axis, e.polarity) for e in cal.edges], False
            )
        finally:
            self.led_off("zero")
        for k, ent in enumerate(m["edges"]):
            if ent["n"] == 0:
                text = _edge_text(ent.get("reason", ""))
                raise ValueError(
                    f"edge {k + 1}: {text}" if len(cal.edges) > 1 else text
                )
        return cal.rezeroed([float(ent["edge_mean"]) for ent in m["edges"]], p)

    def measure(
        self,
        n_frames: int = 20,
        p_um: float | None = None,
        *,
        box: Sequence[Any],
        axis: str | Sequence[str | None] | None = None,
        polarity: int | Sequence[int | None] | None = None,
        return_frame: bool = False,
    ) -> dict:
        """One averaged edge measurement from the worker (loop stopped).

        Parameters
        ----------
        n_frames : int
            Good frames to average per box; all were exposed after the request.
        p_um : float | None
            P the fake camera should follow (a real camera ignores it).
        box : Sequence
            One edge box ``(x, y, w, h)`` in sensor pixels, or a list of one or
            two boxes, all measured on the same frames.
        axis : str | Sequence[str | None] | None
            ``"x"`` / ``"y"`` edge normal; detected on the first good frame
            when None. A list gives one per box.
        polarity : int | Sequence[int | None] | None
            +1 / -1 bright side; detected when None. A list gives one per box.
        return_frame : bool
            Also return one raw full-ROI frame under ``"frame"``.

        Returns
        -------
        dict
            ``p_um, edges, roi`` (and ``frame``). ``edges`` holds, per box,
            ``n, edge_mean, edge_sd, x_mean, y_mean, contrast_mean, angle_mean,
            residual_mean, axis, polarity, reason``; the first box's fields are
            repeated at the top level. ``reason`` is why the last frame was bad
            when ``n`` is 0.
        """
        if self._state == "running":
            raise RuntimeError("stop the loop before measuring")
        sboxes = _as_boxes(box)

        def per_box(value: Any) -> list[Any]:
            if isinstance(value, list | tuple):
                if len(value) != len(sboxes):
                    raise ValueError("axis / polarity need one value per box")
                return list(value)
            if len(sboxes) > 1 and value is not None:
                raise ValueError("pass axis / polarity as lists for two boxes")
            return [value] + [None] * (len(sboxes) - 1)

        specs = list(zip(sboxes, per_box(axis), per_box(polarity), strict=True))
        return self._snap_edges(n_frames, p_um, specs, return_frame)

    def _snap_edges(
        self,
        n_frames: int,
        p_um: float | None,
        specs: Sequence[tuple[Sequence[float], str | None, int | None]],
        return_frame: bool,
    ) -> dict:
        """Ask the worker for *n_frames* fresh fixes of every box; wait for the reply.

        *specs* are ``(box, axis, polarity)``; axis and polarity None are
        detected. The reply has an ``edges`` entry per spec.
        """
        return self._request_measurement(
            {
                "type": "measure",
                "n_frames": int(n_frames),
                "p_um": p_um,
                "boxes": [
                    {"box": [int(v) for v in box], "axis": axis, "polarity": pol}
                    for box, axis, pol in specs
                ],
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
