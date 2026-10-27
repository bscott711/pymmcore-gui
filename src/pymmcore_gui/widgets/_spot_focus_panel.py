"""Spot focus lock panel (the "Spot" mode of the focus-source dock).

The lock (see :mod:`~pymmcore_gui.asi_z_stack.spot_focus_service`) watches the
CRISP LED spot on a side camera in a worker process and holds piezo P by
nudging it rarely. This panel

- starts and stops the loop (it runs until Stop, independent of any MDA);
- calibrates it (a piezo sweep, on a background thread);
- shows the spot (a live ROI thumbnail with the fitted spot and the setpoint),
  a strip chart of the focus error and the commanded piezo over the last five
  minutes, and move statistics;
- edits the loop parameters, camera exposure and CRISP LED intensity.

Everything is written to ``SettingsV1.spot_focus``. The service is created
lazily on Start or Calibrate and shut down with the panel or the app. Its
signals come from the reader thread, so each goes through a GUI-thread
:class:`QObject` signal (Qt queues the call), the same way as in
:mod:`~pymmcore_gui.widgets._image_focus_widget`.
"""

from __future__ import annotations

import html
import itertools
import logging
import threading
import time
from collections import deque
from contextlib import suppress
from typing import TYPE_CHECKING, Any

import numpy as np

from pymmcore_gui._qt.QtCore import QObject, QPointF, QRectF, Qt, QTimer, Signal
from pymmcore_gui._qt.QtGui import QColor, QImage, QPainter, QPen
from pymmcore_gui._qt.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.asi_z_stack.image_focus_events import spot_focus_flag
from pymmcore_gui.asi_z_stack.spot_focus_service import SpotFocusService

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_gui.asi_z_stack.spot_focus import SpotCalibration

logger = logging.getLogger(__name__)

ALERT_COLOR = "#d0342c"
ATTENTION_COLOR = "#e08a00"

#: Strip chart history, seconds.
CHART_WINDOW_S = 300.0
#: Delay before writing edits to disk; they apply in memory immediately.
_FLUSH_DELAY_MS = 500
#: Readout refresh period (move statistics), ms.
_STATS_PERIOD_MS = 1000
_THUMB_SIZE = 256

# (settings field, label, min, max, step, decimals, suffix, tooltip)
_PARAMS: tuple[tuple[str, str, float, float, float, int, str, str], ...] = (
    (
        "deadband_um",
        "Deadband",
        0.0,
        5.0,
        0.01,
        3,
        " µm",
        "Focus errors up to this size are left alone",
    ),
    ("kp", "Kp", 0.0, 5.0, 0.05, 2, "", "Proportional gain"),
    ("ki", "Ki", 0.0, 2.0, 0.01, 3, "", "Integral gain"),
    (
        "max_step_um",
        "Max step",
        0.01,
        5.0,
        0.05,
        2,
        " µm",
        "Largest piezo move in one correction",
    ),
    (
        "max_rate_hz",
        "Max rate",
        0.1,
        20.0,
        0.5,
        1,
        " Hz",
        "Most piezo moves per second",
    ),
)


def source_spec(cfg: Any) -> dict[str, Any]:
    """The worker's frame-source spec for ``SettingsV1.spot_focus`` *cfg*."""
    src = cfg.source
    if src.kind == "fake":
        spec: dict[str, Any] = {"kind": "fake", **src.fake}
    else:
        spec = {"kind": "toupcam"}
        if src.serial:
            spec["serial"] = src.serial
        if src.sdk_dir:
            spec["sdk_dir"] = src.sdk_dir
    spec["analysis"] = {"channel": cfg.channel, "bayer": cfg.bayer}
    return spec


def _colored(text: str, color: str) -> str:
    return f'<span style="color:{color}">{html.escape(text)}</span>'


# ---------------------------------------------------------------------------
# drawing widgets
# ---------------------------------------------------------------------------


class SpotThumbView(QWidget):
    """The live ROI thumbnail with markers at the fitted spot and the setpoint."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(_THUMB_SIZE // 2, _THUMB_SIZE // 2)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._image: QImage | None = None
        self._data: np.ndarray | None = None  # keeps the QImage buffer alive
        self._spot: tuple[float, float] | None = None
        self._setpoint: tuple[float, float] | None = None

    def sizeHint(self):
        """Return the preferred size."""
        from pymmcore_gui._qt.QtCore import QSize

        return QSize(_THUMB_SIZE, _THUMB_SIZE)

    @property
    def has_image(self) -> bool:
        """Whether a thumbnail has been received."""
        return self._image is not None

    @property
    def markers(self) -> tuple[tuple[float, float] | None, tuple[float, float] | None]:
        """``(spot, setpoint)`` in thumbnail pixels (``None`` when unknown)."""
        return self._spot, self._setpoint

    def set_thumb(
        self,
        image: np.ndarray,
        spot: tuple[float, float] | None,
        setpoint: tuple[float, float] | None,
    ) -> None:
        """Show *image* (uint8, 2-D); marker positions are in its pixels."""
        data = np.ascontiguousarray(image, dtype=np.uint8)
        h, w = data.shape
        self._data = data
        self._image = QImage(data.data, w, h, w, QImage.Format.Format_Grayscale8)
        self._spot, self._setpoint = spot, setpoint
        self.update()

    def clear(self) -> None:
        """Forget the thumbnail."""
        self._image = self._data = None
        self._spot = self._setpoint = None
        self.update()

    def paintEvent(self, event) -> None:
        """Draw the thumbnail and the markers."""
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#101010"))
        if self._image is None:
            p.setPen(QColor("#888888"))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "no image")
            return
        iw, ih = self._image.width(), self._image.height()
        scale = min(self.width() / iw, self.height() / ih)
        ox = (self.width() - iw * scale) / 2
        oy = (self.height() - ih * scale) / 2
        p.drawImage(QRectF(ox, oy, iw * scale, ih * scale), self._image)

        def at(pt: tuple[float, float]) -> QPointF:
            return QPointF(ox + (pt[0] + 0.5) * scale, oy + (pt[1] + 0.5) * scale)

        if self._setpoint is not None:  # ring: where the spot should be
            p.setPen(QPen(QColor("#ff9a1f"), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            c = at(self._setpoint)
            p.drawEllipse(c, 9.0, 9.0)
        if self._spot is not None:  # cross: where it is
            p.setPen(QPen(QColor("#3df25a"), 2))
            c = at(self._spot)
            p.drawLine(QPointF(c.x() - 7, c.y()), QPointF(c.x() + 7, c.y()))
            p.drawLine(QPointF(c.x(), c.y() - 7), QPointF(c.x(), c.y() + 7))


class StripChart(QWidget):
    """Focus error and commanded piezo (both in µm) over the last few minutes."""

    def __init__(
        self, window_s: float = CHART_WINDOW_S, parent: QWidget | None = None
    ) -> None:
        super().__init__(parent)
        self.window_s = window_s
        self.setMinimumHeight(110)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        #: ``(t, along_um, p_cmd_rel_um)``
        self._points: deque[tuple[float, float, float]] = deque()
        self._p_start: float | None = None

    def __len__(self) -> int:
        return len(self._points)

    def add(self, t: float, along_um: float, p_cmd_um: float) -> None:
        """Append a sample; ``p_cmd_um`` is plotted relative to the first one."""
        if self._p_start is None:
            self._p_start = p_cmd_um
        self._points.append((t, along_um, p_cmd_um - self._p_start))
        while self._points and t - self._points[0][0] > self.window_s:
            self._points.popleft()
        self.update()

    def clear(self) -> None:
        """Drop all samples."""
        self._points.clear()
        self._p_start = None
        self.update()

    def paintEvent(self, event) -> None:
        """Draw the two traces."""
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#101010"))
        r = QRectF(self.rect()).adjusted(46, 6, -6, -16)
        p.setPen(QColor("#555555"))
        p.drawRect(r)
        if len(self._points) < 2:
            p.setPen(QColor("#888888"))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "no data")
            return
        pts = list(self._points)
        t1 = pts[-1][0]
        t0 = t1 - self.window_s
        span = max(max(abs(a) for _, a, _ in pts), max(abs(c) for _, _, c in pts))
        span = max(span * 1.1, 0.05)

        def xy(t: float, v: float) -> QPointF:
            return QPointF(
                r.left() + (t - t0) / self.window_s * r.width(),
                r.center().y() - v / span * r.height() / 2,
            )

        p.setPen(QPen(QColor("#555555"), 1, Qt.PenStyle.DashLine))
        p.drawLine(
            QPointF(r.left(), r.center().y()), QPointF(r.right(), r.center().y())
        )
        p.setPen(QColor("#aaaaaa"))
        p.drawText(2, int(r.top()) + 10, f"+{span:.2f} µm")
        p.drawText(2, int(r.bottom()), f"-{span:.2f} µm")
        p.drawText(
            int(r.left()), int(self.height()) - 3, f"-{self.window_s / 60:g} min"
        )
        p.drawText(int(r.right()) - 22, int(self.height()) - 3, "now")
        for column, color in ((1, "#56b4e9"), (2, "#ff9a1f")):
            p.setPen(QPen(QColor(color), 1.5))
            for a, b in itertools.pairwise(pts):
                p.drawLine(xy(a[0], a[column]), xy(b[0], b[column]))
        p.setPen(QColor("#56b4e9"))
        p.drawText(int(r.left()) + 4, int(r.top()) + 12, "error (along)")
        p.setPen(QColor("#ff9a1f"))
        p.drawText(int(r.left()) + 90, int(r.top()) + 12, "piezo P (from start)")


# ---------------------------------------------------------------------------
# threading helpers
# ---------------------------------------------------------------------------


class _Relay(QObject):
    """Hands service / worker-thread results to the GUI thread."""

    telemetry = Signal(object)
    thumb = Signal(object)
    fault = Signal(str)
    state = Signal(str)
    moved = Signal(object)
    started = Signal()
    startFailed = Signal(str)
    calibrated = Signal(object)
    calibrateFailed = Signal(str)

    def on_telemetry(self, msg: dict) -> None:
        self.telemetry.emit(msg)

    def on_thumb(self, msg: dict) -> None:
        self.thumb.emit(msg)

    def on_fault(self, reason: str) -> None:
        self.fault.emit(reason)

    def on_state(self, state: str) -> None:
        self.state.emit(state)

    def on_moved(self, info: dict) -> None:
        self.moved.emit(info)


class _Owner:
    """Owns the service so shutdown can run while the panel is being deleted."""

    def __init__(self) -> None:
        self.service: SpotFocusService | None = None
        self.spec: dict | None = None
        self.dead = False
        self.lock = threading.Lock()

    def shutdown(self) -> None:
        with self.lock:
            svc, self.service = self.service, None
        if svc is not None:
            try:
                svc.shutdown()
            except Exception:
                logger.warning("spot focus shutdown failed", exc_info=True)
        spot_focus_flag.active = False


# ---------------------------------------------------------------------------
# the panel
# ---------------------------------------------------------------------------


class SpotFocusPanel(QWidget):
    """Start, stop, calibrate and watch the spot focus lock.

    Parameters
    ----------
    mmcore : CMMCorePlus-like
        The core whose piezo P the loop moves.
    parent : QWidget | None
        Parent widget.
    service_factory : Callable[[dict, float | None], SpotFocusService] | None
        Builds the service from ``(source_spec, led_intensity)``. The default
        uses *mmcore*, the piezo ``PiezoStage:P:34`` and the CRISP device from
        ``SettingsV1.focus.crisp_label``.
    """

    #: Emitted (GUI thread) when Start switched the focus source to Spot.
    modeForced = Signal()

    def __init__(
        self,
        mmcore: Any,
        parent: QWidget | None = None,
        *,
        service_factory: Callable[[dict, float | None], SpotFocusService] | None = None,
    ) -> None:
        super().__init__(parent)
        self._mmc = mmcore
        self._factory = service_factory or self._default_factory
        self._owner = _Owner()
        self._loading = False
        self._busy = ""  # "starting" / "calibrating" while a thread works
        self.state = "idle"
        self._telemetry: dict | None = None
        self._cal: SpotCalibration | None = None

        # -- controls ------------------------------------------------------
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start)
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop)
        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.calibrate)
        buttons = QHBoxLayout()
        for b in (self.start_button, self.stop_button, self.calibrate_button):
            buttons.addWidget(b)
        buttons.addStretch()

        self.state_label = QLabel()
        self.state_label.setTextFormat(Qt.TextFormat.RichText)
        self.fault_label = QLabel()
        self.fault_label.setTextFormat(Qt.TextFormat.RichText)
        self.fault_label.setWordWrap(True)
        self.calibration_label = QLabel()
        self.calibration_label.setTextFormat(Qt.TextFormat.RichText)
        self.calibration_label.setWordWrap(True)

        # -- views ---------------------------------------------------------
        self.thumb_view = SpotThumbView()
        self.chart = StripChart()

        # -- readouts ------------------------------------------------------
        self.readout_labels: dict[str, QLabel] = {}
        readouts = QFormLayout()
        for key, text in (
            ("moves", "Moves / min"),
            ("move_ms", "Move time (mean / max)"),
            ("lateral", "Lateral"),
            ("fps", "Frame rate"),
            ("error", "Error (along)"),
        ):
            label = QLabel("-")
            self.readout_labels[key] = label
            readouts.addRow(text, label)

        # -- parameters ----------------------------------------------------
        self.param_spins: dict[str, QDoubleSpinBox] = {}
        form = QFormLayout()
        for field, label, lo, hi, step, decimals, suffix, tip in _PARAMS:
            spin = self._spin(lo, hi, step, decimals, suffix, tip)
            spin.valueChanged.connect(lambda v, f=field: self._set_param(f, v))
            self.param_spins[field] = spin
            form.addRow(label, spin)
        self.led_spin = self._spin(
            0.0, 100.0, 1.0, 0, " %", "CRISP LED intensity, set on the next Start"
        )
        self.led_spin.setSpecialValueText("unchanged")
        self.led_spin.valueChanged.connect(self._set_led)
        form.addRow("LED", self.led_spin)
        self.exposure_spin = self._spin(
            0.05, 1000.0, 1.0, 2, " ms", "Camera exposure (applies at once)"
        )
        self.exposure_spin.valueChanged.connect(self._set_exposure)
        form.addRow("Exposure", self.exposure_spin)
        params_group = QGroupBox("Parameters")
        params_group.setLayout(form)

        # -- layout --------------------------------------------------------
        top = QHBoxLayout()
        top.addWidget(self.thumb_view, 1)
        right = QVBoxLayout()
        right.addLayout(readouts)
        right.addStretch()
        top.addLayout(right)
        box = QGroupBox("Spot focus lock")
        layout = QVBoxLayout(box)
        layout.addLayout(buttons)
        layout.addWidget(self.state_label)
        layout.addWidget(self.fault_label)
        layout.addWidget(self.calibration_label)
        layout.addLayout(top)
        layout.addWidget(self.chart)
        layout.addWidget(params_group)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(box)

        # -- timers, relay -------------------------------------------------
        self._flush_timer = QTimer(self)
        self._flush_timer.setSingleShot(True)
        self._flush_timer.setInterval(_FLUSH_DELAY_MS)
        self._flush_timer.timeout.connect(lambda: SettingsV1.instance().flush())
        self._stats_timer = QTimer(self)
        self._stats_timer.setInterval(_STATS_PERIOD_MS)
        self._stats_timer.timeout.connect(self._refresh_stats)
        self._stats_timer.start()

        self._relay = _Relay(self)
        self._relay.telemetry.connect(self._on_telemetry)
        self._relay.thumb.connect(self._on_thumb)
        self._relay.fault.connect(self._on_fault)
        self._relay.state.connect(self._on_state)
        self._relay.started.connect(self._on_started)
        self._relay.startFailed.connect(self._on_start_failed)
        self._relay.calibrated.connect(self._on_calibrated)
        self._relay.calibrateFailed.connect(self._on_calibrate_failed)

        # Closures over the owner, not self: they run while self is going away.
        owner = self._owner
        self.destroyed.connect(lambda *_: owner.shutdown())
        if (app := QApplication.instance()) is not None:
            app.aboutToQuit.connect(owner.shutdown)

        self._set_state("idle")
        self.refresh()

    @staticmethod
    def _spin(
        lo: float, hi: float, step: float, decimals: int, suffix: str, tip: str
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setSuffix(suffix)
        spin.setToolTip(tip)
        spin.setKeyboardTracking(False)
        return spin

    # ------------------------------------------------------------------
    # service
    # ------------------------------------------------------------------

    @property
    def service(self) -> SpotFocusService | None:
        """The service, ``None`` until Start or Calibrate created it."""
        return self._owner.service

    def _default_factory(self, spec: dict, led: float | None) -> SpotFocusService:
        crisp = SettingsV1.instance().focus.crisp_label or None
        if crisp is not None:
            with suppress(Exception):
                if crisp not in self._mmc.getLoadedDevices():
                    crisp = None
        return SpotFocusService(
            self._mmc, crisp_label=crisp, source_spec=spec, led_intensity=led
        )

    def _ensure_service(self) -> SpotFocusService:
        """Create (or re-create) the service. Runs on a worker thread."""
        cfg = SettingsV1.instance().spot_focus
        spec = source_spec(cfg)
        owner = self._owner
        with owner.lock:
            svc = owner.service
            if svc is not None and (owner.dead or owner.spec != spec):
                with suppress(Exception):
                    svc.shutdown()
                svc = owner.service = None
            if svc is None:
                svc = self._factory(spec, cfg.led_intensity)
                svc.telemetry.connect(self._relay.on_telemetry)
                svc.thumb_msg.connect(self._relay.on_thumb)
                svc.fault.connect(self._relay.on_fault)
                svc.state.connect(self._relay.on_state)
                svc.moved.connect(self._relay.on_moved)
                owner.service, owner.spec, owner.dead = svc, spec, False
            return svc

    def shutdown(self) -> None:
        """Stop the loop (restoring CRISP) and end the worker process."""
        self._owner.shutdown()
        self._set_state("idle")

    def _run_async(
        self,
        name: str,
        work: Callable[[], Any],
        done: Callable[[Any], None],
        failed: Callable[[str], None],
    ) -> None:
        def target() -> None:
            try:
                result = work()
            except Exception as err:
                logger.warning("spot focus %s failed", name, exc_info=True)
                failed(str(err) or type(err).__name__)
            else:
                done(result)

        threading.Thread(target=target, name=f"spot-focus-{name}", daemon=True).start()

    # ------------------------------------------------------------------
    # settings <-> widgets
    # ------------------------------------------------------------------

    def refresh(self) -> None:
        """Re-read everything shown from ``SettingsV1.spot_focus``."""
        cfg = SettingsV1.instance().spot_focus
        self._loading = True
        try:
            for field, spin in self.param_spins.items():
                spin.setValue(getattr(cfg.params, field))
            self.led_spin.setValue(cfg.led_intensity or 0.0)
            self.exposure_spin.setValue(cfg.exposure_ms)
        finally:
            self._loading = False
        self._cal = cfg.to_calibration()
        self._refresh_calibration()
        self._update_enabled()

    def _refresh_calibration(self) -> None:
        cal = self._cal
        if cal is None:
            self.calibration_label.setText(_colored("not calibrated", ATTENTION_COLOR))
            return
        self.calibration_label.setText(
            html.escape(
                f"{cal.slope_px_per_um:.2f} px/µm along "
                f"({cal.ux:+.2f}, {cal.uy:+.2f}), residual {cal.residual_px:.2f} px, "
                f"{cal.n_points} points, setpoint ({cal.x0:.1f}, {cal.y0:.1f}) "
                f"{cal.created}"
            )
        )

    def _write(self) -> None:
        self._flush_timer.start()

    def _set_param(self, field: str, value: float) -> None:
        if self._loading:
            return
        cfg = SettingsV1.instance().spot_focus
        setattr(cfg.params, field, value)
        self._write()
        if self.state == "running" and (svc := self.service) is not None:
            svc.update_params(cfg.params.to_params())

    def _set_led(self, value: float) -> None:
        if self._loading:
            return
        SettingsV1.instance().spot_focus.led_intensity = value or None
        self._write()

    def _set_exposure(self, value: float) -> None:
        if self._loading:
            return
        SettingsV1.instance().spot_focus.exposure_ms = value
        self._write()
        if (svc := self.service) is not None and self.state != "fault":
            svc.set_camera(exposure_ms=value)

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    def _set_state(self, state: str) -> None:
        self.state = state
        alert = state == "fault"
        text = self._busy or state
        self.state_label.setText(
            _colored(text, ALERT_COLOR) if alert else html.escape(text)
        )
        spot_focus_flag.active = state == "running" or self._busy == "starting"
        self._update_enabled()

    def _update_enabled(self) -> None:
        running = self.state == "running"
        idle = not self._busy and not running
        self.start_button.setEnabled(idle and self._cal is not None)
        self.stop_button.setEnabled(running or self.state == "fault")
        self.calibrate_button.setEnabled(idle)
        if self._cal is None:
            self.start_button.setToolTip("Calibrate first")
        else:
            self.start_button.setToolTip("Hold the spot at its setpoint until Stop")

    def _on_state(self, state: str) -> None:
        if state in ("running", "stopped", "fault", "idle"):
            self._set_state(state)

    def _on_fault(self, reason: str) -> None:
        self.fault_label.setText(_colored(f"Fault: {reason}", ALERT_COLOR))
        if reason == "worker died":
            self._owner.dead = True
        self._set_state("fault")

    # ------------------------------------------------------------------
    # start / stop
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the loop (the Start button)."""
        if self._busy or self.state == "running":
            return
        settings = SettingsV1.instance()
        cfg = settings.spot_focus
        cal = cfg.to_calibration()
        if cal is None:
            self.fault_label.setText(_colored("Calibrate first.", ATTENTION_COLOR))
            return
        # The image lock must not run next to this one.
        if settings.image_focus.mode != "spot":
            settings.image_focus.mode = "spot"
            settings.flush()
            self.modeForced.emit()
        params = cfg.params.to_params()
        exposure, gain, roi, led = cfg.exposure_ms, cfg.gain, cfg.roi, cfg.led_intensity
        self.fault_label.setText("")
        self._telemetry = None
        self.chart.clear()
        self._busy = "starting"
        self._set_state("idle")

        def work() -> None:
            svc = self._ensure_service()
            svc.wait_ready()
            svc.set_camera(exposure_ms=exposure, gain=gain, roi=roi)
            svc.start(params, cal, led)

        self._run_async(
            "start",
            work,
            lambda _: self._relay.started.emit(),
            self._relay.startFailed.emit,
        )

    def _on_started(self) -> None:
        self._busy = ""
        svc = self.service
        # The service's own state signals may have beaten us here.
        self._set_state(svc.current_state if svc is not None else "idle")
        if self.state not in ("running", "fault"):
            self._set_state("running")

    def _on_start_failed(self, message: str) -> None:
        self._busy = ""
        self.fault_label.setText(_colored(f"Could not start: {message}", ALERT_COLOR))
        self._owner.dead = True  # build a fresh service next time
        self._set_state("idle")

    def stop(self) -> None:
        """Stop the loop and restore CRISP (the Stop button). Never moves P."""
        svc = self.service
        if svc is not None:
            try:
                svc.stop()
            except Exception as err:
                logger.warning("spot focus stop failed", exc_info=True)
                self.fault_label.setText(_colored(f"Stop failed: {err}", ALERT_COLOR))
        # After a fault the loop is already silent; Stop acknowledges it.
        self.fault_label.setText(
            self.fault_label.text() if self.state != "fault" else ""
        )
        self._set_state("stopped")

    # ------------------------------------------------------------------
    # calibrate
    # ------------------------------------------------------------------

    def calibrate(self) -> None:
        """Sweep P and fit the spot motion (the Calibrate button)."""
        if self._busy or self.state == "running":
            return
        self.fault_label.setText("")
        self._busy = "calibrating"
        self._set_state(self.state)

        def work() -> SpotCalibration:
            svc = self._ensure_service()
            cfg = SettingsV1.instance().spot_focus
            svc.wait_ready()
            svc.set_camera(exposure_ms=cfg.exposure_ms, gain=cfg.gain, roi=cfg.roi)
            return svc.calibrate()

        self._run_async(
            "calibrate",
            work,
            self._relay.calibrated.emit,
            self._relay.calibrateFailed.emit,
        )

    def _on_calibrated(self, cal: SpotCalibration) -> None:
        self._busy = ""
        settings = SettingsV1.instance()
        settings.spot_focus.calibration = cal.to_dict()
        settings.flush()
        self._cal = cal
        self._refresh_calibration()
        self._set_state(self.state)

    def _on_calibrate_failed(self, message: str) -> None:
        self._busy = ""
        self.fault_label.setText(
            _colored(f"Calibration failed: {message}", ALERT_COLOR)
        )
        self._set_state(self.state)

    # ------------------------------------------------------------------
    # live data
    # ------------------------------------------------------------------

    def _on_telemetry(self, msg: dict) -> None:
        self._telemetry = msg
        along, lateral = msg.get("along_um"), msg.get("lateral_px")
        if along is not None and msg.get("p_cmd_um") is not None:
            self.chart.add(
                float(msg.get("t") or time.monotonic()),
                float(along),
                float(msg["p_cmd_um"]),
            )
        labels = self.readout_labels
        labels["error"].setText("-" if along is None else f"{along:+.3f} µm")
        labels["lateral"].setText("-" if lateral is None else f"{lateral:+.2f} px")
        labels["fps"].setText(f"{msg.get('fps') or 0.0:.1f}")

    def _on_thumb(self, msg: dict) -> None:
        x0, y0, k = msg["x0"], msg["y0"], msg.get("scale", 1.0)

        def place(pt: tuple[float, float] | None) -> tuple[float, float] | None:
            return None if pt is None else ((pt[0] - x0) * k, (pt[1] - y0) * k)

        # Before Start the worker has no calibration yet; use the saved one.
        setpoint = msg.get("setpoint")
        if setpoint is None and self._cal is not None:
            setpoint = (self._cal.x0, self._cal.y0)
        self.thumb_view.set_thumb(msg["image"], place(msg.get("spot")), place(setpoint))

    def _refresh_stats(self) -> None:
        svc = self.service
        if svc is None:
            return
        st = svc.stats()
        self.readout_labels["moves"].setText(
            f"{st['moves_per_min']:.1f} ({st['n_moves']} total)"
        )
        self.readout_labels["move_ms"].setText(
            f"{st['mean_dt_ms']:.1f} / {st['max_dt_ms']:.1f} ms"
        )
