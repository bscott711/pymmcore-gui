"""Spot focus lock panel (the "Spot" mode of the focus-source dock).

The lock (see :mod:`~pymmcore_gui.asi_z_stack.spot_focus_service`) watches the
CRISP LED spot on a side camera in a worker process and holds piezo P by
nudging it rarely. This panel

- starts and stops the loop (it runs until Stop, independent of any MDA);
- calibrates it (a piezo sweep, on a background thread);
- runs the camera: SDK folder, Preview (which opens the full-resolution
  :class:`~pymmcore_gui.widgets._spot_live_view.SpotLiveView`), exposure, gain,
  channel and ROI, all applied live;
- shows a strip chart of the focus error and the commanded piezo over the last
  five minutes, and move statistics;
- edits the loop parameters and CRISP LED intensity.

Everything is written to ``SettingsV1.spot_focus``. The service is created
lazily on Preview, Start or Calibrate and shut down with the panel or the app. Its
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
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pymmcore_gui._qt.QtCore import QObject, QPointF, QRectF, Qt, QTimer, Signal
from pymmcore_gui._qt.QtGui import QColor, QPainter, QPen
from pymmcore_gui._qt.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.asi_z_stack.image_focus_events import spot_focus_flag
from pymmcore_gui.asi_z_stack.spot_focus_service import SpotFocusService
from pymmcore_gui.widgets._spot_live_view import SpotLiveView

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
#: How long Preview waits for the camera to open, s.
_OPEN_TIMEOUT_S = 30.0

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


def _rebuild_key(spec: dict | None) -> dict | None:
    """*spec* without the settings that change live (the analysis channel)."""
    if spec is None:
        return None
    key = dict(spec)
    analysis = dict(key.get("analysis") or {})
    analysis.pop("channel", None)
    key["analysis"] = analysis
    return key


def _colored(text: str, color: str) -> str:
    return f'<span style="color:{color}">{html.escape(text)}</span>'


# ---------------------------------------------------------------------------
# drawing widgets
# ---------------------------------------------------------------------------


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
    fault = Signal(str)
    state = Signal(str)
    moved = Signal(object)
    started = Signal()
    startFailed = Signal(str)
    previewStarted = Signal()
    previewFailed = Signal(str)
    calibrated = Signal(object)
    calibrateFailed = Signal(str)

    def on_telemetry(self, msg: dict) -> None:
        self.telemetry.emit(msg)

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
        self._preview_busy = False  # a thread is opening the camera
        self._sensor: tuple[int, int] | None = None  # known once the camera is open
        self._drop_preview_after_run = False  # view closed while the lock ran
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
        self.chart = StripChart()
        self.live_view = SpotLiveView(parent=self)
        self.live_view.roiDrawn.connect(self._on_roi_drawn)
        self.live_view.windowClosed.connect(self._on_view_closed)
        self.live_view.statsChanged.connect(self._on_view_stats)

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
        params_group = QGroupBox("Parameters")
        params_group.setLayout(form)
        camera_group = self._build_camera_group()

        # -- layout --------------------------------------------------------
        box = QGroupBox("Spot focus lock")
        layout = QVBoxLayout(box)
        layout.addLayout(buttons)
        layout.addWidget(self.state_label)
        layout.addWidget(self.fault_label)
        layout.addWidget(self.calibration_label)
        layout.addWidget(camera_group)
        layout.addWidget(self.chart, 1)
        layout.addLayout(readouts)
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
        self._relay.fault.connect(self._on_fault)
        self._relay.state.connect(self._on_state)
        self._relay.started.connect(self._on_started)
        self._relay.startFailed.connect(self._on_start_failed)
        self._relay.previewStarted.connect(self._on_preview_started)
        self._relay.previewFailed.connect(self._on_preview_failed)
        self._relay.calibrated.connect(self._on_calibrated)
        self._relay.calibrateFailed.connect(self._on_calibrate_failed)

        # Closures over the owner, not self: they run while self is going away.
        owner = self._owner
        self.destroyed.connect(lambda *_: owner.shutdown())
        if (app := QApplication.instance()) is not None:
            app.aboutToQuit.connect(owner.shutdown)

        self._set_state("idle")
        self.refresh()

    def _build_camera_group(self) -> QGroupBox:
        """The Camera section: SDK folder, Preview, exposure/gain/channel, ROI."""
        self.sdk_edit = QLineEdit()
        self.sdk_edit.setPlaceholderText("folder holding the vendor's toupcam.py")
        self.sdk_edit.setToolTip(
            "Folder with the ToupTek SDK's toupcam.py and its native library. "
            "Empty uses the TOUPCAM_SDK_DIR environment variable."
        )
        self.sdk_edit.editingFinished.connect(self._set_sdk_dir)
        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self._browse_sdk)
        sdk_row = QHBoxLayout()
        sdk_row.addWidget(self.sdk_edit, 1)
        sdk_row.addWidget(self.browse_button)

        self.camera_label = QLabel()
        self.camera_label.setTextFormat(Qt.TextFormat.PlainText)
        self.camera_label.setWordWrap(True)

        self.preview_button = QPushButton("Preview")
        self.preview_button.setCheckable(True)
        self.preview_button.setToolTip(
            "Stream the camera into the Spot camera window (full resolution)"
        )
        self.preview_button.clicked.connect(self.set_preview)
        self.show_view_button = QPushButton("Show view")
        self.show_view_button.setToolTip("Bring the Spot camera window to the front")
        self.show_view_button.clicked.connect(self._show_live_view)
        self.camera_readout = QLabel("-")
        preview_row = QHBoxLayout()
        preview_row.addWidget(self.preview_button)
        preview_row.addWidget(self.show_view_button)
        preview_row.addWidget(self.camera_readout, 1)

        self.exposure_spin = self._spin(
            0.1, 15000.0, 1.0, 2, " ms", "Camera exposure (applies at once)"
        )
        self.exposure_spin.valueChanged.connect(self._set_exposure)
        self.gain_spin = QSpinBox()
        self.gain_spin.setRange(100, 5000)
        self.gain_spin.setSingleStep(100)
        self.gain_spin.setSuffix(" %")
        self.gain_spin.setKeyboardTracking(False)
        self.gain_spin.setToolTip("Camera gain, percent (100 = unity)")
        self.gain_spin.valueChanged.connect(self._set_gain)
        self.channel_combo = QComboBox()
        self.channel_combo.addItems(["red", "sum"])
        self.channel_combo.setToolTip(
            "Bayer sites the spot is measured on: red only (the LED is 720 nm) "
            "or the sum of all"
        )
        self.channel_combo.currentTextChanged.connect(self._set_channel)

        self.roi_spins: dict[str, QSpinBox] = {}
        roi_row = QHBoxLayout()
        for name in ("x", "y", "w", "h"):
            spin = QSpinBox()
            spin.setRange(0, 16384)
            spin.setSingleStep(2)
            spin.setKeyboardTracking(False)
            spin.setToolTip(f"ROI {name} in sensor pixels (even); 0 width = full frame")
            self.roi_spins[name] = spin
            roi_row.addWidget(QLabel(name))
            roi_row.addWidget(spin)
        self.apply_roi_button = QPushButton("Apply ROI")
        self.apply_roi_button.clicked.connect(self.apply_roi)
        self.full_frame_button = QPushButton("Full frame")
        self.full_frame_button.clicked.connect(self.full_frame)
        roi_buttons = QHBoxLayout()
        roi_buttons.addWidget(self.apply_roi_button)
        roi_buttons.addWidget(self.full_frame_button)
        roi_buttons.addStretch()

        grid = QGridLayout()
        grid.addWidget(QLabel("Exposure"), 0, 0)
        grid.addWidget(self.exposure_spin, 0, 1)
        grid.addWidget(QLabel("Gain"), 0, 2)
        grid.addWidget(self.gain_spin, 0, 3)
        grid.addWidget(QLabel("Channel"), 0, 4)
        grid.addWidget(self.channel_combo, 0, 5)
        grid.setColumnStretch(6, 1)

        group = QGroupBox("Camera")
        lay = QVBoxLayout(group)
        lay.addWidget(QLabel("SDK folder"))
        lay.addLayout(sdk_row)
        lay.addWidget(self.camera_label)
        lay.addLayout(preview_row)
        lay.addLayout(grid)
        lay.addWidget(QLabel("ROI (sensor pixels)"))
        lay.addLayout(roi_row)
        lay.addLayout(roi_buttons)
        return group

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
            if svc is not None and (
                owner.dead or _rebuild_key(owner.spec) != _rebuild_key(spec)
            ):
                with suppress(Exception):
                    svc.shutdown()
                svc = owner.service = None
            if svc is None:
                svc = self._factory(spec, cfg.led_intensity)
                svc.telemetry.connect(self._relay.on_telemetry)
                svc.fault.connect(self._relay.on_fault)
                svc.state.connect(self._relay.on_state)
                svc.moved.connect(self._relay.on_moved)
                owner.service, owner.spec, owner.dead = svc, spec, False
            return svc

    def shutdown(self) -> None:
        """Stop the loop (restoring CRISP) and end the worker process."""
        self.live_view.set_service(None)
        self.live_view.hide()
        self.preview_button.setChecked(False)
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
            self.gain_spin.setValue(cfg.gain)
            self.channel_combo.setCurrentText(cfg.channel)
            self.sdk_edit.setText(cfg.source.sdk_dir or "")
            self._load_roi_spins(cfg.roi)
        finally:
            self._loading = False
        self._cal = cfg.to_calibration()
        self.live_view.set_setpoint(
            None if self._cal is None else (self._cal.x0, self._cal.y0)
        )
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

    def _apply_camera(self, **kwargs: Any) -> None:
        """Send a live camera change to the worker (if there is one)."""
        svc = self.service
        if svc is None or self.state == "fault":
            return
        try:
            svc.set_camera(**kwargs)
        except Exception as err:
            logger.warning("spot focus set_camera failed", exc_info=True)
            self.fault_label.setText(_colored(f"Camera: {err}", ALERT_COLOR))

    def _set_exposure(self, value: float) -> None:
        if self._loading:
            return
        SettingsV1.instance().spot_focus.exposure_ms = value
        self._write()
        self._apply_camera(exposure_ms=value)

    def _set_gain(self, value: int) -> None:
        if self._loading:
            return
        SettingsV1.instance().spot_focus.gain = int(value)
        self._write()
        self._apply_camera(gain=int(value))

    def _set_channel(self, value: str) -> None:
        if self._loading or value not in ("red", "sum"):
            return
        SettingsV1.instance().spot_focus.channel = value  # type: ignore[assignment]
        self._write()
        self._apply_camera(channel=value)

    # -- SDK folder -----------------------------------------------------

    def _set_sdk_dir(self) -> None:
        if self._loading:
            return
        text = self.sdk_edit.text().strip()
        cfg = SettingsV1.instance().spot_focus
        if (text or None) == cfg.source.sdk_dir:
            return
        cfg.source.sdk_dir = text or None
        self._write()
        # A running camera keeps the old folder; the next Preview rebuilds.
        if self.preview_button.isChecked() and self.state != "running":
            self._stop_preview()

    def _browse_sdk(self) -> None:
        start = self.sdk_edit.text().strip() or str(Path.home())
        folder = QFileDialog.getExistingDirectory(
            self, "Folder with the ToupTek SDK (toupcam.py)", start
        )
        if folder:
            self.sdk_edit.setText(folder)
            self._set_sdk_dir()

    # -- ROI -------------------------------------------------------------

    def _load_roi_spins(self, roi: tuple[int, int, int, int] | None) -> None:
        if roi is None:
            sw, sh = self._sensor or (0, 0)
            roi = (0, 0, sw, sh)
        for spin, value in zip(self.roi_spins.values(), roi, strict=True):
            spin.setValue(int(value))

    def apply_roi(self) -> None:
        """Apply the ROI spins (a zero width or height means full frame)."""
        x, y, w, h = (spin.value() for spin in self.roi_spins.values())
        x, y, w, h = x // 2 * 2, y // 2 * 2, w // 2 * 2, h // 2 * 2
        if w <= 0 or h <= 0:
            self.full_frame()
            return
        if self._sensor is not None:
            sw, sh = self._sensor
            x, y = min(x, sw - 2), min(y, sh - 2)
            w, h = min(w, sw - x) // 2 * 2, min(h, sh - y) // 2 * 2
        self._set_roi((x, y, w, h))

    def full_frame(self) -> None:
        """Use the whole sensor."""
        self._set_roi(None)

    def _set_roi(self, roi: tuple[int, int, int, int] | None) -> None:
        if roi is not None and self._sensor is not None:
            if roi == (0, 0, *self._sensor):
                roi = None
        SettingsV1.instance().spot_focus.roi = roi
        self._write()
        self._loading = True
        try:
            self._load_roi_spins(roi)
        finally:
            self._loading = False
        self._apply_camera(roi=roi or (0, 0, 0, 0))

    def _on_roi_drawn(self, x: int, y: int, w: int, h: int) -> None:
        self._set_roi((x, y, w, h))

    # -- camera info -----------------------------------------------------

    def _apply_camera_info(self, info: dict) -> None:
        """Show the camera and set the spin ranges from what it reported."""
        if not info:
            return
        self._loading = True
        try:
            if rng := info.get("exposure_range_ms"):
                self.exposure_spin.setRange(float(rng[0]), float(rng[1]))
            if rng := info.get("gain_range"):
                self.gain_spin.setRange(int(rng[0]), int(rng[1]))
            sensor = info.get("sensor")
            if sensor:
                self._sensor = (int(sensor[0]), int(sensor[1]))
                sw, sh = self._sensor
                for name, limit in (("x", sw - 2), ("y", sh - 2), ("w", sw), ("h", sh)):
                    self.roi_spins[name].setMaximum(limit)
            cfg = SettingsV1.instance().spot_focus
            self.exposure_spin.setValue(cfg.exposure_ms)
            self.gain_spin.setValue(cfg.gain)
            self._load_roi_spins(cfg.roi)
        finally:
            self._loading = False
        parts = [str(info.get("model") or "camera")]
        if info.get("serial"):
            parts.append(f"serial {info['serial']}")
        if sensor:
            parts.append(f"{sensor[0]} x {sensor[1]}")
        if info.get("bits"):
            parts.append(f"{info['bits']}-bit")
        self.camera_label.setText("  |  ".join(parts))

    # -- preview ---------------------------------------------------------

    def set_preview(self, on: bool) -> None:
        """Turn the camera preview (and the live view window) on or off."""
        if on:
            self._start_preview()
        else:
            self._stop_preview()

    def _start_preview(self) -> None:
        if self._preview_busy:
            return
        if self.state == "running" and self.service is not None:
            # The lock already streams frames; only the window is missing.
            self.preview_button.setChecked(True)
            self._drop_preview_after_run = False
            self._show_live_view()
            return
        cfg = SettingsV1.instance().spot_focus
        exposure, gain, channel = cfg.exposure_ms, cfg.gain, cfg.channel
        roi = cfg.roi or (0, 0, 0, 0)
        self.fault_label.setText("")
        self._preview_busy = True
        self.preview_button.setChecked(True)
        self.preview_button.setEnabled(False)
        self.camera_label.setText("opening the camera...")

        def work() -> SpotFocusService:
            svc = self._ensure_service()
            deadline = time.monotonic() + _OPEN_TIMEOUT_S
            while not svc.wait_ready(0.2):
                if svc.current_state == "fault":
                    raise RuntimeError("the camera could not be opened")
                if time.monotonic() > deadline:
                    raise RuntimeError("the camera did not become ready")
            svc.set_camera(exposure_ms=exposure, gain=gain, roi=roi, channel=channel)
            svc.start_preview()
            return svc

        self._run_async(
            "preview",
            work,
            lambda _: self._relay.previewStarted.emit(),
            self._relay.previewFailed.emit,
        )

    def _on_preview_started(self) -> None:
        self._preview_busy = False
        self.preview_button.setEnabled(True)
        self.preview_button.setChecked(True)
        svc = self.service
        if svc is None:
            return
        if self.state == "fault":
            self._set_state("idle")
        self.fault_label.setText("")
        self._apply_camera_info(self._camera_info(svc))
        self.live_view.set_service(svc)
        self._show_live_view()

    def _on_preview_failed(self, message: str) -> None:
        self._preview_busy = False
        self.preview_button.setEnabled(True)
        self.preview_button.setChecked(False)
        self.camera_label.setText("no camera")
        if self.state != "fault":  # a worker fault already says why
            self.fault_label.setText(_colored(f"Camera: {message}", ALERT_COLOR))
        self._owner.dead = True  # build a fresh service on the next Preview

    def _stop_preview(self) -> None:
        if self._preview_busy:
            return
        if self.state == "running" or self._busy == "starting":
            # The lock needs the camera: leave it streaming.
            self.preview_button.setChecked(True)
            self.fault_label.setText(
                _colored("Preview stays on while the lock runs.", ATTENTION_COLOR)
            )
            return
        self.preview_button.setChecked(False)
        svc = self.service
        if svc is not None:
            try:
                svc.stop_preview()
            except Exception:
                logger.warning("spot focus stop_preview failed", exc_info=True)
        self.live_view.hide()  # hide(), not close(): no windowClosed echo
        self.live_view.clear()
        self.camera_readout.setText("-")

    def _show_live_view(self) -> None:
        svc = self.service
        if svc is None:
            return
        self.live_view.set_service(svc)
        self.live_view.show()
        self.live_view.raise_()
        self.live_view.activateWindow()

    def _on_view_closed(self) -> None:
        """The user closed the Spot camera window."""
        if self.state == "running" or self._busy == "starting":
            self.preview_button.setChecked(False)
            self._drop_preview_after_run = True  # stop streaming after Stop
            return
        self._stop_preview()

    def _on_view_stats(self, fps: float, max_value: int, saturated_pct: float) -> None:
        sat = f"{saturated_pct:.2f}% saturated"
        if saturated_pct >= 1.0:
            sat = _colored(sat, ATTENTION_COLOR)
        else:
            sat = html.escape(sat)
        self.camera_readout.setText(f"{fps:.1f} fps | max {max_value} | {sat}")

    @staticmethod
    def _camera_info(svc: SpotFocusService) -> dict:
        info = getattr(svc, "camera_info", None)
        return dict(info) if info else dict(svc.info)

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
        if state == "preview":
            return  # the camera streams; the lock itself is not running
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
        exposure, gain, led = cfg.exposure_ms, cfg.gain, cfg.led_intensity
        roi, channel = cfg.roi or (0, 0, 0, 0), cfg.channel
        self.fault_label.setText("")
        self._telemetry = None
        self.chart.clear()
        self._busy = "starting"
        self._set_state("idle")

        def work() -> None:
            svc = self._ensure_service()
            svc.wait_ready()
            svc.set_camera(exposure_ms=exposure, gain=gain, roi=roi, channel=channel)
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
        if svc is not None:
            self.live_view.set_service(svc)
            if self.live_view.isVisible():
                self.preview_button.setChecked(True)
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
        if self._drop_preview_after_run:
            self._drop_preview_after_run = False
            self._stop_preview()

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
            svc.set_camera(
                exposure_ms=cfg.exposure_ms,
                gain=cfg.gain,
                roi=cfg.roi or (0, 0, 0, 0),
                channel=cfg.channel,
            )
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
        self.live_view.set_setpoint((cal.x0, cal.y0))
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
