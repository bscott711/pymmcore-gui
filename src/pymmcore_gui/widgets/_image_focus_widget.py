"""Dock widget for the image-based focus lock.

The lock (see :mod:`~pymmcore_gui.asi_z_stack.image_focus_run`) holds the piezo
by watching a box on the camera image instead of using CRISP-P. This panel

- picks the focus source for the next acquisition (CRISP-P / Image / Spot / Off).
  Spot is not tied to an acquisition: its panel
  (:mod:`~pymmcore_gui.widgets._spot_focus_panel`) starts and stops it, and the
  image lock stays out of the way while it is selected;
- shows the focus box drawn in the MDA viewer ("Focus box" button) and whether
  it is calibrated, and runs the piezo-sweep calibration for it;
- shows what the lock is doing while a run is in progress;
- edits the lock's limits.

Everything is written straight to ``SettingsV1.image_focus``. The engine takes
a snapshot of those settings when a run starts, so edits apply from the **next**
acquisition. The lock reports from the MDA thread, so every signal is forwarded
through a ``Signal`` on a GUI-thread :class:`QObject` (Qt queues the call).
"""

from __future__ import annotations

import html
import logging
from contextlib import suppress
from datetime import datetime
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from pymmcore_gui._qt.QtCore import QObject, Qt, QTimer, Signal
from pymmcore_gui._qt.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.asi_z_stack.image_focus_events import (
    FocusLockReport,
    focus_lock_signals,
)
from pymmcore_gui.asi_z_stack.image_focus_run import (
    DEFAULT_CALIBRATION_STEPS_UM,
    build_calibration_sequence,
    window_for_box,
)
from pymmcore_gui.widgets._spot_focus_panel import SpotFocusPanel

if TYPE_CHECKING:
    from collections.abc import Callable

    import useq

    from pymmcore_gui.asi_z_stack.image_focus import FocusCalibration

logger = logging.getLogger(__name__)

#: Rich-text colours for a lock that needs attention / has stopped.
ALERT_COLOR = "#d0342c"
ATTENTION_COLOR = "#e08a00"

#: Delay before writing edits to disk; they apply in memory immediately.
_FLUSH_DELAY_MS = 500

#: ``stopped`` reason sent when a run simply ended (not a problem).
RUN_FINISHED = "run finished"

_SOURCES = {
    "crisp": "CRISP-P (hardware autofocus)",
    "image": "Image (focus box)",
    "spot": "Spot (CRISP LED on side camera)",
    "off": "Off (piezo not touched)",
}

# (settings field, label, min, max, step, decimals, suffix, tooltip)
_LIMITS: tuple[tuple[str, str, float, float, float, int, str, str], ...] = (
    (
        "loop_gain",
        "Loop gain",
        0.05,
        1.0,
        0.05,
        2,
        "",
        "Fraction of the error corrected per volume",
    ),
    (
        "deadband_rows",
        "Deadband",
        0.0,
        20.0,
        0.1,
        2,
        " rows",
        "Errors up to this size are left alone",
    ),
    (
        "max_step_um",
        "Max step",
        0.01,
        5.0,
        0.05,
        2,
        " µm",
        "Largest piezo move in one volume",
    ),
    (
        "max_excursion_um",
        "Max excursion",
        0.1,
        20.0,
        0.5,
        1,
        " µm",
        "Largest distance from where the run started",
    ),
    (
        "min_confidence",
        "Min confidence",
        0.0,
        1.0,
        0.05,
        2,
        "",
        "Minimum peak correlation to trust a measurement",
    ),
    (
        "max_centroid_disagreement_rows",
        "Centroid check",
        0.0,
        50.0,
        0.5,
        1,
        " rows",
        "Largest gap between the correlation and centroid shifts",
    ),
    (
        "min_signal",
        "Min signal",
        0.0,
        1e6,
        1.0,
        1,
        " counts",
        "Minimum background-subtracted profile mean",
    ),
    (
        "max_strikes",
        "Max strikes",
        1,
        100,
        1,
        0,
        "",
        "Rejected measurements in a row before the lock gives up",
    ),
    (
        "max_saturated",
        "Max clipped moves",
        1,
        100,
        1,
        0,
        "",
        "Clipped moves in a row before the lock gives up",
    ),
    (
        "min_r2",
        "Min calibration R²",
        0.0,
        1.0,
        0.01,
        2,
        "",
        "Calibrations fitting worse than this are refused",
    ),
)


def calibration_steps(range_um: float, step_um: float) -> list[float]:
    """Piezo offsets ``-n*step .. +n*step`` (including 0) within ``+/-range_um``."""
    if step_um <= 0:
        raise ValueError("The calibration step must be positive.")
    n = int(range_um / step_um + 1e-9)
    return [round(i * step_um, 6) for i in range(-n, n + 1)]


def age_text(date: str, now: datetime | None = None) -> str:
    """``"2026-10-26 14:03 (2 h ago)"`` for an ISO timestamp (else the input)."""
    try:
        when = datetime.fromisoformat(date)
    except ValueError:
        return date
    seconds = ((now or datetime.now(when.tzinfo)) - when).total_seconds()
    if seconds < 90:
        age = "just now"
    elif seconds < 3600 * 1.5:
        age = f"{seconds / 60:.0f} min ago"
    elif seconds < 86400 * 1.5:
        age = f"{seconds / 3600:.0f} h ago"
    else:
        age = f"{seconds / 86400:.0f} d ago"
    return f"{when:%Y-%m-%d %H:%M} ({age})"


def _colored(text: str, color: str) -> str:
    return f'<span style="color:{color}">{html.escape(text)}</span>'


def _fit_text(fit: FocusCalibration) -> str:
    sign = "+" if fit.rows_per_um >= 0 else "-"
    lo, hi = fit.range_um
    return (
        f"{abs(fit.rows_per_um):.2f} rows/µm (sign {sign}), R² {fit.r2:.3f}, "
        f"linear range {lo:+.2f}..{hi:+.2f} µm, {fit.n_points} points"
    )


class _Relay(QObject):
    """Hands the lock's MDA-thread signals to the GUI thread.

    The ``on_*`` methods are what gets connected to the psygnal signals (they
    run on whichever thread emits); emitting the matching ``Signal`` queues the
    call to the receiver, which lives on the GUI thread.
    """

    report = Signal(object)
    calibrated = Signal(str, object)
    stopped = Signal(str)
    boxChanged = Signal()
    mdaRunning = Signal(bool)

    def on_report(self, report: FocusLockReport) -> None:
        self.report.emit(report)

    def on_calibrated(self, key: str, fit: FocusCalibration) -> None:
        self.calibrated.emit(key, fit)

    def on_stopped(self, reason: str) -> None:
        self.stopped.emit(reason)

    def on_box_changed(self) -> None:
        self.boxChanged.emit()

    def on_mda_started(self, *_: object) -> None:
        self.mdaRunning.emit(True)

    def on_mda_finished(self, *_: object) -> None:
        self.mdaRunning.emit(False)


class ImageFocusWidget(QWidget):
    """Choose, calibrate and watch the image-based focus lock.

    Parameters
    ----------
    mmcore : CMMCorePlus
        The core that runs the calibration sweep.
    parent : QWidget | None
        Parent widget.
    sequence_source : Callable[[], useq.MDASequence] | None
        Returns the sequence a calibration is based on (the MDA widget's current
        plan: its channels and z plan). ``None`` disables calibration.
    """

    def __init__(
        self,
        mmcore: CMMCorePlus | None = None,
        parent: QWidget | None = None,
        *,
        sequence_source: Callable[[], useq.MDASequence] | None = None,
    ) -> None:
        super().__init__(parent)
        self._mmc = mmcore or CMMCorePlus.instance()
        self._sequence_source = sequence_source
        self._mda_running = False
        self._calibrating = False
        self._run_live = False  # reports of the current run have started
        self._start_piezo: float | None = None
        self.state_is_alert = False
        self._loading = False  # filling widgets from settings, don't write back

        # -- focus source ------------------------------------------------
        self.mode_buttons: dict[str, QRadioButton] = {}
        source_box = QGroupBox("Focus source for the piezo")
        source_layout = QVBoxLayout(source_box)
        for key, text in _SOURCES.items():
            button = QRadioButton(text)
            button.toggled.connect(lambda on, k=key: on and self._set_mode(k))
            self.mode_buttons[key] = button
            source_layout.addWidget(button)
        note = QLabel(
            "Applies from the next acquisition. CRISP-Z is not affected; "
            "Image unlocks CRISP-P."
        )
        note.setWordWrap(True)
        source_layout.addWidget(note)

        # -- focus box -----------------------------------------------------
        self.box_label = QLabel()
        self.box_label.setWordWrap(True)
        self.calibration_label = QLabel()
        self.calibration_label.setWordWrap(True)
        self.calibration_label.setTextFormat(Qt.TextFormat.RichText)
        self.clear_button = QPushButton("Clear box")
        self.clear_button.clicked.connect(self._clear_box)
        hint = QLabel("Draw the box in the acquisition viewer with 'Focus box'.")
        hint.setWordWrap(True)
        box_form = QFormLayout()
        box_form.addRow("Box", self.box_label)
        box_form.addRow("Calibration", self.calibration_label)
        box_group = QGroupBox("Focus box")
        box_layout = QVBoxLayout(box_group)
        box_layout.addLayout(box_form)
        clear_row = QHBoxLayout()
        clear_row.addWidget(self.clear_button)
        clear_row.addStretch()
        box_layout.addLayout(clear_row)
        box_layout.addWidget(hint)

        # -- calibrate -----------------------------------------------------
        default_range = max(abs(s) for s in DEFAULT_CALIBRATION_STEPS_UM)
        self.range_spin = QDoubleSpinBox()
        self.range_spin.setRange(0.1, 10.0)
        self.range_spin.setSingleStep(0.5)
        self.range_spin.setDecimals(2)
        self.range_spin.setPrefix("± ")
        self.range_spin.setSuffix(" µm")
        self.range_spin.setValue(default_range)
        self.range_spin.setToolTip(
            "The piezo is swept this far either side of where it is"
        )
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.05, 5.0)
        self.step_spin.setSingleStep(0.1)
        self.step_spin.setDecimals(2)
        self.step_spin.setSuffix(" µm")
        self.step_spin.setValue(0.5)
        self.reps_spin = QSpinBox()
        self.reps_spin.setRange(1, 20)
        self.reps_spin.setValue(2)
        self.reps_spin.setToolTip("Volumes taken at each piezo step")
        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.calibrate)
        self.calibration_result = QLabel()
        self.calibration_result.setWordWrap(True)
        self.calibration_result.setTextFormat(Qt.TextFormat.RichText)
        self.calibration_result.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        cal_form = QFormLayout()
        cal_form.addRow("Range", self.range_spin)
        cal_form.addRow("Step", self.step_spin)
        cal_form.addRow("Repeats", self.reps_spin)
        cal_group = QGroupBox("Calibrate")
        cal_layout = QVBoxLayout(cal_group)
        cal_layout.addLayout(cal_form)
        cal_row = QHBoxLayout()
        cal_row.addWidget(self.calibrate_button)
        cal_row.addStretch()
        cal_layout.addLayout(cal_row)
        cal_layout.addWidget(self.calibration_result)

        # -- status --------------------------------------------------------
        self.state_label = QLabel()
        self.state_label.setTextFormat(Qt.TextFormat.RichText)
        self.state_label.setWordWrap(True)
        self.status_labels: dict[str, QLabel] = {}
        status_form = QFormLayout()
        for key, text in (
            ("volume", "Volume"),
            ("error", "Error"),
            ("move", "Move"),
            ("total", "Total offset"),
            ("confidence", "Confidence"),
            ("message", "Message"),
        ):
            label = QLabel("-")
            label.setWordWrap(True)
            self.status_labels[key] = label
            status_form.addRow(text, label)
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        status_layout.addWidget(self.state_label)
        status_layout.addLayout(status_form)

        # -- limits (collapsed) --------------------------------------------
        self.limit_spins: dict[str, QDoubleSpinBox | QSpinBox] = {}
        limits_form = QFormLayout()
        for field, label, lo, hi, step, decimals, suffix, tip in _LIMITS:
            spin: QDoubleSpinBox | QSpinBox
            if decimals == 0:
                spin = QSpinBox()
                spin.setRange(int(lo), int(hi))
                spin.setSingleStep(int(step))
            else:
                spin = QDoubleSpinBox()
                spin.setRange(lo, hi)
                spin.setSingleStep(step)
                spin.setDecimals(decimals)
            spin.setSuffix(suffix)
            spin.setToolTip(tip)
            spin.setKeyboardTracking(False)
            spin.valueChanged.connect(lambda v, f=field: self._set_limit(f, v))
            self.limit_spins[field] = spin
            limits_form.addRow(label, spin)
        self.end_of_run = QComboBox()
        self.end_of_run.addItem("Return the piezo to where it started", "restore")
        self.end_of_run.addItem("Leave the piezo where the lock put it", "stay")
        self.end_of_run.currentIndexChanged.connect(self._set_end_of_run)
        limits_form.addRow("End of run", self.end_of_run)
        self.limits_group = QGroupBox("Limits")
        self.limits_group.setCheckable(True)
        self.limits_group.setChecked(False)
        limits_body = QWidget()
        limits_body.setLayout(limits_form)
        limits_layout = QVBoxLayout(self.limits_group)
        limits_layout.addWidget(limits_body)
        self.limits_group.toggled.connect(limits_body.setVisible)
        limits_body.setVisible(False)

        self.spot_panel = SpotFocusPanel(self._mmc, self)
        self.spot_panel.modeForced.connect(self.refresh)

        layout = QVBoxLayout(self)
        layout.addWidget(source_box)
        layout.addWidget(self.spot_panel)
        layout.addWidget(box_group)
        layout.addWidget(cal_group)
        layout.addWidget(status_group)
        layout.addWidget(self.limits_group)
        layout.addStretch()

        # Disk writes are debounced so spinning a box doesn't start one per step.
        self._flush_timer = QTimer(self)
        self._flush_timer.setSingleShot(True)
        self._flush_timer.setInterval(_FLUSH_DELAY_MS)
        self._flush_timer.timeout.connect(lambda: SettingsV1.instance().flush())

        # -- signals from the MDA thread -----------------------------------
        self._relay = _Relay(self)
        self._relay.report.connect(self._on_report)
        self._relay.calibrated.connect(self._on_calibrated)
        self._relay.stopped.connect(self._on_stopped)
        self._relay.boxChanged.connect(self.refresh)
        self._relay.mdaRunning.connect(self._set_mda_running)
        self._connections = [
            (focus_lock_signals.report, self._relay.on_report),
            (focus_lock_signals.calibrated, self._relay.on_calibrated),
            (focus_lock_signals.stopped, self._relay.on_stopped),
            (focus_lock_signals.box_changed, self._relay.on_box_changed),
            (self._mmc.mda.events.sequenceStarted, self._relay.on_mda_started),
            (self._mmc.mda.events.sequenceFinished, self._relay.on_mda_finished),
        ]
        for signal, slot in self._connections:
            signal.connect(slot)
        # A closure over the connections, not self: it runs while self is gone.
        connections = list(self._connections)
        self.destroyed.connect(lambda *_: _disconnect_all(connections))

        self._mda_running = bool(self._mmc.mda.is_running())
        self._set_state("idle")
        self.refresh()

    # ------------------------------------------------------------------
    # settings -> widgets
    # ------------------------------------------------------------------

    def refresh(self) -> None:
        """Re-read everything shown from ``SettingsV1.image_focus``."""
        cfg = SettingsV1.instance().image_focus
        self._loading = True
        try:
            self.mode_buttons[cfg.mode].setChecked(True)
            self.spot_panel.setVisible(cfg.mode == "spot")
            for field, spin in self.limit_spins.items():
                spin.setValue(getattr(cfg, field))
            self.end_of_run.setCurrentIndex(
                max(0, self.end_of_run.findData(cfg.end_of_run))
            )
        finally:
            self._loading = False
        self._refresh_box()
        self._update_enabled()

    def focus_box_key(self) -> str | None:
        """The calibration key of the saved box, ``None`` without a box."""
        from pymmcore_gui.asi_z_stack.image_focus import calibration_key

        settings = SettingsV1.instance()
        if (saved := settings.image_focus.box) is None:
            return None
        box = saved.to_box()
        window = window_for_box(box, settings.spectral)
        return calibration_key(box.camera, box.preset, window)

    def _refresh_box(self) -> None:
        settings = SettingsV1.instance()
        cfg = settings.image_focus
        if (box := cfg.box) is None:
            self.box_label.setText("none drawn")
            self.calibration_label.setText(_colored("no box", ATTENTION_COLOR))
            return
        x, y, w, h = box.rect
        window = window_for_box(box.to_box(), settings.spectral)
        self.box_label.setText(
            f"{box.camera}, {box.preset}, slice {box.slice_index}\n"
            f"x {x}, y {y}, {w} x {h} px"
            + (f"\nspectral window: {window}" if window else "")
        )
        record = cfg.calibrations.get(self.focus_box_key() or "")
        if record is None:
            self.calibration_label.setText(_colored("not calibrated", ATTENTION_COLOR))
        else:
            self.calibration_label.setText(
                html.escape(_fit_text(record.to_calibration()))
                + "<br>"
                + html.escape(age_text(record.date))
            )

    def _update_enabled(self) -> None:
        has_box = SettingsV1.instance().image_focus.box is not None
        self.clear_button.setEnabled(has_box and not self._mda_running)
        can = has_box and not self._mda_running and self._sequence_source is not None
        self.calibrate_button.setEnabled(can)
        if self._mda_running:
            tip = "An acquisition is running"
        elif not has_box:
            tip = "Draw a focus box in the acquisition viewer first"
        elif self._sequence_source is None:
            tip = "No acquisition set up to calibrate against"
        else:
            tip = "Sweep the piezo and measure how far the box content moves"
        self.calibrate_button.setToolTip(tip)

    def _set_mda_running(self, running: bool) -> None:
        self._mda_running = running
        self._update_enabled()

    # ------------------------------------------------------------------
    # widgets -> settings
    # ------------------------------------------------------------------

    def _write(self) -> None:
        self._flush_timer.start()

    def _set_mode(self, mode: str) -> None:
        if self._loading:
            return
        if mode != "spot" and self.spot_panel.state in ("running", "fault"):
            self.spot_panel.stop()  # one owner of the piezo at a time
        SettingsV1.instance().image_focus.mode = mode  # type: ignore[assignment]
        SettingsV1.instance().flush()
        self.spot_panel.setVisible(mode == "spot")

    def _set_limit(self, field: str, value: float) -> None:
        if self._loading:
            return
        setattr(SettingsV1.instance().image_focus, field, value)
        self._write()

    def _set_end_of_run(self) -> None:
        if self._loading:
            return
        SettingsV1.instance().image_focus.end_of_run = self.end_of_run.currentData()
        self._write()

    def _clear_box(self) -> None:
        settings = SettingsV1.instance()
        settings.image_focus.box = None
        settings.flush()
        self._refresh_box()
        self._update_enabled()
        # Viewers showing the box hide it.
        try:
            focus_lock_signals.box_changed.emit()
        except Exception:
            logger.warning("A focus box listener raised.", exc_info=True)

    # ------------------------------------------------------------------
    # calibrate
    # ------------------------------------------------------------------

    def calibrate(self) -> None:
        """Run the piezo sweep for the saved box (the Calibrate button)."""
        settings = SettingsV1.instance()
        if (saved := settings.image_focus.box) is None:
            return
        try:
            if self._sequence_source is None:
                raise ValueError("No acquisition set up to calibrate against.")
            box = saved.to_box()
            window = window_for_box(box, settings.spectral)
            sequence = build_calibration_sequence(
                self._sequence_source(),
                box,
                window,
                steps_um=calibration_steps(
                    self.range_spin.value(), self.step_spin.value()
                ),
                reps=self.reps_spin.value(),
            )
            self._calibrating = True
            self._mmc.run_mda(sequence)
        except Exception as err:
            self._calibrating = False
            logger.warning("Could not start the focus calibration: %s", err)
            self.calibration_result.setText(
                _colored(f"Not started: {err}", ALERT_COLOR)
            )
            return
        self.calibration_result.setText("Calibrating: sweeping the piezo...")

    def _on_calibrated(self, key: str, fit: FocusCalibration) -> None:
        self.calibration_result.setText(
            "Calibrated: " + html.escape(_fit_text(fit)) + f"<br>key {html.escape(key)}"
        )
        self._refresh_box()

    # ------------------------------------------------------------------
    # live status
    # ------------------------------------------------------------------

    def _set_state(self, text: str, alert: bool = False) -> None:
        self.state_is_alert = alert
        self.state_label.setText(
            _colored(text, ALERT_COLOR) if alert else html.escape(text)
        )

    def _on_report(self, report: FocusLockReport) -> None:
        if not self._run_live:  # first report of a run
            self._run_live = True
            self._start_piezo = report.piezo_um
        d = report.decision
        labels = self.status_labels
        labels["volume"].setText(", ".join(f"{k}={v}" for k, v in report.index.items()))
        labels["message"].setText(report.message or "-")
        if report.piezo_um is not None and self._start_piezo is not None:
            labels["total"].setText(f"{report.piezo_um - self._start_piezo:+.3f} µm")
        if d is None:
            for key in ("error", "move", "confidence"):
                labels[key].setText("-")
        else:
            labels["error"].setText(f"{d.error_rows:+.2f} rows = {d.error_um:+.3f} µm")
            labels["move"].setText(f"{d.move_um:+.3f} µm")
            labels["confidence"].setText(f"{d.confidence:.2f}")

        if report.mode == "calibrate":
            self._set_state(f"calibrating: {report.message}")
        elif d is None and report.message == "reference captured":
            self._set_state("locked (reference captured)")
        elif d is None:
            self._set_state(f"held: {report.message}")
        elif d.held:
            self._set_state(f"held: {d.reason or 'measurement rejected'}")
        else:
            self._set_state("locked")

    def _on_stopped(self, reason: str) -> None:
        self._run_live = False
        if reason == RUN_FINISHED:
            self._set_state("idle (last run finished)")
        else:
            self._set_state(f"stopped: {reason}", alert=True)
        if self._calibrating:
            self._calibrating = False
            if reason != RUN_FINISHED:
                self.calibration_result.setText(
                    _colored(f"Calibration failed: {reason}", ALERT_COLOR)
                )


def _disconnect_all(connections: list) -> None:
    for signal, slot in connections:
        with suppress(Exception):
            signal.disconnect(slot)
