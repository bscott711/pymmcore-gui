"""Dock widget for the ASI galvo SPIM's advanced per-slice timing.

Edits :class:`~pymmcore_gui._settings.SpimTimingSettingsV1`, which
:class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` reads at the start of
every MDA, and shows what that timing works out to -- slice period, volume
time, how much light each sensor row gets -- using the same model the engine
uses (:mod:`~pymmcore_gui.asi_z_stack.spim_timing`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from pymmcore_gui._qt.QtCore import QPointF, QRectF, Qt
from pymmcore_gui._qt.QtGui import QColor, QPainter, QPen, QPolygonF
from pymmcore_gui._qt.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.asi_z_stack.spim_timing import (
    SliceTiming,
    compute_slice_timing,
    readout_ms_from_camera_timing,
)

if TYPE_CHECKING:
    from pymmcore_gui._qt.QtGui import QPaintEvent

_MODES = {
    "all_rows": "All Rows (pseudo-global, matches Live)",
    "legacy": "Legacy (shared camera/laser pulse)",
}


def _spin(lo: float, hi: float, step: float, suffix: str = " ms") -> QDoubleSpinBox:
    box = QDoubleSpinBox()
    box.setRange(lo, hi)
    box.setSingleStep(step)
    box.setDecimals(2)
    box.setSuffix(suffix)
    return box


class _TimingStrip(QWidget):
    """One slice drawn to scale: settle, trigger, row roll-in, light, readout."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._timing: SliceTiming | None = None
        self.setMinimumHeight(70)

    def set_timing(self, timing: SliceTiming | None) -> None:
        self._timing = timing
        self.update()

    def paintEvent(self, event: QPaintEvent | None) -> None:
        t = self._timing
        if t is None or t.slice_period_ms <= 0:
            return
        painter = QPainter(self)
        w, h = self.width() - 8, self.height()
        scale = w / t.slice_period_ms

        def x(ms: float) -> float:
            return 4 + ms * scale

        trig = t.settle_ms
        s, e = t.readout_ms, t.exposure_ms
        if t.mode == "all_rows" and not t.laser_held_on:
            light = (trig + s, trig + s + e)
        elif t.laser_held_on:
            light = (0.0, t.slice_period_ms)
        else:
            light = (trig, trig + e)
        rows_end = trig + s + (t.camera_pulse_ms if t.mode == "legacy" else e)

        # galvo settle (grey) and hold
        painter.fillRect(QRectF(x(0), 4, trig * scale, 14), QColor(150, 150, 150))
        painter.fillRect(
            QRectF(x(trig), 4, t.hold_ms * scale, 14), QColor(90, 140, 200)
        )
        # rows integrating: a parallelogram from row 0 to the last row
        painter.setPen(QPen(QColor(60, 160, 90), 1))
        painter.setBrush(QColor(60, 160, 90, 90))
        top, bottom = 24.0, h - 22.0
        expo = t.camera_pulse_ms if t.mode == "legacy" else s + e
        painter.drawPolygon(
            QPolygonF(
                [
                    QPointF(x(trig), top),
                    QPointF(x(trig + expo), top),
                    QPointF(x(min(rows_end, t.slice_period_ms)), bottom),
                    QPointF(x(trig + s), bottom),
                ]
            )
        )
        # laser on
        painter.fillRect(
            QRectF(x(light[0]), h - 16, (light[1] - light[0]) * scale, 10),
            QColor(230, 160, 30),
        )
        painter.setPen(QPen(Qt.GlobalColor.darkGray, 1))
        painter.drawText(QRectF(4, h - 16, 80, 14), "laser")
        painter.end()


class SpimTimingWidget(QWidget):
    """Edit and preview the galvo SPIM's advanced slice timing.

    Parameters
    ----------
    parent : QWidget | None
        Parent widget.
    mmcore : CMMCorePlus | None
        Core instance (for disabling edits while an MDA runs).
    """

    def __init__(
        self, *, parent: QWidget | None = None, mmcore: CMMCorePlus | None = None
    ) -> None:
        super().__init__(parent)
        self._mmc = mmcore or CMMCorePlus.instance()
        cfg = SettingsV1.instance().spim_timing

        self.mode = QComboBox()
        for key, text in _MODES.items():
            self.mode.addItem(text, key)
        self.mode.setCurrentIndex(max(0, self.mode.findData(cfg.mode)))

        self.settle = _spin(0.25, 1000, 0.25)
        self.settle.setValue(cfg.galvo_settle_ms)
        self.settle.setToolTip("Galvo settle after each slice step, before the trigger")
        self.volume_settle = _spin(0, 1000, 0.25)
        self.volume_settle.setValue(cfg.volume_start_settle_ms)
        self.volume_settle.setToolTip(
            "Extra settle before each volume's first slice (galvo jump back)"
        )
        self.readout_auto = QCheckBox("from cameras")
        self.readout_auto.setChecked(cfg.readout_ms_override is None)
        self.readout = _spin(0, 100, 0.05)
        self.readout.setValue(cfg.readout_ms_override or cfg.fallback_readout_ms)
        self.margin = _spin(0, 20, 0.25)
        self.margin.setValue(cfg.margin_ms)
        self.trigger = _spin(0.25, 10, 0.25)
        self.trigger.setValue(cfg.trigger_pulse_ms)
        self.bnc = {label: self._bnc_spin(n) for label, n in cfg.expose_out_bnc.items()}

        readout_row = QHBoxLayout()
        readout_row.addWidget(self.readout)
        readout_row.addWidget(self.readout_auto)

        form = QFormLayout()
        form.addRow("Mode", self.mode)
        form.addRow("Galvo settle", self.settle)
        form.addRow("Volume-start settle", self.volume_settle)
        form.addRow("Camera readout", readout_row)
        form.addRow("Galvo hold margin", self.margin)
        form.addRow("Camera trigger pulse", self.trigger)
        for label, spin in self.bnc.items():
            form.addRow(f"{label} Expose Out -> PLogic BNC", spin)
        settings_box = QGroupBox("Timing (used by the next MDA)")
        settings_box.setLayout(form)

        self.preview_exposure = _spin(0.1, 10_000, 1)
        self.preview_exposure.setValue(self._live_exposure())
        self.preview_slices = QSpinBox()
        self.preview_slices.setRange(1, 10_000)
        self.preview_slices.setValue(81)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.summary.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.strip = _TimingStrip(self)
        preview_form = QFormLayout()
        preview_form.addRow("Exposure", self.preview_exposure)
        preview_form.addRow("Slices", self.preview_slices)
        preview_box = QGroupBox("Preview")
        preview_layout = QVBoxLayout(preview_box)
        preview_layout.addLayout(preview_form)
        preview_layout.addWidget(self.strip)
        preview_layout.addWidget(self.summary)

        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self._save)
        refresh = QPushButton("Read camera readout")
        refresh.setToolTip(
            "Use the readout time the cameras reported on their last arm"
        )
        refresh.clicked.connect(self._refresh_readout)
        buttons = QHBoxLayout()
        buttons.addWidget(refresh)
        buttons.addStretch()
        buttons.addWidget(self.save_button)

        layout = QVBoxLayout(self)
        layout.addWidget(settings_box)
        layout.addWidget(preview_box)
        layout.addLayout(buttons)
        layout.addStretch()

        for box in (
            self.settle,
            self.volume_settle,
            self.readout,
            self.margin,
            self.trigger,
            self.preview_exposure,
        ):
            box.valueChanged.connect(self._update_preview)
        self.preview_slices.valueChanged.connect(self._update_preview)
        self.mode.currentIndexChanged.connect(self._update_preview)
        self.readout_auto.toggled.connect(self._on_auto_toggled)

        mda = self._mmc.mda.events
        mda.sequenceStarted.connect(lambda *_: self._set_editable(False))
        mda.sequenceFinished.connect(lambda *_: self._set_editable(True))

        self._on_auto_toggled(self.readout_auto.isChecked())

    # ------------------------------------------------------------------

    @staticmethod
    def _bnc_spin(value: int) -> QSpinBox:
        box = QSpinBox()
        box.setRange(1, 8)
        box.setValue(value)
        return box

    def _live_exposure(self) -> float:
        from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

        if (svc := CameraWorkerService.get_active()) is not None:
            return svc.last_known_exposure_ms
        try:
            return float(self._mmc.getExposure())
        except Exception:
            return 10.0

    def _camera_readout_ms(self) -> float | None:
        from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

        if (svc := CameraWorkerService.get_active()) is None:
            return None
        values = [
            r
            for t in svc.camera_timing().values()
            if (r := readout_ms_from_camera_timing(t)) is not None
        ]
        return max(values) if values else None

    def _on_auto_toggled(self, auto: bool) -> None:
        self.readout.setEnabled(not auto)
        if auto:
            self._refresh_readout()
        self._update_preview()

    def _refresh_readout(self) -> None:
        if (measured := self._camera_readout_ms()) is not None:
            self.readout.setValue(measured)

    def _set_editable(self, editable: bool) -> None:
        self.save_button.setEnabled(editable)

    def timing(self) -> SliceTiming:
        """The slice timing the current fields work out to (preview exposure)."""
        return compute_slice_timing(
            self.mode.currentData(),
            self.preview_exposure.value(),
            self.readout.value(),
            self.settle.value(),
            margin_ms=self.margin.value(),
            trigger_pulse_ms=self.trigger.value(),
            volume_start_settle_ms=self.volume_settle.value(),
        )

    def _update_preview(self) -> None:
        timing = self.timing()
        self.strip.set_timing(timing)
        text = timing.describe(self.preview_slices.value())
        if self.readout_auto.isChecked() and self._camera_readout_ms() is None:
            text += (
                "\nCameras haven't reported a readout yet (Snap or Live once); "
                "the MDA will use the fallback until they do."
            )
        if timing.mode == "all_rows":
            text += (
                "\nAll Rows needs each camera's Expose Out cabled into the PLogic "
                "BNCs above; without it the lasers never fire (dark frames)."
            )
        self.summary.setText(text)

    def _save(self) -> None:
        settings = SettingsV1.instance()
        cfg = settings.spim_timing
        cfg.mode = self.mode.currentData()
        cfg.galvo_settle_ms = self.settle.value()
        cfg.volume_start_settle_ms = self.volume_settle.value()
        cfg.readout_ms_override = (
            None if self.readout_auto.isChecked() else self.readout.value()
        )
        cfg.margin_ms = self.margin.value()
        cfg.trigger_pulse_ms = self.trigger.value()
        cfg.expose_out_bnc = {label: spin.value() for label, spin in self.bnc.items()}
        settings.flush()
