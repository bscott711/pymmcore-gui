"""Minimal Camera ROI control for cameras owned by a persistent worker service.

The stock ``pymmcore_widgets.CameraRoiWidget`` calls ``mmcore.setROI``/
``getROI`` directly against the main-process core. Once
:class:`~pymmcore_gui.asi_z_stack.camera_worker_service.CameraWorkerService`
is active, Camera-1/Camera-2 are never loaded there -- this widget is a small,
purpose-built replacement that talks to the service's worker-process ROI
protocol instead (see ``worker_pool.py``'s ``set_roi``/``get_roi``).

Deliberately does not replicate the stock widget's "Auto Snap" checkbox or
fractional-crop preset combo -- pure UI sugar, addable later without
architectural impact.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerServiceState

if TYPE_CHECKING:
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

logger = logging.getLogger("pymmcore_gui")

_POLL_INTERVAL_MS = 500


class WorkerCameraRoiWidget(QWidget):
    """Set/read hardware ROI on worker-owned cameras.

    Parameters
    ----------
    parent : QWidget | None
        Parent widget.
    service : CameraWorkerService
        The active persistent camera worker service.
    """

    def __init__(self, parent: QWidget | None, service: CameraWorkerService) -> None:
        super().__init__(parent)
        self._service = service

        self._camera_combo = QComboBox()
        self._camera_combo.addItems(service.camera_labels)

        self._x_spin = self._make_spinbox()
        self._y_spin = self._make_spinbox()
        self._w_spin = self._make_spinbox()
        self._h_spin = self._make_spinbox()

        geometry = service.geometry
        if geometry is not None:
            for spin, bound in (
                (self._x_spin, geometry.image_width),
                (self._w_spin, geometry.image_width),
                (self._y_spin, geometry.image_height),
                (self._h_spin, geometry.image_height),
            ):
                spin.setMaximum(bound)

        self._set_button = QPushButton("Set ROI")
        self._set_button.clicked.connect(self._on_set_clicked)
        self._full_chip_button = QPushButton("Full Chip")
        self._full_chip_button.clicked.connect(self._on_full_chip_clicked)

        self._status_label = QLabel("")
        self._status_label.setWordWrap(True)

        form = QFormLayout()
        form.addRow("Camera", self._camera_combo)
        form.addRow("X", self._x_spin)
        form.addRow("Y", self._y_spin)
        form.addRow("Width", self._w_spin)
        form.addRow("Height", self._h_spin)

        buttons = QHBoxLayout()
        buttons.addWidget(self._set_button)
        buttons.addWidget(self._full_chip_button)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addWidget(self._status_label)
        layout.addStretch()

        self._camera_combo.currentTextChanged.connect(self._refresh_fields)

        # Polls rather than wiring a dedicated "state changed" signal from
        # the service: this widget needs to reflect Live *and* MDA
        # transitions (the service only emits liveStateChanged for the
        # former), and polling is self-correcting regardless of which path
        # changed the state -- the same approach the Camera Alignment
        # widget already uses for its own live-state-dependent polling.
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._update_enabled)
        self._poll_timer.start(_POLL_INTERVAL_MS)

        self._update_enabled()
        self._refresh_fields()

    @staticmethod
    def _make_spinbox() -> QSpinBox:
        spin = QSpinBox()
        spin.setMaximum(100_000)
        return spin

    def _current_camera(self) -> str | None:
        return self._camera_combo.currentText() or None

    def _update_enabled(self) -> None:
        idle = self._service.state is CameraWorkerServiceState.IDLE
        for w in (
            self._camera_combo,
            self._x_spin,
            self._y_spin,
            self._w_spin,
            self._h_spin,
            self._set_button,
            self._full_chip_button,
        ):
            w.setEnabled(idle)
        if not idle:
            self._status_label.setText(
                f"Camera busy ({self._service.state.name.title()}) -- "
                "stop Live/MDA to change ROI."
            )
        elif not self._status_label.text().startswith("ROI"):
            self._status_label.setText("")

    def _refresh_fields(self) -> None:
        camera = self._current_camera()
        if camera is None or self._service.state is not CameraWorkerServiceState.IDLE:
            return
        try:
            x, y, w, h = self._service.get_roi(camera)
        except Exception as exc:
            logger.warning("Failed to read ROI for %s: %s", camera, exc)
            return
        for spin, value in (
            (self._x_spin, x),
            (self._y_spin, y),
            (self._w_spin, w),
            (self._h_spin, h),
        ):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    def _on_set_clicked(self) -> None:
        camera = self._current_camera()
        if camera is None:
            return
        try:
            x, y, w, h = self._service.set_roi(
                camera,
                self._x_spin.value(),
                self._y_spin.value(),
                self._w_spin.value(),
                self._h_spin.value(),
            )
        except Exception as exc:
            self._status_label.setText(f"Failed to set ROI: {exc}")
            return
        self._status_label.setText(f"ROI set: ({x}, {y}, {w}, {h})")
        self._refresh_fields()

    def _on_full_chip_clicked(self) -> None:
        camera = self._current_camera()
        geometry = self._service.geometry
        if camera is None or geometry is None:
            return
        try:
            self._service.set_roi(
                camera, 0, 0, geometry.image_width, geometry.image_height
            )
        except Exception as exc:
            self._status_label.setText(f"Failed to clear ROI: {exc}")
            return
        self._status_label.setText("ROI cleared (full chip).")
        self._refresh_fields()
