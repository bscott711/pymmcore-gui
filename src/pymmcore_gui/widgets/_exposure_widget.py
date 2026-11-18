"""Exposure control for Snap/Live that also works with worker-owned cameras.

When a persistent :class:`~pymmcore_gui.asi_z_stack.camera_worker_service.
CameraWorkerService` is active, the main-process core has no camera
(``setExposure`` raises "Camera not loaded or initialized"), so exposure goes
to the camera workers via :meth:`CameraWorkerService.set_exposure`. Otherwise
(demo/single-camera/non-ASI configs) it's plain ``mmc.setExposure``.

In EPI mode exposure goes to the EPI (Orca) camera service instead
(:func:`~pymmcore_gui.epi._camera_service.active_camera_service`); the widget
re-syncs on the mode controller's ``modeChanged``.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QWidget,
)

from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    has_camera,
)
from pymmcore_gui.epi._camera_service import active_camera_service
from pymmcore_gui.epi._mode import ScopeMode, active_controller, current_mode

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus


class ExposureWidget(QWidget):
    """A labeled exposure spin box (milliseconds) for the camera toolbar.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    parent : QWidget | None
        Optional parent widget.
    """

    def __init__(self, mmc: CMMCorePlus, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._mmc = mmc
        self._service: CameraWorkerService | None = None

        self.spin = QDoubleSpinBox()
        self.spin.setRange(0.01, 60_000.0)
        self.spin.setDecimals(2)
        self.spin.setSuffix(" ms")
        self.spin.setToolTip("Snap/Live exposure")
        # Apply on Enter/focus-out/arrow steps, not on every keystroke: with
        # worker-owned cameras each apply restarts Live.
        self.spin.setKeyboardTracking(False)
        self.spin.valueChanged.connect(self._apply)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 0, 4, 0)
        layout.addWidget(QLabel("Exposure"))
        layout.addWidget(self.spin)

        mmc.events.systemConfigurationLoaded.connect(self._refresh)
        mmc.events.exposureChanged.connect(self._on_core_exposure_changed)
        self.destroyed.connect(self._disconnect)
        self._controller = active_controller()
        if (signal := getattr(self._controller, "modeChanged", None)) is not None:
            signal.connect(self._on_mode_changed)
        self._refresh()

    def _on_mode_changed(self, _mode: object) -> None:
        self._refresh()

    def _refresh(self) -> None:
        """Re-sync to the active service (if any) and show the current exposure."""
        svc = active_camera_service()
        if svc is not self._service:
            if self._service is not None:
                with suppress(RuntimeError, TypeError):
                    self._service.exposureChanged.disconnect(self._show)
            if svc is not None:
                svc.exposureChanged.connect(self._show)
            self._service = svc

        if current_mode() is ScopeMode.EPI:
            self.setEnabled(svc is not None)
        else:
            self.setEnabled(has_camera(self._mmc))
        if svc is not None:
            self._show(svc.last_known_exposure_ms)
        elif self._mmc.getCameraDevice():
            self._show(self._mmc.getExposure())

    def _apply(self, exposure_ms: float) -> None:
        try:
            if (svc := active_camera_service()) is not None:
                svc.set_exposure(exposure_ms)
            else:
                self._mmc.setExposure(exposure_ms)
        except Exception as exc:
            QMessageBox.warning(self, "Cannot set exposure", str(exc))
            self._refresh()  # back to what's actually in effect

    def _on_core_exposure_changed(self, device: str, value: float) -> None:
        if active_camera_service() is None:
            self._show(value)

    def _show(self, exposure_ms: float) -> None:
        with suppress(RuntimeError):  # spin box already deleted
            self.spin.blockSignals(True)
            try:
                self.spin.setValue(exposure_ms)
            finally:
                self.spin.blockSignals(False)

    def _disconnect(self) -> None:
        with suppress(RuntimeError, TypeError):
            self._mmc.events.systemConfigurationLoaded.disconnect(self._refresh)
            self._mmc.events.exposureChanged.disconnect(self._on_core_exposure_changed)
        if (signal := getattr(self._controller, "modeChanged", None)) is not None:
            with suppress(RuntimeError, TypeError):
                signal.disconnect(self._on_mode_changed)
