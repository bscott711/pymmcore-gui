"""Dock widget for configuring image-splitter spectral-channel crop regions."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._qt.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1, SpectralChannelConfig

if TYPE_CHECKING:
    from pymmcore_gui._main_window import MicroManagerGUI
    from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

_WRITER_FORMATS = ("ome-zarr", "ome-tiff", "tiff-sequence")


class _ChannelRow(QGroupBox):
    """Editable fields for one spectral channel."""

    def __init__(
        self,
        config: SpectralChannelConfig,
        cameras: list[str],
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(config.name, parent)
        self.channel_name = config.name

        self.camera = QComboBox()
        self.camera.addItems(cameras)

        self.laser_preset = QComboBox()
        self.laser_preset.setEditable(True)

        self.writer_format = QComboBox()
        self.writer_format.addItems(_WRITER_FORMATS)

        self.enabled = QCheckBox("Save this channel")

        # Note: named *_spin (not x/y/w/h) to avoid shadowing QWidget.x()/.y().
        self.x_spin = QSpinBox()
        self.y_spin = QSpinBox()
        self.w_spin = QSpinBox()
        self.h_spin = QSpinBox()
        for spinbox in (self.x_spin, self.y_spin, self.w_spin, self.h_spin):
            spinbox.setRange(0, 100_000)

        self.draw_button = QPushButton("Draw on Live View...")

        form = QFormLayout(self)
        form.addRow("Camera", self.camera)
        form.addRow("Laser preset", self.laser_preset)
        form.addRow("Writer format", self.writer_format)
        form.addRow(self.enabled)

        rect_row = QHBoxLayout()
        for label, spinbox in (
            ("x", self.x_spin),
            ("y", self.y_spin),
            ("w", self.w_spin),
            ("h", self.h_spin),
        ):
            rect_row.addWidget(QLabel(label))
            rect_row.addWidget(spinbox)
        form.addRow("Rect (px)", rect_row)
        form.addRow(self.draw_button)

        self.load(config, cameras)

    def load(self, config: SpectralChannelConfig, cameras: list[str]) -> None:
        """Populate fields from *config*, adding a missing camera label if needed."""
        if config.camera not in cameras:
            self.camera.addItem(config.camera)
        self.camera.setCurrentText(config.camera)
        self.laser_preset.setCurrentText(config.laser_preset)
        self.writer_format.setCurrentText(config.writer_format)
        self.enabled.setChecked(config.enabled)
        self.set_rect(config.rect or (0, 0, 0, 0))

    def set_rect(self, rect: tuple[int, int, int, int]) -> None:
        x, y, w, h = rect
        self.x_spin.setValue(x)
        self.y_spin.setValue(y)
        self.w_spin.setValue(w)
        self.h_spin.setValue(h)

    def to_config(self) -> SpectralChannelConfig:
        w, h = self.w_spin.value(), self.h_spin.value()
        rect = (self.x_spin.value(), self.y_spin.value(), w, h) if w and h else None
        return SpectralChannelConfig(
            name=self.channel_name,
            camera=self.camera.currentText(),
            laser_preset=self.laser_preset.currentText(),
            rect=rect,
            enabled=self.enabled.isChecked(),
            writer_format=self.writer_format.currentText(),
        )


class SpectralChannelConfigWidget(QWidget):
    """Configure the fixed image-splitter spectral-channel crop regions.

    Lets the user pick which camera/laser-preset each region belongs to, draw
    its rectangle directly on that camera's live view, and persist the result
    so :class:`~pymmcore_gui._spectral_channel_handler.SpectralChannelHandler`
    can crop and save each region to its own file during MDA acquisitions.
    """

    def __init__(
        self, *, parent: QWidget | None = None, mmcore: CMMCorePlus | None = None
    ) -> None:
        super().__init__(parent)
        self._mmc = mmcore or CMMCorePlus.instance()

        spectral = SettingsV1.instance().spectral
        cameras = physical_camera_labels(self._mmc)

        self.master_enabled = QCheckBox("Enable spectral-channel cropping")
        self.master_enabled.setChecked(spectral.enabled)

        self._rows = [_ChannelRow(c, cameras, self) for c in spectral.channels]
        for row in self._rows:
            self._populate_laser_presets(row)
            row.draw_button.clicked.connect(lambda _checked=False, r=row: self._draw(r))

        save_button = QPushButton("Save")
        save_button.clicked.connect(self._save)

        rows_layout = QVBoxLayout()
        rows_layout.setContentsMargins(0, 0, 0, 0)
        for row in self._rows:
            rows_layout.addWidget(row)
        rows_container = QWidget()
        rows_container.setLayout(rows_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(rows_container)

        layout = QVBoxLayout(self)
        layout.addWidget(self.master_enabled)
        layout.addWidget(scroll)
        layout.addWidget(save_button)

    def _populate_laser_presets(self, row: _ChannelRow) -> None:
        laser_group = SettingsV1.instance().spectral.laser_config_group
        try:
            presets = list(self._mmc.getAvailableConfigs(laser_group))
        except Exception:
            presets = []
        current = row.laser_preset.currentText()
        row.laser_preset.clear()
        row.laser_preset.addItems(presets)
        if current:
            row.laser_preset.setCurrentText(current)

    def _main_window(self) -> MicroManagerGUI | None:
        from pymmcore_gui.actions.widget_actions import _get_mm_main_window

        # _get_mm_main_window is typed against PyQt6.QtCore.QObject, while this
        # module's QWidget resolves to PySide6 under TYPE_CHECKING (see
        # pymmcore_gui._qt); both bindings are duck-type compatible at runtime.
        return _get_mm_main_window(self)  # type: ignore[arg-type]

    def _draw(self, row: _ChannelRow) -> None:
        win = self._main_window()
        if win is None:
            return
        preview = win.viewers_manager.get_or_create_camera_preview(
            row.camera.currentText()
        )
        preview.begin_roi_draw(lambda rect: self._on_rect_drawn(row, preview, rect))

    def _on_rect_drawn(
        self, row: _ChannelRow, preview: PygfxPreview, rect: tuple[int, int, int, int]
    ) -> None:
        row.set_rect(rect)
        preview.set_roi_overlays([(row.channel_name, rect)])

    def _save(self) -> None:
        settings = SettingsV1.instance()
        settings.spectral.enabled = self.master_enabled.isChecked()
        settings.spectral.channels = [row.to_config() for row in self._rows]
        settings.flush()
        if (win := self._main_window()) is not None:
            win.viewers_manager.refresh_roi_overlays()
