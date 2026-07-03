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
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1, SpectralChannelConfig
from pymmcore_gui._spectral_channel_handler import next_fft_friendly

if TYPE_CHECKING:
    from pymmcore_gui._main_window import MicroManagerGUI
    from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview


class _ChannelRow(QGroupBox):
    """Editable fields for one spectral channel (camera, laser, position).

    The region *size* is shared across all channels (see
    :class:`SpectralChannelConfigWidget`), so a row only owns its top-left
    ``(x, y)`` position.
    """

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

        # Note: named *_spin (not x/y) to avoid shadowing QWidget.x()/.y().
        self.x_spin = QSpinBox()
        self.y_spin = QSpinBox()
        for spinbox in (self.x_spin, self.y_spin):
            spinbox.setRange(0, 100_000)

        self.draw_button = QPushButton("Draw on Live View...")
        self.move_button = QPushButton("Move on Live View...")

        form = QFormLayout(self)
        form.addRow("Camera", self.camera)
        form.addRow("Laser preset", self.laser_preset)

        pos_row = QHBoxLayout()
        for label, spinbox in (("x", self.x_spin), ("y", self.y_spin)):
            pos_row.addWidget(QLabel(label))
            pos_row.addWidget(spinbox)
        form.addRow("Position (px)", pos_row)

        buttons = QHBoxLayout()
        buttons.addWidget(self.draw_button)
        buttons.addWidget(self.move_button)
        form.addRow(buttons)

        self.load(config, cameras)

    def load(self, config: SpectralChannelConfig, cameras: list[str]) -> None:
        """Populate fields from *config*, adding a missing camera label if needed."""
        if config.camera not in cameras:
            self.camera.addItem(config.camera)
        self.camera.setCurrentText(config.camera)
        self.laser_preset.setCurrentText(config.laser_preset)
        x, y = (config.rect[0], config.rect[1]) if config.rect else (0, 0)
        self.set_position(x, y)

    def set_position(self, x: int, y: int) -> None:
        self.x_spin.setValue(x)
        self.y_spin.setValue(y)

    @property
    def position(self) -> tuple[int, int]:
        return self.x_spin.value(), self.y_spin.value()

    def to_config(self, size: tuple[int, int]) -> SpectralChannelConfig:
        w, h = size
        x, y = self.position
        rect = (x, y, w, h) if w and h else None
        return SpectralChannelConfig(
            name=self.channel_name,
            camera=self.camera.currentText(),
            laser_preset=self.laser_preset.currentText(),
            rect=rect,
        )


class SpectralChannelConfigWidget(QWidget):
    """Configure the fixed image-splitter spectral-channel crop regions.

    Lets the user pick which camera/laser-preset each region belongs to, draw
    (or drag to reposition) its rectangle directly on that camera's live view,
    and persist the result so
    :class:`~pymmcore_gui._spectral_channel_handler.SpectralChannelHandler`
    can crop and save each region to its own file during MDA acquisitions.

    All regions share one width/height (required by this feature); which regions
    are actually saved is decided per-MDA from the lasers it uses, not here.
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

        # Shared size — every region must be identical for deconvolution.
        self.width_spin = QSpinBox()
        self.height_spin = QSpinBox()
        for spinbox in (self.width_spin, self.height_spin):
            spinbox.setRange(0, 100_000)
        w, h = self._initial_size(spectral.channels)
        self.width_spin.setValue(w)
        self.height_spin.setValue(h)
        self.width_spin.editingFinished.connect(self._maybe_extend_for_fft)
        self.height_spin.editingFinished.connect(self._maybe_extend_for_fft)

        size_row = QHBoxLayout()
        size_row.addWidget(QLabel("w"))
        size_row.addWidget(self.width_spin)
        size_row.addWidget(QLabel("h"))
        size_row.addWidget(self.height_spin)
        size_box = QGroupBox("Shared ROI size (px) — all regions share one size")
        size_box.setLayout(size_row)

        self._rows = [_ChannelRow(c, cameras, self) for c in spectral.channels]
        for row in self._rows:
            self._populate_laser_presets(row)
            row.draw_button.clicked.connect(lambda _checked=False, r=row: self._draw(r))
            row.move_button.clicked.connect(lambda _checked=False, r=row: self._move(r))

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
        layout.addWidget(size_box)
        layout.addWidget(scroll)
        layout.addWidget(save_button)

    # ------------------------------------------------------------------
    # Size helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _initial_size(channels: list[SpectralChannelConfig]) -> tuple[int, int]:
        """Seed the shared size from the first channel that has a drawn rect."""
        for c in channels:
            if c.rect is not None:
                return c.rect[2], c.rect[3]
        return 0, 0

    @property
    def _size(self) -> tuple[int, int]:
        return self.width_spin.value(), self.height_spin.value()

    def _maybe_extend_for_fft(self) -> None:
        """Offer to pad the shared size up to the next FFT-friendly size."""
        w, h = self._size
        if not (w and h):
            return
        nw, nh = next_fft_friendly(w), next_fft_friendly(h)
        if (nw, nh) == (w, h):
            return
        ret = QMessageBox.question(
            self,
            "Non-optimal ROI size",
            f"ROI size {w}x{h} px isn't optimal for GPU/FFT deconvolution.\n"
            f"Extend to the next FFT-friendly size {nw}x{nh} px, keeping each "
            "region centered on its current area?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if ret == QMessageBox.StandardButton.Yes:
            self._resize_centered(nw, nh)

    def _resize_centered(self, new_w: int, new_h: int) -> None:
        """Set the shared size to *new_w*/*new_h*, re-centering every region."""
        old_w, old_h = self._size
        for row in self._rows:
            x, y = row.position
            cx, cy = x + old_w / 2, y + old_h / 2
            row.set_position(
                max(0, round(cx - new_w / 2)), max(0, round(cy - new_h / 2))
            )
        self.width_spin.setValue(new_w)
        self.height_spin.setValue(new_h)

    # ------------------------------------------------------------------
    # Draw / move on the live view
    # ------------------------------------------------------------------

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

    def _preview_for(self, row: _ChannelRow) -> PygfxPreview | None:
        win = self._main_window()
        if win is None:
            return None
        return win.viewers_manager.get_or_create_camera_preview(
            row.camera.currentText()
        )

    def _draw(self, row: _ChannelRow) -> None:
        if (preview := self._preview_for(row)) is not None:
            preview.begin_roi_draw(lambda rect: self._on_rect_drawn(row, preview, rect))

    def _move(self, row: _ChannelRow) -> None:
        w, h = self._size
        if not (w and h):
            QMessageBox.information(
                self,
                "Draw a region first",
                "Draw a region first to set the shared size, then you can drag "
                "it to reposition.",
            )
            return
        if (preview := self._preview_for(row)) is not None:
            x, y = row.position
            preview.begin_roi_move(
                (x, y, w, h), lambda rect: self._on_rect_moved(row, preview, rect)
            )

    def _on_rect_drawn(
        self, row: _ChannelRow, preview: PygfxPreview, rect: tuple[int, int, int, int]
    ) -> None:
        x, y, w, h = rect
        row.set_position(x, y)
        self.width_spin.setValue(w)
        self.height_spin.setValue(h)
        self._maybe_extend_for_fft()
        self._refresh_camera_overlays(preview, row.camera.currentText())

    def _on_rect_moved(
        self, row: _ChannelRow, preview: PygfxPreview, rect: tuple[int, int, int, int]
    ) -> None:
        x, y, _w, _h = rect
        row.set_position(x, y)
        self._refresh_camera_overlays(preview, row.camera.currentText())

    def _refresh_camera_overlays(
        self, preview: PygfxPreview, camera_label: str
    ) -> None:
        """Live-preview all this camera's (unsaved) regions on *preview*."""
        w, h = self._size
        rois = (
            [
                (row.channel_name, (row.x_spin.value(), row.y_spin.value(), w, h))
                for row in self._rows
                if row.camera.currentText() == camera_label
            ]
            if (w and h)
            else []
        )
        preview.set_roi_overlays(rois)

    def _save(self) -> None:
        settings = SettingsV1.instance()
        settings.spectral.enabled = self.master_enabled.isChecked()
        settings.spectral.channels = [row.to_config(self._size) for row in self._rows]
        settings.flush()
        if (win := self._main_window()) is not None:
            win.viewers_manager.refresh_roi_overlays()
