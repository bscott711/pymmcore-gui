"""Config Groups widget that also shows groups owned by camera worker processes.

Once a persistent :class:`~pymmcore_gui.asi_z_stack.camera_worker_service.
CameraWorkerService` is active, config groups that touch Camera-1/Camera-2
(e.g. a ``Cameras`` group switching PVCAM ``Port``) are moved off the
main-process core -- see :class:`~pymmcore_gui.asi_z_stack.
camera_worker_service.WorkerConfigGroup`. The stock
``GroupPresetTableWidget`` only lists core groups, so this subclass appends
one row per worker-owned group with a preset combo that applies through the
service.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_widgets import GroupPresetTableWidget
from qtpy.QtCore import Qt
from qtpy.QtWidgets import QComboBox, QMessageBox, QTableWidgetItem

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from qtpy.QtWidgets import QWidget

    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService


class WorkerGroupPresetTableWidget(GroupPresetTableWidget):
    """``GroupPresetTableWidget`` plus rows for worker-owned config groups.

    Parameters
    ----------
    service : CameraWorkerService
        The active camera worker service.
    parent : QWidget | None
        Optional parent widget.
    mmcore : CMMCorePlus | None
        The main-process core.
    """

    def __init__(
        self,
        service: CameraWorkerService,
        *,
        parent: QWidget | None = None,
        mmcore: CMMCorePlus | None = None,
    ) -> None:
        # _populate_table runs inside super().__init__, and reads this.
        self._service = service
        self._worker_combos: dict[str, QComboBox] = {}
        super().__init__(parent=parent, mmcore=mmcore)
        service.workerConfigChanged.connect(self._on_worker_config_changed)

    def _populate_table(self) -> None:
        super()._populate_table()
        self._worker_combos = {}
        for group, cfg in self._service.worker_config_groups.items():
            row = self.table_wdg.rowCount()
            self.table_wdg.insertRow(row)
            item = QTableWidgetItem(group)
            # Not selectable: the stock edit/delete buttons act on core
            # groups via mmc, which no longer has this one.
            item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            item.setToolTip("Applied through the camera worker processes.")
            self.table_wdg.setItem(row, 0, item)

            combo = QComboBox()
            combo.addItems(list(cfg.presets))
            if cfg.current in cfg.presets:
                combo.setCurrentText(cfg.current)
            else:
                combo.setCurrentIndex(-1)
            combo.textActivated.connect(
                lambda preset, g=group: self._apply_worker_preset(g, preset)
            )
            self.table_wdg.setCellWidget(row, 1, combo)
            self._worker_combos[group] = combo
        self.table_wdg.resizeColumnToContents(0)

    def _apply_worker_preset(self, group: str, preset: str) -> None:
        try:
            self._service.apply_worker_config(group, preset)
        except Exception as exc:
            QMessageBox.warning(self, f"Cannot apply {group}", str(exc))
            # Revert the combo to what's actually in effect.
            self._on_worker_config_changed(
                group, self._service.worker_config_groups[group].current
            )

    def _on_worker_config_changed(self, group: str, preset: str) -> None:
        combo = self._worker_combos.get(group)
        if combo is None:
            return
        with suppress(RuntimeError):  # combo deleted by a table rebuild
            combo.blockSignals(True)
            try:
                if preset:
                    combo.setCurrentText(preset)
                else:
                    combo.setCurrentIndex(-1)
            finally:
                combo.blockSignals(False)
