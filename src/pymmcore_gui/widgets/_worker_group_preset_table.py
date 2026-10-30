"""Config Groups widget that also shows groups owned by camera worker processes.

Once a persistent :class:`~pymmcore_gui.asi_z_stack.camera_worker_service.
CameraWorkerService` is active, config groups that touch Camera-1/Camera-2
(e.g. a ``Cameras`` group switching PVCAM ``Port``) are moved off the
main-process core -- see :class:`~pymmcore_gui.asi_z_stack.
camera_worker_service.WorkerConfigGroup`. The stock
``GroupPresetTableWidget`` only lists core groups, so this subclass appends
one row per worker-owned group with a preset combo that applies through the
service, and routes the panel's edit buttons for those rows to the service
(see :mod:`pymmcore_gui.widgets._worker_group_editor`) instead of the
main-process core.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_widgets import GroupPresetTableWidget
from qtpy.QtCore import QItemSelectionModel
from qtpy.QtWidgets import (
    QComboBox,
    QFileDialog,
    QMenu,
    QMessageBox,
    QTableWidgetItem,
)

from pymmcore_gui.asi_z_stack.config_save import save_system_configuration
from pymmcore_gui.widgets._worker_group_editor import (
    WorkerGroupDialog,
    WorkerPresetDialog,
)

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_plus import CMMCorePlus
    from qtpy.QtWidgets import QDialog, QWidget

    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService


class WorkerGroupPresetTableWidget(GroupPresetTableWidget):
    """``GroupPresetTableWidget`` plus rows for worker-owned config groups.

    Worker rows are selectable and the Edit/Add/Delete buttons (and Save cfg)
    act on them through the service; core rows keep the stock behaviour.

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
        self._worker_dialog: QDialog | None = None
        super().__init__(parent=parent, mmcore=mmcore)
        service.workerConfigChanged.connect(self._on_worker_config_changed)
        service.workerConfigGroupsChanged.connect(self._on_worker_groups_changed)

    def _populate_table(self) -> None:
        super()._populate_table()
        self._worker_combos = {}
        for group, cfg in self._service.worker_config_groups.items():
            row = self.table_wdg.rowCount()
            self.table_wdg.insertRow(row)
            item = QTableWidgetItem(group)
            item.setToolTip("Applied and edited through the camera worker processes.")
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

    # ------------------------------------------------------------------
    # Selection helpers
    # ------------------------------------------------------------------

    def _row_group(self, row: int) -> str | None:
        item = self.table_wdg.item(row, 0)
        return item.text() if item is not None else None

    def _selected_rows(self) -> list[int]:
        return sorted({idx.row() for idx in self.table_wdg.selectedIndexes()})

    def _selected_worker_groups(self) -> list[str]:
        groups: list[str] = []
        for row in self._selected_rows():
            group = self._row_group(row)
            if group is not None and group in self._service.worker_config_groups:
                groups.append(group)
        return groups

    def _single_worker_group(self) -> str | None:
        """The selected group if exactly one row is selected and it is a worker's."""
        rows = self._selected_rows()
        if len(rows) != 1:
            return None
        group = self._row_group(rows[0])
        if group is not None and group in self._service.worker_config_groups:
            return group
        return None

    def _shown_preset(self, group: str) -> str:
        combo = self._worker_combos.get(group)
        return combo.currentText() if combo is not None else ""

    def _set_rows_selected(self, rows: list[int], selected: bool) -> None:
        selection = self.table_wdg.selectionModel()
        model = self.table_wdg.model()
        if selection is None or model is None:
            return
        flag = (
            QItemSelectionModel.SelectionFlag.Select
            if selected
            else QItemSelectionModel.SelectionFlag.Deselect
        )
        for row in rows:
            selection.select(
                model.index(row, 0), flag | QItemSelectionModel.SelectionFlag.Rows
            )

    def _select_groups(self, groups: set[str]) -> None:
        rows = [
            r for r in range(self.table_wdg.rowCount()) if self._row_group(r) in groups
        ]
        self._set_rows_selected(rows, True)

    def _deselect_worker_rows(self) -> None:
        workers = self._service.worker_config_groups
        rows = [
            r
            for r in range(self.table_wdg.rowCount())
            if (g := self._row_group(r)) is not None and g in workers
        ]
        self._set_rows_selected(rows, False)

    def _on_worker_groups_changed(self) -> None:
        """Rebuild the table after a structural edit, keeping the selection."""
        selected = {
            g for r in self._selected_rows() if (g := self._row_group(r)) is not None
        }
        self._populate_table()
        self._select_groups(selected)

    # ------------------------------------------------------------------
    # Handlers: worker group -> worker path, otherwise stock
    # ------------------------------------------------------------------

    def _close_if_hasattr(self) -> None:
        super()._close_if_hasattr()
        if self._worker_dialog is not None:
            with suppress(RuntimeError):  # already deleted
                self._worker_dialog.close()

    def _open_worker_dialog(self, dialog: QDialog) -> None:
        self._close_if_hasattr()
        self._worker_dialog = dialog
        dialog.show()

    def _edit_preset(self) -> None:
        group = self._single_worker_group()
        if group is None:
            super()._edit_preset()
            return
        preset = self._shown_preset(group)
        if not preset:
            QMessageBox.warning(self, "Edit preset", f"{group} has no preset shown.")
            return
        self._open_worker_dialog(
            WorkerPresetDialog(
                self._service, self._mmc, group, preset=preset, parent=self
            )
        )

    def _add_preset(self) -> None:
        group = self._single_worker_group()
        if group is None:
            super()._add_preset()
            return
        self._open_worker_dialog(
            WorkerPresetDialog(self._service, self._mmc, group, parent=self)
        )

    def _edit_group(self) -> None:
        group = self._single_worker_group()
        if group is None:
            super()._edit_group()
            return
        self._open_worker_dialog(
            WorkerGroupDialog(self._service, self._mmc, group, parent=self)
        )

    def _add_group(self) -> None:
        menu = QMenu(self)
        for text, slot in (
            ("Standard group (main core)...", self._add_core_group),
            ("Camera-worker group...", self._add_worker_group),
        ):
            action = menu.addAction(text)
            if action is not None:
                action.triggered.connect(slot)
        menu.exec(
            self.groups_add_btn.mapToGlobal(self.groups_add_btn.rect().bottomLeft())
        )

    def _add_core_group(self) -> None:
        super()._add_group()

    def _add_worker_group(self) -> None:
        """Open the dialog that creates a camera-worker group."""
        dialog = WorkerGroupDialog(self._service, self._mmc, parent=self)
        dialog.groupSaved.connect(lambda _name, d=dialog: self._edit_new_group(d))
        self._open_worker_dialog(dialog)

    def _edit_new_group(self, dialog: WorkerGroupDialog) -> None:
        """Open the first preset of a freshly added worker group for editing."""
        self._open_worker_dialog(
            WorkerPresetDialog(
                self._service,
                self._mmc,
                dialog.saved_group,
                preset=dialog.saved_preset,
                parent=self,
            )
        )

    def _delete_group(self) -> None:
        workers = self._selected_worker_groups()
        if not workers:
            super()._delete_group()
            return
        msg = self._msg_box(f"Delete '{',  '.join(workers)}'?")
        if msg != QMessageBox.StandardButton.Ok:
            return
        for group in workers:
            try:
                self._service.delete_worker_group(group)
            except KeyError:
                continue
        self._finish_mixed_delete(super()._delete_group)

    def _delete_preset(self) -> None:
        workers = self._selected_worker_groups()
        if not workers:
            super()._delete_preset()
            return
        shown = [(g, self._shown_preset(g)) for g in workers]
        missing = [g for g, preset in shown if not preset]
        entries = [(g, preset) for g, preset in shown if preset]
        if missing:
            QMessageBox.warning(
                self, "Delete preset", f"No preset is shown for {', '.join(missing)}."
            )
        if entries:
            text = ",  ".join(f"({g},  {preset})" for g, preset in entries)
            if self._msg_box(f"Delete '{text}'?") != QMessageBox.StandardButton.Ok:
                return
            for group, preset in entries:
                try:
                    self._service.delete_worker_preset(group, preset)
                except KeyError as exc:
                    QMessageBox.warning(self, "Cannot delete preset", str(exc))
        self._finish_mixed_delete(super()._delete_preset)

    def _finish_mixed_delete(self, stock_delete: Callable[[], None]) -> None:
        """Hand any core rows that are still selected to the stock handler."""
        self._deselect_worker_rows()
        if self._selected_rows():
            stock_delete()

    def _save_cfg(self) -> None:
        (filename, _) = QFileDialog.getSaveFileName(
            self, "Save Micro-Manager Configuration."
        )
        if not filename:
            return
        path = filename if str(filename).endswith(".cfg") else f"{filename}.cfg"
        try:
            save_system_configuration(self._mmc, path)
        except RuntimeError as exc:
            QMessageBox.warning(self, "Cannot save configuration", str(exc))
