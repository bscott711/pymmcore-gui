"""Dialogs that edit config groups owned by the camera worker processes.

The stock Config Groups dialogs (``EditPresetWidget`` and friends) are
hard-wired to ``CMMCorePlus.instance()`` and to ``PropertyWidget``, which
raises for devices that are not loaded in the main process. Worker-owned
groups reference exactly such devices (Camera-1/-2, the ``Multi Camera``
composite), so this module provides small replacements driven by the property
metadata the :class:`~pymmcore_gui.asi_z_stack.camera_worker_service.
CameraWorkerService` captured while those devices were still loaded.

None of these dialogs applies anything to hardware: they only edit the stored
presets, like the stock dialogs.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from PyQt6.QtCore import pyqtSignal
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from pymmcore_gui.asi_z_stack.camera_handoff import (
    PropertyInfo,
    snapshot_device_properties,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

_INT32_MIN = -(2**31)
_INT32_MAX = 2**31 - 1
_FLOAT_DECIMALS = 6

DEFAULT_PRESET_NAME = "NewPreset"


# ----------------------------------------------------------------------
# Property metadata
# ----------------------------------------------------------------------


class WorkerPropertyMetadata:
    """Property metadata for devices that may be released to the workers.

    Parameters
    ----------
    service : CameraWorkerService
        The active camera worker service (source of released-device metadata).
    mmc : CMMCorePlus
        The main-process core (source of metadata for devices still loaded).
    """

    def __init__(self, service: CameraWorkerService, mmc: CMMCorePlus) -> None:
        self._service = service
        self._mmc = mmc
        self._released = service.released_device_properties()
        self._core_cache: dict[str, dict[str, PropertyInfo]] = {}

    @property
    def released_devices(self) -> set[str]:
        """Labels of the devices owned by the workers (or gone with them)."""
        return set(self._released) | set(self._service.camera_labels)

    def device_properties(self, device: str) -> dict[str, PropertyInfo]:
        """Metadata of every property of *device* ({} if it is not known)."""
        if device in self._released:
            return self._released[device]
        if device not in self._core_cache:
            loaded = device in self._mmc.getLoadedDevices()
            self._core_cache[device] = (
                snapshot_device_properties(self._mmc, device) if loaded else {}
            )
        return self._core_cache[device]

    def info(self, device: str, prop: str) -> PropertyInfo | None:
        """Metadata of one property, or ``None`` if the device is unknown."""
        return self.device_properties(device).get(prop)

    def candidates(self) -> list[tuple[str, str, PropertyInfo]]:
        """Writable, post-init properties of released and main-core devices."""
        out: list[tuple[str, str, PropertyInfo]] = []
        devices = list(self._released)
        devices.extend(
            d
            for d in self._mmc.getLoadedDevices()
            if d != "Core" and d not in self._released
        )
        for device in devices:
            for prop, info in self.device_properties(device).items():
                if not info.read_only and not info.pre_init:
                    out.append((device, prop, info))
        return out


# ----------------------------------------------------------------------
# Value editors
# ----------------------------------------------------------------------


class _IntEditor(QSpinBox):
    """Integer spin box that reports the stored text until it is touched."""

    def __init__(self, lower: int, upper: int, stored: str) -> None:
        super().__init__()
        self._stored = stored
        self._touched = False
        self.setRange(lower, upper)
        self.setValue(int(stored))
        self.valueChanged.connect(self._mark_touched)

    def _mark_touched(self) -> None:
        self._touched = True

    def stored_value(self) -> str:
        return str(self.value()) if self._touched else self._stored


class _FloatEditor(QDoubleSpinBox):
    """Float spin box that reports the stored text until it is touched."""

    def __init__(self, lower: float, upper: float, stored: str) -> None:
        super().__init__()
        self._stored = stored
        self._touched = False
        self.setDecimals(_FLOAT_DECIMALS)
        self.setRange(lower, upper)
        self.setValue(float(stored))
        self.valueChanged.connect(self._mark_touched)

    def _mark_touched(self) -> None:
        self._touched = True

    def stored_value(self) -> str:
        if not self._touched:
            return self._stored
        return str(round(self.value(), _FLOAT_DECIMALS))


def _try_int_editor(info: PropertyInfo, stored: str) -> QWidget | None:
    if info.lower is None or info.upper is None:
        return None
    lower, upper = info.lower, info.upper
    if not (_INT32_MIN <= lower <= upper <= _INT32_MAX):
        return None
    try:
        value = int(stored)
    except ValueError:
        return None
    if not lower <= value <= upper:
        return None
    return _IntEditor(int(lower), int(upper), stored)


def _try_float_editor(info: PropertyInfo, stored: str) -> QWidget | None:
    if info.lower is None or info.upper is None:
        return None
    lower, upper = info.lower, info.upper
    try:
        value = float(stored)
    except ValueError:
        return None
    if not all(math.isfinite(x) for x in (lower, upper, value)):
        return None
    if not lower <= value <= upper:
        return None
    return _FloatEditor(lower, upper, stored)


def make_value_editor(info: PropertyInfo | None, stored: str) -> QWidget:
    """Create the editor widget for one property value.

    Mirrors the choice the stock ``PropertyWidget`` makes: read-only -> label,
    allowed values -> combo box, Integer/Float with limits -> spin box, anything
    else (including unknown metadata) -> line edit. A stored value the chosen
    editor cannot represent (outside the limits, not a number) falls back to a
    line edit so it is never silently altered.

    Parameters
    ----------
    info : PropertyInfo | None
        Cached metadata of the property, if known.
    stored : str
        The value currently stored in the preset.

    Returns
    -------
    QWidget
        Read its value back with :func:`editor_value`.
    """
    if info is not None:
        if info.read_only:
            return QLabel(stored)
        if info.allowed:
            # Cached allowed values can be stale (PVCAM values depend on
            # Port), so a stored value outside the list is kept selectable.
            combo = QComboBox()
            items = list(info.allowed)
            if stored not in items:
                items.insert(0, stored)
            combo.addItems(items)
            combo.setCurrentText(stored)
            return combo
        editor: QWidget | None = None
        if info.ptype == "Integer":
            editor = _try_int_editor(info, stored)
        elif info.ptype == "Float":
            editor = _try_float_editor(info, stored)
        if editor is not None:
            return editor
    return QLineEdit(stored)


def editor_value(editor: QWidget) -> str:
    """Return the value held by an editor made by :func:`make_value_editor`.

    Parameters
    ----------
    editor : QWidget
        An editor created by :func:`make_value_editor`.

    Returns
    -------
    str
        The value as it would be stored in the preset (untouched values are
        returned verbatim, never reformatted).
    """
    if isinstance(editor, _IntEditor | _FloatEditor):
        return editor.stored_value()
    if isinstance(editor, QComboBox):
        return editor.currentText()
    if isinstance(editor, QLineEdit):
        return editor.text()
    if isinstance(editor, QLabel):
        return editor.text()
    raise TypeError(f"Unsupported value editor: {type(editor).__name__}")


def _readonly_item(text: str) -> QTableWidgetItem:
    item = QTableWidgetItem(text)
    item.setFlags(Qt.ItemFlag.ItemIsEnabled)
    return item


def _configure_table(table: QTableWidget, labels: list[str]) -> None:
    table.setColumnCount(len(labels))
    table.setHorizontalHeaderLabels(labels)
    table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
    if (vh := table.verticalHeader()) is not None:
        vh.setVisible(False)
    if (hh := table.horizontalHeader()) is not None:
        hh.setStretchLastSection(True)
        hh.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)


# ----------------------------------------------------------------------
# Preset dialog
# ----------------------------------------------------------------------


class WorkerPresetDialog(QDialog):
    """Add or edit one preset of a worker-owned config group.

    Saving only changes the stored preset; it does not apply the preset to
    the cameras (the stock Edit Preset dialog behaves the same way).

    Parameters
    ----------
    service : CameraWorkerService
        The active camera worker service.
    mmc : CMMCorePlus
        The main-process core (for metadata of devices still loaded there).
    group : str
        A key of ``service.worker_config_groups``.
    preset : str | None
        The preset to edit. ``None`` adds a new preset instead: its rows are the
        (device, property) union over the group's presets, valued from the group's
        current preset (or the first one).
    parent : QWidget | None
        Optional parent widget.
    """

    def __init__(
        self,
        service: CameraWorkerService,
        mmc: CMMCorePlus,
        group: str,
        *,
        preset: str | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._service = service
        self._group = group
        self._edited_preset = preset
        self._meta = WorkerPropertyMetadata(service, mmc)

        cfg = service.worker_config_groups[group]
        if preset is not None:
            self.setWindowTitle(f"Edit preset {group}/{preset}")
            settings = list(cfg.presets[preset])
        else:
            self.setWindowTitle(f"Add preset to {group}")
            settings = self._new_preset_rows(cfg.presets, cfg.current)

        self.name_edit = QLineEdit(preset or "")
        self.name_edit.setToolTip(
            "Saving only edits the stored preset; it is not applied to the cameras."
        )
        self.table = QTableWidget(0, 3)
        _configure_table(self.table, ["Device", "Property", "Value"])
        self._editors: dict[tuple[str, str], QWidget] = {}
        for device, prop, value in settings:
            self._add_row(device, prop, value)

        self.button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

        form = QFormLayout()
        form.addRow("Group:", QLabel(group))
        form.addRow("Preset name:", self.name_edit)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.table)
        layout.addWidget(self.button_box)
        self.resize(560, 360)

    def _new_preset_rows(
        self,
        presets: dict[str, tuple[tuple[str, str, str], ...]],
        current: str,
    ) -> list[tuple[str, str, str]]:
        base_name = current if current in presets else next(iter(presets), "")
        base = {(d, p): v for d, p, v in presets.get(base_name, ())}
        rows: list[tuple[str, str, str]] = []
        seen: set[tuple[str, str]] = set()
        for settings in presets.values():
            for device, prop, _ in settings:
                if (device, prop) in seen:
                    continue
                seen.add((device, prop))
                value = base.get((device, prop))
                if value is None:
                    info = self._meta.info(device, prop)
                    value = info.value if info is not None else ""
                rows.append((device, prop, value))
        return rows

    def _add_row(self, device: str, prop: str, value: str) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, _readonly_item(device))
        self.table.setItem(row, 1, _readonly_item(prop))
        editor = make_value_editor(self._meta.info(device, prop), value)
        self.table.setCellWidget(row, 2, editor)
        self._editors[(device, prop)] = editor

    def editor(self, device: str, prop: str) -> QWidget:
        """Return the value editor of the ``(device, prop)`` row."""
        return self._editors[(device, prop)]

    def settings(self) -> list[tuple[str, str, str]]:
        """Return the table's ``(device, property, value)`` settings, in order."""
        return [(d, p, editor_value(w)) for (d, p), w in self._editors.items()]

    def accept(self) -> None:
        """Store the preset; on invalid input warn and keep the dialog open."""
        try:
            self._service.define_worker_preset(
                self._group,
                self.name_edit.text().strip(),
                self.settings(),
                replace=self._edited_preset,
            )
        except (ValueError, KeyError) as exc:
            QMessageBox.warning(self, "Cannot save preset", str(exc))
            return
        super().accept()


# ----------------------------------------------------------------------
# Group dialog
# ----------------------------------------------------------------------


class WorkerGroupDialog(QDialog):
    """Add a worker-owned config group, or edit which properties one contains.

    Parameters
    ----------
    service : CameraWorkerService
        The active camera worker service.
    mmc : CMMCorePlus
        The main-process core.
    group : str | None
        The worker group to edit; ``None`` adds a new group, created with one
        preset (named in the dialog) holding every checked property's current
        value.
    parent : QWidget | None
        Optional parent widget.

    Attributes
    ----------
    saved_group, saved_preset : str
        Name of the group (and, when adding, of its first preset) written by
        the last successful accept.
    """

    #: Emitted with the saved group name after a successful accept.
    groupSaved = pyqtSignal(str)

    def __init__(
        self,
        service: CameraWorkerService,
        mmc: CMMCorePlus,
        group: str | None = None,
        *,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._service = service
        self._mmc = mmc
        self._edited_group = group
        self._meta = WorkerPropertyMetadata(service, mmc)
        self.saved_group = ""
        self.saved_preset = ""
        self.setWindowTitle(
            "Add camera-worker group" if group is None else "Edit group"
        )

        existing: dict[tuple[str, str], str] = {}
        if group is not None:
            for settings in service.worker_config_groups[group].presets.values():
                for device, prop, value in settings:
                    existing.setdefault((device, prop), value)

        self.name_edit = QLineEdit(group or "")
        self.preset_edit = QLineEdit(DEFAULT_PRESET_NAME)
        self.preset_edit.setToolTip("Name of the group's first preset.")
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("Filter devices / properties...")
        self.filter_edit.textChanged.connect(self._apply_filter)

        self.table = QTableWidget(0, 3)
        _configure_table(self.table, ["Device", "Property", "Value"])
        self._rows: dict[tuple[str, str], int] = {}
        self._values: dict[tuple[str, str], str] = {}
        for (device, prop), value in existing.items():
            self._add_candidate(device, prop, value, checked=True)
        for device, prop, info in self._meta.candidates():
            if (device, prop) not in self._rows:
                self._add_candidate(device, prop, info.value, checked=False)

        self.button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

        form = QFormLayout()
        form.addRow("Group name:", self.name_edit)
        if group is None:
            form.addRow("First preset:", self.preset_edit)
        else:
            self.preset_edit.hide()
        form.addRow("Filter:", self.filter_edit)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.table)
        layout.addWidget(self.button_box)
        self.resize(560, 480)

    def _add_candidate(
        self, device: str, prop: str, value: str, *, checked: bool
    ) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        check = QTableWidgetItem(device)
        check.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsUserCheckable)
        check.setCheckState(
            Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked
        )
        self.table.setItem(row, 0, check)
        self.table.setItem(row, 1, _readonly_item(prop))
        self.table.setItem(row, 2, _readonly_item(value))
        self._rows[(device, prop)] = row
        self._values[(device, prop)] = value

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for (device, prop), row in self._rows.items():
            self.table.setRowHidden(row, needle not in f"{device} {prop}".lower())

    def set_checked(self, device: str, prop: str, checked: bool = True) -> None:
        """Check or uncheck the ``(device, prop)`` row."""
        item = self.table.item(self._rows[(device, prop)], 0)
        if item is not None:
            item.setCheckState(
                Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked
            )

    def checked_pairs(self) -> list[tuple[str, str]]:
        """Return the checked ``(device, property)`` pairs, in table order."""
        out: list[tuple[str, str]] = []
        for pair, row in sorted(self._rows.items(), key=lambda kv: kv[1]):
            item = self.table.item(row, 0)
            if item is not None and item.checkState() == Qt.CheckState.Checked:
                out.append(pair)
        return out

    def _warn(self, message: str) -> None:
        QMessageBox.warning(self, "Cannot save group", message)

    def accept(self) -> None:
        """Create or update the group; on invalid input warn and stay open."""
        name = self.name_edit.text().strip()
        if not name:
            self._warn("The group needs a name.")
            return
        pairs = self.checked_pairs()
        if not pairs:
            self._warn("Check at least one property.")
            return
        if not any(d in self._meta.released_devices for d, _ in pairs):
            self._warn(
                "Check at least one property of a camera-worker device "
                "(Camera-1/Camera-2 ...). A group of main-core devices only "
                "should be a standard group."
            )
            return

        try:
            if self._edited_group is None:
                self._add_group(name, pairs)
            else:
                self._edit_group(name, pairs)
        except (ValueError, KeyError) as exc:
            self._warn(str(exc))
            return
        self.saved_group = name
        super().accept()
        self.groupSaved.emit(name)

    def _add_group(self, name: str, pairs: list[tuple[str, str]]) -> None:
        if (
            name in self._service.worker_config_groups
            or name in self._mmc.getAvailableConfigGroups()
        ):
            raise ValueError(f"A config group named {name!r} already exists.")
        preset = self.preset_edit.text().strip()
        self._service.set_worker_group(
            name, {preset: [(d, p, self._values[(d, p)]) for d, p in pairs]}
        )
        self.saved_preset = preset

    def _edit_group(self, name: str, pairs: list[tuple[str, str]]) -> None:
        assert self._edited_group is not None
        old = self._service.worker_config_groups[self._edited_group]
        keep = set(pairs)
        known = {(d, p) for settings in old.presets.values() for d, p, _ in settings}
        added = [(d, p, self._values[(d, p)]) for d, p in pairs if (d, p) not in known]
        presets = {
            preset: [(d, p, v) for d, p, v in settings if (d, p) in keep] + added
            for preset, settings in old.presets.items()
        }
        self._service.set_worker_group(name, presets, rename_from=self._edited_group)
