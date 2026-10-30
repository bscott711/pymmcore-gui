"""Editing worker-owned config groups from the Config Groups panel."""

from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest
from pymmcore_widgets import GroupPresetTableWidget
from qtpy.QtCore import QItemSelectionModel
from qtpy.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QLabel,
    QLineEdit,
    QMenu,
    QMessageBox,
    QSpinBox,
)

from pymmcore_gui.asi_z_stack.camera_handoff import PropertyInfo
from pymmcore_gui.widgets._worker_group_editor import (
    WorkerGroupDialog,
    WorkerPresetDialog,
    WorkerPropertyMetadata,
    editor_value,
    make_value_editor,
)
from pymmcore_gui.widgets._worker_group_preset_table import (
    WorkerGroupPresetTableWidget,
)
from test_camera_worker_service import _FakePool, _service

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator
    from pathlib import Path

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot

    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

_OK = QMessageBox.StandardButton.Ok
_CANCEL = QMessageBox.StandardButton.Cancel
_TABLE_MOD = "pymmcore_gui.widgets._worker_group_preset_table"


@pytest.fixture
def core(mmcore: CMMCorePlus) -> CMMCorePlus:
    # The global instance: the stock PresetsWidget ignores the table's core.
    mmcore.loadSystemConfiguration()
    return mmcore


@pytest.fixture(autouse=True)
def warn() -> Iterator[MagicMock]:
    """Nothing may block on a message box."""
    with patch("qtpy.QtWidgets.QMessageBox.warning") as mock:
        yield mock


@pytest.fixture
def confirm() -> Iterator[MagicMock]:
    with patch.object(
        WorkerGroupPresetTableWidget, "_msg_box", return_value=_OK
    ) as mock:
        yield mock


def _with_metadata(svc: CameraWorkerService) -> CameraWorkerService:
    assert svc._snapshot is not None
    svc._snapshot.device_properties = {
        "Camera-1": {
            "Port": PropertyInfo("Speed", allowed=("Sensitivity", "Speed")),
            "Gain": PropertyInfo("1", ptype="Integer", lower=1, upper=4),
            "Exposure": PropertyInfo("10.50", ptype="Float", lower=0, upper=1000),
            "Temperature": PropertyInfo("20", read_only=True),
            "Serial": PropertyInfo("abc", pre_init=True),
        },
        "Camera-2": {"Port": PropertyInfo("Speed", allowed=("Sensitivity", "Speed"))},
        "Multi Camera": {
            "Physical Camera 1": PropertyInfo("Camera-1", allowed=("Camera-1",))
        },
    }
    return svc


@pytest.fixture
def panel(
    qtbot: QtBot, core: CMMCorePlus
) -> tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool]:
    pool = _FakePool()
    svc = _with_metadata(_service(core, pool))
    wdg = WorkerGroupPresetTableWidget(svc, mmcore=core)
    qtbot.addWidget(wdg)
    return wdg, svc, pool


def _row(wdg: WorkerGroupPresetTableWidget, group: str) -> int:
    for r in range(wdg.table_wdg.rowCount()):
        if wdg._row_group(r) == group:
            return r
    raise AssertionError(f"no row for {group!r}")


def _has_row(wdg: WorkerGroupPresetTableWidget, group: str) -> bool:
    return any(wdg._row_group(r) == group for r in range(wdg.table_wdg.rowCount()))


def _select(wdg: WorkerGroupPresetTableWidget, *groups: str) -> None:
    table = wdg.table_wdg
    table.clearSelection()
    sel, model = table.selectionModel(), table.model()
    assert sel is not None
    assert model is not None
    for group in groups:
        sel.select(
            model.index(_row(wdg, group), 0),
            QItemSelectionModel.SelectionFlag.Select
            | QItemSelectionModel.SelectionFlag.Rows,
        )


def _preset_dialog(wdg: WorkerGroupPresetTableWidget) -> WorkerPresetDialog:
    dlg = wdg._worker_dialog
    assert isinstance(dlg, WorkerPresetDialog)
    return dlg


def _group_dialog(wdg: WorkerGroupPresetTableWidget) -> WorkerGroupDialog:
    dlg = wdg._worker_dialog
    assert isinstance(dlg, WorkerGroupDialog)
    return dlg


def _combo(wdg: WorkerGroupPresetTableWidget, group: str) -> QComboBox:
    return wdg._worker_combos[group]


# ----------------------------------------------------------------------
# Edit / add preset
# ----------------------------------------------------------------------


def test_edit_preset_changes_values_and_renames(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, svc, pool = panel
    _select(wdg, "Cameras")
    wdg._edit_preset()

    dlg = _preset_dialog(wdg)
    assert dlg.name_edit.text() == "Speed"  # the preset shown in the combo
    # Prefilled with the stored values.
    assert dlg.settings() == list(svc.worker_config_groups["Cameras"].presets["Speed"])

    port = dlg.editor("Camera-1", "Port")
    assert isinstance(port, QComboBox)
    port.setCurrentText("Sensitivity")
    dlg.name_edit.setText("Fast")
    dlg.accept()

    cfg = svc.worker_config_groups["Cameras"]
    assert list(cfg.presets) == ["Sensitivity", "Fast"]
    assert cfg.presets["Fast"] == (
        ("Camera-1", "Port", "Sensitivity"),
        ("Camera-2", "Port", "Speed"),
    )
    assert cfg.current == "Fast"  # follows the rename
    # The table was rebuilt: combo updated, row still selected.
    combo = _combo(wdg, "Cameras")
    assert [combo.itemText(i) for i in range(combo.count())] == ["Sensitivity", "Fast"]
    assert combo.currentText() == "Fast"
    assert wdg._selected_rows() == [_row(wdg, "Cameras")]
    assert pool.calls == []  # editing never touches hardware


def test_edit_preset_invalid_name_warns_and_stays_open(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    warn: MagicMock,
) -> None:
    wdg, svc, _ = panel
    _select(wdg, "Cameras")
    wdg._edit_preset()
    dlg = _preset_dialog(wdg)
    dlg.name_edit.setText("Sensitivity")  # clashes with the other preset
    dlg.accept()

    warn.assert_called_once()
    assert dlg.isVisible()
    assert dict(svc.worker_config_groups["Cameras"].presets)["Speed"] == (
        ("Camera-1", "Port", "Speed"),
        ("Camera-2", "Port", "Speed"),
    )


def test_edit_preset_without_shown_preset_warns(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    warn: MagicMock,
) -> None:
    wdg, svc, _ = panel
    svc.worker_config_groups["Cameras"].current = ""
    wdg._populate_table()
    _select(wdg, "Cameras")
    wdg._edit_preset()
    warn.assert_called_once()
    assert wdg._worker_dialog is None


def test_add_preset_copies_rows_from_current_preset(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, svc, pool = panel
    _select(wdg, "Cameras")
    wdg._add_preset()

    dlg = _preset_dialog(wdg)
    assert dlg.name_edit.text() == ""
    # Union of the group's (device, property) pairs, valued from "Speed".
    assert [(d, p) for d, p, _ in dlg.settings()] == [
        ("Camera-1", "Port"),
        ("Camera-2", "Port"),
        ("Multi Camera", "Physical Camera 1"),
        ("Dichroic", "Label"),
    ]
    assert dlg.settings()[0][2] == "Speed"
    # Dichroic is still loaded in the main core: its metadata comes from there.
    assert isinstance(dlg.editor("Dichroic", "Label"), QComboBox)

    dlg.accept()  # no name
    assert list(svc.worker_config_groups["Cameras"].presets) == ["Sensitivity", "Speed"]

    port = dlg.editor("Camera-1", "Port")
    assert isinstance(port, QComboBox)
    port.setCurrentText("Sensitivity")
    dlg.name_edit.setText("Mixed")
    dlg.accept()

    cfg = svc.worker_config_groups["Cameras"]
    assert list(cfg.presets) == ["Sensitivity", "Speed", "Mixed"]
    assert ("Camera-1", "Port", "Sensitivity") in cfg.presets["Mixed"]
    assert ("Camera-2", "Port", "Speed") in cfg.presets["Mixed"]
    assert cfg.current == "Speed"
    assert "Mixed" in [
        _combo(wdg, "Cameras").itemText(i)
        for i in range(_combo(wdg, "Cameras").count())
    ]
    assert pool.calls == []


def test_service_edit_rebuilds_the_table(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, svc, _ = panel
    svc.define_worker_preset("Cameras", "Extra", [("Camera-1", "Port", "Speed")])
    assert _combo(wdg, "Cameras").findText("Extra") >= 0
    svc.delete_worker_group("Cameras")
    assert not _has_row(wdg, "Cameras")


# ----------------------------------------------------------------------
# Delete preset / group
# ----------------------------------------------------------------------


def test_delete_preset_and_last_preset_removes_group(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    confirm: MagicMock,
) -> None:
    wdg, svc, _ = panel
    _select(wdg, "Cameras")
    wdg._delete_preset()  # deletes the shown preset, "Speed"

    confirm.assert_called_once()
    cfg = svc.worker_config_groups["Cameras"]
    assert list(cfg.presets) == ["Sensitivity"]
    assert cfg.current == ""
    assert wdg._selected_rows() == []  # worker rows are deselected afterwards

    _select(wdg, "Cameras")
    _combo(wdg, "Cameras").setCurrentText("Sensitivity")
    wdg._delete_preset()
    assert "Cameras" not in svc.worker_config_groups
    assert not _has_row(wdg, "Cameras")


def test_delete_preset_cancelled_changes_nothing(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    confirm: MagicMock,
) -> None:
    wdg, svc, _ = panel
    confirm.return_value = _CANCEL
    _select(wdg, "Cameras")
    wdg._delete_preset()
    wdg._delete_group()
    assert list(svc.worker_config_groups["Cameras"].presets) == ["Sensitivity", "Speed"]


def test_delete_group(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    confirm: MagicMock,
    core: CMMCorePlus,
) -> None:
    wdg, svc, _ = panel
    n_core = len(core.getAvailableConfigGroups())
    _select(wdg, "Cameras")
    wdg._delete_group()

    confirm.assert_called_once()
    assert "Cameras" not in svc.worker_config_groups
    assert not _has_row(wdg, "Cameras")
    assert len(core.getAvailableConfigGroups()) == n_core


def test_mixed_selection_delete_group_still_deletes_core_rows(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    confirm: MagicMock,
    core: CMMCorePlus,
) -> None:
    wdg, svc, _ = panel
    assert "Channel" in core.getAvailableConfigGroups()
    _select(wdg, "Cameras", "Channel")
    wdg._delete_group()

    assert "Cameras" not in svc.worker_config_groups
    assert "Channel" not in core.getAvailableConfigGroups()
    assert not _has_row(wdg, "Channel")
    assert confirm.call_count == 2  # one per path


def test_mixed_selection_delete_preset_still_deletes_core_preset(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    confirm: MagicMock,
    core: CMMCorePlus,
) -> None:
    wdg, svc, _ = panel
    n_presets = len(core.getAvailableConfigs("Channel"))
    assert n_presets > 1
    _select(wdg, "Cameras", "Channel")
    wdg._delete_preset()

    assert list(svc.worker_config_groups["Cameras"].presets) == ["Sensitivity"]
    assert len(core.getAvailableConfigs("Channel")) == n_presets - 1


# ----------------------------------------------------------------------
# Add / edit group
# ----------------------------------------------------------------------


def test_add_worker_group_then_edit_its_preset(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, svc, pool = panel
    wdg._add_worker_group()
    dlg = _group_dialog(wdg)
    dlg.name_edit.setText("Tuning")
    dlg.set_checked("Camera-1", "Gain")
    dlg.set_checked("Camera-2", "Port")
    dlg.accept()

    cfg = svc.worker_config_groups["Tuning"]
    assert cfg.presets == {
        "NewPreset": (("Camera-1", "Gain", "1"), ("Camera-2", "Port", "Speed"))
    }
    assert cfg.current == ""
    assert _has_row(wdg, "Tuning")
    # The new group's first preset opens for editing right away.
    preset_dlg = _preset_dialog(wdg)
    assert preset_dlg.name_edit.text() == "NewPreset"
    assert isinstance(preset_dlg.editor("Camera-1", "Gain"), QSpinBox)
    assert pool.calls == []


def test_add_worker_group_validation(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    warn: MagicMock,
) -> None:
    wdg, svc, _ = panel
    wdg._add_worker_group()
    dlg = _group_dialog(wdg)

    dlg.accept()  # no name
    dlg.name_edit.setText("Tuning")
    dlg.accept()  # nothing checked
    assert warn.call_count == 2

    dlg.set_checked("Dichroic", "Label")  # a main-core device only
    dlg.accept()
    assert warn.call_count == 3

    dlg.set_checked("Camera-1", "Gain")
    dlg.name_edit.setText("Cameras")  # clashes with a worker group
    dlg.accept()
    assert warn.call_count == 4
    dlg.name_edit.setText("Channel")  # clashes with a core group
    dlg.accept()
    assert warn.call_count == 5
    assert set(svc.worker_config_groups) == {"Cameras"}
    assert dlg.isVisible()


def test_add_worker_group_candidates_skip_read_only_and_pre_init(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, _svc, _ = panel
    wdg._add_worker_group()
    dlg = _group_dialog(wdg)
    pairs = set(dlg._rows)
    assert ("Camera-1", "Gain") in pairs
    assert ("Camera-1", "Temperature") not in pairs  # read-only
    assert ("Camera-1", "Serial") not in pairs  # pre-init
    assert not any(d == "Core" for d, _ in pairs)
    assert ("Dichroic", "Label") in pairs  # loaded in the main core

    dlg.filter_edit.setText("gain")
    assert not dlg.table.isRowHidden(dlg._rows[("Camera-1", "Gain")])
    assert dlg.table.isRowHidden(dlg._rows[("Camera-2", "Port")])


def test_edit_group_renames_adds_and_removes_properties(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, svc, _ = panel
    _select(wdg, "Cameras")
    wdg._edit_group()

    dlg = _group_dialog(wdg)
    assert dlg.name_edit.text() == "Cameras"
    assert dlg.checked_pairs() == [
        ("Camera-1", "Port"),
        ("Camera-2", "Port"),
        ("Multi Camera", "Physical Camera 1"),
        ("Dichroic", "Label"),
    ]
    dlg.name_edit.setText("Cams")
    dlg.set_checked("Dichroic", "Label", False)
    dlg.set_checked("Camera-1", "Gain")
    dlg.accept()

    assert list(svc.worker_config_groups) == ["Cams"]
    cfg = svc.worker_config_groups["Cams"]
    assert cfg.presets["Sensitivity"] == (
        ("Camera-1", "Port", "Sensitivity"),
        ("Camera-2", "Port", "Sensitivity"),
        ("Multi Camera", "Physical Camera 1", "Camera-1"),
        ("Camera-1", "Gain", "1"),
    )
    assert cfg.presets["Speed"] == (
        ("Camera-1", "Port", "Speed"),
        ("Camera-2", "Port", "Speed"),
        ("Camera-1", "Gain", "1"),
    )
    assert cfg.current == "Speed"
    assert _has_row(wdg, "Cams")
    assert not _has_row(wdg, "Cameras")


def test_edit_group_needs_a_worker_device(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    warn: MagicMock,
) -> None:
    wdg, svc, _ = panel
    _select(wdg, "Cameras")
    wdg._edit_group()
    dlg = _group_dialog(wdg)
    for device, prop in dlg.checked_pairs():
        if device != "Dichroic":
            dlg.set_checked(device, prop, False)
    dlg.accept()

    warn.assert_called_once()
    assert len(svc.worker_config_groups["Cameras"].presets["Speed"]) == 2


def test_add_group_button_offers_a_choice(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, _svc, _ = panel

    def trigger(index: int) -> Callable[..., None]:
        def run(menu: QMenu, *_args: object) -> None:
            menu.actions()[index].trigger()

        return run

    with patch.object(QMenu, "exec", new=trigger(1)):
        wdg._add_group()
    assert isinstance(wdg._worker_dialog, WorkerGroupDialog)

    with (
        patch.object(GroupPresetTableWidget, "_add_group") as stock,
        patch.object(QMenu, "exec", new=trigger(0)),
    ):
        wdg._add_group()
    stock.assert_called_once()


# ----------------------------------------------------------------------
# Core groups are untouched
# ----------------------------------------------------------------------


def test_core_group_handlers_reach_the_stock_path(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    confirm: MagicMock,
    core: CMMCorePlus,
) -> None:
    wdg, svc, _ = panel
    _select(wdg, "Channel")

    wdg._edit_preset()
    assert wdg._worker_dialog is None
    stock_edit = wdg._edit_preset_wgd
    assert stock_edit.isVisible()

    wdg._add_preset()
    assert wdg._worker_dialog is None
    assert wdg._add_preset_wdg.isVisible()
    assert not stock_edit.isVisible()  # _close_if_hasattr closed it

    wdg._edit_group()
    assert wdg._worker_dialog is None
    assert wdg._edit_group_wdg.isVisible()

    wdg._delete_group()
    assert "Channel" not in core.getAvailableConfigGroups()
    assert "Cameras" in svc.worker_config_groups


def test_opening_a_stock_dialog_closes_the_worker_dialog(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
) -> None:
    wdg, _svc, _ = panel
    _select(wdg, "Cameras")
    wdg._edit_preset()
    dlg = _preset_dialog(wdg)
    assert dlg.isVisible()
    wdg._close_if_hasattr()
    assert not dlg.isVisible()


# ----------------------------------------------------------------------
# Save cfg
# ----------------------------------------------------------------------


def test_save_cfg_uses_worker_aware_writer(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    core: CMMCorePlus,
    tmp_path: Path,
    warn: MagicMock,
) -> None:
    wdg, _svc, _ = panel
    target = tmp_path / "out"
    with (
        patch(
            f"{_TABLE_MOD}.QFileDialog.getSaveFileName", return_value=(str(target), "")
        ),
        patch(f"{_TABLE_MOD}.save_system_configuration") as save,
    ):
        wdg._save_cfg()
        save.assert_called_once_with(core, f"{target}.cfg")

        save.reset_mock()
        save.side_effect = RuntimeError("no file loaded")
        wdg._save_cfg()
        warn.assert_called_once()

    with (
        patch(f"{_TABLE_MOD}.QFileDialog.getSaveFileName", return_value=("", "")),
        patch(f"{_TABLE_MOD}.save_system_configuration") as save,
    ):
        wdg._save_cfg()
        save.assert_not_called()


# ----------------------------------------------------------------------
# Value editors and metadata
# ----------------------------------------------------------------------


def test_value_editor_choice_per_metadata_kind(qtbot: QtBot) -> None:
    label = make_value_editor(PropertyInfo("7", read_only=True), "7")
    assert isinstance(label, QLabel)
    assert editor_value(label) == "7"

    combo = make_value_editor(PropertyInfo("A", allowed=("A", "B")), "C")
    assert isinstance(combo, QComboBox)
    assert not combo.isEditable()
    # A stored value outside the (possibly stale) allowed list is kept.
    assert [combo.itemText(i) for i in range(combo.count())] == ["C", "A", "B"]
    assert editor_value(combo) == "C"
    combo.setCurrentText("B")
    assert editor_value(combo) == "B"

    spin = make_value_editor(PropertyInfo("2", ptype="Integer", lower=1, upper=4), "2")
    assert isinstance(spin, QSpinBox)
    assert editor_value(spin) == "2"
    spin.setValue(3)
    assert editor_value(spin) == "3"

    dspin = make_value_editor(
        PropertyInfo("10.50", ptype="Float", lower=0, upper=100), "10.50"
    )
    assert isinstance(dspin, QDoubleSpinBox)
    assert editor_value(dspin) == "10.50"  # untouched: never reformatted
    dspin.setValue(12.25)
    assert editor_value(dspin) == "12.25"

    # Anything the typed editors cannot represent exactly stays a line edit.
    overflow = PropertyInfo("5", ptype="Integer", lower=0, upper=2**40)
    out_of_range = PropertyInfo("9", ptype="Integer", lower=1, upper=4)
    no_limits = PropertyInfo("1.5", ptype="Float")
    for info, stored in [
        (overflow, "5"),
        (out_of_range, "9"),
        (no_limits, "1.5"),
        (PropertyInfo("x", ptype="Integer", lower=0, upper=4), "x"),
        (PropertyInfo("s"), "s"),
        (None, "anything"),
    ]:
        edit = make_value_editor(info, stored)
        assert isinstance(edit, QLineEdit)
        assert editor_value(edit) == stored


def test_metadata_sources(
    panel: tuple[WorkerGroupPresetTableWidget, CameraWorkerService, _FakePool],
    core: CMMCorePlus,
) -> None:
    _wdg, svc, _ = panel
    meta = WorkerPropertyMetadata(svc, core)

    released = meta.info("Camera-1", "Port")
    assert released is not None
    assert released.allowed == ("Sensitivity", "Speed")
    loaded = meta.info("Dichroic", "Label")
    assert loaded is not None
    assert loaded.value == core.getProperty("Dichroic", "Label")
    assert meta.info("NotADevice", "Prop") is None
    assert meta.info("Camera-1", "NotAProp") is None
    assert meta.released_devices == {"Camera-1", "Camera-2", "Multi Camera"}
