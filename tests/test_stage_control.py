"""The stage panel: no "Zero" (set origin) for piezo stages."""

from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import MagicMock

from pymmcore_plus import CMMCorePlus

from pymmcore_gui.widgets._stage_control import StagesControlWidget, is_piezo

if TYPE_CHECKING:
    from pytestqt.qtbot import QtBot


def _core(name: str, description: str) -> MagicMock:
    mmc = MagicMock()
    mmc.getDeviceName.return_value = name
    mmc.getDeviceDescription.return_value = description
    return mmc


def test_an_asi_piezo_is_a_piezo() -> None:
    assert is_piezo(_core("PiezoStage:P:34", "ASI Piezo Stage"), "PiezoStage:P:34")


def test_an_asi_z_stage_is_not() -> None:
    assert not is_piezo(_core("ZStage:Z:32", "ASI Z Stage"), "ZStage:Z:32")


def test_an_unreadable_device_is_not() -> None:
    mmc = MagicMock()
    mmc.getDeviceName.side_effect = RuntimeError("no such device")
    assert not is_piezo(mmc, "gone")


def test_the_panel_hides_zero_only_for_piezos(qtbot: QtBot) -> None:
    core = CMMCorePlus()
    core.loadSystemConfiguration()  # the demo config: XY and Z stages, no piezo
    widget = StagesControlWidget(mmcore=core)
    qtbot.addWidget(widget)
    from pymmcore_widgets import StageWidget

    stages = widget.findChildren(StageWidget)
    assert stages
    if not hasattr(stages[0], "enable_set_origin"):
        return  # an older pymmcore-widgets: nothing to hide
    assert all(not s._origin_btn.isHidden() for s in stages)

    # The same panel when the demo Z stage reports itself as a piezo.
    real_description = core.getDeviceDescription
    core.getDeviceDescription = lambda label: (  # type: ignore[method-assign]
        "A Piezo Stage" if label == "Z" else real_description(label)
    )
    widget._on_cfg_loaded()
    hidden = {
        s._device: s._origin_btn.isHidden() for s in widget.findChildren(StageWidget)
    }
    assert hidden["Z"] is True
    assert hidden["XY"] is False
