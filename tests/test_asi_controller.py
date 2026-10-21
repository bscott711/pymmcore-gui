"""Unit tests for PLogic serial-command helpers in ``asi_controller``.

These mock the module-level ``mmc`` (a ``CMMCorePlus`` singleton reference)
rather than constructing a real core, since the functions under test only
ever call a handful of its methods (``getLoadedDevices``, ``hasProperty``,
``setProperty``) to drive raw Tiger serial commands.
"""

from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import MagicMock

from pymmcore_gui.asi_z_stack import asi_controller
from pymmcore_gui.asi_z_stack.asi_controller import configure_plogic_for_dual_nrt_pulses
from pymmcore_gui.asi_z_stack.common import AcquisitionSettings

if TYPE_CHECKING:
    import pytest

TIGER_COMM_HUB_LABEL = "TigerCommHub"
PLOGIC_LABEL = "PLogic:E:36"


def _sent_commands(fake_mmc: MagicMock) -> list[str]:
    """Extract the ``SerialCommand`` strings sent to the mocked comm hub."""
    return [
        call.args[2]
        for call in fake_mmc.setProperty.call_args_list
        if call.args[:2] == (TIGER_COMM_HUB_LABEL, "SerialCommand")
    ]


def test_configure_plogic_no_longer_loads_a_laser_preset(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Regression test: no unconditional PLogic preset load before pulse setup.

    ``configure_plogic_for_dual_nrt_pulses`` used to send a bare
    ``CCA X=<preset>`` as its very first command, with no ``M E=`` pointer set
    beforehand. Confirmed against ASI's own Tiger PLogic documentation, that
    preset (30, the default ``plogic_laser_preset_num``) is "diSPIM:
    simultaneous 4-color" -- it wires *all four* laser BNC outputs to fire
    together off cell 10, a completely different preset family from the
    single-wavelength selection the ``"Lasers"`` ConfigGroup's
    ``OutputChannel`` property already performs per-channel. That step is
    removed: this function must now only program the camera/laser NRT pulse
    cells and route the camera cell to BNC1, leaving laser BNC routing
    entirely to the ConfigGroup switch.
    """
    fake_mmc = MagicMock()
    fake_mmc.getLoadedDevices.return_value = [TIGER_COMM_HUB_LABEL]
    fake_mmc.hasProperty.return_value = False
    monkeypatch.setattr(asi_controller, "mmc", fake_mmc)

    settings = AcquisitionSettings(camera_exposure_ms=10.0, laser_trig_duration_ms=10.0)
    configure_plogic_for_dual_nrt_pulses(
        settings,
        PLOGIC_LABEL,
        TIGER_COMM_HUB_LABEL,
        plogic_camera_cell=11,
        pulses_per_ms=4.0,
        plogic_4khz_clock_addr=192,
        plogic_trigger_ttl_addr=41,
        plogic_laser_on_cell=10,
    )

    commands = _sent_commands(fake_mmc)
    assert commands, "no serial commands were sent"
    # The old first command -- a bare, unguarded preset load -- must be gone.
    assert "36CCA X=30" not in commands
    # The very first command must now address the camera cell directly,
    # not load a preset.
    assert commands[0] == "M E=11"


def test_configure_plogic_for_all_rows_gates_laser_on_expose_out(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """All Rows: short camera trigger on BNC1, laser-on = AND of Expose Out inputs."""
    from pymmcore_gui.asi_z_stack.asi_controller import configure_plogic_for_all_rows

    fake_mmc = MagicMock()
    fake_mmc.getLoadedDevices.return_value = [TIGER_COMM_HUB_LABEL]
    fake_mmc.hasProperty.return_value = False
    monkeypatch.setattr(asi_controller, "mmc", fake_mmc)

    configure_plogic_for_all_rows(
        PLOGIC_LABEL,
        TIGER_COMM_HUB_LABEL,
        plogic_camera_cell=11,
        plogic_laser_on_cell=10,
        pulses_per_ms=4.0,
        plogic_4khz_clock_addr=192,
        plogic_trigger_ttl_addr=41,
        trigger_pulse_ms=1.0,
        expose_out_addrs=[34, 36],
    )

    assert _sent_commands(fake_mmc) == [
        # Expose Out BNCs become inputs first (never left as push-pull outputs)
        "M E=34",
        "36CCA Y=0",
        "M E=36",
        "36CCA Y=0",
        "36SS Z",
        # camera: 1 ms NRT one-shot off the galvo slice TTL
        "M E=11",
        "36CCA Y=14 Z=4",
        "36CCB X=41 Y=192 Z=0",
        # laser on = Expose Out(Camera-1) AND Expose Out(Camera-2)
        "M E=10",
        "36CCA Y=5 Z=0",
        "36CCB X=34 Y=36",
        # camera cell -> BNC1
        "M E=33",
        "36CCA Z=11",
        "36SS Z",
    ]


def test_configure_plogic_for_all_rows_single_camera_ties_high(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from pymmcore_gui.asi_z_stack.asi_controller import configure_plogic_for_all_rows

    fake_mmc = MagicMock()
    fake_mmc.getLoadedDevices.return_value = [TIGER_COMM_HUB_LABEL]
    fake_mmc.hasProperty.return_value = False
    monkeypatch.setattr(asi_controller, "mmc", fake_mmc)

    configure_plogic_for_all_rows(
        PLOGIC_LABEL, TIGER_COMM_HUB_LABEL, 11, 10, 4.0, 192, 41, 1.0, [34]
    )

    assert "36CCB X=34 Y=64" in _sent_commands(fake_mmc)
