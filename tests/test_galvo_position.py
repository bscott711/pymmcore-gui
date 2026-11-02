"""Tests for the static galvo slice-axis move helper."""

from __future__ import annotations

import logging
from unittest.mock import MagicMock

import pytest

from pymmcore_gui.asi_z_stack import galvo_position
from pymmcore_gui.asi_z_stack.galvo_position import (
    move_galvo_slice,
    read_galvo_position,
)

DEV = "Scanner:AB:33"


def _core(positions: list, props: dict[str, str] | None = None) -> MagicMock:
    """Fake core whose successive getGalvoPosition calls return ``positions``."""
    core = MagicMock()
    core.getGalvoPosition.side_effect = list(positions)
    props = props or {}
    core.hasProperty.side_effect = lambda _dev, prop: prop in props
    core.getProperty.side_effect = lambda _dev, prop: props[prop]
    return core


def test_move_preserves_x_and_returns_readback() -> None:
    core = _core([[1.5, 0.0], [1.5, 2.0]])
    assert move_galvo_slice(core, DEV, 2.0) == (1.5, 2.0)
    core.setGalvoPosition.assert_called_once_with(DEV, 1.5, 2.0)


def test_mismatch_warns_with_diagnostics(caplog: pytest.LogCaptureFixture) -> None:
    props = {"BeamEnabled": "No", "SingleAxisYMode": "Enabled", "SPIMState": "Idle"}
    core = _core([(0.0, 0.0), (0.0, 0.0)], props)
    with caplog.at_level(logging.WARNING, logger=galvo_position.__name__):
        assert move_galvo_slice(core, DEV, 2.0) == (0.0, 0.0)
    text = caplog.text
    assert "BeamEnabled=No" in text
    assert "SingleAxisYMode=Enabled" in text
    assert "SPIMState=Idle" in text


def test_missing_diagnostic_props_are_skipped(
    caplog: pytest.LogCaptureFixture,
) -> None:
    core = _core([(0.0, 0.0), (0.0, 0.0)], {"BeamEnabled": "Yes"})
    with caplog.at_level(logging.WARNING, logger=galvo_position.__name__):
        move_galvo_slice(core, DEV, 2.0)
    assert "BeamEnabled=Yes" in caplog.text
    assert "SingleAxisYMode" not in caplog.text


def test_unreadable_x_falls_back_to_zero() -> None:
    core = MagicMock()
    core.getGalvoPosition.side_effect = [RuntimeError("no"), (0.0, 1.0)]
    assert move_galvo_slice(core, DEV, 1.0) == (0.0, 1.0)
    core.setGalvoPosition.assert_called_once_with(DEV, 0.0, 1.0)


def test_set_exception_does_not_propagate() -> None:
    core = _core([(0.5, 0.0)])
    core.setGalvoPosition.side_effect = RuntimeError("boom")
    assert move_galvo_slice(core, DEV, 1.0) is None


def test_get_exception_does_not_propagate() -> None:
    core = MagicMock()
    core.getGalvoPosition.side_effect = RuntimeError("boom")
    assert read_galvo_position(core, DEV) is None
    assert move_galvo_slice(core, DEV, 1.0) is None


def test_magicmock_core_does_not_crash() -> None:
    core = MagicMock()
    assert read_galvo_position(core, DEV) is None
    move_galvo_slice(core, DEV, 1.0)


def test_demo_galvo_round_trip() -> None:
    from pymmcore_plus import CMMCorePlus

    core = CMMCorePlus()
    try:
        if "DGalvo" not in core.getAvailableDevices("DemoCamera"):
            pytest.skip("DemoCamera has no DGalvo")
        core.loadDevice("Galvo", "DemoCamera", "DGalvo")
        core.initializeDevice("Galvo")
    except Exception as e:
        pytest.skip(f"cannot load demo galvo: {e!r}")
    result = move_galvo_slice(core, "Galvo", 0.5)
    if result is None:
        pytest.skip("demo galvo does not support position round trip")
    assert result[1] == pytest.approx(0.5, abs=0.01)
    assert read_galvo_position(core, "Galvo") == pytest.approx(result)
