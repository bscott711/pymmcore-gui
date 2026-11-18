"""Persisting and restoring the galvo Live offset."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import pytest

from pymmcore_gui.asi_z_stack.live_offset import (
    OFFSET_PROP,
    restore_live_offset,
    save_live_offset_deg,
    saved_live_offset_deg,
)

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._settings import Settings

GALVO = "Scanner:AB:33"
LOGGER = "pymmcore_gui.asi_z_stack.live_offset"


class _Mda:
    def __init__(self) -> None:
        self.running = False

    def is_running(self) -> bool:
        return self.running


class _FakeCore:
    """Just enough of CMMCorePlus for restore_live_offset."""

    def __init__(
        self,
        limits: tuple[float, float] | None = (-2.0, 2.0),
        loaded: bool = True,
        has_prop: bool = True,
    ) -> None:
        self.mda = _Mda()
        self.limits = limits
        self.loaded = loaded
        self.has_prop = has_prop
        self.writes: list[tuple[str, str, str]] = []
        self.moves: list[tuple[str, float, float]] = []
        self.position = (0.3, 0.0)
        self.set_property_error: Exception | None = None

    def getLoadedDevices(self) -> tuple[str, ...]:
        return (GALVO,) if self.loaded else ("Other",)

    def hasProperty(self, device: str, prop: str) -> bool:
        return self.has_prop

    def hasPropertyLimits(self, device: str, prop: str) -> bool:
        return self.limits is not None

    def getPropertyLowerLimit(self, device: str, prop: str) -> float:
        assert self.limits is not None
        return self.limits[0]

    def getPropertyUpperLimit(self, device: str, prop: str) -> float:
        assert self.limits is not None
        return self.limits[1]

    def setProperty(self, device: str, prop: str, value: str) -> None:
        if self.set_property_error is not None:
            raise self.set_property_error
        self.writes.append((device, prop, value))

    def setGalvoPosition(self, device: str, x: float, y: float) -> None:
        self.moves.append((device, x, y))
        self.position = (x, y)

    def getGalvoPosition(self, device: str) -> tuple[float, float]:
        return self.position


def _core(**kwargs: Any) -> tuple[_FakeCore, CMMCorePlus]:
    core = _FakeCore(**kwargs)
    return core, core  # type: ignore[return-value]


def test_default_is_minus_26_um(settings: Settings) -> None:
    assert settings.galvo.live_offset_deg == pytest.approx(-0.26)
    assert saved_live_offset_deg() == pytest.approx(-0.26)


def test_save_round_trips(settings: Settings) -> None:
    save_live_offset_deg(0.125)
    assert settings.galvo.live_offset_deg == 0.125
    assert saved_live_offset_deg() == 0.125
    save_live_offset_deg(0.0)
    assert saved_live_offset_deg() == 0.0


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_is_not_saved(settings: Settings, bad: float) -> None:
    save_live_offset_deg(0.5)
    save_live_offset_deg(bad)
    assert saved_live_offset_deg() == 0.5


def test_saved_non_finite_falls_back_to_default(
    settings: Settings, caplog: pytest.LogCaptureFixture
) -> None:
    settings.galvo.live_offset_deg = float("nan")
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        assert saved_live_offset_deg() == pytest.approx(-0.26)
    assert any(r.levelno == logging.WARNING for r in caplog.records)


def test_save_never_raises(
    settings: Settings,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    def _boom(*_: Any, **__: Any) -> None:
        raise OSError("disk full")

    monkeypatch.setattr(type(settings), "flush", _boom)
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        save_live_offset_deg(0.5)
    assert any(r.levelno == logging.WARNING for r in caplog.records)


def test_restore_writes_property_moves_mirror_and_logs(
    settings: Settings, caplog: pytest.LogCaptureFixture
) -> None:
    save_live_offset_deg(-0.26)
    fake, core = _core()
    with caplog.at_level(logging.INFO, logger=LOGGER):
        applied = restore_live_offset(core)
    assert applied == pytest.approx(-0.26)
    assert fake.writes == [(GALVO, OFFSET_PROP, "-0.260000")]
    # static move keeps X, sets Y
    assert fake.moves == [(GALVO, 0.3, pytest.approx(-0.26))]
    infos = [
        r for r in caplog.records if r.name == LOGGER and r.levelno == logging.INFO
    ]
    assert len(infos) == 1
    msg = infos[0].getMessage()
    assert GALVO in msg
    assert "-0.2600 deg" in msg
    assert "-26.00 um" in msg
    assert "read-back -0.2600 deg" in msg


def test_restore_uses_the_default_when_nothing_was_saved(settings: Settings) -> None:
    fake, core = _core()
    assert restore_live_offset(core) == pytest.approx(-0.26)
    assert fake.writes[0][2] == "-0.260000"


def test_restore_reports_an_unreadable_mirror(
    settings: Settings,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    fake, core = _core()

    def _boom(device: str) -> tuple[float, float]:
        raise RuntimeError("no read")

    monkeypatch.setattr(fake, "getGalvoPosition", _boom)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        assert restore_live_offset(core) == pytest.approx(-0.26)
    assert any("unreadable" in r.getMessage() for r in caplog.records)


def test_restore_clamps_to_the_property_limits(settings: Settings) -> None:
    save_live_offset_deg(5.0)
    fake, core = _core(limits=(-2.0, 2.0))
    assert restore_live_offset(core) == 2.0
    assert fake.writes[-1][2] == "2.000000"
    assert fake.moves[-1][2] == 2.0
    save_live_offset_deg(-9.0)
    assert restore_live_offset(core) == -2.0
    assert fake.writes[-1][2] == "-2.000000"


def test_restore_without_limits_does_not_clamp(settings: Settings) -> None:
    save_live_offset_deg(5.0)
    fake, core = _core(limits=None)
    assert restore_live_offset(core) == 5.0


def test_restore_explicit_device(settings: Settings) -> None:
    fake, core = _core()
    monkey_device = "Scanner:CD:34"
    fake.getLoadedDevices = lambda: (monkey_device,)  # type: ignore[method-assign]
    assert restore_live_offset(core, monkey_device) == pytest.approx(-0.26)
    assert fake.writes[0][0] == monkey_device


def test_restore_is_a_noop_during_an_mda(settings: Settings) -> None:
    fake, core = _core()
    fake.mda.running = True
    assert restore_live_offset(core) is None
    assert fake.writes == []
    assert fake.moves == []


def test_restore_is_a_noop_when_the_device_is_missing(settings: Settings) -> None:
    fake, core = _core(loaded=False)
    assert restore_live_offset(core) is None
    assert fake.writes == []
    assert fake.moves == []


def test_restore_is_a_noop_when_the_property_is_missing(settings: Settings) -> None:
    fake, core = _core(has_prop=False)
    assert restore_live_offset(core) is None
    assert fake.writes == []
    assert fake.moves == []


def test_restore_never_raises_when_the_write_fails(
    settings: Settings, caplog: pytest.LogCaptureFixture
) -> None:
    fake, core = _core()
    fake.set_property_error = RuntimeError("serial down")
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        assert restore_live_offset(core) is None
    assert fake.moves == []
    assert any(
        r.levelno == logging.WARNING and "serial down" in r.getMessage()
        for r in caplog.records
    )


def test_restore_writes_only_the_offset_property(settings: Settings) -> None:
    fake, core = _core()
    restore_live_offset(core)
    assert {prop for _, prop, _ in fake.writes} == {OFFSET_PROP}
    assert len(fake.writes) == 1
