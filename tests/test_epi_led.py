from __future__ import annotations

import importlib.util
import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock, patch

import pytest
from pydantic import ValidationError

from pymmcore_gui._mmcore_shutdown import shutdown_mmcore
from pymmcore_gui._settings import EpiLedChannel, SettingsV1, _default_epi_leds
from pymmcore_gui.epi import _mode
from pymmcore_gui.epi._led import (
    LED_INTENSITY_PROP,
    all_leds_off,
    apply_led,
    channel_for_preset,
    classify_led_label,
    pct_to_prop,
    resolve_led_channels,
)
from pymmcore_gui.epi._mode import ScopeMode, current_mode, set_active_controller

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pymmcore_plus import CMMCorePlus

TGLED = ["LED:L:37:1", "LED:L:37:2", "LED:L:37:3", "LED:L:37:4"]
DECOY_X = "LED:X:31"  # unrelated X-axis LED on the XY card
DECOY_OTHER_CARD = "LED:L:38:1"


class FakeLedCore:
    """The slice of CMMCorePlus the LED code uses, shaped like the ASITiger adapter.

    - ``LED:L:37:1..4`` are the TGLED channels; ``LED:X:31`` and ``LED:L:38:1``
      are decoys, and ``LED:X:31`` is also the Core Shutter (as in
      ``20261103-LED_Only.cfg``).
    - ``LED Intensity(%)`` is an Integer property limited to 1..100; floats are
      truncated like the adapter's ``atol``.
    - Writing the intensity while the shutter is closed only updates a cache
      (``cache``); the card (``on_card``) takes it when the shutter opens.
    - ``events`` / ``mda`` come from a real core when one is given, so widgets
      that connect to them behave normally.
    """

    def __init__(
        self,
        real: CMMCorePlus | None = None,
        labels: list[str] | None = None,
    ) -> None:
        if real is not None:
            self.events = real.events
            self.mda = real.mda
        self.labels = [*TGLED, DECOY_X, DECOY_OTHER_CARD] if labels is None else labels
        self.library: dict[str, str] = dict.fromkeys(self.labels, "ASITiger")
        self.open = dict.fromkeys(self.labels, False)
        self.cache = dict.fromkeys(self.labels, 50)
        self.on_card = dict.fromkeys(self.labels, 0)  # 0 = dark
        self.fail_on: set[str] = set()
        self.calls: list[tuple[Any, ...]] = []

    # ---- discovery
    def getLoadedDevicesOfType(self, _dev_type: Any) -> list[str]:
        return [*self.labels, "Shutter"]  # "Shutter" = a non-LED shutter

    def getDeviceLibrary(self, label: str) -> str:
        return self.library.get(label, "DemoCamera")

    def getShutterDevice(self) -> str:
        return DECOY_X if DECOY_X in self.labels else "Shutter"

    def getDevicePropertyNames(self, label: str) -> list[str]:
        return ["Name", LED_INTENSITY_PROP, "State"]

    # ---- state
    def getShutterOpen(self, label: str) -> bool:
        self.calls.append(("getShutterOpen", label))
        return self.open[label]

    def setShutterOpen(self, label: str, state: bool) -> None:
        self.calls.append(("setShutterOpen", label, state))
        if label in self.fail_on:
            raise RuntimeError("serial error")
        self.open[label] = state
        self.on_card[label] = self.cache[label] if state else 0

    def getProperty(self, label: str, prop: str) -> str:
        self.calls.append(("getProperty", label, prop))
        if prop == LED_INTENSITY_PROP:
            return str(self.cache[label])
        if prop == "State":
            return "1" if self.open[label] else "0"
        raise KeyError(prop)

    def setProperty(self, label: str, prop: str, value: Any) -> None:
        self.calls.append(("setProperty", label, prop, value))
        if prop == "State":
            self.setShutterOpen(label, bool(int(value)))
            return
        if prop != LED_INTENSITY_PROP:
            raise KeyError(prop)
        iv = int(float(value))  # Integer property: atol truncates
        if not 1 <= iv <= 100:
            raise ValueError(f"{iv} outside 1..100")
        self.cache[label] = iv
        if self.open[label]:
            self.on_card[label] = iv

    # ---- helpers for assertions
    def writes(self) -> list[tuple[Any, ...]]:
        return [c for c in self.calls if c[0] in ("setProperty", "setShutterOpen")]

    def touched(self, label: str) -> list[tuple[Any, ...]]:
        return [c for c in self.calls if c[1] == label]


class _EpiController:
    mode = ScopeMode.EPI


class _OpmController:
    mode = ScopeMode.OPM


@pytest.fixture(autouse=True)
def _reset_mode() -> Iterator[None]:
    set_active_controller(None)
    yield
    set_active_controller(None)


@pytest.fixture
def epi_mode() -> None:
    set_active_controller(_EpiController())


@pytest.fixture
def fake() -> FakeLedCore:
    return FakeLedCore()


# ------------------------------------------------------------------ mode


def test_mode_defaults_to_opm_without_a_controller() -> None:
    assert _mode._active_controller is None
    assert current_mode() is ScopeMode.OPM


def test_mode_follows_the_controller() -> None:
    set_active_controller(_EpiController())
    assert current_mode() is ScopeMode.EPI
    set_active_controller(_OpmController())
    assert current_mode() is ScopeMode.OPM
    set_active_controller(None)
    assert current_mode() is ScopeMode.OPM


# ------------------------------------------------------------- pct_to_prop


@pytest.mark.parametrize(
    ("pct", "expected"),
    [(-5, 1), (0, 1), (0.4, 1), (42.4, 42), (42.6, 43), (100, 100), (250, 100)],
)
def test_pct_to_prop(pct: float, expected: int) -> None:
    out = pct_to_prop(pct)
    assert out == expected
    assert type(out) is int


# ----------------------------------------------------------- classification


@pytest.mark.parametrize(
    ("label", "expected"),
    [
        ("LED:L:37:1", 1),
        ("LED:L:37:4", 4),
        ("LED:X:37:2", 2),  # any axis letter
        ("LED:Z:37:3", 3),
        ("LED:X:31", None),  # 3 tokens, the unrelated X-axis LED
        ("LED:X:37", None),  # legacy 3-token form, even at 37
        ("LED:L:38:1", None),  # other card
        ("LED:L:370:1", None),  # address must match exactly
        ("LED:L:37:0", None),
        ("LED:L:37:5", None),
        ("LED:L:37:12", None),
        ("LED:L:37:x", None),
        ("LED:L:37:1:2", None),
        ("XLED:L:37:1", None),
        ("LED", None),
        ("", None),
    ],
)
def test_classify_led_label(label: str, expected: int | None) -> None:
    assert classify_led_label(label) == expected


def test_classify_led_label_other_card_address() -> None:
    assert classify_led_label("LED:L:38:2", "38") == 2
    assert classify_led_label("LED:L:37:2", "38") is None


# ------------------------------------------------------ resolve_led_channels


def test_resolve_returns_the_four_channels_and_ignores_decoys(
    fake: FakeLedCore,
) -> None:
    got = resolve_led_channels(fake)  # type: ignore[arg-type]
    assert got == dict(zip((1, 2, 3, 4), TGLED, strict=False))
    assert DECOY_X not in got.values()
    assert DECOY_OTHER_CARD not in got.values()


def test_resolve_accepts_any_axis_letter() -> None:
    labels = ["LED:A:37:1", "LED:B:37:2"]
    fake = FakeLedCore(labels=labels)
    assert resolve_led_channels(fake) == {1: "LED:A:37:1", 2: "LED:B:37:2"}  # type: ignore[arg-type]


def test_resolve_with_only_the_x_led_finds_nothing() -> None:
    fake = FakeLedCore(labels=[DECOY_X])
    assert resolve_led_channels(fake) == {}  # type: ignore[arg-type]


def test_resolve_requires_the_asitiger_library(fake: FakeLedCore) -> None:
    fake.library["LED:L:37:2"] = "DemoCamera"
    assert 2 not in resolve_led_channels(fake)  # type: ignore[arg-type]


def test_resolve_honours_card_address(fake: FakeLedCore) -> None:
    assert resolve_led_channels(fake, "38") == {1: DECOY_OTHER_CARD}  # type: ignore[arg-type]


# --------------------------------------------------------- channel_for_preset


def test_channel_for_preset_maps_led_presets_to_settings_names() -> None:
    leds = SettingsV1().epi.leds
    ch = channel_for_preset("LED488", leds)
    assert ch is not None
    assert (ch.channel, ch.name) == (1, "488")
    ch = channel_for_preset("LED642", leds)
    assert ch is not None
    assert ch.channel == 3
    # channel 4 exists in settings (hidden) but has no preset in the cfg
    assert channel_for_preset("LED405", leds) is not None
    assert channel_for_preset("488nm", leds) is None  # a Lasers preset
    assert channel_for_preset("LED999", leds) is None
    assert channel_for_preset("LED", leds) is None


# --------------------------------------------------------------- apply_led


def test_apply_led_refuses_in_opm_and_touches_nothing(fake: FakeLedCore) -> None:
    with pytest.raises(RuntimeError, match="EPI"):
        apply_led(fake, "LED:L:37:1", 50)  # type: ignore[arg-type]
    assert fake.writes() == []


def test_apply_led_writes_percent_before_opening(
    fake: FakeLedCore, epi_mode: None
) -> None:
    apply_led(fake, "LED:L:37:2", 42.6)  # type: ignore[arg-type]
    assert fake.writes() == [
        ("setProperty", "LED:L:37:2", LED_INTENSITY_PROP, 43),
        ("setShutterOpen", "LED:L:37:2", True),
    ]
    assert fake.on_card["LED:L:37:2"] == 43  # lit at the new value, not a stale one


@pytest.mark.parametrize(("pct", "expected"), [(0, 1), (-3, 1), (500, 100)])
def test_apply_led_never_writes_an_illegal_percent(
    fake: FakeLedCore, epi_mode: None, pct: float, expected: int
) -> None:
    apply_led(fake, "LED:L:37:1", pct)  # type: ignore[arg-type]
    assert fake.cache["LED:L:37:1"] == expected


def test_fake_intensity_write_while_closed_only_updates_cache(
    fake: FakeLedCore,
) -> None:
    fake.setProperty("LED:L:37:1", LED_INTENSITY_PROP, 30)
    assert fake.cache["LED:L:37:1"] == 30
    assert fake.on_card["LED:L:37:1"] == 0  # still dark
    fake.setShutterOpen("LED:L:37:1", True)
    assert fake.on_card["LED:L:37:1"] == 30


# ------------------------------------------------------------- all_leds_off


def test_all_leds_off_closes_all_four_channels_and_never_writes_intensity(
    fake: FakeLedCore,
) -> None:
    for lbl in TGLED:
        fake.open[lbl] = True
    all_leds_off(fake)  # type: ignore[arg-type]  # works in OPM (no controller)
    assert fake.writes() == [("setShutterOpen", lbl, False) for lbl in TGLED]
    assert not any(fake.open[lbl] for lbl in TGLED)
    assert all(c[0] != "setProperty" for c in fake.calls)


def test_all_leds_off_never_touches_the_decoys(fake: FakeLedCore) -> None:
    fake.open[DECOY_X] = True
    all_leds_off(fake)  # type: ignore[arg-type]
    assert fake.touched(DECOY_X) == []
    assert fake.touched(DECOY_OTHER_CARD) == []
    assert fake.open[DECOY_X] is True


def test_all_leds_off_is_allowed_in_epi_too(fake: FakeLedCore, epi_mode: None) -> None:
    all_leds_off(fake)  # type: ignore[arg-type]
    assert len(fake.writes()) == 4


def test_all_leds_off_continues_past_a_failing_device(
    fake: FakeLedCore, caplog: pytest.LogCaptureFixture
) -> None:
    fake.fail_on.add("LED:L:37:2")
    with caplog.at_level(logging.ERROR, logger="pymmcore_gui"):
        all_leds_off(fake)  # type: ignore[arg-type]
    closed = [c[1] for c in fake.writes() if c[2] is False]
    assert closed == TGLED
    assert any("LED:L:37:2" in r.getMessage() for r in caplog.records)


def test_all_leds_off_tolerates_missing_devices(
    caplog: pytest.LogCaptureFixture,
) -> None:
    all_leds_off(FakeLedCore(labels=[]))  # type: ignore[arg-type]  # nothing loaded
    broken = MagicMock()
    broken.getLoadedDevicesOfType.side_effect = RuntimeError("core gone")
    with caplog.at_level(logging.ERROR, logger="pymmcore_gui"):
        all_leds_off(broken)  # must not raise
    assert caplog.records


def test_all_leds_off_with_only_the_x_led_loaded_does_nothing() -> None:
    fake = FakeLedCore(labels=[DECOY_X])
    all_leds_off(fake)  # type: ignore[arg-type]
    assert fake.calls == []


# ----------------------------------------------------------------- shutdown


def _shutdown_core(fake: FakeLedCore) -> MagicMock:
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = False
    mmc.mda.is_running.return_value = False
    mmc.getLoadedDevicesOfType.side_effect = fake.getLoadedDevicesOfType
    mmc.getDeviceLibrary.side_effect = fake.getDeviceLibrary
    mmc.setShutterOpen.side_effect = fake.setShutterOpen
    return mmc


@pytest.mark.parametrize("mode_ctrl", [None, _EpiController()])
def test_shutdown_closes_every_tgled_channel_and_never_the_decoys(
    fake: FakeLedCore, mode_ctrl: object
) -> None:
    set_active_controller(mode_ctrl)
    mmc = _shutdown_core(fake)
    with (
        patch("pymmcore_gui.asi_z_stack.asi_controller.close_all_lasers", MagicMock()),
        patch("pymmcore_gui._mmcore_shutdown._quiesce_background_reads", MagicMock()),
    ):
        shutdown_mmcore(mmc)
    assert [c[1] for c in fake.writes()] == TGLED
    assert fake.touched(DECOY_X) == []
    mmc.unloadAllDevices.assert_called_once_with()


def test_shutdown_still_unloads_when_led_close_fails(fake: FakeLedCore) -> None:
    mmc = _shutdown_core(fake)
    mmc.getLoadedDevicesOfType.side_effect = RuntimeError("boom")
    with (
        patch("pymmcore_gui.asi_z_stack.asi_controller.close_all_lasers", MagicMock()),
        patch("pymmcore_gui._mmcore_shutdown._quiesce_background_reads", MagicMock()),
    ):
        shutdown_mmcore(mmc)  # must not raise
    mmc.unloadAllDevices.assert_called_once_with()


# ----------------------------------------------------------------- settings


def test_default_epi_settings() -> None:
    epi = SettingsV1().epi
    assert epi.card_address == "37"
    assert [(c.channel, c.name, c.visible, c.percent) for c in epi.leds] == [
        (1, "488", True, 50),
        (2, "561", True, 50),
        (3, "642", True, 50),
        (4, "405", False, 50),
    ]


def test_default_led_map_is_the_confirmed_one() -> None:
    # ASI X/Y/Z/F = channel 1..4; user-confirmed X=488, Y=561, Z=642; 405 unused.
    leds = _default_epi_leds()
    assert {c.channel: c.name for c in leds} == {
        1: "488",
        2: "561",
        3: "642",
        4: "405",
    }
    assert [c.visible for c in leds] == [True, True, True, False]


def test_default_settings_dump_is_still_empty() -> None:
    assert SettingsV1().model_dump_json(exclude_defaults=True) == "{}"


def test_epi_settings_round_trip_keeps_leds() -> None:
    s = SettingsV1()
    s.epi.leds[1].percent = 80
    s.epi.leds[3].visible = True
    dumped = s.model_dump_json(exclude_defaults=True)
    assert set(json.loads(dumped)) == {"epi"}
    again = SettingsV1.model_validate_json(dumped)
    assert again.epi.leds[1].percent == 80
    assert again.epi.leds[3].visible is True
    assert [c.name for c in again.epi.leds] == ["488", "561", "642", "405"]


@pytest.mark.parametrize("pct", [0, 101, -1])
def test_led_percent_must_be_1_to_100(pct: int) -> None:
    with pytest.raises(ValidationError):
        EpiLedChannel(channel=1, name="488", percent=pct)


@pytest.mark.parametrize("channel", [0, 5])
def test_led_channel_must_be_1_to_4(channel: int) -> None:
    with pytest.raises(ValidationError):
        EpiLedChannel(channel=channel, name="x")


# --------------------------------------------------------- discover_tgled.py


def _load_discover_script() -> Any:
    path = Path(__file__).parent.parent / "scripts" / "discover_tgled.py"
    spec = importlib.util.spec_from_file_location("discover_tgled", path)
    assert spec is not None
    assert spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_discover_script_labels_tgled_and_other_leds() -> None:
    mod = _load_discover_script()
    installed = [
        "XYStage:XY:31",
        "LED:X:31",
        "LED:L:37:1",
        "LED:L:37:4",
        "LED:L:38:1",
    ]
    lines = [mod.device_line(n) for n in installed]
    assert lines == [
        "Device,XYStage:XY:31,ASITiger,XYStage:XY:31",
        "Device,LED:X:31,ASITiger,LED:X:31  # other LED, not the TGLED",
        "Device,LED:L:37:1,ASITiger,LED:L:37:1  # TGLED channel 1",
        "Device,LED:L:37:4,ASITiger,LED:L:37:4  # TGLED channel 4",
        "Device,LED:L:38:1,ASITiger,LED:L:38:1  # other LED, not the TGLED",
    ]


def test_discover_script_main_uses_installed_device_list(
    capsys: pytest.CaptureFixture[str],
) -> None:
    mod = _load_discover_script()
    core = MagicMock()
    core.getInstalledDevices.return_value = [DECOY_X, *TGLED]
    with patch.object(mod, "CMMCorePlus", return_value=core):
        mod.main("COM4")
    out = capsys.readouterr().out
    assert "Device,LED:X:31,ASITiger,LED:X:31  # other LED, not the TGLED" in out
    assert out.count("# TGLED channel") == 4
    assert "No TGLED" not in out
