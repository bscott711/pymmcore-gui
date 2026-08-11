"""Unit tests for the CRISP piezo-tuning bench helpers.

Exercises the property-dump/diff and fluctuation-sampling logic against a
minimal stub core (see ``_StubCore`` -- same convention as
``test_multi_camera.py``/``test_spectral_channel_handler.py``), so no ASI
hardware or CRISPy Qt panel is required.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from pymmcore_gui.asi_z_stack.crisp_piezo_tuning import (
    GAIN_PROP,
    UPDATE_RATE_PROP,
    apply_and_sample,
    capture_focus_curve,
    diff_crisp_state,
    dump_crisp_state,
    sample_fluctuation,
)

if TYPE_CHECKING:
    from collections.abc import Iterator


class _StubCore:
    """Minimal stand-in exposing only what crisp_piezo_tuning needs.

    ``queue_sequence`` lets a single (label, property) pair yield a
    different value on each successive ``getProperty`` call, standing in
    for a live sensor value (e.g. ``"Dither Error"``) changing between polls.
    """

    def __init__(self, devices: dict[str, dict[str, str]]) -> None:
        self._devices: dict[str, dict[str, str]] = {
            label: dict(props) for label, props in devices.items()
        }
        self._sequences: dict[tuple[str, str], Iterator[str]] = {}
        self.set_calls: list[tuple[str, str, str]] = []

    def queue_sequence(self, label: str, prop: str, values: list[Any]) -> None:
        self._sequences[(label, prop)] = iter(str(v) for v in values)

    def getLoadedDevices(self) -> list[str]:
        return list(self._devices)

    def getDevicePropertyNames(self, label: str) -> list[str]:
        return list(self._devices[label])

    def getProperty(self, label: str, prop: str) -> str:
        key = (label, prop)
        if key in self._sequences:
            return next(self._sequences[key])
        return self._devices[label][prop]

    def setProperty(self, label: str, prop: str, value: str) -> None:
        self._devices.setdefault(label, {})[prop] = value
        self.set_calls.append((label, prop, value))

    def hasProperty(self, label: str, prop: str) -> bool:
        return prop in self._devices.get(label, {})

    def getLoadedDevicesOfType(self, device_type: object) -> list[str]:
        # No paired stage device modeled -> classify_focus_actuator sees none
        # and reports "unknown", which is fine: nothing under test depends
        # on the actuator-type classification itself.
        return []


def _piezo_props(**overrides: str) -> dict[str, str]:
    base = {
        "AxisLetter": "P",
        "TigerHexAddress": "34",
        "CRISP State": "Lock",
        "Calibration Gain": "-1000",
        "Calibration Range(um)": "0.5",  # sensitivity = 1000/0.5 = 2000 counts/um
        UPDATE_RATE_PROP: "5",
        GAIN_PROP: "4",
        "Max Lock Range(mm)": "2.0000",
    }
    base.update(overrides)
    return base


def test_dump_crisp_state_reads_all_properties() -> None:
    core = _StubCore({"CRISPAFocus:P:34": _piezo_props()})
    dump = dump_crisp_state(["CRISPAFocus:P:34"], mmcore=core)
    assert dump == {"CRISPAFocus:P:34": _piezo_props()}


def test_dump_crisp_state_skips_unloaded_labels() -> None:
    core = _StubCore({"CRISPAFocus:P:34": _piezo_props()})
    dump = dump_crisp_state(["CRISPAFocus:P:34", "CRISPAFocus:Z:32"], mmcore=core)
    assert list(dump) == ["CRISPAFocus:P:34"]


def test_diff_crisp_state_reports_only_changed_properties() -> None:
    before = {"CRISPAFocus:P:34": _piezo_props()}
    after = {
        "CRISPAFocus:P:34": _piezo_props(**{UPDATE_RATE_PROP: "15", GAIN_PROP: "8"})
    }
    changed = diff_crisp_state(before, after)
    assert changed == {
        "CRISPAFocus:P:34": {
            UPDATE_RATE_PROP: ("5", "15"),
            GAIN_PROP: ("4", "8"),
        }
    }


def test_diff_crisp_state_empty_when_nothing_changed() -> None:
    props = {"CRISPAFocus:P:34": _piezo_props()}
    assert diff_crisp_state(props, props) == {}


def test_sample_fluctuation_converts_counts_to_microns() -> None:
    core = _StubCore({"CRISPAFocus:P:34": _piezo_props()})
    # sensitivity = 2000 counts/um (see _piezo_props); error swings +-1um.
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", [0, 2000, -2000, 0])
    stats = sample_fluctuation(
        "CRISPAFocus:P:34", n_samples=4, interval_s=0.0, mmcore=core
    )
    assert stats.n_samples == 4
    assert stats.sensitivity_counts_per_um == 2000.0
    assert stats.ptp_counts == 4000.0
    assert stats.ptp_um is not None
    assert stats.ptp_um == 2.0


def test_sample_fluctuation_without_calibration_returns_none_um() -> None:
    core = _StubCore(
        {
            "CRISPAFocus:P:34": _piezo_props(
                **{"Calibration Gain": "0", "Calibration Range(um)": "0"}
            )
        }
    )
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", [10, 20, 10])
    stats = sample_fluctuation(
        "CRISPAFocus:P:34", n_samples=3, interval_s=0.0, mmcore=core
    )
    assert stats.sensitivity_counts_per_um is None
    assert stats.std_um is None
    assert stats.ptp_um is None


def test_apply_and_sample_writes_update_rate_and_gain() -> None:
    core = _StubCore({"CRISPAFocus:P:34": _piezo_props()})
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", [0, 0, 0])
    apply_and_sample(
        "CRISPAFocus:P:34",
        update_rate_ms=15,
        gain_multiplier=8,
        n_samples=3,
        interval_s=0.0,
        settle_s=0.0,
        prompt=False,
        mmcore=core,
    )
    assert (
        "CRISPAFocus:P:34",
        UPDATE_RATE_PROP,
        "15",
    ) in core.set_calls
    assert ("CRISPAFocus:P:34", GAIN_PROP, "8") in core.set_calls


def test_capture_focus_curve_collects_changed_lines_only() -> None:
    core = _StubCore(
        {
            "CRISPAFocus:P:34": _piezo_props(),
            "TigerCommHub": {"SerialResponse": ""},
        }
    )
    # Repeats a value once (should be deduped) then produces a fresh one.
    core.queue_sequence(
        "TigerCommHub", "SerialResponse", ["T=1 Z=0.1", "T=1 Z=0.1", "T=2 Z=0.2"]
    )
    # duration_s gives comfortable margin over poll_interval_s (~15 polls) so
    # the 3-value queue is exhausted well before the deadline; further polls
    # after exhaustion raise StopIteration, which capture_focus_curve catches
    # and treats as "no new value" -- so this isn't timing-sensitive.
    lines = capture_focus_curve(
        "CRISPAFocus:P:34", duration_s=0.15, poll_interval_s=0.01, mmcore=core
    )
    assert lines == ["T=1 Z=0.1", "T=2 Z=0.2"]
    assert ("CRISPAFocus:P:34", "CRISP State", "Curve") in core.set_calls
