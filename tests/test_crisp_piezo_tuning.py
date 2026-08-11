"""Unit tests for the CRISP piezo-tuning bench helpers.

Exercises the property-dump/diff and fluctuation-sampling logic against a
minimal stub core (see ``_StubCore`` -- same convention as
``test_multi_camera.py``/``test_spectral_channel_handler.py``), so no ASI
hardware or CRISPy Qt panel is required.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

import pymmcore_gui.asi_z_stack.crisp_piezo_tuning as cpt
from pymmcore_gui.asi_z_stack.asi_controller import _HW
from pymmcore_gui.asi_z_stack.crisp_piezo_tuning import (
    GAIN_PROP,
    UPDATE_RATE_PROP,
    _focus_score,
    apply_and_sample,
    capture_focus_curve,
    diff_crisp_state,
    dump_crisp_state,
    log_crisp_drift,
    sample_fluctuation,
)

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path


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
        self._positions: dict[str, float] = {}
        self._position_sequences: dict[str, Iterator[float]] = {}
        self._images: Iterator[np.ndarray] | None = None
        self._last_image: np.ndarray | None = None
        self.set_calls: list[tuple[str, str, str]] = []
        self.camera_device: str | None = None
        self.fail_snap: Exception | None = None

    def queue_sequence(self, label: str, prop: str, values: list[Any]) -> None:
        self._sequences[(label, prop)] = iter(str(v) for v in values)

    def queue_positions(self, label: str, values: list[float]) -> None:
        self._position_sequences[label] = iter(values)

    def queue_images(self, images: list[np.ndarray]) -> None:
        self._images = iter(images)

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

    def getPosition(self, label: str) -> float:
        if label in self._position_sequences:
            return next(self._position_sequences[label])
        return self._positions.get(label, 0.0)

    def setCameraDevice(self, label: str) -> None:
        self.camera_device = label

    def snapImage(self) -> None:
        if self.fail_snap is not None:
            raise self.fail_snap
        if self._images is not None:
            self._last_image = next(self._images)

    def getImage(self) -> np.ndarray:
        if self._last_image is not None:
            return self._last_image
        return np.zeros((8, 8), dtype=np.uint16)


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


def test_focus_score_ranks_sharp_above_blurred() -> None:
    flat = np.full((16, 16), 100, dtype=np.uint16)
    checkerboard = np.indices((16, 16)).sum(axis=0) % 2 * 65535
    checkerboard = checkerboard.astype(np.uint16)
    assert _focus_score(flat) == 0.0
    assert _focus_score(checkerboard) > _focus_score(flat)


def test_log_crisp_drift_writes_telemetry_and_position(tmp_path: Path) -> None:
    core = _StubCore(
        {
            "CRISPAFocus:P:34": _piezo_props(),
            "PiezoStage:P:34": {},
        }
    )
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", ["0"] * 10)
    core.queue_positions("PiezoStage:P:34", [-40.180, -40.181, -40.183, -40.190])

    csv_path = log_crisp_drift(
        {"CRISPAFocus:P:34": "PiezoStage:P:34"},
        duration_s=0.25,
        interval_s=0.1,
        out_dir=tmp_path,
        mmcore=core,
    )

    assert csv_path.exists()
    rows = csv_path.read_text().strip().splitlines()
    header, *data_rows = rows
    assert "actuator_position_um" in header
    assert len(data_rows) >= 2
    assert all("CRISPAFocus:P:34" in row for row in data_rows)
    assert all("PiezoStage:P:34" in row for row in data_rows)
    # position values from queue_positions should show up (Python's str()
    # drops the trailing zero: -40.180 -> "-40.18")
    assert "-40.18" in data_rows[0]


def test_log_crisp_drift_captures_camera_snapshots(tmp_path: Path) -> None:
    core = _StubCore(
        {
            "CRISPAFocus:P:34": _piezo_props(),
            "PiezoStage:P:34": {},
        }
    )
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", ["0"] * 10)
    checkerboard = (np.indices((16, 16)).sum(axis=0) % 2 * 65535).astype(np.uint16)
    core.queue_images([checkerboard, checkerboard])

    csv_path = log_crisp_drift(
        {"CRISPAFocus:P:34": "PiezoStage:P:34"},
        duration_s=0.25,
        interval_s=0.1,
        camera_label="Camera-1",
        snapshot_interval_s=0.1,
        out_dir=tmp_path,
        mmcore=core,
    )

    rows = csv_path.read_text().strip().splitlines()
    camera_rows = [r for r in rows if "__camera__" in r]
    assert camera_rows, "expected at least one camera snapshot row"
    assert core.camera_device == "Camera-1"
    saved = list((tmp_path / "snapshots").glob("*.tiff"))
    assert saved


def test_log_crisp_drift_records_snapshot_error_in_csv(tmp_path: Path) -> None:
    core = _StubCore({"CRISPAFocus:P:34": _piezo_props(), "PiezoStage:P:34": {}})
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", ["0"] * 10)
    core.fail_snap = RuntimeError("camera busy")

    csv_path = log_crisp_drift(
        {"CRISPAFocus:P:34": "PiezoStage:P:34"},
        duration_s=0.25,
        interval_s=0.1,
        camera_label="Camera-1",
        snapshot_interval_s=0.1,
        out_dir=tmp_path,
        mmcore=core,
    )

    rows = csv_path.read_text().strip().splitlines()
    camera_rows = [r for r in rows if "__camera__" in r]
    assert camera_rows
    # failed attempt: error captured, no snapshot file/path/score written
    assert "camera busy" in camera_rows[0]
    assert not list((tmp_path / "snapshots").glob("*.tiff"))


def test_log_crisp_drift_rejects_unknown_laser(tmp_path: Path) -> None:
    core = _StubCore({"CRISPAFocus:P:34": _piezo_props(), "PiezoStage:P:34": {}})
    with pytest.raises(ValueError, match="Unknown laser"):
        log_crisp_drift(
            {"CRISPAFocus:P:34": "PiezoStage:P:34"},
            duration_s=1.0,
            camera_label="Camera-1",
            laser="not-a-real-laser",
            out_dir=tmp_path,
            mmcore=core,
        )


def test_log_crisp_drift_gates_laser_around_each_snapshot(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[tuple[str, tuple[int, ...], bool] | tuple[str]] = []
    monkeypatch.setattr(
        cpt, "ensure_global_shutter_open", lambda: calls.append(("shutter",))
    )

    def fake_set_laser_outputs(
        plogic_label: str,
        hub_label: str,
        bnc_addrs: list[int],
        on: bool,
        always_on_cell: int,
    ) -> None:
        calls.append(("laser", tuple(bnc_addrs), on))

    monkeypatch.setattr(cpt, "set_laser_outputs", fake_set_laser_outputs)

    core = _StubCore({"CRISPAFocus:P:34": _piezo_props(), "PiezoStage:P:34": {}})
    core.queue_sequence("CRISPAFocus:P:34", "Dither Error", ["0"] * 10)
    checkerboard = (np.indices((16, 16)).sum(axis=0) % 2 * 65535).astype(np.uint16)
    core.queue_images([checkerboard])

    # duration_s < interval_s so the loop breaks after exactly one outer
    # iteration -- one snapshot attempt, consuming the single queued image.
    csv_path = cpt.log_crisp_drift(
        {"CRISPAFocus:P:34": "PiezoStage:P:34"},
        duration_s=0.05,
        interval_s=0.1,
        camera_label="Camera-1",
        snapshot_interval_s=0.1,
        laser="488nm",
        out_dir=tmp_path,
        mmcore=core,
    )

    assert ("shutter",) in calls
    laser_calls = [c for c in calls if c[0] == "laser"]
    assert len(laser_calls) == 2
    on_call, off_call = laser_calls
    expected_addr = (_HW.laser_bnc_addr["488nm"],)
    assert on_call == ("laser", expected_addr, True)
    assert off_call == ("laser", expected_addr, False)

    rows = csv_path.read_text().strip().splitlines()
    assert any("__camera__" in r and "snapshots" in r for r in rows)
