import json
from pathlib import Path
from unittest.mock import patch

import pytest
from pydantic_settings import BaseSettings

from pymmcore_gui import _settings
from pymmcore_gui._settings import MMGuiUserPrefsSource, SettingsV1


def test_settings() -> None:
    settings = SettingsV1()
    assert settings.version == "1.0"
    assert settings.version_tuple == (1, 0, "")

    # settings ignores unrecognized fields
    # (this is important for backwards compatibility)
    v = SettingsV1(random_value="asdf")  # pyright: ignore[reportCallIssue]
    assert not hasattr(v, "random_value")


def test_user_settings(tmp_path: Path) -> None:
    fake_settings = tmp_path / "settings.json"
    with patch.object(_settings, "SETTINGS_FILE_NAME", fake_settings):
        assert not MMGuiUserPrefsSource.exists()
        assert MMGuiUserPrefsSource.values() == {}

        fake_settings.touch()
        assert MMGuiUserPrefsSource.exists()
        assert MMGuiUserPrefsSource(BaseSettings)() == {}

        # test unrecognized fields are ignored
        fake_settings.write_text('{"a": 1}')
        with pytest.warns(RuntimeWarning, match="not found in model"):
            assert MMGuiUserPrefsSource(BaseSettings)() == {"a": 1}

        # test that the presence of invalid fields don't invalidate
        # the entire settings file
        fake_settings.write_text(
            '{"window": {"geometry": "AAAC", "window_state": [1,2,3] } }'
        )
        with pytest.warns(
            RuntimeWarning, match="Could not validate key 'window_state'"
        ):
            assert MMGuiUserPrefsSource(BaseSettings)() == {
                "window": {"geometry": "AAAC"}
            }

        # test invalid json doesn't ruin everything
        fake_settings.write_text("[]")
        with pytest.warns(RuntimeWarning, match="Failed to read settings"):
            assert MMGuiUserPrefsSource(BaseSettings)() == {}

        obj = SettingsV1.instance()
        assert obj.auto_load_last_config is None
        obj.auto_load_last_config = True
        with patch("pymmcore_gui._settings.TESTING", False):
            obj.flush(timeout=0.2)

        txt = fake_settings.read_text()
        obj2 = SettingsV1.model_validate_json(txt)
        assert obj2.auto_load_last_config is True

        assert fake_settings.exists()
        with patch("pymmcore_gui._settings.TESTING", False):
            _settings.reset_to_defaults()
        assert not fake_settings.exists()


def test_focus_offset_settings_roundtrip() -> None:
    s = SettingsV1()
    assert s.focus.enabled is False
    assert s.focus.offsets == {}
    assert s.focus.locked_preset == ""

    s.focus.enabled = True
    s.focus.locked_preset = "488nm"
    s.focus.counts_per_um = 558.5
    s.focus.apply_live = True
    s.focus.offsets = {"488nm": 0.0, "561nm": 0.35}

    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    assert s2.focus.enabled is True
    assert s2.focus.locked_preset == "488nm"
    assert s2.focus.counts_per_um == 558.5
    assert s2.focus.apply_live is True
    assert s2.focus.offsets == {"488nm": 0.0, "561nm": 0.35}


def test_focus_offset_bad_offsets_entry_degrades_gracefully() -> None:
    # A single unparsable offset drops only `offsets` (back to {}), leaving
    # the rest of the `focus` section intact -- not the whole section.
    data = {"focus": {"enabled": True, "offsets": {"561nm": "banana"}}}
    with pytest.warns(RuntimeWarning, match="Could not validate key 'offsets'"):
        cleaned = _settings._good_data_only(SettingsV1, data)
    assert cleaned == {"focus": {"enabled": True}}


def test_removed_image_focus_section_is_ignored() -> None:
    # The row-shift image lock was dropped; a settings file written while it
    # existed still carries an ``image_focus`` section. It must load, with the
    # other sections intact.
    old = {
        "version": "1.0",
        "focus": {"enabled": True},
        "image_focus": {
            "mode": "image",
            "loop_gain": 0.4,
            "box": {
                "camera": "Camera-1",
                "preset": "488nm",
                "slice_index": 1,
                "rect": [100, 60, 40, 30],
            },
            "calibrations": {},
        },
    }
    with pytest.warns(RuntimeWarning, match="image_focus"):
        cleaned = _settings._good_data_only(SettingsV1, old)
    s = SettingsV1.model_validate(cleaned)
    assert s.focus.enabled is True
    assert s.version == "1.0"
    assert not hasattr(s, "image_focus")
    assert "image_focus" not in s.model_dump()
    # the JSON path (no cleaning) ignores it as well
    s2 = SettingsV1.model_validate_json(json.dumps(old))
    assert s2.focus.enabled is True
    assert not hasattr(s2, "image_focus")


def test_spot_edge_disagreement_and_hold_band_settings() -> None:
    s = SettingsV1()
    p = s.spot_focus.params
    assert (p.max_edge_disagreement_um, p.hold_band_um) == (1.0, 0.15)
    assert s.spot_focus.params.to_params().hold_band_um == 0.15

    p.hold_band_um = 0.2
    p.max_edge_disagreement_um = 0.8
    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    assert s2.spot_focus.params.hold_band_um == 0.2
    loop = s2.spot_focus.params.to_params()
    assert (loop.max_edge_disagreement_um, loop.hold_band_um) == (0.8, 0.2)

    # a file written before hold_band_um existed loads with the default
    old = '{"version": "1.0", "spot_focus": {"params": {"kp": 0.5}}}'
    s3 = SettingsV1.model_validate_json(old)
    assert s3.spot_focus.params.kp == 0.5
    assert s3.spot_focus.params.hold_band_um == 0.15


def test_spot_edge_detector_settings() -> None:
    s = SettingsV1()
    assert s.spot_focus.detector == "gaussderiv"
    assert s.spot_focus.detector_sigma_px == 6.0

    s.spot_focus.detector = "halfmax"
    s.spot_focus.detector_sigma_px = 8.0
    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    assert (s2.spot_focus.detector, s2.spot_focus.detector_sigma_px) == ("halfmax", 8.0)

    # a settings file from before the detector was selectable loads the defaults
    old = '{"version": "1.0", "spot_focus": {"edge_box": [10, 10, 40, 40]}}'
    s3 = SettingsV1.model_validate_json(old)
    assert (s3.spot_focus.detector, s3.spot_focus.detector_sigma_px) == (
        "gaussderiv",
        6.0,
    )

    for bad in (
        '{"version": "1.0", "spot_focus": {"detector": "sobel"}}',
        '{"version": "1.0", "spot_focus": {"detector_sigma_px": 1.0}}',
        '{"version": "1.0", "spot_focus": {"detector_sigma_px": 20.0}}',
    ):
        with pytest.raises(ValueError):
            SettingsV1.model_validate_json(bad)


def test_spot_calibration_in_settings_keeps_its_detector() -> None:
    from CRISPy.spot.spot_focus import EdgeCalibration, FocusCalibration

    edge = EdgeCalibration(
        box=(232, 210, 80, 60),
        axis="x",
        polarity=1,
        slope_px_per_um=2.0,
        e0_px=272.0,
        p0_um=50.0,
        detector="gaussderiv",
        detector_sigma_px=7.0,
        edge_width_px=12.5,
        box_normal_px=80,
    )
    cal = FocusCalibration(edges=[edge], p0_um=50.0, created="t")
    s = SettingsV1()
    s.spot_focus.calibration = cal.to_dict()
    s2 = SettingsV1.model_validate_json(s.model_dump_json())
    back = s2.spot_focus.to_calibration()
    assert back is not None and back.edges[0] == edge
    assert (back.edges[0].detector, back.edges[0].detector_sigma_px) == (
        "gaussderiv",
        7.0,
    )
    assert back.edges[0].edge_width_px == 12.5 and back.edges[0].box_normal_px == 80

    # a calibration saved before the detector existed loads as halfmax
    old = cal.to_dict()
    for e in old["edges"]:
        for k in ("detector", "detector_sigma_px", "edge_width_px", "box_normal_px"):
            e.pop(k)
    s3 = SettingsV1()
    s3.spot_focus.calibration = old
    loaded = s3.spot_focus.to_calibration()
    assert loaded is not None and loaded.edges[0].detector == "halfmax"
    assert s3.spot_focus.calibration_status()[1] == ""


def test_rig_spot_focus_settings_survive_the_round_trip() -> None:
    """A rig-shaped ``spot_focus`` section loads and dumps back unchanged.

    The section's models live in ``CRISPy.spot.settings``; the rig's saved
    ``pmm_settings.json`` must keep loading with no migration.
    """
    calibration = {
        "kind": "edges",
        "edges": [
            {
                "kind": "edge",
                "box": [232, 210, 80, 60],
                "axis": "x",
                "polarity": 1,
                "slope_px_per_um": 2.0,
                "e0_px": 272.0,
                "p0_um": -50.25,
                "detector": "halfmax",
                "detector_sigma_px": 6.0,
                "edge_width_px": 12.5,
                "box_normal_px": 80,
            }
        ],
        "p0_um": -50.25,
        "created": "2026-10-26T12:00:00",
        "record_path": "spot_calibration_20261026.npz",
        "warnings": ["residual high"],
    }
    spot = {
        "source": {"serial": "TP123", "sdk_dir": "C:/sdk/toupcam"},
        "exposure_ms": 7.5,
        "gain": 300,
        "roi": [100, 120, 640, 480],
        "channel": "sum",
        "led_intensity": 12.0,
        "params": {"kp": 0.4, "hold_band_um": 0.2, "p_min_um": -80.0},
        "calibration": calibration,
        "edge_box": [232, 210, 80, 60],
        "edge_box_2": [400, 300, 60, 80],
        "detector": "halfmax",
        "detector_sigma_px": 8.0,
        "sweep_span_um": 3.0,
        "sweep_steps": 11,
        "sweep_frames": 4,
    }
    s = SettingsV1.model_validate({"version": "1.0", "spot_focus": spot})
    sf = s.spot_focus
    assert sf.source.serial == "TP123"
    assert sf.source.sdk_dir == "C:/sdk/toupcam"
    assert sf.roi == (100, 120, 640, 480)
    assert sf.edge_box == (232, 210, 80, 60)
    assert sf.edge_box_2 == (400, 300, 60, 80)
    assert sf.detector == "halfmax" and sf.detector_sigma_px == 8.0
    assert sf.calibration == calibration
    assert sf.to_calibration() is not None
    assert (sf.params.kp, sf.params.hold_band_um, sf.params.p_min_um) == (
        0.4,
        0.2,
        -80.0,
    )
    dumped = s.model_dump(mode="json", exclude_defaults=True)["spot_focus"]
    assert dumped == spot

    # the legacy 0..150 um piezo range loads as "no range" and is not written back
    legacy = {"spot_focus": {"params": {"p_min_um": 0.0, "p_max_um": 150.0, "kp": 0.5}}}
    s2 = SettingsV1.model_validate(legacy)
    assert (s2.spot_focus.params.p_min_um, s2.spot_focus.params.p_max_um) == (
        None,
        None,
    )
    assert s2.model_dump(mode="json", exclude_defaults=True)["spot_focus"] == {
        "params": {"kp": 0.5}
    }
