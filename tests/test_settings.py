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
    # A single unparseable offset drops only `offsets` (back to {}), leaving
    # the rest of the `focus` section intact -- not the whole section.
    data = {"focus": {"enabled": True, "offsets": {"561nm": "banana"}}}
    with pytest.warns(RuntimeWarning, match="Could not validate key 'offsets'"):
        cleaned = _settings._good_data_only(SettingsV1, data)
    assert cleaned == {"focus": {"enabled": True}}
