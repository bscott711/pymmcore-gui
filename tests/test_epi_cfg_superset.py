"""20261118-OPM_EPI.cfg must stay a pure superset of 20260630-OPM.cfg.

Micro-Manager config rules used here (MMCore ``loadSystemConfigurationImpl``):
a line starting with ``#`` is a comment, other non-empty lines are
comma-separated commands. ``Device`` takes exactly 4 tokens, ``Parent`` and
``Property`` 3-4, ``ConfigGroup`` 6 (or 5/2).
"""

from __future__ import annotations

from pathlib import Path

import pytest

PROFILES = Path(__file__).parent.parent / "hardware_profiles"
OPM_CFG = PROFILES / "20260630-OPM.cfg"
EPI_CFG = PROFILES / "20261118-OPM_EPI.cfg"

# Marker the added comment block carries on every line.
_ADDED_COMMENT = "# EPI "


def _lines(path: Path) -> list[str]:
    return path.read_text(encoding="ascii").splitlines()


def _is_added_led_line(line: str) -> bool:
    tokens = line.split(",")
    if tokens[0] in ("Device", "Parent"):
        return tokens[1].startswith("LED:")
    return tokens[0] == "ConfigGroup" and tokens[1] == "EPI LEDs"


def _added(epi: list[str], opm: list[str]) -> list[str]:
    """Lines of the EPI cfg that are not part of the OPM cfg (in order)."""
    return [
        line
        for line in epi
        if line.startswith(_ADDED_COMMENT)
        or (line and not line.startswith("#") and _is_added_led_line(line))
    ]


def test_epi_cfg_minus_additions_equals_opm_cfg_line_for_line() -> None:
    opm = _lines(OPM_CFG)
    epi = _lines(EPI_CFG)
    added = set(_added(epi, opm))
    rest = [line for line in epi if line not in added or line in opm]
    assert rest == opm


def test_original_cfg_has_no_led_lines() -> None:
    # the OPM cfg stays untouched: no LED device, no EPI group
    for line in _lines(OPM_CFG):
        assert not line.startswith(_ADDED_COMMENT)
        assert "LED:" not in line
        assert "EPI LEDs" not in line


def test_added_lines_are_only_comments_led_devices_parents_and_epi_group() -> None:
    opm = _lines(OPM_CFG)
    epi = _lines(EPI_CFG)
    opm_set = set(opm)
    extra = [line for line in epi if line not in opm_set]
    assert extra  # something was added
    assert [line for line in extra if line in _added(epi, opm)] == extra


def test_added_lines_name_only_address_37() -> None:
    epi = _lines(EPI_CFG)
    opm = set(_lines(OPM_CFG))
    led_lines = [
        line
        for line in epi
        if line not in opm and not line.startswith("#") and "LED:" in line
    ]
    assert led_lines
    for line in led_lines:
        tokens = line.split(",")
        labels = [t for t in tokens if t.startswith("LED:")]
        assert labels, line
        for label in labels:
            parts = label.split(":")
            assert len(parts) == 4, line
            assert parts[2] == "37", line
    assert not any("LED:X:31" in line for line in epi)


def test_added_lines_do_not_touch_core_system_or_startup() -> None:
    epi = _lines(EPI_CFG)
    opm = set(_lines(OPM_CFG))
    for line in epi:
        if line in opm or line.startswith("#"):
            continue
        tokens = line.split(",")
        assert "Core" not in tokens, line
        assert "System" not in tokens, line
        assert "Startup" not in tokens, line
        assert tokens[0] != "Property", line  # no property writes at all


def test_epi_group_has_the_three_presets_and_no_405() -> None:
    group = [
        line.split(",")
        for line in _lines(EPI_CFG)
        if line.startswith("ConfigGroup,EPI")
    ]
    assert group == [
        ["ConfigGroup", "EPI LEDs", "LED488", "LED:L:37:1", "State", "0"],
        ["ConfigGroup", "EPI LEDs", "LED561", "LED:L:37:2", "State", "0"],
        ["ConfigGroup", "EPI LEDs", "LED642", "LED:L:37:3", "State", "0"],
    ]


def test_all_four_led_channels_are_loaded_with_a_hub_parent() -> None:
    epi = _lines(EPI_CFG)
    for n in range(1, 5):
        assert f"Device,LED:L:37:{n},ASITiger,LED:L:37:{n}" in epi
        assert f"Parent,LED:L:37:{n},TigerCommHub" in epi


def test_comment_block_records_the_placeholders() -> None:
    comments = "\n".join(
        line for line in _lines(EPI_CFG) if line.startswith(_ADDED_COMMENT)
    )
    assert "Confirmed channel map" in comments
    assert "1/X = 488, 2/Y = 561, 3/Z = 642, 4/F = 405" in comments
    assert "PLACEHOLDER (O2)" in comments
    assert "20260630-OPM.cfg" in comments


# Expected token counts per MMCore's parser, for every line in both files.
_TOKEN_COUNTS: dict[str, set[int]] = {
    "Device": {4},
    "Property": {3, 4},
    "Parent": {3},
    "Delay": {3},
    "FocusDirection": {3},
    "Label": {4},
    "ConfigGroup": {2, 5, 6},
    "ImageSynchro": {3},
    "PixelSize_um": {3},
    "PixelSizeAffine": {8},
    "PixelSizeConfig": {3},
    "Equipment": {2},
}


@pytest.mark.parametrize("path", [OPM_CFG, EPI_CFG], ids=lambda p: p.name)
def test_every_line_parses_like_mmcore(path: Path) -> None:
    for line in _lines(path):
        if not line or line.startswith("#"):
            continue
        tokens = line.split(",")
        assert tokens[0] in _TOKEN_COUNTS, line
        assert len(tokens) in _TOKEN_COUNTS[tokens[0]], line
