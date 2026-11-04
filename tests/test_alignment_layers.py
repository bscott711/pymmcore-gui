from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from pymmcore_gui._settings import SpectralChannelConfig
from pymmcore_gui.widgets.camera_alignment._layer_controls import (
    LayerControlsPanel,
    LayerControlsRow,
)
from pymmcore_gui.widgets.camera_alignment._layers import (
    PALETTE,
    LayerDisplay,
    LayerSpec,
    camera_layer_specs,
    crop_layer_frame,
    default_color_for,
    default_reference_key,
    region_layer_specs,
    spectral_signature,
)

if TYPE_CHECKING:
    import pytest
    from pytestqt.qtbot import QtBot

GREEN = PALETTE["Green"]
MAGENTA = PALETTE["Magenta"]
RED = PALETTE["Red"]
CYAN = PALETTE["Cyan"]


def _channels() -> list[SpectralChannelConfig]:
    return [
        SpectralChannelConfig(
            name="g488",
            camera="Camera",
            laser_preset="488nm",
            position="top",
            rect=(0, 0, 100, 50),
        ),
        SpectralChannelConfig(
            name="c405",
            camera="Camera",
            laser_preset="405nm",
            position="bottom",
            rect=(0, 60, 100, 50),
        ),
        SpectralChannelConfig(
            name="m561",
            camera="Camera-2",
            laser_preset="561nm",
            position="top",
            rect=(0, 0, 100, 50),
        ),
        SpectralChannelConfig(
            name="r638",
            camera="Camera-2",
            laser_preset="638nm",
            position="bottom",
            rect=(0, 60, 100, 50),
        ),
    ]


# ------------------------------------------------------------------ _layers


def test_region_layer_specs_default_channels() -> None:
    specs = region_layer_specs(_channels(), ["Camera", "Camera-2"])
    assert [s.key for s in specs] == [
        "region:g488",
        "region:c405",
        "region:m561",
        "region:r638",
    ]
    assert [s.default_color for s in specs] == [GREEN, CYAN, MAGENTA, RED]
    assert specs[0].camera == "Camera"
    assert specs[0].rect == (0, 0, 100, 50)
    assert specs[2].laser_preset == "561nm"


def test_region_layer_specs_skips_no_rect_and_absent_camera() -> None:
    chans = _channels()
    chans[1] = chans[1].model_copy(update={"rect": None})
    specs = region_layer_specs(chans, ["Camera"])
    assert [s.name for s in specs] == ["g488"]


def test_camera_layer_specs_colors() -> None:
    specs = camera_layer_specs(["A", "B", "C", "D"])
    assert [s.key for s in specs] == ["camera:A", "camera:B", "camera:C", "camera:D"]
    assert all(s.rect is None for s in specs)
    assert specs[0].default_color == MAGENTA
    assert specs[1].default_color == GREEN
    # then the first unused palette colours
    assert specs[2].default_color == RED
    assert specs[3].default_color == CYAN


def test_default_color_for() -> None:
    assert default_color_for("488nm") == GREEN
    assert default_color_for("405") == CYAN
    assert default_color_for("Laser 561") == MAGENTA
    assert default_color_for("640nm") == RED
    # duplicate heuristic colour -> first unused palette colour
    assert default_color_for("488nm", {GREEN}) == MAGENTA
    # unknown / unparsable -> first unused palette colour
    assert default_color_for("mystery") == GREEN
    assert default_color_for("mystery", {GREEN, MAGENTA}) == RED
    # palette exhausted -> gray
    assert default_color_for("", set(PALETTE.values())) == PALETTE["Gray"]


def test_default_reference_key() -> None:
    specs = region_layer_specs(_channels(), ["Camera", "Camera-2"])
    assert default_reference_key(specs) == "region:g488"
    assert default_reference_key(specs, hidden={"region:g488"}) == "region:c405"
    assert default_reference_key(specs[1:]) == "region:c405"
    assert default_reference_key([]) is None
    assert default_reference_key(specs, hidden={s.key for s in specs}) is None


def test_crop_layer_frame_origin_offset() -> None:
    frame = np.arange(20 * 30, dtype=np.uint16).reshape(20, 30)
    spec = LayerSpec(key="region:x", name="x", camera="c", rect=(105, 52, 10, 5))
    out = crop_layer_frame(frame, spec, origin=(100, 50))
    assert out is not None
    np.testing.assert_array_equal(out, frame[2:7, 5:15])


def test_crop_layer_frame_out_of_frame_and_whole() -> None:
    frame = np.zeros((20, 30), dtype=np.uint16)
    spec = LayerSpec(key="region:x", name="x", camera="c", rect=(25, 0, 10, 5))
    assert crop_layer_frame(frame, spec, origin=(0, 0)) is None
    whole = LayerSpec(key="camera:c", name="c", camera="c")
    assert crop_layer_frame(frame, whole, origin=(7, 7)) is frame


def test_spectral_signature_changes_with_rect() -> None:
    chans = _channels()
    before = spectral_signature(chans)
    assert hash(before) == hash(spectral_signature(_channels()))
    assert before == spectral_signature(_channels())
    chans[0] = chans[0].model_copy(update={"rect": (1, 0, 100, 50)})
    assert spectral_signature(chans) != before


# --------------------------------------------------------------------- Qt


def _spec(key: str = "region:g488") -> LayerSpec:
    return LayerSpec(key=key, name=key.split(":")[1], camera="Camera")


def _row(qtbot: QtBot, display: LayerDisplay | None = None) -> LayerControlsRow:
    row = LayerControlsRow(_spec(), display or LayerDisplay(color=GREEN))
    qtbot.addWidget(row)
    return row


def test_row_initial_state_emits_nothing(qtbot: QtBot) -> None:
    row = _row(qtbot, LayerDisplay(color=CYAN, visible=False, auto_contrast=False))
    assert not row._visible_cb.isChecked()
    assert not row._auto_cb.isChecked()
    assert row._min_spin.isEnabled()
    assert row._color_combo.currentData() == CYAN
    assert row.key == "region:g488"


def test_row_visibility(qtbot: QtBot) -> None:
    row = _row(qtbot)
    with qtbot.waitSignal(row.visibilityChanged) as blocker:
        row._visible_cb.setChecked(False)
    assert blocker.args == ["region:g488", False]


def test_row_palette_color(qtbot: QtBot) -> None:
    row = _row(qtbot)
    idx = row._color_combo.findData(RED)
    with qtbot.waitSignal(row.colorChanged) as blocker:
        row._color_combo.setCurrentIndex(idx)
    assert blocker.args == ["region:g488", RED]


def test_row_custom_color_and_cancel(
    qtbot: QtBot, monkeypatch: pytest.MonkeyPatch
) -> None:
    row = _row(qtbot)
    custom_idx = row._color_combo.count() - 1

    monkeypatch.setattr(row, "_ask_custom_color", lambda initial: "#123456")
    with qtbot.waitSignal(row.colorChanged) as blocker:
        row._color_combo.setCurrentIndex(custom_idx)
    assert blocker.args == ["region:g488", "#123456"]
    assert row._color_combo.currentData() == "#123456"
    assert "#123456" in row._color_combo.currentText()

    # cancel -> revert to the previous colour, no signal
    monkeypatch.setattr(row, "_ask_custom_color", lambda initial: None)
    emitted: list[tuple[str, str]] = []
    row.colorChanged.connect(lambda k, c: emitted.append((k, c)))
    row._color_combo.setCurrentIndex(row._color_combo.count() - 1)
    assert emitted == []
    assert row._color_combo.currentData() == "#123456"


def test_row_set_color_is_silent(qtbot: QtBot) -> None:
    row = _row(qtbot)
    emitted: list[object] = []
    row.colorChanged.connect(lambda k, c: emitted.append(c))
    row.set_color(MAGENTA)
    row.set_color("#abcdef")
    assert emitted == []
    assert row._color_combo.currentData() == "#abcdef"


def test_row_auto_toggle_and_spins(qtbot: QtBot) -> None:
    row = _row(qtbot)
    assert not row._min_spin.isEnabled()
    row.set_auto_range(100.0, 2000.0)
    assert (row._min_spin.value(), row._max_spin.value()) == (100, 2000)

    with qtbot.waitSignal(row.climChanged) as blocker:
        row._auto_cb.setChecked(False)
    assert blocker.args == ["region:g488", (100.0, 2000.0)]
    assert row._min_spin.isEnabled()

    with qtbot.waitSignal(row.climChanged) as blocker:
        row._max_spin.setValue(3000)
    assert blocker.args == ["region:g488", (100.0, 3000.0)]

    # max <= min is clamped to min + 1
    with qtbot.waitSignal(row.climChanged) as blocker:
        row._min_spin.setValue(4000)
    assert blocker.args == ["region:g488", (4000.0, 4001.0)]
    assert row._max_spin.value() == 4001

    with qtbot.waitSignal(row.climChanged) as blocker:
        row._auto_cb.setChecked(True)
    assert blocker.args == ["region:g488", None]
    assert not row._min_spin.isEnabled()


def test_row_set_auto_range_emits_nothing(qtbot: QtBot) -> None:
    row = _row(qtbot)
    emitted: list[object] = []
    row.climChanged.connect(lambda k, v: emitted.append(v))
    row.set_auto_range(5, 500)
    assert emitted == []
    assert (row._min_spin.value(), row._max_spin.value()) == (5, 500)


def test_row_pick_and_status(qtbot: QtBot) -> None:
    row = _row(qtbot)
    with qtbot.waitSignal(row.pickRequested) as blocker:
        row._pick_btn.click()
    assert blocker.args == ["region:g488"]

    row.set_pick_enabled(False)
    assert not row._pick_btn.isEnabled()
    row.set_status("outside frame")
    assert row._status_label.text() == "outside frame"
    assert row._status_label.toolTip() == "outside frame"
    row.set_status("")
    assert row._status_label.text() == ""


def test_panel_set_layers_and_forwarding(qtbot: QtBot) -> None:
    panel = LayerControlsPanel()
    qtbot.addWidget(panel)
    specs = [_spec("region:a"), _spec("region:b")]
    displays = {s.key: LayerDisplay(color=GREEN) for s in specs}
    panel.set_layers(specs, displays)
    assert [r.key for r in panel.rows()] == ["region:a", "region:b"]
    assert panel.row("region:b") is panel.rows()[1]
    assert panel.row("nope") is None

    row_b = panel.row("region:b")
    assert row_b is not None
    with qtbot.waitSignal(panel.visibilityChanged) as blocker:
        row_b._visible_cb.setChecked(False)
    assert blocker.args == ["region:b", False]
    with qtbot.waitSignal(panel.colorChanged) as blocker:
        row_b._color_combo.setCurrentIndex(row_b._color_combo.findData(RED))
    assert blocker.args == ["region:b", RED]
    with qtbot.waitSignal(panel.climChanged) as blocker:
        row_b._auto_cb.setChecked(False)
    assert blocker.args[0] == "region:b"
    with qtbot.waitSignal(panel.pickRequested) as blocker:
        row_b._pick_btn.click()
    assert blocker.args == ["region:b"]

    # rebuild replaces the rows
    panel.set_layers([_spec("region:c")], {"region:c": LayerDisplay(color=RED)})
    assert [r.key for r in panel.rows()] == ["region:c"]
    assert panel.row("region:a") is None
