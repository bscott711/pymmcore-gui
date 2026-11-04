from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from pymmcore_gui.widgets.camera_alignment._overlay_canvas import OverlayCanvas

if TYPE_CHECKING:
    from pytestqt.qtbot import QtBot


@pytest.fixture
def canvas(qtbot: QtBot) -> OverlayCanvas:
    c = OverlayCanvas()
    qtbot.addWidget(c)
    c.set_layers([("a", "magenta"), ("b", "green"), ("c", "red")])
    return c


def _frame(lo: float = 2, hi: float = 9, shape: tuple[int, int] = (8, 6)) -> np.ndarray:
    data = np.full(shape, lo, dtype=np.uint16)
    data[0, 0] = hi
    return data


def _in_scene(c: OverlayCanvas, obj: object) -> bool:
    return obj in c._scene.children


def test_set_layers_order_and_rebuild(canvas: OverlayCanvas) -> None:
    assert canvas.layer_keys() == ["a", "b", "c"]
    old_nodes = [layer.node for layer in canvas._layers.values()]
    assert all(_in_scene(canvas, n) for n in old_nodes)

    canvas.set_marker("a", (3, 4))
    old_marker = list(canvas._markers["a"])
    assert all(_in_scene(canvas, m) for m in old_marker)

    canvas.set_layers([("x", "red"), ("y", "blue")])
    assert canvas.layer_keys() == ["x", "y"]
    assert not any(_in_scene(canvas, n) for n in old_nodes)
    assert not any(_in_scene(canvas, m) for m in old_marker)
    assert canvas._markers == {}
    assert all(not canvas._layers[k].node.visible for k in "xy")
    assert canvas.is_visible("x")
    assert canvas.is_auto_clim("x")
    assert canvas.layer_data("x") is None


def test_set_frame_unknown_key_ignored(canvas: OverlayCanvas) -> None:
    canvas.set_frame("nope", _frame())
    assert canvas.layer_keys() == ["a", "b", "c"]
    assert all(canvas.layer_data(k) is None for k in "abc")


def test_node_hidden_before_data_visible_after(canvas: OverlayCanvas) -> None:
    assert not canvas._layers["a"].node.visible
    canvas.set_frame("a", _frame())
    assert canvas._layers["a"].node.visible
    assert not canvas._layers["b"].node.visible
    data = canvas.layer_data("a")
    assert data is not None
    assert data.shape == (8, 6)


def test_set_visible_hides_node_and_marker(canvas: OverlayCanvas) -> None:
    canvas.set_frame("a", _frame())
    canvas.set_marker("a", (3, 4))
    assert all(m.visible for m in canvas._markers["a"])

    canvas.set_visible("a", False)
    assert not canvas.is_visible("a")
    assert not canvas._layers["a"].node.visible
    assert not any(m.visible for m in canvas._markers["a"])

    # frames pushed while hidden do not re-show the node
    canvas.set_frame("a", _frame())
    assert not canvas._layers["a"].node.visible

    canvas.set_visible("a", True)
    assert canvas.is_visible("a")
    assert canvas._layers["a"].node.visible
    assert all(m.visible for m in canvas._markers["a"])


def test_set_visible_without_data_keeps_node_hidden(canvas: OverlayCanvas) -> None:
    canvas.set_visible("b", False)
    canvas.set_visible("b", True)
    assert not canvas._layers["b"].node.visible


def test_marker_hidden_when_layer_hidden(canvas: OverlayCanvas) -> None:
    canvas.set_visible("a", False)
    canvas.set_marker("a", (1, 1))
    assert not any(m.visible for m in canvas._markers["a"])


def test_marker_default_color_and_removal(canvas: OverlayCanvas) -> None:
    canvas.set_marker("a", (3, 4))
    magenta = (1.0, 0.0, 1.0, 1.0)
    for line in canvas._markers["a"]:
        assert tuple(line.material.color) == pytest.approx(magenta)
    canvas.set_marker("unknown", (1, 1))  # falls back to white, no layer
    for line in canvas._markers["unknown"]:
        assert tuple(line.material.color) == pytest.approx((1.0, 1.0, 1.0, 1.0))
    canvas.set_marker("a", None)
    assert "a" not in canvas._markers


def test_set_color(canvas: OverlayCanvas) -> None:
    canvas.set_frame("a", _frame())
    canvas.set_marker("a", (3, 4))
    layer = canvas._layers["a"]
    old_map = layer.material.map
    old_last = np.array(old_map.texture.data)[-1].copy()
    old_marker_color = tuple(canvas._markers["a"][0].material.color)

    canvas.set_color("a", "blue")

    assert canvas.color("a") == "blue"
    assert layer.material.map is not old_map
    new_last = np.array(layer.material.map.texture.data)[-1]
    assert not np.allclose(old_last, new_last)
    for line in canvas._markers["a"]:
        assert tuple(line.material.color) != old_marker_color
    # clim and data untouched by a recolour
    assert canvas.clim("a") == (2.0, 9.0)
    assert canvas.layer_data("a") is not None
    assert layer.node.visible


def test_auto_clim_is_min_max(canvas: OverlayCanvas) -> None:
    canvas.set_frame("a", _frame(2, 9))
    assert canvas.is_auto_clim("a")
    assert canvas.clim("a") == (2.0, 9.0)
    canvas.set_frame("a", _frame(5, 100))
    assert canvas.clim("a") == (5.0, 100.0)


def test_auto_clim_flat_frame_guarded(canvas: OverlayCanvas) -> None:
    canvas.set_frame("a", np.full((8, 6), 7, dtype=np.uint16))
    assert canvas.clim("a") == (7.0, 8.0)


def test_manual_clim_fixed_across_frames(canvas: OverlayCanvas) -> None:
    canvas.set_frame("a", _frame(2, 9))
    canvas.set_clim("a", (10, 500))
    assert not canvas.is_auto_clim("a")
    assert canvas.clim("a") == (10.0, 500.0)
    canvas.set_frame("a", _frame(3, 60))
    assert canvas.clim("a") == (10.0, 500.0)


def test_manual_clim_guard(canvas: OverlayCanvas) -> None:
    canvas.set_clim("a", (5, 5))
    assert canvas.clim("a") == (5.0, 6.0)
    canvas.set_clim("a", (5, 1))
    assert canvas.clim("a") == (5.0, 6.0)


def test_back_to_auto_reapplies_min_max(canvas: OverlayCanvas) -> None:
    canvas.set_frame("a", _frame(2, 9))
    canvas.set_clim("a", (10, 500))
    canvas.set_clim("a", None)
    assert canvas.is_auto_clim("a")
    assert canvas.clim("a") == (2.0, 9.0)
    canvas.set_frame("a", _frame(4, 40))
    assert canvas.clim("a") == (4.0, 40.0)


def test_set_clim_auto_without_data_is_safe(canvas: OverlayCanvas) -> None:
    canvas.set_clim("b", None)
    assert canvas.is_auto_clim("b")


def test_reset_view_no_data_or_hidden_first_layer(canvas: OverlayCanvas) -> None:
    canvas.reset_view()  # no data anywhere
    canvas.set_frame("a", _frame())
    canvas.set_visible("a", False)
    canvas.reset_view()  # only layer with data is hidden
    canvas.set_frame("b", _frame())
    canvas.reset_view()  # falls through to "b"
    empty = OverlayCanvas()
    empty.reset_view()  # no layers at all
    empty.deleteLater()


def test_pick_replaces_pending_handler(canvas: OverlayCanvas) -> None:
    calls: list[tuple[float, float]] = []
    canvas.begin_point_pick(calls.append)
    first = canvas._pick_handler
    assert first is not None
    assert not canvas._controller.enabled
    canvas.begin_point_pick(calls.append)
    assert canvas._pick_handler is not first
    canvas._detach_pick_handler()
    assert canvas._pick_handler is None
    assert canvas._controller.enabled
