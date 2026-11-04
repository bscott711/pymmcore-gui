"""Multi-layer composite pygfx canvas for manual optical alignment.

Renders N keyed frames (e.g. the 4 spectral regions: 2 cameras x top/bottom
crops) as independently-tinted, additively-blended ``pygfx.Image`` nodes at
the same (unshifted) world position -- i.e. there is exactly one pixel<->world
mapping, shared by every layer: a single click's world ``(x, y)`` converts to
pixel coordinates the same way regardless of which layer it's meant to seed.

This deliberately does not subclass
:class:`~pymmcore_gui.widgets.image_preview._pygfx_image.PygfxImagePreview`:
that class is built around exactly one texture/image node, while this widget
needs several independent textures composited in one scene. The renderer/
camera/controller boilerplate and the pixel<->world coordinate conversion are
mirrored from it instead (see ``_screen_to_world_xy`` there).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import pygfx
import pylinalg as la
from cmap import Colormap

from pymmcore_gui._qt.QtCore import QObject, QSize
from pymmcore_gui._qt.QtWidgets import QVBoxLayout, QWidget

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    import rendercanvas.qt

    class QRenderWidget(rendercanvas.qt.QRenderWidget, QWidget): ...  # pyright: ignore [reportIncompatibleMethodOverride]

else:
    from rendercanvas.qt import QRenderWidget

# Additive blend override: overlapping tinted regions ADD (e.g. magenta +
# green -> near-white) instead of alpha-compositing (which would just show
# whichever layer is drawn on top) -- this is what makes overlap visually
# diagnostic for alignment. Confirmed against the installed pygfx version:
# `alpha_config`'s setter requires a "method" key alongside "mode".
_ADDITIVE_ALPHA_CONFIG = {
    "mode": "custom",
    "method": "blended",
    "color_op": "add",
    "color_src": "one",
    "color_dst": "one",
    "alpha_op": "add",
    "alpha_src": "one",
    "alpha_dst": "one",
}


class _OverlayLayer:
    """One keyed texture + tinted, additively-blended material + image node."""

    def __init__(self, key: str, color: str) -> None:
        self.key = key
        self.color = color
        self.user_visible = True
        self.auto_clim = True
        self.has_data = False
        self.texture = pygfx.Texture(dim=2, size=(1, 1), format="1xf4")
        self.geometry = pygfx.Geometry(grid=self.texture)
        self.material = pygfx.ImageBasicMaterial(
            clim=(0, 1), map=Colormap(["black", color]).to_pygfx(), depth_test=False
        )
        self.material.alpha_config = _ADDITIVE_ALPHA_CONFIG
        self.node = pygfx.Image(self.geometry, self.material, visible=False)

    def _update_node_visible(self) -> None:
        self.node.visible = self.user_visible and self.has_data

    def set_data(self, data: np.ndarray) -> bool:
        """Push new frame data; returns True if the texture had to be recreated."""
        if self.auto_clim:
            self.apply_auto_clim(data)
        recreated = False
        try:
            self.texture.set_data(data)
        except (ValueError, AttributeError):
            # texture has wrong shape or format, recreate it
            self.texture = pygfx.Texture(data, dim=2)
            self.geometry.grid = self.texture
            recreated = True
        self.has_data = True
        self._update_node_visible()
        return recreated

    def apply_auto_clim(self, data: np.ndarray) -> None:
        lo, hi = float(np.min(data)), float(np.max(data))
        self.material.clim = (lo, hi if hi > lo else lo + 1.0)

    def set_user_visible(self, visible: bool) -> None:
        self.user_visible = visible
        self._update_node_visible()

    def set_color(self, color: str) -> None:
        self.color = color
        # ImageBasicMaterial.map has a setter (it wraps a Texture in a
        # TextureMap), so the colour map is swapped in place; no need to
        # rebuild the material.
        self.material.map = Colormap(["black", color]).to_pygfx()

    @property
    def data(self) -> np.ndarray | None:
        if not self.has_data:
            return None
        return self.texture.data  # type: ignore [no-any-return]


class OverlayCanvas(QWidget):
    """Superimposed live view of N keyed layers, tinted for visual alignment.

    Layers are additively blended at the same (unshifted) world position and
    configured with :meth:`set_layers`; each has its own visibility, colour,
    contrast limits and (optional) point marker.

    Parameters
    ----------
    parent : QWidget | None
        Optional parent widget. By default, None.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._layers: dict[str, _OverlayLayer] = {}
        self._markers: dict[str, list[pygfx.WorldObject]] = {}
        self._pick_handler: Callable[[pygfx.PointerEvent], None] | None = None

        self._scene = pygfx.Scene()
        top = np.array((50, 50, 50, 255)) / 255
        bot = np.array((30, 30, 30, 255)) / 255
        self._scene.add(pygfx.Background(None, pygfx.BackgroundMaterial(bot, top)))

        self._canvas = QRenderWidget()
        self._renderer = pygfx.WgpuRenderer(self._canvas)
        self._camera = pygfx.OrthographicCamera()
        # Flip the y-axis so row 0 of the image array is at the top, matching
        # PygfxImagePreview's convention.
        self._camera.local.scale_y = -1
        self._scene.add(self._camera)
        self._controller = pygfx.PanZoomController(
            self._camera, register_events=self._renderer, damping=2
        )
        self._canvas.request_draw(self._draw_function)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._canvas)

        if isinstance(parent, QObject):
            parent.destroyed.connect(self._detach_pick_handler)

    def sizeHint(self) -> QSize:
        return self._canvas.sizeHint()

    # ------------------------------ layers -------------------------------

    def set_layers(self, layers: Sequence[tuple[str, str]]) -> None:
        """Replace all layers with ordered ``(key, color)`` pairs.

        Removes every existing layer node and every marker from the scene,
        then adds the new layers in order. New layers have no data (node
        hidden), are user-visible and use auto contrast.

        Parameters
        ----------
        layers : Sequence[tuple[str, str]]
            ``(key, color)`` pairs; *color* is a ``cmap``-compatible color
            name for the layer's black->color tint LUT.
        """
        for layer in self._layers.values():
            self._scene.remove(layer.node)
        for objs in self._markers.values():
            for obj in objs:
                self._scene.remove(obj)
        self._markers.clear()
        self._layers = {key: _OverlayLayer(key, color) for key, color in layers}
        for layer in self._layers.values():
            self._scene.add(layer.node)
        self._canvas.request_draw(self._draw_function)

    def layer_keys(self) -> list[str]:
        """Return the layer keys in order."""
        return list(self._layers)

    def set_frame(self, key: str, data: np.ndarray) -> None:
        """Push a 2-D frame to layer *key*; unknown keys are ignored.

        Parameters
        ----------
        key : str
            Layer key.
        data : np.ndarray
            2-D frame. With auto contrast the layer's clim becomes the
            frame's (min, max).
        """
        layer = self._layers.get(key)
        if layer is None:
            return
        if layer.set_data(data):
            self.reset_view()
        self._canvas.request_draw(self._draw_function)

    def set_visible(self, key: str, visible: bool) -> None:
        """Show/hide layer *key* (and its marker); unknown keys are ignored.

        Parameters
        ----------
        key : str
            Layer key.
        visible : bool
            User visibility. The image node is only shown if the layer also
            has data.
        """
        layer = self._layers.get(key)
        if layer is None:
            return
        layer.set_user_visible(visible)
        for obj in self._markers.get(key, []):
            obj.visible = visible
        self._canvas.request_draw(self._draw_function)

    def is_visible(self, key: str) -> bool:
        """Return the user-visibility flag of layer *key* (KeyError if unknown)."""
        return self._layers[key].user_visible

    def set_color(self, key: str, color: str) -> None:
        """Recolour layer *key* and its marker; unknown keys are ignored.

        Parameters
        ----------
        key : str
            Layer key.
        color : str
            ``cmap``-compatible color name.
        """
        layer = self._layers.get(key)
        if layer is None:
            return
        layer.set_color(color)
        for obj in self._markers.get(key, []):
            obj.material.color = color
        self._canvas.request_draw(self._draw_function)

    def color(self, key: str) -> str:
        """Return the color name of layer *key* (KeyError if unknown)."""
        return self._layers[key].color

    def set_clim(self, key: str, clim: tuple[float, float] | None) -> None:
        """Set contrast limits for layer *key*; unknown keys are ignored.

        Parameters
        ----------
        key : str
            Layer key.
        clim : tuple[float, float] | None
            ``None`` selects auto contrast (per-frame min/max, re-applied to
            the current data immediately). ``(lo, hi)`` fixes the limits
            (``hi <= lo`` is bumped to ``lo + 1``).
        """
        layer = self._layers.get(key)
        if layer is None:
            return
        if clim is None:
            layer.auto_clim = True
            data = layer.data
            if data is not None:
                layer.apply_auto_clim(data)
        else:
            layer.auto_clim = False
            lo, hi = float(clim[0]), float(clim[1])
            layer.material.clim = (lo, hi if hi > lo else lo + 1.0)
        self._canvas.request_draw(self._draw_function)

    def clim(self, key: str) -> tuple[float, float]:
        """Return the contrast limits currently applied to layer *key*."""
        lo, hi = self._layers[key].material.clim
        return float(lo), float(hi)

    def is_auto_clim(self, key: str) -> bool:
        """Return True if layer *key* uses auto contrast (KeyError if unknown)."""
        return self._layers[key].auto_clim

    def layer_data(self, key: str) -> np.ndarray | None:
        """Return layer *key*'s texture data, or None if it has no data yet."""
        return self._layers[key].data

    def reset_view(self) -> None:
        """Frame the view on the first user-visible layer that has data."""
        for layer in self._layers.values():
            if layer.user_visible and layer.has_data:
                self._camera.show_object(layer.node)  # pyright: ignore [reportArgumentType]
                return

    # ------------------------------ picking ------------------------------

    def _screen_to_world_xy(self, x: float, y: float) -> tuple[float, float]:
        """Convert a canvas-space pointer position to world (x, y).

        Same math as ``PygfxImagePreview._screen_to_world_xy``.
        """
        w, h = self._renderer.logical_size
        ndc_x = 2.0 * x / w - 1.0
        ndc_y = 1.0 - 2.0 * y / h  # canvas y is down; NDC y is up
        cam = self._camera
        view_pos = la.vec_transform((ndc_x, ndc_y, 0.0), cam.projection_matrix_inverse)
        world_pos = la.vec_transform(view_pos, cam.world.matrix)
        return float(world_pos[0]), float(world_pos[1])

    @staticmethod
    def world_to_pixel(world_xy: tuple[float, float]) -> tuple[float, float]:
        """Convert a world (x, y) to pixel (x, y).

        Inverse of the pixel -> world ``-0.5`` shift used by every layer
        (none is offset relative to another, so one conversion serves all).
        """
        return world_xy[0] + 0.5, world_xy[1] + 0.5

    def begin_point_pick(self, on_done: Callable[[tuple[float, float]], None]) -> None:
        """Start a single-click pick; calls ``on_done(world_xy)`` on the next click.

        Simpler than ``PygfxImagePreview.begin_roi_draw``/``begin_roi_move``
        (no drag): suspends pan/zoom, listens for exactly one
        ``pointer_down``, converts to world coordinates, restores pan/zoom,
        and calls *on_done*. No clamping to a single layer's frame bounds
        happens here -- that's the caller's job once it knows which
        layer's array the resulting pixel coordinate is meant to index.
        A pick that is already pending is replaced (its handler removed).

        Parameters
        ----------
        on_done : Callable[[tuple[float, float]], None]
            Called with the clicked world ``(x, y)``.
        """
        self._remove_pick_handler()
        self._controller.enabled = False

        def _on_pointer(event: pygfx.PointerEvent) -> None:
            self._remove_pick_handler()
            self._controller.enabled = True
            on_done(self._screen_to_world_xy(event.x, event.y))

        self._pick_handler = _on_pointer
        self._renderer.add_event_handler(_on_pointer, "pointer_down")

    def _remove_pick_handler(self) -> None:
        if self._pick_handler is not None:
            self._renderer.remove_event_handler(self._pick_handler, "pointer_down")
            self._pick_handler = None

    def _detach_pick_handler(self, _obj: Any = None) -> None:
        # Best-effort: re-enable panning if the widget is torn down mid-pick.
        self._remove_pick_handler()
        self._controller.enabled = True

    # ------------------------------ markers -------------------------------

    def set_marker(
        self, key: str, xy: tuple[float, float] | None, color: str | None = None
    ) -> None:
        """Draw/move/remove a small crosshair at pixel ``xy`` for layer *key*.

        The marker is hidden while the layer is user-hidden.

        Parameters
        ----------
        key : str
            Layer key this marker belongs to, so each layer's marker can be
            independently updated every frame without touching the others.
        xy : tuple[float, float] | None
            Pixel coordinates to draw the crosshair at, or ``None`` to
            remove the marker (e.g. tracking lost / cleared).
        color : str | None
            Color of the crosshair lines. ``None`` uses the layer's color
            (``"white"`` if there is no such layer).
        """
        for obj in self._markers.pop(key, []):
            self._scene.remove(obj)
        if xy is not None:
            layer = self._layers.get(key)
            if color is None:
                color = layer.color if layer is not None else "white"
            visible = layer.user_visible if layer is not None else True
            wx, wy = xy[0] - 0.5, xy[1] - 0.5
            half = 6.0
            h_line = pygfx.Line(
                pygfx.Geometry(
                    positions=np.array(
                        [[wx - half, wy, 0], [wx + half, wy, 0]], dtype=np.float32
                    )
                ),
                pygfx.LineMaterial(thickness=2.0, color=color, depth_test=False),
                visible=visible,
            )
            v_line = pygfx.Line(
                pygfx.Geometry(
                    positions=np.array(
                        [[wx, wy - half, 0], [wx, wy + half, 0]], dtype=np.float32
                    )
                ),
                pygfx.LineMaterial(thickness=2.0, color=color, depth_test=False),
                visible=visible,
            )
            self._scene.add(h_line, v_line)
            self._markers[key] = [h_line, v_line]
        self._canvas.request_draw(self._draw_function)

    # ----------------------------

    def _draw_function(self) -> None:
        self._renderer.render(self._scene, self._camera)
        self._renderer.request_draw()
