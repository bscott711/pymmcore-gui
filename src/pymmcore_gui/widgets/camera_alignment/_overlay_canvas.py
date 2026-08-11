"""Two-camera composite pygfx canvas for manual optical alignment.

Renders both cameras' frames as two independently-tinted, additively-blended
``pygfx.Image`` nodes at the same (unshifted) world position -- i.e. there is
exactly one pixel<->world mapping, shared by both cameras: a single click's
world ``(x, y)`` converts to pixel coordinates the same way regardless of
which camera it's meant to seed.

This deliberately does not subclass
:class:`~pymmcore_gui.widgets.image_preview._pygfx_image.PygfxImagePreview`:
that class is built around exactly one texture/image node, while this widget
needs two independent textures composited in one scene. The renderer/camera/
controller boilerplate and the pixel<->world coordinate conversion are
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
    from collections.abc import Callable

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


class _CameraLayer:
    """One camera's texture + tinted, additively-blended material + image node."""

    def __init__(self, color: str) -> None:
        self.color = color
        self.texture = pygfx.Texture(dim=2, size=(1, 1), format="1xf4")
        self.geometry = pygfx.Geometry(grid=self.texture)
        cmap = Colormap(["black", color])
        self.material = pygfx.ImageBasicMaterial(
            clim=(0, 1), map=cmap.to_pygfx(), depth_test=False
        )
        self.material.alpha_config = _ADDITIVE_ALPHA_CONFIG
        self.node = pygfx.Image(self.geometry, self.material, visible=False)

    def set_data(self, data: np.ndarray) -> bool:
        """Push new frame data; returns True if the texture had to be recreated."""
        self.material.clim = float(np.min(data)), float(np.max(data))
        recreated = False
        try:
            self.texture.set_data(data)
        except (ValueError, AttributeError):
            # texture has wrong shape or format, recreate it
            self.texture = pygfx.Texture(data, dim=2)
            self.geometry.grid = self.texture
            recreated = True
        self.node.visible = True
        return recreated

    @property
    def data(self) -> np.ndarray | None:
        return self.texture.data  # type: ignore [no-any-return]


class OverlayCanvas(QWidget):
    """Superimposed live view of two cameras, tinted for visual alignment.

    Parameters
    ----------
    parent : QWidget | None
        Optional parent widget. By default, None.
    color1 : str
        ``cmap``-compatible color name for camera 1's black->color tint LUT.
    color2 : str
        ``cmap``-compatible color name for camera 2's black->color tint LUT.
        Default magenta/green: overlap reads near-white, misalignment reads
        as two separated colored blobs.
    """

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        color1: str = "magenta",
        color2: str = "green",
    ) -> None:
        super().__init__(parent)
        self._layer1 = _CameraLayer(color1)
        self._layer2 = _CameraLayer(color2)
        self._markers: dict[str, list[pygfx.WorldObject]] = {}

        self._scene = pygfx.Scene()
        top = np.array((50, 50, 50, 255)) / 255
        bot = np.array((30, 30, 30, 255)) / 255
        self._scene.add(pygfx.Background(None, pygfx.BackgroundMaterial(bot, top)))
        self._scene.add(self._layer1.node, self._layer2.node)

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

    def set_frame1(self, data: np.ndarray) -> None:
        """Push a new frame for camera 1 (the ``color1``-tinted layer)."""
        if self._layer1.set_data(data):
            self.reset_view()
        self._canvas.request_draw(self._draw_function)

    def set_frame2(self, data: np.ndarray) -> None:
        """Push a new frame for camera 2 (the ``color2``-tinted layer)."""
        if self._layer2.set_data(data):
            self.reset_view()
        self._canvas.request_draw(self._draw_function)

    def reset_view(self) -> None:
        """Frame the view on whichever camera layer currently has data."""
        target = self._layer1.node if self._layer1.node.visible else self._layer2.node
        if target.visible:
            self._camera.show_object(target)  # pyright: ignore [reportArgumentType]

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

        Inverse of the pixel -> world ``-0.5`` shift used by both camera
        layers (neither is offset relative to the other, so one conversion
        serves both).
        """
        return world_xy[0] + 0.5, world_xy[1] + 0.5

    def begin_point_pick(self, on_done: Callable[[tuple[float, float]], None]) -> None:
        """Start a single-click pick; calls ``on_done(world_xy)`` on the next click.

        Simpler than ``PygfxImagePreview.begin_roi_draw``/``begin_roi_move``
        (no drag): suspends pan/zoom, listens for exactly one
        ``pointer_down``, converts to world coordinates, restores pan/zoom,
        and calls *on_done*. No clamping to a single camera's frame bounds
        happens here -- that's the caller's job once it knows which
        camera's array the resulting pixel coordinate is meant to index.
        """
        self._controller.enabled = False

        def _on_pointer(event: pygfx.PointerEvent) -> None:
            self._renderer.remove_event_handler(_on_pointer, "pointer_down")
            self._controller.enabled = True
            on_done(self._screen_to_world_xy(event.x, event.y))

        self._renderer.add_event_handler(_on_pointer, "pointer_down")

    def _detach_pick_handler(self, _obj: Any = None) -> None:
        # Best-effort: re-enable panning if the widget is torn down mid-pick.
        self._controller.enabled = True

    # ------------------------------ markers -------------------------------

    def set_marker(self, key: str, xy: tuple[float, float] | None, color: str) -> None:
        """Draw/move/remove a small crosshair at pixel ``xy``, tagged *key*.

        Parameters
        ----------
        key : str
            Identifies this marker (e.g. ``"cam1"``/``"cam2"``) so each
            tracked spot's marker can be independently updated every frame
            without touching the other.
        xy : tuple[float, float] | None
            Pixel coordinates to draw the crosshair at, or ``None`` to
            remove the marker (e.g. tracking lost / cleared).
        color : str
            Color of the crosshair lines.
        """
        for obj in self._markers.pop(key, []):
            self._scene.remove(obj)
        if xy is not None:
            wx, wy = xy[0] - 0.5, xy[1] - 0.5
            half = 6.0
            h_line = pygfx.Line(
                pygfx.Geometry(
                    positions=np.array(
                        [[wx - half, wy, 0], [wx + half, wy, 0]], dtype=np.float32
                    )
                ),
                pygfx.LineMaterial(thickness=2.0, color=color, depth_test=False),
            )
            v_line = pygfx.Line(
                pygfx.Geometry(
                    positions=np.array(
                        [[wx, wy - half, 0], [wx, wy + half, 0]], dtype=np.float32
                    )
                ),
                pygfx.LineMaterial(thickness=2.0, color=color, depth_test=False),
            )
            self._scene.add(h_line, v_line)
            self._markers[key] = [h_line, v_line]
        self._canvas.request_draw(self._draw_function)

    # ----------------------------

    def _draw_function(self) -> None:
        self._renderer.render(self._scene, self._camera)
        self._renderer.request_draw()
