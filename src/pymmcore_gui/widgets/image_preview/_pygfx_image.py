from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal

import numpy as np
import pygfx
import pylinalg as la
from cmap import Colormap

from pymmcore_gui._qt.QtCore import QObject, QSize
from pymmcore_gui._qt.QtWidgets import QVBoxLayout, QWidget

from ._preview_base import ImagePreviewBase

if TYPE_CHECKING:
    from collections.abc import Callable

    import rendercanvas.qt
    from cmap._colormap import ColorStopsLike
    from pymmcore_plus import CMMCorePlus

    class QRenderWidget(rendercanvas.qt.QRenderWidget, QWidget): ...  # pyright: ignore [reportIncompatibleMethodOverride]

else:
    from rendercanvas.qt import QRenderWidget

_DEFAULT_WAIT = 10


class PygfxImagePreview(ImagePreviewBase):
    """A Widget that displays the last image snapped by active core.

    This widget will automatically update when the active core snaps an image, when the
    active core starts streaming or when a Multi-Dimensional Acquisition is running.

    This is a single-image-node viewer optimized for speed.

    Parameters
    ----------
    parent : QWidget | None
        Optional parent widget. By default, None.
    mmcore : CMMCorePlus | None
        Optional [`pymmcore_plus.CMMCorePlus`][] micromanager core.
        By default, None. If not specified, the widget will use the active
        (or create a new)
        [`CMMCorePlus.instance`][pymmcore_plus.core._mmcore_plus.CMMCorePlus.instance].
    use_with_mda: bool
        If False, the widget will not update when a Multi-Dimensional Acquisition is
        running. By default, True.
    mouse_wheel_sensitivity: float
        The sensitivity of the mouse wheel for zooming. By default, 0.004.  Higher
        means faster zooming.

    Attributes
    ----------
    data : np.ndarray | None
        The data of the image.
    clims : tuple[float, float]
        The contrast limits of the image.
    cmap : Colormap
        The colormap (lookup table) of the image.
    use_with_mda : bool
        Whether the widget updates when a Multi-Dimensional Acquisition is running.
    """

    def __init__(
        self,
        parent: QWidget | None,
        mmcore: CMMCorePlus,
        *,
        use_with_mda: bool = False,
        mouse_wheel_sensitivity: float = 0.004,
    ):
        super().__init__(parent, mmcore, use_with_mda=use_with_mda)

        self._clims: tuple[float, float] | Literal["auto"] = "auto"
        self._cmap: Colormap = Colormap("gray")
        self._roi_overlays: list[pygfx.WorldObject] = []

        # IMAGE NODE

        self._texture = pygfx.Texture(dim=2, size=(1, 1), format="1xf4")
        self._geometry = pygfx.Geometry(grid=self._texture)
        self._material = pygfx.ImageBasicMaterial(
            clim=(0, 1), map=self._cmap.to_pygfx()
        )
        self._image_node = pygfx.Image(self._geometry, self._material, visible=False)

        # SCENE

        self._scene = scene = pygfx.Scene()
        # slight gradient background
        top = np.array((50, 50, 50, 255)) / 255
        bot = np.array((30, 30, 30, 255)) / 255
        scene.add(pygfx.Background(None, pygfx.BackgroundMaterial(bot, top)))

        self._canvas = QRenderWidget()
        self._renderer = renderer = pygfx.WgpuRenderer(self._canvas)
        self._camera = camera = pygfx.OrthographicCamera()
        # Flip the y-axis so row 0 of the image array is at the top (image/array
        # convention), matching ndv/napari.  Without this pygfx shows images
        # vertically mirrored.
        camera.local.scale_y = -1
        self._scene.add(self._image_node)
        self._scene.add(self._camera)
        self.reset_view()
        self._controller = pygfx.PanZoomController(
            camera, register_events=renderer, damping=2
        )

        # faster mouse wheel
        self._controller.controls["wheel"] = (
            "zoom_to_point",
            "push",
            -mouse_wheel_sensitivity,
        )
        self._controller.add_camera(self._camera)
        self._canvas.request_draw(self._draw_function)  # critical for showing

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._canvas)

        if isinstance(parent, QObject):
            parent.destroyed.connect(self.detach)

    def sizeHint(self) -> QSize:
        return self._canvas.sizeHint()

    @property
    def data(self) -> np.ndarray | None:
        """Return current texture data."""
        return self._texture.data  # type: ignore [no-any-return]

    def append(self, data: np.ndarray) -> None:
        """Set texture data.

        The dtype must be compatible with wgpu texture formats.
        Will also apply contrast limits if _clims is "auto".
        """
        if self._clims == "auto":
            self._material.clim = np.min(data), np.max(data)
        try:
            self._texture.set_data(data)
        except (ValueError, AttributeError):
            # texture has wrong shape or format, recreate it
            self._texture = pygfx.Texture(data, dim=2)
            self._geometry.grid = self._texture
            self.reset_view()
        self._image_node.visible = True

    @property
    def clims(self) -> tuple[float, float]:
        """Get the contrast limits of the image."""
        return self._material.clim  # type: ignore [no-any-return]

    def set_clims(self, clims: tuple[float, float] | Literal["auto"]) -> None:
        """Set the contrast limits of the image.

        Parameters
        ----------
        clims : tuple[float, float], or "auto"
            The contrast limits to set.
        """
        self._clims = clims
        if clims == "auto":
            if self.data is not None:
                self._material.clim = np.min(self.data), np.max(self.data)
        else:
            self._material.clim = clims

    @property
    def cmap(self) -> Colormap:
        """Get the colormap (lookup table) of the image."""
        return self._cmap

    def set_cmap(self, cmap: ColorStopsLike) -> None:
        """Set the colormap (lookup table) of the image.

        Parameters
        ----------
        cmap : str
            The colormap to use.
        """
        self._cmap = cm = Colormap(cmap)
        self._material.map = cm.to_pygfx()

    @property
    def interpolation(self) -> Literal["nearest", "linear"]:
        """Return the interpolation method."""
        return self._material.interpolation  # type: ignore [no-any-return]

    def set_interpolation(self, interpolation: Literal["nearest", "linear"]) -> None:
        """Set the interpolation method."""
        self._material.interpolation = interpolation

    def reset_view(self, scale: float = 0.8) -> None:
        """Reset the view so that the image fills the widget area."""
        self._camera.show_object(self._image_node, scale=scale)  # pyright: ignore [reportArgumentType]

    def set_roi_overlays(
        self, rois: list[tuple[str, tuple[int, int, int, int]]]
    ) -> None:
        """Draw rectangle outlines over fixed pixel regions, e.g. splitter ROIs.

        Parameters
        ----------
        rois : list[tuple[str, tuple[int, int, int, int]]]
            ``(label, (x, y, w, h))`` pairs, in full-frame pixel coordinates
            (``x``/``y`` are the top-left corner, in array column/row units).
            Any previously drawn overlays are cleared first.
        """
        self.clear_roi_overlays()
        for _label, (x, y, w, h) in rois:
            # The pygfx Image quad spans local/world x in [-0.5, size_x - 0.5]
            # (pixel *centers* at integer coordinates), so pixel -> world is a
            # flat -0.5 shift on both axes; see pygfx's image_common.wgsl
            # get_im_geometry(). The overlay lives in the same (identity
            # transform) object space as the image node, so no other
            # adjustment -- including the camera's y-flip -- is needed.
            x0, y0 = x - 0.5, y - 0.5
            x1, y1 = x + w - 0.5, y + h - 0.5
            pts = np.array(
                [[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0], [x0, y0, 0]],
                dtype=np.float32,
            )
            line = pygfx.Line(
                pygfx.Geometry(positions=pts),
                pygfx.LineMaterial(thickness=2.0, color="#00e5ff", depth_test=False),
            )
            self._scene.add(line)
            self._roi_overlays.append(line)
        self._canvas.request_draw(self._draw_function)

    def clear_roi_overlays(self) -> None:
        """Remove any rectangle overlays added by :meth:`set_roi_overlays`."""
        for obj in self._roi_overlays:
            self._scene.remove(obj)
        self._roi_overlays = []

    def _screen_to_world_xy(self, x: float, y: float) -> tuple[float, float]:
        """Convert a canvas-space pointer position to world (x, y).

        Inverse of the camera's world -> NDC transform used by pygfx to
        render (see e.g. ``get_screen_vectors_in_world_cords`` in
        ``pygfx.controllers``): NDC -> (inverse projection) -> view ->
        (camera world matrix) -> world. Valid for any depth for an
        orthographic camera, since it applies no perspective foreshortening.
        """
        w, h = self._renderer.logical_size
        ndc_x = 2.0 * x / w - 1.0
        ndc_y = 1.0 - 2.0 * y / h  # canvas y is down; NDC y is up
        cam = self._camera
        view_pos = la.vec_transform((ndc_x, ndc_y, 0.0), cam.projection_matrix_inverse)
        world_pos = la.vec_transform(view_pos, cam.world.matrix)
        return float(world_pos[0]), float(world_pos[1])

    def begin_roi_draw(
        self, on_done: Callable[[tuple[int, int, int, int]], None]
    ) -> None:
        """Start an interactive click-drag to define one rectangular ROI.

        Pan/zoom is suspended for the duration of the drag. On release, the
        dragged rectangle is converted to full-frame pixel coordinates
        (clamped to the current frame's shape) and passed to *on_done* as
        ``(x, y, w, h)``. No-ops if there's no image displayed yet.
        """
        if self.data is None:
            return
        frame_h, frame_w = self.data.shape[-2:]
        self._controller.enabled = False
        drag: dict[str, Any] = {}
        event_types = ("pointer_down", "pointer_move", "pointer_up")

        def _set_rubber_band(x0: float, y0: float, x1: float, y1: float) -> None:
            if (line := drag.get("line")) is not None:
                self._scene.remove(line)
            pts = np.array(
                [[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0], [x0, y0, 0]],
                dtype=np.float32,
            )
            line = pygfx.Line(
                pygfx.Geometry(positions=pts),
                pygfx.LineMaterial(thickness=2.0, color="#ffcc00", depth_test=False),
            )
            self._scene.add(line)
            drag["line"] = line
            self._canvas.request_draw(self._draw_function)

        def _finish(rect: tuple[int, int, int, int] | None) -> None:
            self._renderer.remove_event_handler(_on_pointer, *event_types)
            if (line := drag.pop("line", None)) is not None:
                self._scene.remove(line)
            self._controller.enabled = True
            self._canvas.request_draw(self._draw_function)
            if rect is not None:
                on_done(rect)

        def _on_pointer(event: pygfx.PointerEvent) -> None:
            if event.type == "pointer_down":
                drag["start"] = (event.x, event.y)
            elif event.type == "pointer_move" and "start" in drag:
                x0, y0 = self._screen_to_world_xy(*drag["start"])
                x1, y1 = self._screen_to_world_xy(event.x, event.y)
                _set_rubber_band(x0, y0, x1, y1)
            elif event.type == "pointer_up" and "start" in drag:
                px0, py0 = self._screen_to_world_xy(*drag["start"])
                px1, py1 = self._screen_to_world_xy(event.x, event.y)
                # world -> pixel is the inverse of set_roi_overlays' pixel -> world.
                px0, py0, px1, py1 = px0 + 0.5, py0 + 0.5, px1 + 0.5, py1 + 0.5
                x = max(0, min(round(min(px0, px1)), frame_w - 1))
                y = max(0, min(round(min(py0, py1)), frame_h - 1))
                w = max(1, min(round(abs(px1 - px0)), frame_w - x))
                h = max(1, min(round(abs(py1 - py0)), frame_h - y))
                _finish((x, y, w, h))

        self._renderer.add_event_handler(_on_pointer, *event_types)

    # ----------------------------

    def _draw_function(self) -> None:
        self._renderer.render(self._scene, self._camera)
        self._renderer.request_draw()
