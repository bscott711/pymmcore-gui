"""The spot camera live view: native-resolution drawing, ROI drawing, navigation.

No camera, worker or MM install: frames are built in the test.

Run WITHOUT ``--noconftest`` and with ``QT_QPA_PLATFORM=offscreen``: with both
PyQt6 and PySide6 installed, pytest-qt otherwise picks PySide6 and
``qtbot.addWidget`` fails ("Need to pass a QWidget").
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest
from CRISPy.spot.live_view import (
    EDGE_BOX2_COLOR,
    EDGE_BOX_COLOR,
    ROI_COLOR,
    SETPOINT2_COLOR,
    SETPOINT_COLOR,
    SPOT2_COLOR,
    SpotLiveView,
    bayer_to_grey,
    frame_to_qimage,
    snap_edge_box,
    snap_roi,
)
from CRISPy.spot.spot_frame_shm import LiveFrame

from pymmcore_gui._qt.QtCore import QPoint, QPointF, Qt
from pymmcore_gui._qt.QtGui import QPainter

if TYPE_CHECKING:
    from pytestqt.qtbot import QtBot

SENSOR = (2048, 1536)


def make_frame(
    roi=(0, 0, 2048, 1536),
    *,
    seq: int = 1,
    bits: int = 8,
    spot=(100.0, 80.0),
    spot_ok: bool = True,
    fill: int | None = None,
) -> LiveFrame:
    _x, _y, w, h = roi
    dtype = np.uint8 if bits <= 8 else np.uint16
    rng = np.random.default_rng(seq)
    top = (1 << bits) - 1
    image = (
        np.full((h, w), fill, dtype=dtype)
        if fill is not None
        else rng.integers(0, top + 1, (h, w)).astype(dtype)
    )
    return LiveFrame(
        image=image,
        seq=seq,
        t=float(seq),
        roi=roi,
        sensor=SENSOR,
        bits=bits,
        spot=spot,
        spot_ok=spot_ok,
        saturated_frac=0.01,
        max_value=int(image.max()),
    )


class StubService:
    """Hands out frames like ``SpotFocusService.latest_frame``."""

    def __init__(self) -> None:
        self.frame: LiveFrame | None = None
        self.calls: list[int | None] = []

    def latest_frame(self, last_seq: int | None = None) -> LiveFrame | None:
        self.calls.append(last_seq)
        f = self.frame
        if f is None or (last_seq is not None and f.seq <= last_seq):
            return None
        return f


@pytest.fixture
def view(qtbot: QtBot) -> SpotLiveView:
    v = SpotLiveView()
    qtbot.addWidget(v)
    v.resize(900, 700)
    v.show()
    qtbot.waitExposed(v)
    return v


def _drag(v: SpotLiveView, qtbot: QtBot, a: QPointF, b: QPointF) -> None:
    """Left-drag between two scene points on the view's viewport."""
    pa = v.view.mapFromScene(a)
    pb = v.view.mapFromScene(b)
    vp = v.view.viewport()
    qtbot.mousePress(vp, Qt.MouseButton.LeftButton, pos=pa)
    qtbot.mouseMove(vp, pos=QPoint((pa.x() + pb.x()) // 2, (pa.y() + pb.y()) // 2))
    qtbot.mouseMove(vp, pos=pb)
    qtbot.mouseRelease(vp, Qt.MouseButton.LeftButton, pos=pb)


# ---------------------------------------------------------------------------
# pure helpers
# ---------------------------------------------------------------------------


def test_snap_roi_is_even_and_clamped() -> None:
    assert snap_roi((101.3, 51.1), (300.2, 200.9), SENSOR) == (100, 50, 202, 152)
    # dragged backwards and past the sensor edge: clamped, still even
    assert snap_roi((3000, 2000), (1900.4, 1400.0), SENSOR) == (1900, 1400, 148, 136)
    assert snap_roi((-50, -10), (40, 30), SENSOR) == (0, 0, 40, 30)
    # a click or a sliver is not an ROI
    assert snap_roi((10, 10), (12, 12), SENSOR) is None
    assert snap_roi((10, 10), (500, 12), SENSOR) is None
    x, y, w, h = snap_roi((7.7, 9.1), (333.3, 444.4), SENSOR)  # type: ignore[misc]
    assert all(v % 2 == 0 for v in (x, y, w, h))


def test_snap_edge_box_is_even_clamped_and_at_least_16() -> None:
    assert snap_edge_box((232.3, 210.6), (311.2, 269.9), SENSOR) == (232, 210, 80, 60)
    # same snapping as the ROI: backwards, past the sensor edge
    assert snap_edge_box((3000, 2000), (1900.4, 1400.0), SENSOR) == (
        1900,
        1400,
        148,
        136,
    )
    # between the ROI minimum (8) and the edge box minimum (16): a ROI, no box
    assert snap_roi((10, 10), (22, 22), SENSOR) == (10, 10, 12, 12)
    assert snap_edge_box((10, 10), (22, 22), SENSOR) is None
    assert snap_edge_box((10, 10), (500, 20), SENSOR) is None
    x, y, w, h = snap_edge_box((7.7, 9.1), (333.3, 444.4), SENSOR)  # type: ignore[misc]
    assert all(v % 2 == 0 for v in (x, y, w, h))


def test_qimage_formats_and_levels() -> None:
    u8 = np.arange(12, dtype=np.uint8).reshape(3, 4)
    qimg, buf = frame_to_qimage(u8, 8)
    assert (qimg.width(), qimg.height()) == (4, 3)
    assert qimg.pixelColor(2, 1).red() == int(u8[1, 2])  # fixed levels: unchanged

    u12 = np.array([[0, 4095]], dtype=np.uint16)
    qimg, _buf = frame_to_qimage(u12, 12)
    assert qimg.format().name.endswith("Grayscale16")
    assert qimg.pixelColor(0, 0).red() == 0
    assert qimg.pixelColor(1, 0).red() >= 254  # full scale maps to white

    dim = np.array([[10, 20, 30]], dtype=np.uint8)
    qimg, _buf = frame_to_qimage(dim, 8, auto_levels=True)
    assert [qimg.pixelColor(i, 0).red() for i in range(3)] == [0, 127, 255]
    flat, _ = frame_to_qimage(np.full((2, 2), 9, np.uint8), 8, auto_levels=True)
    assert flat.pixelColor(0, 0).red() == 0  # constant frame: no divide by zero


def _rggb_mosaic(
    h: int, w: int, dtype: type = np.uint8, blob: bool = False, top: int = 255
) -> np.ndarray:
    """A synthetic RGGB mosaic: colour gains tiled, optionally under a blob."""
    # sites (0,0) (0,1) (1,0) (1,1), like the rig's 118 / 167 / 167 / 58 under white
    gains = np.tile(np.array([[118.0, 167.0], [167.0, 58.0]]) / 255.0, (h // 2, w // 2))
    if blob:
        yy, xx = np.mgrid[0:h, 0:w]
        gains = gains * np.exp(
            -(((xx - w / 2) / (w / 6)) ** 2 + ((yy - h / 2) / (h / 6)) ** 2) / 2
        )
    return np.rint(gains * top).astype(dtype)


def _sublattice_means(a: np.ndarray) -> list[float]:
    return [float(a[i::2, j::2].mean()) for i in (0, 1) for j in (0, 1)]


@pytest.mark.parametrize("dtype,top", [(np.uint8, 255), (np.uint16, 65535)])
def test_bayer_to_grey_flat_colour_is_constant(dtype: type, top: int) -> None:
    mosaic = _rggb_mosaic(32, 48, dtype, top=top)
    out = bayer_to_grey(mosaic)
    assert out.shape == mosaic.shape and out.dtype == mosaic.dtype
    inner = out[2:-2, 2:-2].astype(np.float64)
    assert inner.std() < 1e-9  # the 2x2 cell is gone
    # R:G:B = 1:2:1 weighting of the same cell
    assert inner.mean() == pytest.approx(mosaic.mean(), abs=top / 255)
    assert mosaic[2:-2, 2:-2].astype(np.float64).std() > 0.1 * top / 4  # was a mosaic


@pytest.mark.parametrize("dtype,top", [(np.uint8, 255), (np.uint16, 65535)])
def test_bayer_to_grey_kills_nyquist_power(dtype: type, top: int) -> None:
    mosaic = _rggb_mosaic(192, 256, dtype, blob=True, top=top)
    out = bayer_to_grey(mosaic)
    assert out.shape == mosaic.shape and out.dtype == mosaic.dtype
    h, w = mosaic.shape

    def nyquist_power(a: np.ndarray) -> list[float]:
        f = np.abs(np.fft.fft2(a.astype(np.float64))) ** 2
        return [float(f[h // 2, w // 2]), float(f[h // 2, 0]), float(f[0, w // 2])]

    before, after = nyquist_power(mosaic), nyquist_power(out)
    assert before[0] > 0  # the CFA put energy at (h/2, w/2)
    for b, a in zip(before, after, strict=True):
        assert a <= 0.01 * max(b, 1.0)
    assert after[0] < 1e-3 * before[0]


def test_bayer_to_grey_is_fast_enough_and_rejects_3d() -> None:
    import time

    img = _rggb_mosaic(1536, 2048)
    bayer_to_grey(img)  # warm up
    t0 = time.perf_counter()
    for _ in range(5):
        bayer_to_grey(img)
    per_frame = (time.perf_counter() - t0) / 5
    assert per_frame < 0.25  # generous; ~10-20 ms measured, 12 fps needs < 80 ms
    with pytest.raises(ValueError):
        bayer_to_grey(np.zeros((2, 2, 3), np.uint8))


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------


def test_roi_frame_drawn_at_offset_and_native_size(view: SpotLiveView) -> None:
    view.show_frame(make_frame(roi=(400, 300, 640, 480)))
    item = view.pixmap_item
    assert (item.pos().x(), item.pos().y()) == (400, 300)
    pm = item.pixmap()
    assert (pm.width(), pm.height()) == (640, 480)  # not downsampled
    assert view.sensor == SENSOR
    rect = view.scene.sceneRect()
    assert (rect.width(), rect.height()) == SENSOR
    # the sensor was fitted to the window (zoom < 1): resampled smoothly ...
    assert view.zoom < 1.0
    assert view.view.renderHints() & QPainter.RenderHint.SmoothPixmapTransform
    assert item.transformationMode() == Qt.TransformationMode.SmoothTransformation
    # ... but at 1:1 the pixels stay hard-edged
    view.one_to_one()
    assert not (view.view.renderHints() & QPainter.RenderHint.SmoothPixmapTransform)
    assert item.transformationMode() == Qt.TransformationMode.FastTransformation

    # a full frame keeps its full size too
    view.show_frame(make_frame(seq=2))
    pm = view.pixmap_item.pixmap()
    assert (pm.width(), pm.height()) == SENSOR
    assert (view.pixmap_item.pos().x(), view.pixmap_item.pos().y()) == (0, 0)


def test_overlays_follow_the_frame(view: SpotLiveView) -> None:
    view.set_setpoint((120.0, 90.0))
    view.show_frame(make_frame(roi=(64, 32, 400, 300), spot=(150.5, 100.0)))
    assert view._roi_item.isVisible()
    r = view._roi_item.rect()
    assert (r.x(), r.y(), r.width(), r.height()) == (64, 32, 400, 300)
    assert view._spot_item.isVisible()
    assert view._spot_item.pos().x() == pytest.approx(151.0)
    assert view._spot_item.pen().color().name() == "#3df25a"  # ok: green
    assert view._setpoint_item.isVisible()
    assert view._setpoint_item.pos().x() == pytest.approx(120.5)

    view.show_frame(make_frame(roi=(64, 32, 400, 300), seq=2, spot_ok=False))
    assert view._spot_item.pen().color().name() == "#ff9a1f"  # not ok: orange
    view.show_frame(make_frame(seq=3, spot=None))
    assert not view._spot_item.isVisible()
    assert not view._roi_item.isVisible()  # full frame: only the sensor outline
    view.set_setpoint(None)
    assert not view._setpoint_item.isVisible()
    assert not view.grab().isNull()  # paints without raising


def test_edge_box_overlay_is_dashed_and_distinct_from_the_roi(
    view: SpotLiveView,
) -> None:
    assert not view._edge_box_item.isVisible()
    assert view.edge_box is None
    view.set_edge_box((232, 210, 80, 60))
    assert view._edge_box_item.isVisible()
    assert view.edge_box == (232, 210, 80, 60)
    r = view._edge_box_item.rect()
    assert (r.x(), r.y(), r.width(), r.height()) == (232, 210, 80, 60)
    pen = view._edge_box_item.pen()
    assert pen.style() == Qt.PenStyle.DashLine and pen.isCosmetic()
    assert pen.color().name() == EDGE_BOX_COLOR
    assert EDGE_BOX_COLOR != ROI_COLOR
    assert view._roi_item.pen().color().name() == ROI_COLOR
    assert view._edge_box_item is not view._roi_item

    # it is the user's box, not frame data: frames and clear() leave it alone
    view.show_frame(make_frame(roi=(64, 32, 400, 300)))
    assert view._edge_box_item.isVisible() and view._roi_item.isVisible()
    r = view._roi_item.rect()
    assert (r.x(), r.y(), r.width(), r.height()) == (64, 32, 400, 300)
    view.clear()
    assert view._edge_box_item.isVisible()
    assert not view.grab().isNull()  # paints without raising
    view.set_edge_box(None)
    assert not view._edge_box_item.isVisible() and view.edge_box is None


def test_edge_box_2_overlay_is_dashed_and_has_its_own_colour(
    view: SpotLiveView,
) -> None:
    assert not view._edge_box2_item.isVisible()
    assert view.edge_box2 is None
    view.set_edge_box2((288, 154, 64, 80))
    assert view._edge_box2_item.isVisible()
    assert view.edge_box2 == (288, 154, 64, 80)
    r = view._edge_box2_item.rect()
    assert (r.x(), r.y(), r.width(), r.height()) == (288, 154, 64, 80)
    pen = view._edge_box2_item.pen()
    assert pen.style() == Qt.PenStyle.DashLine and pen.isCosmetic()
    assert pen.color().name() == EDGE_BOX2_COLOR
    assert len({EDGE_BOX2_COLOR, EDGE_BOX_COLOR, ROI_COLOR}) == 3
    assert view._edge_box2_item is not view._edge_box_item
    assert not view._edge_box_item.isVisible()  # box 1 is independent
    # the user's box: frames and clear() leave it alone
    view.show_frame(make_frame(roi=(64, 32, 400, 300)))
    view.clear()
    assert view._edge_box2_item.isVisible()
    assert not view.grab().isNull()  # paints without raising
    view.set_edge_box2(None)
    assert not view._edge_box2_item.isVisible() and view.edge_box2 is None


def test_secondary_point_and_setpoint_have_their_own_markers(
    view: SpotLiveView,
) -> None:
    assert not view._spot2_item.isVisible()
    assert not view._setpoint2_item.isVisible()
    view.set_setpoint((120.0, 90.0))
    view.set_secondary_point((320.0, 194.25))
    view.set_secondary_setpoint((320.0, 194.0))
    assert view._spot2_item.isVisible() and view._setpoint2_item.isVisible()
    assert view._spot2_item.pos().x() == pytest.approx(320.5)
    assert view._spot2_item.pos().y() == pytest.approx(194.75)
    assert view._setpoint2_item.pos().y() == pytest.approx(194.5)
    assert view._spot2_item.pen().color().name() == SPOT2_COLOR
    assert view._setpoint2_item.pen().color().name() == SETPOINT2_COLOR
    assert SETPOINT2_COLOR != SETPOINT_COLOR
    view.set_secondary_point((320.0, 194.25), ok=False)
    assert view._spot2_item.pen().color().name() == "#ff9a1f"
    # the first ring and cross are untouched by the second
    assert view._setpoint_item.pos().x() == pytest.approx(120.5)
    assert not view._spot_item.isVisible()
    # a frame does not move or hide the panel-fed second cross; clear() hides it
    view.show_frame(make_frame(spot=(150.5, 100.0)))
    assert view._spot_item.isVisible() and view._spot2_item.isVisible()
    assert not view.grab().isNull()
    view.clear()
    assert not view._spot2_item.isVisible()
    assert view._setpoint2_item.isVisible()  # a zero marker, not frame data
    view.set_secondary_point(None)
    view.set_secondary_setpoint(None)
    assert not view._spot2_item.isVisible()
    assert not view._setpoint2_item.isVisible()
    assert view._setpoint_item.isVisible()


def test_status_text_says_edge(view: SpotLiveView) -> None:
    view.show_frame(make_frame(spot=(150.5, 100.0)))
    text = view.status_label.text()
    assert "edge (150.5, 100.0)" in text and "spot" not in text
    view.show_frame(make_frame(seq=2, spot=(150.5, 100.0), spot_ok=False))
    assert "edge (150.5, 100.0) (not ok)" in view.status_label.text()


def test_pixel_readout_and_status(view: SpotLiveView) -> None:
    frame = make_frame(roi=(100, 50, 20, 10), fill=0)
    frame.image[3, 5] = 77
    view.show_frame(frame)
    assert view.pixel_at(105, 53) == 77
    assert view.pixel_at(99, 53) is None  # outside the ROI
    view._on_cursor(QPointF(105.4, 53.9))
    assert view.cursor_label.text() == "x 105  y 53  value 77"
    view._on_cursor(None)
    assert view.cursor_label.text() == ""
    text = view.status_label.text()
    assert "max" in text and "saturated" in text and "ROI 20x10 at (100, 50)" in text


def test_timer_polls_and_repaints_only_new_frames(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    svc = StubService()
    view.set_service(svc)
    assert view.poll() is False  # no frame yet
    svc.frame = make_frame(seq=5)
    assert view.poll() is True
    assert view.poll() is False  # same seq: nothing to do
    assert svc.calls[-1] == 5
    pm_key = view.pixmap_item.pixmap().cacheKey()
    view.poll()
    assert view.pixmap_item.pixmap().cacheKey() == pm_key  # not rebuilt
    svc.frame = make_frame(seq=6)
    assert view.poll() is True

    # The real timer is running while the window is visible.
    svc.frame = make_frame(seq=7)
    qtbot.waitUntil(
        lambda: view.frame is not None and view.frame.seq == 7, timeout=2000
    )


def test_polling_can_be_switched_off_while_visible(view: SpotLiveView) -> None:
    assert view.polling and view._timer.isActive()
    view.set_polling(False)
    assert not view.polling and not view._timer.isActive()
    view.hide()
    view.show()  # showing does not restart a disabled timer
    assert not view._timer.isActive()
    view.set_polling(True)
    assert view._timer.isActive()
    view.hide()
    assert not view._timer.isActive()


def test_view_can_be_a_child_and_a_window_again(qtbot: QtBot) -> None:
    from pymmcore_gui._qt.QtWidgets import QVBoxLayout, QWidget

    host = QWidget()
    qtbot.addWidget(host)  # owns the view below
    view = SpotLiveView()
    QVBoxLayout(host).addWidget(view)  # docked: a child of the host
    host.show()
    assert not view.isWindow() and view.window() is host
    assert view._timer.isActive()
    zoom_before = view.zoom
    view.setParent(host, Qt.WindowType.Window)  # floated again, same scene
    view.show()
    assert view.isWindow() and view.zoom == pytest.approx(zoom_before)


def test_stats_signal(qtbot: QtBot, view: SpotLiveView) -> None:
    with qtbot.waitSignal(view.statsChanged, timeout=1000) as sig:
        view.show_frame(make_frame(fill=200))
    _fps, mx, sat = sig.args
    assert mx == 200
    assert sat == pytest.approx(1.0)


def test_auto_levels_rebuilds_the_pixmap(view: SpotLiveView) -> None:
    view.bayer_grey.setChecked(False)  # this test is about raw levels
    view.show_frame(make_frame(roi=(0, 0, 8, 8), fill=40))
    before = view.pixmap_item.pixmap().toImage().pixelColor(0, 0).red()
    assert before == 40  # fixed levels: 0..255 untouched
    ramp = make_frame(roi=(0, 0, 8, 8), fill=40)
    ramp.image[0, 0] = 20
    view.show_frame(ramp)
    view.auto_levels.setChecked(True)
    img = view.pixmap_item.pixmap().toImage()
    assert img.pixelColor(0, 0).red() == 0
    assert img.pixelColor(1, 1).red() == 255


def _mosaic_frame(roi: tuple[int, int, int, int] = (0, 0, 64, 48)) -> LiveFrame:
    _x, _y, w, h = roi
    frame = make_frame(roi=roi, fill=0)
    frame.image[:] = _rggb_mosaic(h, w)
    return frame


def test_bayer_to_grey_checkbox_controls_the_pixmap(view: SpotLiveView) -> None:
    assert view.bayer_grey.isChecked()  # the rig sensor is colour: default on
    frame = _mosaic_frame()
    view.show_frame(frame)
    img = view.pixmap_item.pixmap().toImage()
    assert (img.width(), img.height()) == (64, 48)  # still native size
    grey = np.array(
        [[img.pixelColor(x, y).red() for x in range(64)] for y in range(48)]
    )
    inner = grey[2:-2, 2:-2]
    means = _sublattice_means(inner)
    assert max(means) - min(means) < 1.0  # no period-2 checker left
    # the raw values (pixel readout) are untouched
    assert view.pixel_at(0, 0) == int(frame.image[0, 0])
    assert view.frame is not None and np.array_equal(
        view.frame.image, _rggb_mosaic(48, 64)
    )

    view.bayer_grey.setChecked(False)  # re-runs _redraw: raw mosaic shown
    img = view.pixmap_item.pixmap().toImage()
    raw = np.array([[img.pixelColor(x, y).red() for x in range(64)] for y in range(48)])
    assert np.array_equal(raw, frame.image)
    means = _sublattice_means(raw[2:-2, 2:-2])
    assert max(means) - min(means) > 50


def test_resampling_follows_the_zoom(view: SpotLiveView) -> None:
    smooth = QPainter.RenderHint.SmoothPixmapTransform
    view.show_frame(make_frame())  # full 2048x1536 sensor in 900x700
    view.fit()
    assert view.zoom < 1.0
    assert view.view.renderHints() & smooth
    assert (
        view.pixmap_item.transformationMode()
        == Qt.TransformationMode.SmoothTransformation
    )

    view.one_to_one()
    assert not (view.view.renderHints() & smooth)
    assert (
        view.pixmap_item.transformationMode()
        == Qt.TransformationMode.FastTransformation
    )

    view.view.zoom_by(0.5)  # wheel out below 100%: smooth again
    assert view.view.renderHints() & smooth
    view.view.zoom_by(8.0)  # wheel in past 100%: crisp pixels
    assert not (view.view.renderHints() & smooth)
    assert (
        view.pixmap_item.transformationMode()
        == Qt.TransformationMode.FastTransformation
    )


# ---------------------------------------------------------------------------
# Draw ROI
# ---------------------------------------------------------------------------


def test_draw_roi_emits_even_clamped_values_and_turns_off(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    view.show_frame(make_frame())
    view.fit()
    view.draw_roi_button.setChecked(True)
    assert view.view.draw_mode
    # Drag from inside the sensor to well past its lower-right corner.
    with qtbot.waitSignal(view.roiDrawn, timeout=2000) as sig:
        _drag(view, qtbot, QPointF(401.0, 301.0), QPointF(2200.0, 1700.0))
    x, y, w, h = sig.args
    assert all(v % 2 == 0 for v in (x, y, w, h))
    assert x + w <= SENSOR[0] and y + h <= SENSOR[1]
    assert (x, y) == (pytest.approx(400, abs=4), pytest.approx(300, abs=4))
    assert x + w == SENSOR[0] and y + h == SENSOR[1]  # clamped to the sensor
    assert not view.draw_roi_button.isChecked()  # mode turned itself off
    assert not view.view.draw_mode


def test_draw_roi_ignores_a_click(qtbot: QtBot, view: SpotLiveView) -> None:
    view.show_frame(make_frame())
    view.fit()
    got: list[tuple] = []
    view.roiDrawn.connect(lambda *a: got.append(a))
    view.draw_roi_button.setChecked(True)
    _drag(view, qtbot, QPointF(500.0, 500.0), QPointF(502.0, 502.0))
    assert got == []
    assert view.draw_roi_button.isChecked()  # still armed


def test_draw_edge_box_emits_even_clamped_values_not_a_roi(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    view.show_frame(make_frame())
    view.fit()
    rois: list[tuple] = []
    view.roiDrawn.connect(lambda *a: rois.append(a))
    modes: list[bool] = []
    view.edgeDrawModeChanged.connect(modes.append)
    assert view.draw_target is None
    view.set_edge_draw_mode(True)  # the panel's way in
    assert view.draw_edge_button.isChecked()
    assert view.draw_target == "edge" and view.view.draw_mode
    assert not view.draw_roi_button.isChecked()
    with qtbot.waitSignal(view.edgeBoxDrawn, timeout=2000) as sig:
        _drag(view, qtbot, QPointF(401.0, 301.0), QPointF(2200.0, 1700.0))
    x, y, w, h = sig.args
    assert all(v % 2 == 0 for v in (x, y, w, h))
    assert (x, y) == (pytest.approx(400, abs=4), pytest.approx(300, abs=4))
    assert x + w == SENSOR[0] and y + h == SENSOR[1]  # clamped to the sensor
    assert rois == []  # a different signal from Draw ROI
    assert not view.draw_edge_button.isChecked()  # one box per click
    assert view.draw_target is None and not view.view.draw_mode
    assert modes == [True, False]


def test_draw_edge_box_2_emits_its_own_signal(qtbot: QtBot, view: SpotLiveView) -> None:
    view.show_frame(make_frame())
    view.fit()
    rois: list[tuple] = []
    boxes: list[tuple] = []
    view.roiDrawn.connect(lambda *a: rois.append(a))
    view.edgeBoxDrawn.connect(lambda *a: boxes.append(a))
    modes: list[bool] = []
    view.edge2DrawModeChanged.connect(modes.append)
    view.set_edge2_draw_mode(True)  # the panel's way in
    assert view.draw_edge2_button.isChecked()
    assert view.draw_target == "edge2" and view.view.draw_mode
    assert not view.draw_roi_button.isChecked()
    assert not view.draw_edge_button.isChecked()
    with qtbot.waitSignal(view.edgeBox2Drawn, timeout=2000) as sig:
        _drag(view, qtbot, QPointF(401.0, 301.0), QPointF(2200.0, 1700.0))
    x, y, w, h = sig.args
    assert all(v % 2 == 0 for v in (x, y, w, h))
    assert x + w == SENSOR[0] and y + h == SENSOR[1]  # clamped to the sensor
    assert rois == [] and boxes == []  # different signals
    assert not view.draw_edge2_button.isChecked()  # one box per click
    assert view.draw_target is None and not view.view.draw_mode
    assert modes == [True, False]


def test_draw_edge_box_2_ignores_a_click_and_a_sliver(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    view.show_frame(make_frame())
    view.fit()
    got: list[tuple] = []
    view.edgeBox2Drawn.connect(lambda *a: got.append(a))
    view.draw_edge2_button.setChecked(True)
    _drag(view, qtbot, QPointF(500.0, 500.0), QPointF(502.0, 502.0))
    _drag(view, qtbot, QPointF(500.0, 500.0), QPointF(900.0, 504.0))
    assert got == []
    assert view.draw_edge2_button.isChecked()  # still armed


def test_all_three_draw_modes_are_exclusive(qtbot: QtBot, view: SpotLiveView) -> None:
    view.show_frame(make_frame())
    view.fit()
    buttons = {
        "roi": view.draw_roi_button,
        "edge": view.draw_edge_button,
        "edge2": view.draw_edge2_button,
    }
    for target, button in buttons.items():
        button.setChecked(True)
        assert view.draw_target == target and view.view.draw_mode
        assert [n for n, b in buttons.items() if b.isChecked()] == [target]
    buttons["edge2"].setChecked(False)
    assert view.draw_target is None and not view.view.draw_mode
    # Draw edge box 1 still emits only its own signal after box 2 was armed
    first: list[tuple] = []
    second: list[tuple] = []
    view.edgeBoxDrawn.connect(lambda *a: first.append(a))
    view.edgeBox2Drawn.connect(lambda *a: second.append(a))
    view.draw_edge2_button.setChecked(True)
    view.draw_edge_button.setChecked(True)
    with qtbot.waitSignal(view.edgeBoxDrawn, timeout=2000):
        _drag(view, qtbot, QPointF(401.0, 301.0), QPointF(900.0, 700.0))
    assert len(first) == 1 and second == []


def test_draw_edge_box_ignores_a_click_and_a_sliver(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    view.show_frame(make_frame())
    view.fit()
    got: list[tuple] = []
    view.edgeBoxDrawn.connect(lambda *a: got.append(a))
    view.draw_edge_button.setChecked(True)
    _drag(view, qtbot, QPointF(500.0, 500.0), QPointF(502.0, 502.0))
    assert got == []
    # wide but only a few px high: a sliver (the 8 to 16 px band is covered by
    # test_snap_edge_box_is_even_clamped_and_at_least_16 at exact coordinates)
    _drag(view, qtbot, QPointF(500.0, 500.0), QPointF(900.0, 504.0))
    assert got == []
    assert view.draw_edge_button.isChecked()  # still armed


def test_draw_roi_and_edge_box_modes_are_exclusive(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    view.show_frame(make_frame())
    view.fit()
    view.draw_roi_button.setChecked(True)
    assert view.draw_target == "roi"
    view.draw_edge_button.setChecked(True)
    assert view.draw_target == "edge" and not view.draw_roi_button.isChecked()
    assert view.view.draw_mode
    view.draw_roi_button.setChecked(True)
    assert view.draw_target == "roi" and not view.draw_edge_button.isChecked()
    # Draw ROI still emits roiDrawn only
    boxes: list[tuple] = []
    view.edgeBoxDrawn.connect(lambda *a: boxes.append(a))
    with qtbot.waitSignal(view.roiDrawn, timeout=2000):
        _drag(view, qtbot, QPointF(401.0, 301.0), QPointF(900.0, 700.0))
    assert boxes == []
    assert view.draw_target is None and not view.view.draw_mode
    # turning the armed mode off returns to panning
    view.draw_edge_button.setChecked(True)
    view.draw_edge_button.setChecked(False)
    assert view.draw_target is None and not view.view.draw_mode


def test_closing_the_window_leaves_draw_modes(qtbot: QtBot, view: SpotLiveView) -> None:
    view.draw_edge_button.setChecked(True)
    with qtbot.waitSignal(view.windowClosed, timeout=1000):
        view.close()
    assert not view.draw_edge_button.isChecked() and view.draw_target is None
    view.show()
    view.draw_edge2_button.setChecked(True)
    with qtbot.waitSignal(view.windowClosed, timeout=1000):
        view.close()
    assert not view.draw_edge2_button.isChecked() and view.draw_target is None


# ---------------------------------------------------------------------------
# navigation
# ---------------------------------------------------------------------------


def test_fit_and_one_to_one(qtbot: QtBot, view: SpotLiveView) -> None:
    view.show_frame(make_frame())
    # Fit: the whole sensor is visible, scaled to the viewport.
    view.fit()
    expect = min(
        view.view.viewport().width() / SENSOR[0],
        view.view.viewport().height() / SENSOR[1],
    )
    assert view.zoom == pytest.approx(expect, rel=0.02)
    visible = view.view.mapToScene(view.view.viewport().rect()).boundingRect()
    assert visible.contains(view.scene.sceneRect().adjusted(1, 1, -1, -1))
    assert view.fit_mode

    # 1:1 : one image pixel per screen pixel (device pixel ratio 1 offscreen).
    view.one_to_one()
    assert view.zoom == pytest.approx(1.0 / max(view.devicePixelRatioF(), 1.0))
    assert not view.fit_mode
    assert view.zoom_label.text() == "100%"
    # centred on the spot
    centre = view.view.mapToScene(view.view.viewport().rect().center())
    assert centre.x() == pytest.approx(100.5, abs=2)
    assert centre.y() == pytest.approx(80.5, abs=2)

    # Fit mode follows a resize; 1:1 does not.
    z = view.zoom
    view.resize(1100, 800)
    qtbot.wait(50)
    assert view.zoom == pytest.approx(z)
    view.fit()
    z_small = view.zoom
    view.resize(500, 400)
    qtbot.wait(50)
    assert view.zoom < z_small


def test_wheel_zoom_keeps_the_point_under_the_cursor(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    view.show_frame(make_frame())
    view.one_to_one()
    vp = view.view.viewport()
    pos = QPoint(vp.width() // 3, vp.height() // 3)
    before = view.view.mapToScene(pos)
    z0 = view.zoom
    view.view.zoom_by(2.0, pos)  # the wheel handler calls this
    assert view.zoom == pytest.approx(2 * z0)
    after = view.view.mapToScene(pos)
    assert after.x() == pytest.approx(before.x(), abs=1.5)
    assert after.y() == pytest.approx(before.y(), abs=1.5)
    assert not view.fit_mode
    view.view.zoom_by(1e6)
    assert view.zoom <= 128.0 + 1e-6  # clamped


def test_window_closed_signal(qtbot: QtBot, view: SpotLiveView) -> None:
    with qtbot.waitSignal(view.windowClosed, timeout=1000):
        view.close()
    assert not view.isVisible()
    view.show()  # reusable
    assert view.isVisible()


def test_real_wheel_event_zooms_around_the_cursor(
    qtbot: QtBot, view: SpotLiveView
) -> None:
    from pymmcore_gui._qt.QtCore import QPointF as PF
    from pymmcore_gui._qt.QtGui import QWheelEvent
    from pymmcore_gui._qt.QtWidgets import QApplication

    view.show_frame(make_frame())
    view.one_to_one()
    vp = view.view.viewport()
    pos = QPoint(vp.width() // 4, vp.height() // 2)
    before = view.view.mapToScene(pos)
    z0 = view.zoom
    ev = QWheelEvent(
        PF(pos),
        PF(vp.mapToGlobal(pos)),
        QPoint(0, 0),
        QPoint(0, 120),
        Qt.MouseButton.NoButton,
        Qt.KeyboardModifier.NoModifier,
        Qt.ScrollPhase.NoScrollPhase,
        False,
    )
    QApplication.sendEvent(vp, ev)
    assert view.zoom == pytest.approx(z0 * 1.25)
    after = view.view.mapToScene(pos)
    assert after.x() == pytest.approx(before.x(), abs=1.5)
    assert after.y() == pytest.approx(before.y(), abs=1.5)
