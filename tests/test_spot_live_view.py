"""The spot camera live view: native-resolution drawing, ROI drawing, navigation.

No camera, worker or MM install: frames are built in the test. Runs with
``--noconftest`` like ``test_spot_focus_widget.py``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from pymmcore_gui._qt.QtCore import QPoint, QPointF, Qt
from pymmcore_gui._qt.QtGui import QPainter
from pymmcore_gui.asi_z_stack.spot_frame_shm import LiveFrame
from pymmcore_gui.widgets._spot_live_view import (
    SpotLiveView,
    frame_to_qimage,
    snap_roi,
)

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
    x, y, w, h = roi
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


def test_qimage_formats_and_levels() -> None:
    u8 = np.arange(12, dtype=np.uint8).reshape(3, 4)
    qimg, buf = frame_to_qimage(u8, 8)
    assert (qimg.width(), qimg.height()) == (4, 3)
    assert qimg.pixelColor(2, 1).red() == int(u8[1, 2])  # fixed levels: unchanged

    u12 = np.array([[0, 4095]], dtype=np.uint16)
    qimg, buf = frame_to_qimage(u12, 12)
    assert qimg.format().name.endswith("Grayscale16")
    assert qimg.pixelColor(0, 0).red() == 0
    assert qimg.pixelColor(1, 0).red() >= 254  # full scale maps to white

    dim = np.array([[10, 20, 30]], dtype=np.uint8)
    qimg, buf = frame_to_qimage(dim, 8, auto_levels=True)
    assert [qimg.pixelColor(i, 0).red() for i in range(3)] == [0, 127, 255]
    flat, _ = frame_to_qimage(np.full((2, 2), 9, np.uint8), 8, auto_levels=True)
    assert flat.pixelColor(0, 0).red() == 0  # constant frame: no divide by zero


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
    # no smoothing: zoomed pixels stay hard-edged
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


def test_stats_signal(qtbot: QtBot, view: SpotLiveView) -> None:
    with qtbot.waitSignal(view.statsChanged, timeout=1000) as sig:
        view.show_frame(make_frame(fill=200))
    fps, mx, sat = sig.args
    assert mx == 200
    assert sat == pytest.approx(1.0)


def test_auto_levels_rebuilds_the_pixmap(view: SpotLiveView) -> None:
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
