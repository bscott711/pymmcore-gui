"""Live camera view of the spot focus lock ("Spot camera" window).

The user aligns the laser spot, places the ROI and sets the exposure with this
window, so it shows the camera at **full resolution**: a
:class:`~pymmcore_gui._qt.QtWidgets.QGraphicsView` whose scene is the sensor,
with each frame drawn as a pixmap item at its ROI offset, one image pixel per
scene unit. Nothing is downsampled; when zoomed in, pixels are drawn
nearest-neighbour (``SmoothPixmapTransform`` off).

Frames come from the service (:meth:`SpotFocusService.latest_frame`, a shared
memory read) on a 30 Hz timer; the scene is repainted only when ``seq``
changes. Overlays (sensor outline, ROI, spot cross, setpoint ring) are scene
items with cosmetic pens, so they keep their on-screen size at any zoom.

See ``docs/spot_focus_lock.md``, "Live camera view and controls".
"""

from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from pymmcore_gui._qt.QtCore import QPoint, QPointF, QRectF, QSize, Qt, QTimer, Signal
from pymmcore_gui._qt.QtGui import (
    QBrush,
    QColor,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
)
from pymmcore_gui._qt.QtWidgets import (
    QCheckBox,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

if TYPE_CHECKING:
    from PyQt6.QtGui import QMouseEvent, QResizeEvent, QWheelEvent

    from pymmcore_gui.asi_z_stack.spot_frame_shm import LiveFrame

logger = logging.getLogger(__name__)

#: Frame poll period (30 Hz), ms.
POLL_MS = 33
#: Smallest rectangle a Draw ROI drag may produce, sensor px.
MIN_ROI_PX = 8
#: Zoom limits, view pixels per image pixel.
MIN_ZOOM = 0.02
MAX_ZOOM = 128.0
#: Frames older than this are flagged as stale in the status line, s.
STALE_S = 1.5
#: Stats (fps, max, saturation) are re-emitted at most this often, s.
_STATS_PERIOD_S = 0.25

OK_COLOR = "#3df25a"
BAD_COLOR = "#ff9a1f"
SETPOINT_COLOR = "#ff4fd8"
ROI_COLOR = "#56b4e9"


# ---------------------------------------------------------------------------
# pure helpers (tested without a window)
# ---------------------------------------------------------------------------


def snap_roi(
    p0: tuple[float, float],
    p1: tuple[float, float],
    sensor: tuple[int, int],
) -> tuple[int, int, int, int] | None:
    """The ROI ``(x, y, w, h)`` for a drag from *p0* to *p1* (sensor px).

    The rectangle is clamped to the sensor, then snapped outwards to even
    integers. Returns ``None`` when it is smaller than :data:`MIN_ROI_PX`.
    """
    sw, sh = (int(sensor[0]) // 2 * 2, int(sensor[1]) // 2 * 2)

    def span(a: float, b: float, limit: int) -> tuple[int, int]:
        lo, hi = sorted((min(max(a, 0.0), limit), min(max(b, 0.0), limit)))
        start = math.floor(lo) // 2 * 2
        end = (math.ceil(hi) + 1) // 2 * 2
        return start, min(end, limit)

    x0, x1 = span(p0[0], p1[0], sw)
    y0, y1 = span(p0[1], p1[1], sh)
    if x1 - x0 < MIN_ROI_PX or y1 - y0 < MIN_ROI_PX:
        return None
    return x0, y0, x1 - x0, y1 - y0


def frame_to_qimage(
    image: np.ndarray, bits: int, auto_levels: bool = False
) -> tuple[QImage, np.ndarray]:
    """Linear grey QImage of *image*, plus the buffer that backs it.

    ``uint8`` data becomes ``Grayscale8`` and ``uint16`` data ``Grayscale16``.
    Levels are linear: fixed ``0 .. 2**bits - 1`` by default (a 12-bit frame is
    shifted up into the 16-bit range), or min to max with *auto_levels*. Every
    image pixel stays one pixel of the QImage.
    """
    img = np.ascontiguousarray(image)
    if img.ndim != 2:
        raise ValueError(f"expected a 2-D frame, got shape {img.shape}")
    is8 = img.dtype == np.uint8
    top = 255 if is8 else 65535
    fmt = QImage.Format.Format_Grayscale8 if is8 else QImage.Format.Format_Grayscale16
    if auto_levels:
        lo, hi = int(img.min()), int(img.max())
        if hi <= lo:
            data = np.zeros_like(img)
        else:
            scaled = (img.astype(np.float32) - lo) * (top / (hi - lo))
            data = np.clip(scaled, 0, top).astype(img.dtype)
    elif is8:
        data = (
            img
            if bits >= 8
            else (img.astype(np.uint16) << (8 - int(bits)))
            .clip(0, 255)
            .astype(np.uint8)
        )
    elif bits >= 16:
        data = img
    else:
        data = (
            (img.astype(np.uint32) << (16 - int(bits))).clip(0, 65535).astype(np.uint16)
        )
    data = np.ascontiguousarray(data)
    h, w = data.shape
    qimg = QImage(data.data, w, h, w * data.itemsize, fmt)
    return qimg, data


# ---------------------------------------------------------------------------
# the graphics view
# ---------------------------------------------------------------------------


class _View(QGraphicsView):
    """Wheel zoom around the cursor, drag pan, and the Draw ROI rubber band."""

    cursorMoved = Signal(object)  # QPointF (scene px) or None when it leaves
    rectDrawn = Signal(object, object)  # QPointF, QPointF
    zoomed = Signal()  # the user changed the zoom (leaves Fit mode)
    resized = Signal()

    def __init__(self, scene: QGraphicsScene) -> None:
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
        self.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.NoAnchor)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QBrush(QColor("#1c1c1c")))
        self.setFrameShape(QGraphicsView.Shape.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.draw_mode = False
        self._drag_start: QPointF | None = None
        self._rubber = QGraphicsRectItem()
        pen = QPen(QColor("#ffe14d"), 1.5, Qt.PenStyle.DashLine)
        pen.setCosmetic(True)
        self._rubber.setPen(pen)
        self._rubber.setZValue(50)
        self._rubber.setVisible(False)
        scene.addItem(self._rubber)

    @property
    def zoom(self) -> float:
        """View pixels per image pixel."""
        return self.transform().m11()

    def set_draw_mode(self, on: bool) -> None:
        """Switch between pan (hand) and rectangle drawing (cross cursor)."""
        self.draw_mode = on
        self._drag_start = None
        self._rubber.setVisible(False)
        self.setDragMode(
            QGraphicsView.DragMode.NoDrag
            if on
            else QGraphicsView.DragMode.ScrollHandDrag
        )
        self.viewport().setCursor(
            Qt.CursorShape.CrossCursor if on else Qt.CursorShape.ArrowCursor
        )

    def zoom_by(self, factor: float, at: QPoint | None = None) -> None:
        """Zoom by *factor* (clamped), keeping the point under *at* fixed.

        *at* is a viewport position; the default is the viewport centre.
        """
        if at is None:
            at = self.viewport().rect().center()
        new = min(max(self.zoom * factor, MIN_ZOOM), MAX_ZOOM)
        factor = new / self.zoom
        if abs(factor - 1.0) > 1e-9:
            anchor = self.mapToScene(at)
            self.scale(factor, factor)
            drift = self.mapFromScene(anchor) - at
            hbar, vbar = self.horizontalScrollBar(), self.verticalScrollBar()
            hbar.setValue(hbar.value() + drift.x())
            vbar.setValue(vbar.value() + drift.y())
        self.zoomed.emit()

    def wheelEvent(self, event: QWheelEvent) -> None:
        """Zoom around the cursor."""
        dy = event.angleDelta().y()
        if dy:
            self.zoom_by(1.25 ** (dy / 120.0), event.position().toPoint())
        event.accept()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        """Start a rectangle in Draw ROI mode, else pan."""
        if self.draw_mode and event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = self.mapToScene(event.position().toPoint())
            self._rubber.setRect(QRectF(self._drag_start, self._drag_start))
            self._rubber.setVisible(True)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        """Track the cursor; stretch the rectangle."""
        scene_pt = self.mapToScene(event.position().toPoint())
        self.cursorMoved.emit(scene_pt)
        if self._drag_start is not None:
            self._rubber.setRect(QRectF(self._drag_start, scene_pt).normalized())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        """Finish the rectangle."""
        if self._drag_start is not None and event.button() == Qt.MouseButton.LeftButton:
            start, self._drag_start = self._drag_start, None
            self._rubber.setVisible(False)
            self.rectDrawn.emit(start, self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event: Any) -> None:
        """Clear the pixel readout."""
        self.cursorMoved.emit(None)
        super().leaveEvent(event)

    def resizeEvent(self, event: QResizeEvent) -> None:
        """Let the window decide whether to re-fit."""
        super().resizeEvent(event)
        self.resized.emit()


# ---------------------------------------------------------------------------
# the window
# ---------------------------------------------------------------------------


class SpotLiveView(QWidget):
    """The "Spot camera" window: full-resolution live view with overlays.

    Parameters
    ----------
    service : SpotFocusService-like | None
        Anything with ``latest_frame(last_seq) -> LiveFrame | None``; set later
        with :meth:`set_service`.
    parent : QWidget | None
        Owner (the window stays top-level).
    """

    #: ``(x, y, w, h)`` drawn in Draw ROI mode (even, clamped to the sensor).
    roiDrawn = Signal(int, int, int, int)
    #: The window was closed by the user.
    windowClosed = Signal()
    #: ``(fps, max_value, saturated_percent)`` a few times a second.
    statsChanged = Signal(float, int, float)

    def __init__(
        self, service: Any | None = None, parent: QWidget | None = None
    ) -> None:
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle("Spot camera")
        self.resize(900, 700)
        self._service = service
        self._last_seq: int | None = None
        self._frame: LiveFrame | None = None
        self._sensor: tuple[int, int] | None = None
        self._buf: np.ndarray | None = None  # keeps the QImage buffer alive
        self._fit = True
        self._setpoint: tuple[float, float] | None = None
        self._cursor: QPointF | None = None
        self._poll_error_logged = False
        self.fps = 0.0
        self._fps_mark: tuple[int, float] | None = None
        self._last_new = 0.0
        self._last_stats = 0.0

        # -- scene ---------------------------------------------------------
        self.scene = QGraphicsScene(self)
        self._sensor_item = QGraphicsRectItem()
        self._sensor_item.setBrush(QBrush(QColor("#000000")))
        pen = QPen(QColor("#808080"), 1)
        pen.setCosmetic(True)
        self._sensor_item.setPen(pen)
        self._sensor_item.setZValue(-1)
        self.scene.addItem(self._sensor_item)

        self._pixmap_item = QGraphicsPixmapItem()
        self._pixmap_item.setTransformationMode(
            Qt.TransformationMode.FastTransformation
        )
        self._pixmap_item.setZValue(0)
        self.scene.addItem(self._pixmap_item)

        self._roi_item = QGraphicsRectItem()
        pen = QPen(QColor(ROI_COLOR), 1.5, Qt.PenStyle.DashLine)
        pen.setCosmetic(True)
        self._roi_item.setPen(pen)
        self._roi_item.setZValue(10)
        self._roi_item.setVisible(False)
        self.scene.addItem(self._roi_item)

        self._setpoint_item = QGraphicsEllipseItem(-10.0, -10.0, 20.0, 20.0)
        pen = QPen(QColor(SETPOINT_COLOR), 2)
        pen.setCosmetic(True)
        self._setpoint_item.setPen(pen)
        self._setpoint_item.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations
        )
        self._setpoint_item.setZValue(20)
        self._setpoint_item.setVisible(False)
        self.scene.addItem(self._setpoint_item)

        cross = QPainterPath()
        cross.moveTo(-9, 0)
        cross.lineTo(9, 0)
        cross.moveTo(0, -9)
        cross.lineTo(0, 9)
        self._spot_item = QGraphicsPathItem(cross)
        self._spot_item.setPen(QPen(QColor(OK_COLOR), 2))
        self._spot_item.setFlag(
            QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations
        )
        self._spot_item.setZValue(21)
        self._spot_item.setVisible(False)
        self.scene.addItem(self._spot_item)

        self.view = _View(self.scene)
        self.view.cursorMoved.connect(self._on_cursor)
        self.view.rectDrawn.connect(self._on_rect_drawn)
        self.view.zoomed.connect(self._on_user_zoom)
        self.view.resized.connect(self._on_resized)

        # -- toolbar -------------------------------------------------------
        self.fit_button = QPushButton("Fit")
        self.fit_button.setToolTip("Scale the whole sensor to the window")
        self.fit_button.clicked.connect(self.fit)
        self.one_to_one_button = QPushButton("1:1")
        self.one_to_one_button.setToolTip("One image pixel per screen pixel")
        self.one_to_one_button.clicked.connect(self.one_to_one)
        self.draw_roi_button = QPushButton("Draw ROI")
        self.draw_roi_button.setCheckable(True)
        self.draw_roi_button.setToolTip(
            "Drag a rectangle on the image to set the camera ROI"
        )
        self.draw_roi_button.toggled.connect(self.view.set_draw_mode)
        self.auto_levels = QCheckBox("Auto levels")
        self.auto_levels.setToolTip(
            "Stretch the display from the frame's min to its max (linear). "
            "Off: fixed 0 to full scale."
        )
        self.auto_levels.toggled.connect(lambda _: self._redraw())
        self.zoom_label = QLabel("")
        bar = QHBoxLayout()
        for w in (
            self.fit_button,
            self.one_to_one_button,
            self.draw_roi_button,
            self.auto_levels,
        ):
            bar.addWidget(w)
        bar.addStretch()
        bar.addWidget(self.zoom_label)

        self.status_label = QLabel("Waiting for the camera...")
        self.status_label.setTextFormat(Qt.TextFormat.PlainText)
        self.status_label.setMinimumWidth(10)
        self.cursor_label = QLabel("")
        self.cursor_label.setMinimumWidth(10)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addLayout(bar)
        layout.addWidget(self.view, 1)
        layout.addWidget(self.cursor_label)
        layout.addWidget(self.status_label)

        self._timer = QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self.poll)

    # ------------------------------------------------------------------
    # properties
    # ------------------------------------------------------------------

    def sizeHint(self) -> QSize:
        """Preferred window size."""
        return QSize(900, 700)

    @property
    def sensor(self) -> tuple[int, int] | None:
        """``(W, H)`` of the sensor, ``None`` before the first frame."""
        return self._sensor

    @property
    def frame(self) -> LiveFrame | None:
        """The frame on screen."""
        return self._frame

    @property
    def pixmap_item(self) -> QGraphicsPixmapItem:
        """The item that shows the current frame."""
        return self._pixmap_item

    @property
    def zoom(self) -> float:
        """View pixels per image pixel (device-independent)."""
        return self.view.zoom

    @property
    def fit_mode(self) -> bool:
        """Whether the view re-fits the sensor when the window is resized."""
        return self._fit

    def set_service(self, service: Any | None) -> None:
        """Poll *service* for frames (``None`` stops polling and keeps the image)."""
        self._service = service
        self._last_seq = None
        self._fps_mark = None
        self._poll_error_logged = False

    def set_setpoint(self, xy: tuple[float, float] | None) -> None:
        """Mark the calibrated setpoint (sensor px), or hide the ring."""
        self._setpoint = xy
        self._place_setpoint()

    def clear(self) -> None:
        """Forget the image and overlays (the camera went away)."""
        self._frame = None
        self._buf = None
        self._last_seq = None
        self._pixmap_item.setPixmap(QPixmap())
        self._spot_item.setVisible(False)
        self._roi_item.setVisible(False)
        self.status_label.setText("Waiting for the camera...")
        self.cursor_label.setText("")

    # ------------------------------------------------------------------
    # window events
    # ------------------------------------------------------------------

    def showEvent(self, event: Any) -> None:
        """Start polling."""
        super().showEvent(event)
        self._timer.start()

    def hideEvent(self, event: Any) -> None:
        """Stop polling."""
        self._timer.stop()
        super().hideEvent(event)

    def closeEvent(self, event: Any) -> None:
        """Tell the owner; the window is reused, not destroyed."""
        self._timer.stop()
        self.draw_roi_button.setChecked(False)
        super().closeEvent(event)
        self.windowClosed.emit()

    # ------------------------------------------------------------------
    # frames
    # ------------------------------------------------------------------

    def poll(self) -> bool:
        """Fetch the newest frame and repaint if it is new; ``True`` if it was."""
        svc = self._service
        if svc is None:
            return False
        try:
            frame = svc.latest_frame(self._last_seq)
        except Exception:
            if not self._poll_error_logged:
                logger.warning("spot live view: latest_frame failed", exc_info=True)
                self._poll_error_logged = True
            return False
        now = time.monotonic()
        if frame is None:
            if self._frame is not None and now - self._last_new > STALE_S:
                self._set_status(stale=True)
            return False
        self.show_frame(frame)
        return True

    def show_frame(self, frame: LiveFrame) -> None:
        """Draw *frame* at its ROI offset, at native size."""
        now = time.monotonic()
        self._frame = frame
        self._last_seq = frame.seq
        self._last_new = now
        self._poll_error_logged = False
        sensor = (int(frame.sensor[0]), int(frame.sensor[1]))
        if sensor != self._sensor:
            self._set_sensor(sensor)
        self._update_fps(frame.seq, now)
        self._redraw()
        self._place_overlays(frame)
        self._set_status()
        self._update_cursor_label()
        if now - self._last_stats >= _STATS_PERIOD_S:
            self._last_stats = now
            self.statsChanged.emit(
                self.fps, int(frame.max_value), float(frame.saturated_frac) * 100.0
            )

    def _update_fps(self, seq: int, now: float) -> None:
        if self._fps_mark is None:
            self._fps_mark = (seq, now)
            return
        seq0, t0 = self._fps_mark
        if now - t0 >= 0.5:
            self.fps = max(seq - seq0, 0) / (now - t0)
            self._fps_mark = (seq, now)

    def _redraw(self) -> None:
        """(Re)build the pixmap from the current frame and levels."""
        frame = self._frame
        if frame is None:
            return
        qimg, self._buf = frame_to_qimage(
            frame.image, int(frame.bits), self.auto_levels.isChecked()
        )
        roi_x, roi_y = int(frame.roi[0]), int(frame.roi[1])
        self._pixmap_item.setPixmap(QPixmap.fromImage(qimg))
        self._pixmap_item.setPos(roi_x, roi_y)

    def _set_sensor(self, sensor: tuple[int, int]) -> None:
        first = self._sensor is None
        self._sensor = sensor
        rect = QRectF(0, 0, sensor[0], sensor[1])
        self.scene.setSceneRect(rect)
        # The view may pan past the sensor edge, so a spot near a corner can
        # still be centred; Fit and 1:1 use the sensor rect itself.
        margin = max(sensor) / 2
        self.view.setSceneRect(rect.adjusted(-margin, -margin, margin, margin))
        self._sensor_item.setRect(rect)
        self._place_setpoint()
        if first or self._fit:
            self.fit()

    def _place_overlays(self, frame: LiveFrame) -> None:
        x, y, w, h = (int(v) for v in frame.roi)
        sw, sh = frame.sensor
        full = (x, y, w, h) == (0, 0, int(sw), int(sh))
        self._roi_item.setRect(QRectF(x, y, w, h))
        self._roi_item.setVisible(not full)
        if frame.spot is None:
            self._spot_item.setVisible(False)
        else:
            self._spot_item.setPos(frame.spot[0] + 0.5, frame.spot[1] + 0.5)
            color = OK_COLOR if frame.spot_ok else BAD_COLOR
            self._spot_item.setPen(QPen(QColor(color), 2))
            self._spot_item.setVisible(True)

    def _place_setpoint(self) -> None:
        sp = self._setpoint
        if sp is None:
            self._setpoint_item.setVisible(False)
            return
        self._setpoint_item.setPos(sp[0] + 0.5, sp[1] + 0.5)
        self._setpoint_item.setVisible(True)

    def _set_status(self, stale: bool = False) -> None:
        frame = self._frame
        if frame is None:
            return
        x, y, w, h = (int(v) for v in frame.roi)
        parts = [
            f"{self.fps:.1f} fps" if self.fps else "- fps",
            f"max {int(frame.max_value)}",
            f"{float(frame.saturated_frac) * 100.0:.2f}% saturated",
            f"{int(frame.bits)}-bit",
            f"ROI {w}x{h} at ({x}, {y})",
        ]
        if frame.spot is not None:
            parts.append(
                f"spot ({frame.spot[0]:.1f}, {frame.spot[1]:.1f})"
                + ("" if frame.spot_ok else " (not ok)")
            )
        text = "  |  ".join(parts)
        if stale:
            text = "No new frames.  " + text
        self.status_label.setText(text)

    # ------------------------------------------------------------------
    # cursor readout
    # ------------------------------------------------------------------

    def pixel_at(self, x: int, y: int) -> int | None:
        """Raw value of sensor pixel ``(x, y)``, ``None`` outside the frame."""
        frame = self._frame
        if frame is None:
            return None
        rx, ry, rw, rh = (int(v) for v in frame.roi)
        if not (rx <= x < rx + rw and ry <= y < ry + rh):
            return None
        return int(frame.image[y - ry, x - rx])

    def _on_cursor(self, pt: QPointF | None) -> None:
        self._cursor = pt
        self._update_cursor_label()

    def _update_cursor_label(self) -> None:
        pt, sensor = self._cursor, self._sensor
        if pt is None or sensor is None:
            self.cursor_label.setText("")
            return
        x, y = math.floor(pt.x()), math.floor(pt.y())
        if not (0 <= x < sensor[0] and 0 <= y < sensor[1]):
            self.cursor_label.setText("")
            return
        value = self.pixel_at(x, y)
        self.cursor_label.setText(
            f"x {x}  y {y}  value {'-' if value is None else value}"
        )

    # ------------------------------------------------------------------
    # navigation
    # ------------------------------------------------------------------

    def fit(self) -> None:
        """Scale the whole sensor into the window."""
        self._fit = True
        if self._sensor is None:
            return
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        self._update_zoom_label()

    def one_to_one(self) -> None:
        """One image pixel per screen pixel, centred on the ROI (or the spot)."""
        self._fit = False
        dpr = max(self.devicePixelRatioF(), 1.0)
        self.view.resetTransform()
        if dpr != 1.0:
            self.view.scale(1.0 / dpr, 1.0 / dpr)
        frame = self._frame
        if frame is not None:
            if frame.spot is not None:
                self.view.centerOn(frame.spot[0] + 0.5, frame.spot[1] + 0.5)
            else:
                x, y, w, h = (int(v) for v in frame.roi)
                self.view.centerOn(x + w / 2, y + h / 2)
        self._update_zoom_label()

    def _on_user_zoom(self) -> None:
        self._fit = False
        self._update_zoom_label()

    def _on_resized(self) -> None:
        if self._fit:
            self.fit()

    def _update_zoom_label(self) -> None:
        dpr = max(self.devicePixelRatioF(), 1.0)
        self.zoom_label.setText(f"{self.zoom * dpr * 100:.0f}%")

    # ------------------------------------------------------------------
    # Draw ROI
    # ------------------------------------------------------------------

    def _on_rect_drawn(self, p0: QPointF, p1: QPointF) -> None:
        if self._sensor is None:
            return
        roi = snap_roi((p0.x(), p0.y()), (p1.x(), p1.y()), self._sensor)
        if roi is None:  # a click or a sliver: stay in Draw mode
            return
        self.draw_roi_button.setChecked(False)
        self.roiDrawn.emit(*roi)
