"""Lightweight, dependency-free live strip chart for the alignment displacement.

A fixed-length ring buffer of ``(t, dx, dy, magnitude)`` samples redrawn via
``paintEvent`` -- deliberately not built on a plotting library (no
pyqtgraph/matplotlib in this codebase): no zoom/pan/axis-editing, just a
scrolling trace of three values against elapsed time.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

from pymmcore_gui._qt.QtCore import QPointF, Qt
from pymmcore_gui._qt.QtGui import QColor, QPainter, QPaintEvent, QPen, QPolygonF
from pymmcore_gui._qt.QtWidgets import QWidget

if TYPE_CHECKING:
    from collections.abc import Callable

_DX_COLOR = "#ff5555"
_DY_COLOR = "#55aaff"
_MAG_COLOR = "#dddddd"
_MIN_RANGE = 5.0  # px -- floors the y-range so a converged trace isn't razor-thin


@dataclass(frozen=True, slots=True)
class _Sample:
    t: float
    dx: float
    dy: float
    mag: float


class DisplacementStripChart(QWidget):
    """Scrolling line plot of dx/dy/magnitude (px) over recent wall-clock time.

    Parameters
    ----------
    parent : QWidget | None
        Optional parent widget. By default, None.
    max_samples : int
        Ring-buffer capacity; older samples are dropped (bounded memory, no
        persistence). By default, 600.
    """

    def __init__(
        self, parent: QWidget | None = None, *, max_samples: int = 600
    ) -> None:
        super().__init__(parent)
        self._samples: deque[_Sample] = deque(maxlen=max_samples)
        self._t0: float | None = None
        self.setMinimumHeight(120)

    def add_sample(self, dx: float, dy: float, mag: float) -> None:
        """Append one ``(dx, dy, magnitude)`` sample (px) and schedule a repaint."""
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        self._samples.append(_Sample(now - self._t0, dx, dy, mag))
        self.update()

    def clear(self) -> None:
        """Discard all samples, e.g. when spots are re-picked."""
        self._samples.clear()
        self._t0 = None
        self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        if len(self._samples) < 2:
            painter.end()
            return

        vals = [v for s in self._samples for v in (s.dx, s.dy, s.mag)]
        lo, hi = min(vals), max(vals)
        if hi - lo < _MIN_RANGE:
            mid = (hi + lo) / 2
            lo, hi = mid - _MIN_RANGE / 2, mid + _MIN_RANGE / 2
        t_span = max(self._samples[-1].t - self._samples[0].t, 1e-6)

        def _pt(t: float, v: float) -> QPointF:
            x = rect.left() + (t / t_span) * rect.width()
            y = rect.bottom() - ((v - lo) / (hi - lo)) * rect.height()
            return QPointF(x, y)

        painter.setPen(QPen(QColor("#555555"), 1, Qt.PenStyle.DashLine))
        zero_y = _pt(0, 0).y()
        painter.drawLine(QPointF(rect.left(), zero_y), QPointF(rect.right(), zero_y))

        getters: tuple[tuple[str, Callable[[_Sample], float]], ...] = (
            (_DX_COLOR, lambda s: s.dx),
            (_DY_COLOR, lambda s: s.dy),
            (_MAG_COLOR, lambda s: s.mag),
        )
        for color, getter in getters:
            painter.setPen(QPen(QColor(color), 1.5))
            poly = QPolygonF([_pt(s.t, getter(s)) for s in self._samples])
            painter.drawPolyline(poly)

        painter.setPen(QColor(_DX_COLOR))
        painter.drawText(4, 12, "dx")
        painter.setPen(QColor(_DY_COLOR))
        painter.drawText(28, 12, "dy")
        painter.setPen(QColor(_MAG_COLOR))
        painter.drawText(52, 12, "|d|")
        painter.end()
