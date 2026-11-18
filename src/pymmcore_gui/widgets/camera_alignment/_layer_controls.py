"""Per-layer controls (visibility, colour, contrast, pick) for the alignment overlay."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_gui._qt.QtCore import QSize, Signal
from pymmcore_gui._qt.QtGui import QColor, QIcon, QPixmap
from pymmcore_gui._qt.QtWidgets import (
    QCheckBox,
    QColorDialog,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui.widgets.camera_alignment._layers import PALETTE

if TYPE_CHECKING:
    from collections.abc import Mapping, Sequence

    from pymmcore_gui.widgets.camera_alignment._layers import LayerDisplay, LayerSpec

_MAX_VALUE = 65535
_CUSTOM_LABEL = "Custom..."
_STATUS_STYLE = "color: #c0392b; font-style: italic;"


def _swatch(color: str) -> QIcon:
    pix = QPixmap(14, 14)
    pix.fill(QColor(color))
    return QIcon(pix)


def _norm(color: str) -> str:
    """Normalise a colour string to lowercase ``#rrggbb``."""
    return QColor(color).name().lower()


class LayerControlsRow(QWidget):
    """One compact controls row for a single overlay layer.

    Parameters
    ----------
    spec : LayerSpec
        The layer this row controls.
    display : LayerDisplay
        Initial display state (no signals are emitted while applying it).
    parent : QWidget | None
        Parent widget.
    """

    visibilityChanged = Signal(str, bool)
    """``(key, visible)``."""
    colorChanged = Signal(str, str)
    """``(key, hex colour)``."""
    climChanged = Signal(str, object)
    """``(key, (lo, hi))``, or ``(key, None)`` for auto contrast."""
    pickRequested = Signal(str)
    """``(key)``: the user wants to pick a point on this layer."""

    def __init__(
        self,
        spec: LayerSpec,
        display: LayerDisplay,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._key = spec.key
        self._color = _norm(display.color)
        self._updating = False

        self._visible_cb = QCheckBox()
        self._visible_cb.setToolTip("Show this layer")
        self._visible_cb.setChecked(display.visible)

        self._name_label = QLabel(spec.name)
        self._name_label.setMinimumWidth(90)

        self._color_combo = QComboBox()
        self._color_combo.setIconSize(QSize(14, 14))
        for name, hex_ in PALETTE.items():
            self._color_combo.addItem(_swatch(hex_), name, hex_)
        self._color_combo.addItem(_CUSTOM_LABEL, None)

        self._auto_cb = QCheckBox("Auto")
        self._auto_cb.setToolTip("Auto contrast")
        self._auto_cb.setChecked(display.auto_contrast)

        self._min_spin = self._make_spin(display.clim[0])
        self._max_spin = self._make_spin(display.clim[1])
        self._min_spin.setToolTip("Minimum")
        self._max_spin.setToolTip("Maximum")
        self._sync_spin_enabled()

        self._pick_btn = QPushButton("Pick")
        self._pick_btn.setToolTip("Click a point on this layer to align it")

        self._status_label = QLabel()
        self._status_label.setStyleSheet(_STATUS_STYLE)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 0, 2, 0)
        for w in (
            self._visible_cb,
            self._name_label,
            self._color_combo,
            self._min_spin,
            self._max_spin,
            self._auto_cb,
            self._pick_btn,
            self._status_label,
        ):
            layout.addWidget(w)
        layout.addStretch(1)

        self.set_color(display.color)

        self._visible_cb.toggled.connect(self._on_visible_toggled)
        self._color_combo.currentIndexChanged.connect(self._on_color_index_changed)
        self._auto_cb.toggled.connect(self._on_auto_toggled)
        self._min_spin.valueChanged.connect(self._on_spin_changed)
        self._max_spin.valueChanged.connect(self._on_spin_changed)
        self._pick_btn.clicked.connect(lambda: self.pickRequested.emit(self._key))

    @property
    def key(self) -> str:
        """The layer key this row controls."""
        return self._key

    # ------------------------------------------------------------------ public

    def set_auto_range(self, lo: float, hi: float) -> None:
        """Show the live auto range in the spin boxes without emitting signals.

        Only has an effect while Auto is checked.

        Parameters
        ----------
        lo : float
            Current automatic minimum.
        hi : float
            Current automatic maximum.
        """
        if not self._auto_cb.isChecked():
            return
        self._set_spins(lo, hi)

    def set_status(self, text: str) -> None:
        """Show a short per-layer hint (e.g. ``"outside frame"``); ``""`` clears.

        Parameters
        ----------
        text : str
            Hint text, also used as the tooltip.
        """
        self._status_label.setText(text)
        self._status_label.setToolTip(text)

    def set_pick_enabled(self, enabled: bool) -> None:
        """Enable or disable the Pick button.

        Parameters
        ----------
        enabled : bool
            Whether the button is clickable.
        """
        self._pick_btn.setEnabled(enabled)

    def set_color(self, color: str) -> None:
        """Programmatically select a colour without emitting ``colorChanged``.

        Parameters
        ----------
        color : str
            Hex colour. Colours not in the palette get a ``Custom (#rrggbb)``
            entry.
        """
        self._color = _norm(color)
        self._updating = True
        try:
            self._select_color_item(self._color)
        finally:
            self._updating = False
        self._name_label.setStyleSheet(f"color: {self._color}; font-weight: bold;")

    # ----------------------------------------------------------------- private

    def _ask_custom_color(self, initial: str) -> str | None:
        """Open a colour dialog; return the chosen hex colour or ``None``."""
        chosen = QColorDialog.getColor(QColor(initial), self)
        return _norm(chosen.name()) if chosen.isValid() else None

    @staticmethod
    def _make_spin(value: float) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(0, _MAX_VALUE)
        spin.setKeyboardTracking(False)
        spin.setValue(round(value))
        spin.setMinimumWidth(64)
        return spin

    def _set_spins(self, lo: float, hi: float) -> None:
        for spin, val in ((self._min_spin, lo), (self._max_spin, hi)):
            spin.blockSignals(True)
            spin.setValue(max(0, min(_MAX_VALUE, round(val))))
            spin.blockSignals(False)

    def _sync_spin_enabled(self) -> None:
        manual = not self._auto_cb.isChecked()
        self._min_spin.setEnabled(manual)
        self._max_spin.setEnabled(manual)

    def _select_color_item(self, color: str) -> None:
        """Select (inserting if needed) the combo entry for *color*."""
        combo = self._color_combo
        for i in range(combo.count()):
            data = combo.itemData(i)
            if data is not None and _norm(str(data)) == color:
                combo.setCurrentIndex(i)
                return
        # A custom colour: reuse a previous "Custom (#...)" entry or add one
        # just before the trailing "Custom..." item.
        custom_index = combo.count() - 1
        for i in range(len(PALETTE), custom_index):
            if _norm(str(combo.itemData(i))) == color:
                combo.setCurrentIndex(i)
                return
        combo.insertItem(custom_index, _swatch(color), f"Custom ({color})", color)
        combo.setCurrentIndex(custom_index)

    def _on_visible_toggled(self, checked: bool) -> None:
        self.visibilityChanged.emit(self._key, checked)

    def _on_color_index_changed(self, index: int) -> None:
        if self._updating or index < 0:
            return
        data = self._color_combo.itemData(index)
        if data is None:  # the trailing "Custom..." entry
            chosen = self._ask_custom_color(self._color)
            if chosen is None:
                self.set_color(self._color)  # cancelled: revert
                return
            self.set_color(chosen)
        else:
            self.set_color(str(data))
        self.colorChanged.emit(self._key, self._color)

    def _on_auto_toggled(self, checked: bool) -> None:
        self._sync_spin_enabled()
        if checked:
            self.climChanged.emit(self._key, None)
        else:
            self.climChanged.emit(self._key, self._current_clim())

    def _on_spin_changed(self, _value: int) -> None:
        if self._auto_cb.isChecked():
            return
        lo, hi = self._min_spin.value(), self._max_spin.value()
        if hi <= lo:
            if lo < _MAX_VALUE:
                hi = lo + 1
            else:
                lo, hi = _MAX_VALUE - 1, _MAX_VALUE
            self._set_spins(lo, hi)
        self.climChanged.emit(self._key, (float(lo), float(hi)))

    def _current_clim(self) -> tuple[float, float]:
        return float(self._min_spin.value()), float(self._max_spin.value())


class LayerControlsPanel(QWidget):
    """A vertical stack of :class:`LayerControlsRow` widgets.

    Parameters
    ----------
    parent : QWidget | None
        Parent widget.
    """

    visibilityChanged = Signal(str, bool)
    """``(key, visible)``."""
    colorChanged = Signal(str, str)
    """``(key, hex colour)``."""
    climChanged = Signal(str, object)
    """``(key, (lo, hi))``, or ``(key, None)`` for auto contrast."""
    pickRequested = Signal(str)
    """``(key)``: the user wants to pick a point on this layer."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._rows: list[LayerControlsRow] = []
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(2)
        self._layout.addStretch(1)

    def set_layers(
        self, specs: Sequence[LayerSpec], displays: Mapping[str, LayerDisplay]
    ) -> None:
        """Replace all rows with one row per spec.

        Parameters
        ----------
        specs : Sequence[LayerSpec]
            Layers to show, in display order.
        displays : Mapping[str, LayerDisplay]
            Display state keyed by ``spec.key``; every spec must be present.
        """
        for old in self._rows:
            self._layout.removeWidget(old)
            old.hide()
            old.setParent(None)
            old.deleteLater()
        self._rows = []
        for spec in specs:
            row = LayerControlsRow(spec, displays[spec.key], self)
            row.visibilityChanged.connect(self.visibilityChanged)
            row.colorChanged.connect(self.colorChanged)
            row.climChanged.connect(self.climChanged)
            row.pickRequested.connect(self.pickRequested)
            self._layout.insertWidget(self._layout.count() - 1, row)
            self._rows.append(row)

    def row(self, key: str) -> LayerControlsRow | None:
        """Return the row for *key*, or ``None``.

        Parameters
        ----------
        key : str
            Layer key.
        """
        return next((r for r in self._rows if r.key == key), None)

    def rows(self) -> list[LayerControlsRow]:
        """Return the current rows in display order."""
        return list(self._rows)


__all__ = ["LayerControlsPanel", "LayerControlsRow"]
