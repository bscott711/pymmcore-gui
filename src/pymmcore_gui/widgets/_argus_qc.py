"""Argus live QC: what the processing server thinks of each timepoint.

Argus (``celldet-live-qc``) judges every streamed timepoint about a second
after it lands. It checks whether the cell is cut off by a face of the imaged
volume, drifting toward one, defocused or bleaching, and says what to change.
Those verdicts arrive as ``MSG_QC`` (see ``_argus_stream._protocol.QCHeader``).
This panel shows the newest one, its advice (tagged *now* when it can change
mid-run, *next run* when it changes the volume's shape), the cell's margin on
every face, and a strip of recent verdicts.

It is advisory: nothing here changes the acquisition.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from PyQt6.QtWidgets import QLabel, QListWidget, QVBoxLayout, QWidget

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping

VERDICT_COLORS = {
    "ok": "#2e9d4a",
    "warn": "#e08a00",
    "act": "#d0342c",
    "no_cell": "#808080",
}
HISTORY_LEN = 40


def verdict_text(rec: Mapping[str, Any]) -> str:
    text = f"T={rec.get('t', '?')}  {str(rec.get('verdict', '?')).upper()}"
    flags = rec.get("flags") or []
    if flags:
        text += "  ·  " + ", ".join(flags)
    return text


def advice_lines(rec: Mapping[str, Any]) -> list[str]:
    out = []
    for a in rec.get("advice") or []:
        when = "now" if a.get("when") == "now" else "next run"
        out.append(f"[{when}] {a.get('text', a.get('action', ''))}")
    return out


def margins_text(rec: Mapping[str, Any]) -> str:
    """``depth 26.7 / 9.1 um · scan 20.5 / 0.0 um · ...`` (low / high side)."""
    margins = (rec.get("metrics") or {}).get("margins_um") or {}
    parts = []
    for axis in ("depth", "scan", "lateral"):
        m = margins.get(axis)
        if m:
            parts.append(f"{axis} {m.get('low', 0):.1f} / {m.get('high', 0):.1f} um")
    return "Margins (low / high): " + " · ".join(parts) if parts else ""


def history_html(verdicts: Iterable[str]) -> str:
    dots = (
        f'<span style="color:{VERDICT_COLORS.get(v, "#808080")}">●</span>'
        for v in verdicts
    )
    return "".join(dots)


class ArgusQCWidget(QWidget):
    """Newest Argus QC verdict, its advice, and recent history."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._verdict = QLabel("Waiting for Argus QC…")
        self._verdict.setWordWrap(True)
        self._margins = QLabel("")
        self._margins.setWordWrap(True)
        self._advice = QListWidget()
        self._advice.setWordWrap(True)
        self._history = QLabel("")
        self._recent: list[str] = []
        self._raw_t: dict[int, Mapping[str, Any]] = {}

        layout = QVBoxLayout(self)
        layout.addWidget(self._verdict)
        layout.addWidget(self._margins)
        layout.addWidget(QLabel("Advice:"))
        layout.addWidget(self._advice)
        layout.addWidget(self._history)

    def update_qc(self, rec: Mapping[str, Any]) -> None:
        """Show one ``MSG_QC`` header (call on the GUI thread)."""
        if rec.get("stage") == "dsr":
            # The deskewed-volume verdict repeats the raw one and adds only a
            # box; keep the raw one's metrics on screen.
            rec = {**self._raw_t.get(rec.get("t", -1), {}), **rec}
        else:
            self._raw_t[rec.get("t", -1)] = rec
            self._raw_t = dict(list(self._raw_t.items())[-HISTORY_LEN:])
            self._recent = [*self._recent, str(rec.get("verdict", ""))][-HISTORY_LEN:]
        color = VERDICT_COLORS.get(str(rec.get("verdict")), "#808080")
        self._verdict.setText(verdict_text(rec))
        self._verdict.setStyleSheet(f"font-weight: bold; color: {color};")
        self._margins.setText(margins_text(rec))
        self._advice.clear()
        self._advice.addItems(advice_lines(rec) or ["(none)"])
        self._history.setText(history_html(self._recent))
