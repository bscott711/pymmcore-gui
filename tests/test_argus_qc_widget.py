from __future__ import annotations

from typing import TYPE_CHECKING

from pymmcore_gui.widgets._argus_qc import (
    ArgusQCWidget,
    advice_lines,
    margins_text,
    verdict_text,
)

if TYPE_CHECKING:
    from pytestqt.qtbot import QtBot

RAW = {
    "seq": 7,
    "t": 12,
    "stage": "raw",
    "verdict": "act",
    "flags": ["clipped_scan_high"],
    "advice": [
        {"action": "scan_center", "when": "now", "text": "Shift the scan window."},
        {"action": "more_slices", "when": "next_run", "text": "Add 3 slices."},
    ],
    "metrics": {
        "margins_um": {
            "depth": {"low": 26.74, "high": 9.07},
            "scan": {"low": 20.5, "high": 0.0},
            "lateral": {"low": 40.0, "high": 138.0},
        }
    },
}


def test_formatting() -> None:
    assert verdict_text(RAW) == "T=12  ACT  ·  clipped_scan_high"
    assert advice_lines(RAW) == [
        "[now] Shift the scan window.",
        "[next run] Add 3 slices.",
    ]
    assert margins_text(RAW) == (
        "Margins (low / high): depth 26.7 / 9.1 um · scan 20.5 / 0.0 um · "
        "lateral 40.0 / 138.0 um"
    )
    assert margins_text({}) == ""


def test_widget_shows_the_newest_verdict_and_keeps_raw_metrics(qtbot: QtBot) -> None:
    w = ArgusQCWidget()
    qtbot.addWidget(w)
    w.update_qc(RAW)
    assert "ACT" in w._verdict.text()
    assert w._advice.count() == 2
    # The deskewed-volume verdict adds a box but no metrics: margins stay.
    w.update_qc({"t": 12, "stage": "dsr", "verdict": "act", "boxes": {}})
    assert "scan 20.5 / 0.0 um" in w._margins.text()
    w.update_qc({"t": 13, "stage": "raw", "verdict": "ok", "flags": [], "advice": []})
    assert w._advice.item(0).text() == "(none)"
    assert w._history.text().count("●") == 2
