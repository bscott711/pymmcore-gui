from __future__ import annotations

import importlib.metadata
from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock

from pymmcore_gui import _plugins

if TYPE_CHECKING:
    import pytest


class FakeEP:
    def __init__(self, name: str, obj: Any = None, load_error: Exception | None = None):
        self.name = name
        self._obj = obj
        self._load_error = load_error

    def load(self) -> Any:
        if self._load_error:
            raise self._load_error
        return self._obj


def _patch_eps(monkeypatch: pytest.MonkeyPatch, eps: list[FakeEP]) -> list[str]:
    groups: list[str] = []

    def fake_entry_points(**kwargs: Any) -> list[FakeEP]:
        groups.append(kwargs.get("group", ""))
        return eps

    monkeypatch.setattr(importlib.metadata, "entry_points", fake_entry_points)
    return groups


def test_plugin_called_with_window(monkeypatch: pytest.MonkeyPatch) -> None:
    win = MagicMock()
    plugin = MagicMock()
    groups = _patch_eps(monkeypatch, [FakeEP("good", plugin)])
    _plugins.load_plugins(win)
    plugin.assert_called_once_with(win)
    assert groups == ["pymmcore_gui.plugins"]
    win.nm.show_warning_message.assert_not_called()


def test_bad_plugin_does_not_block_others(monkeypatch: pytest.MonkeyPatch) -> None:
    win = MagicMock()
    raising = MagicMock(side_effect=RuntimeError("boom"))
    good = MagicMock()
    _patch_eps(
        monkeypatch,
        [
            FakeEP("import-fails", load_error=ImportError("nope")),
            FakeEP("raises", raising),
            FakeEP("good", good),
        ],
    )
    _plugins.load_plugins(win)
    raising.assert_called_once_with(win)
    good.assert_called_once_with(win)
    msgs = [c.args[0] for c in win.nm.show_warning_message.call_args_list]
    assert len(msgs) == 2
    assert "import-fails" in msgs[0]
    assert "raises" in msgs[1]


def test_no_plugins_is_noop(monkeypatch: pytest.MonkeyPatch) -> None:
    win = MagicMock()
    _patch_eps(monkeypatch, [])
    _plugins.load_plugins(win)
    assert win.mock_calls == []


def test_missing_notification_manager_is_tolerated(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    win = MagicMock()
    win.nm.show_warning_message.side_effect = AttributeError("no nm")
    good = MagicMock()
    _patch_eps(
        monkeypatch,
        [FakeEP("raises", MagicMock(side_effect=ValueError("x"))), FakeEP("g", good)],
    )
    _plugins.load_plugins(win)
    good.assert_called_once_with(win)
