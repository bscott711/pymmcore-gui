"""Tests for commit_headroom.py: the probe never raises; the margin rule."""

from __future__ import annotations

import ctypes
import sys
from typing import TYPE_CHECKING

from pymmcore_gui.asi_z_stack import commit_headroom as ch

if TYPE_CHECKING:
    import pytest

GIB = 1024**3
MIB = 1024**2


def test_margin_is_two_gib_or_eight_frames() -> None:
    assert ch.MIN_COMMIT_HEADROOM_BYTES == 2 * GIB
    assert ch.required_headroom_bytes(0) == 2 * GIB
    assert ch.required_headroom_bytes(int(3.18 * MIB)) == 2 * GIB
    # A frame big enough that 8 of them exceed 2 GiB takes over.
    assert ch.required_headroom_bytes(512 * MIB) == 8 * 512 * MIB
    assert ch.required_headroom_bytes(-5) == 2 * GIB


def test_available_commit_is_none_off_windows(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    assert ch.available_commit_bytes() is None
    assert ch.process_private_bytes() is None


def test_available_commit_never_raises_on_ctypes_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(sys, "platform", "win32")

    class _Boom:
        @property
        def kernel32(self) -> object:
            raise OSError("no kernel32")

    # ctypes.windll does not exist off Windows; raising=False creates it.
    monkeypatch.setattr(ctypes, "windll", _Boom(), raising=False)
    assert ch.available_commit_bytes() is None


def test_available_commit_reads_ull_avail_page_file(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(sys, "platform", "win32")

    class _Kernel32:
        def GlobalMemoryStatusEx(self, ref: object) -> int:
            status = ref._obj  # type: ignore[attr-defined]
            assert status.dwLength == ctypes.sizeof(ch._MemoryStatusEx)
            status.ullAvailPageFile = 123 * MIB
            return 1

    class _Windll:
        kernel32 = _Kernel32()

    monkeypatch.setattr(ctypes, "windll", _Windll(), raising=False)
    assert ch.available_commit_bytes() == 123 * MIB

    _Kernel32.GlobalMemoryStatusEx = lambda self, ref: 0  # type: ignore[method-assign]
    assert ch.available_commit_bytes() is None


def test_process_private_bytes_never_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setitem(sys.modules, "psutil", None)  # import psutil -> ImportError
    assert ch.process_private_bytes() is None


def test_error_message_has_free_and_required_mib() -> None:
    exc = ch.CommitHeadroomError(34 * MIB, 2048 * MIB)
    assert isinstance(exc, MemoryError)
    text = str(exc)
    assert "34 MiB free" in text
    assert "2048 MiB needed" in text
    assert "Close MDA viewer tabs" in text


def test_format_gib() -> None:
    assert ch.format_gib(None) == "n/a"
    assert ch.format_gib(3 * GIB // 2) == "1.5 GiB"


def test_module_does_not_import_qt() -> None:
    from pathlib import Path

    text = Path(ch.__file__).read_text()
    assert "PyQt" not in text
    assert "PySide" not in text
