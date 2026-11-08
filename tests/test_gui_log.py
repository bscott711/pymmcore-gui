from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from typing import TYPE_CHECKING

import pytest

from pymmcore_gui import _gui_log

if TYPE_CHECKING:
    from pathlib import Path


@pytest.fixture
def clean_gui_logger() -> object:
    logger = logging.getLogger("pymmcore_gui")
    old_level = logger.level
    before = list(logger.handlers)
    yield logger
    for h in list(logger.handlers):
        if h not in before:
            logger.removeHandler(h)
            h.close()
    logger.setLevel(old_level)


def _file_handlers(logger: logging.Logger) -> list[RotatingFileHandler]:
    return [h for h in logger.handlers if isinstance(h, RotatingFileHandler)]


def test_install_gui_file_log_writes_info_and_is_idempotent(
    tmp_path: Path, clean_gui_logger: logging.Logger
) -> None:
    logger = clean_gui_logger
    n_before = len(_file_handlers(logger))
    path = _gui_log.install_gui_file_log(tmp_path)
    assert path == tmp_path / "pymmcore-gui.log"
    assert path is not None
    again = _gui_log.install_gui_file_log(tmp_path)
    assert again == path
    assert len(_file_handlers(logger)) == n_before + 1

    logging.getLogger("pymmcore_gui.x").info("hello persistent log")
    for h in _file_handlers(logger):
        h.flush()
    text = path.read_text(encoding="utf-8")
    assert "hello persistent log" in text
    assert "pymmcore_gui.x" in text
    assert "[INFO]" in text


def test_install_gui_file_log_oserror_returns_none(
    tmp_path: Path, clean_gui_logger: logging.Logger
) -> None:
    blocker = tmp_path / "file"
    blocker.write_text("x")
    # a directory below a regular file cannot be created
    assert _gui_log.install_gui_file_log(blocker / "sub") is None
