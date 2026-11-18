from __future__ import annotations

import logging

from pymmcore_gui._exceptions import is_persistent_notice, report_background_exception


class _Persistent(Exception):
    persistent_notice = True


def test_is_persistent_notice_plain_error() -> None:
    assert is_persistent_notice(RuntimeError("boom")) is False


def test_is_persistent_notice_class_attribute() -> None:
    assert is_persistent_notice(_Persistent("x")) is True


def test_is_persistent_notice_follows_cause_and_context() -> None:
    wrapper = RuntimeError("wrapped")
    wrapper.__cause__ = _Persistent("inner")
    assert is_persistent_notice(wrapper) is True

    ctx = RuntimeError("during handling")
    ctx.__context__ = _Persistent("inner")
    assert is_persistent_notice(ctx) is True


def test_is_persistent_notice_survives_cycles() -> None:
    a, b = RuntimeError("a"), RuntimeError("b")
    a.__cause__ = b
    b.__context__ = a
    assert is_persistent_notice(a) is False


class _ListHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__(logging.DEBUG)
        self.records: list[logging.LogRecord] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.records.append(record)


def test_report_background_exception_logs_to_pymmcore_plus_logger() -> None:
    handler = _ListHandler()
    plus = logging.getLogger("pymmcore-plus")
    plus.addHandler(handler)
    try:
        try:
            raise ValueError("kaput")
        except ValueError as exc:
            report_background_exception(exc, context="MDA save")
    finally:
        plus.removeHandler(handler)

    errors = [r for r in handler.records if r.levelno >= logging.ERROR]
    assert errors, "nothing reached the pymmcore-plus logger"
    rec = errors[0]
    assert rec.exc_info is not None
    assert isinstance(rec.exc_info[1], ValueError)
    assert "MDA save" in rec.getMessage()
