"""Unit tests for camera_worker.py's pure message-dispatch helpers.

``_drain_incoming`` and ``_wait_for_free_slot`` take only a
``Connection``-like object and a plain ``list[int]`` -- no real camera,
shared memory, or subprocess -- so their ``StopCmd``/``ShutdownCmd``
dispatch and priority logic is directly unit-testable with a mocked
connection, unlike ``_drain_sequence``/``run_camera_worker``, which need a
real ``SharedMemory`` segment and are left to bench/integration
verification (consistent with this package's existing convention).
"""

from __future__ import annotations

from unittest.mock import MagicMock

from pymmcore_gui.asi_z_stack.camera_worker import _drain_incoming, _wait_for_free_slot
from pymmcore_gui.asi_z_stack.worker_messages import ShutdownCmd, SlotFreeCmd, StopCmd


def _mock_conn(messages: list[object]) -> MagicMock:
    """Build a mocked ``Connection`` whose ``poll``/``recv`` drain *messages* in order.

    Parameters
    ----------
    messages : list[object]
        Messages to hand back from successive ``recv()`` calls; ``poll()``
        reports messages available until this queue is exhausted.
    """
    conn = MagicMock()
    remaining = list(messages)
    conn.poll.side_effect = lambda timeout=0.0: bool(remaining)
    conn.recv.side_effect = lambda: remaining.pop(0)
    return conn


def test_drain_incoming_collects_slot_free_and_returns_none() -> None:
    """Only ``SlotFreeCmd`` messages: all collected, no stop signal."""
    conn = _mock_conn([SlotFreeCmd(0), SlotFreeCmd(1)])
    free_slots: list[int] = []

    assert _drain_incoming(conn, free_slots) is None
    assert free_slots == [0, 1]


def test_drain_incoming_returns_stop_cmd() -> None:
    """A ``StopCmd`` amid ``SlotFreeCmd``s is reported, and slots still collected."""
    conn = _mock_conn([SlotFreeCmd(0), StopCmd()])
    free_slots: list[int] = []

    assert _drain_incoming(conn, free_slots) is StopCmd
    assert free_slots == [0]


def test_drain_incoming_shutdown_wins_over_stop_regardless_of_order() -> None:
    """``ShutdownCmd`` always outranks a ``StopCmd`` seen in the same drain pass."""
    assert _drain_incoming(_mock_conn([StopCmd(), ShutdownCmd()]), []) is ShutdownCmd
    assert _drain_incoming(_mock_conn([ShutdownCmd(), StopCmd()]), []) is ShutdownCmd


def test_wait_for_free_slot_returns_none_once_slot_frees() -> None:
    """Blocks until a ``SlotFreeCmd`` arrives, then returns ``None``."""
    conn = _mock_conn([SlotFreeCmd(3)])
    free_slots: list[int] = []

    assert _wait_for_free_slot(conn, free_slots, "cam0") is None
    assert free_slots == [3]


def test_wait_for_free_slot_returns_shutdown_cmd() -> None:
    """A ``ShutdownCmd`` while waiting for a slot is reported, not swallowed.

    Regression test for the production deadlock: previously neither
    ``_wait_for_free_slot`` nor ``_drain_incoming`` recognized
    ``ShutdownCmd`` at all, so a worker blocked here waiting for a free
    ring-buffer slot would never be unblocked by ``shutdown_all()``.
    """
    assert _wait_for_free_slot(_mock_conn([ShutdownCmd()]), [], "cam0") is ShutdownCmd


def test_wait_for_free_slot_returns_stop_cmd() -> None:
    """A ``StopCmd`` while waiting for a slot is reported."""
    assert _wait_for_free_slot(_mock_conn([StopCmd()]), [], "cam0") is StopCmd
