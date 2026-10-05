"""Unit tests for camera_worker.py's pure message-dispatch helpers, plus
``_drain_live``'s newest-frame/rate-cap/in-flight-cap logic.

``_drain_incoming`` and ``_wait_for_free_slot`` take only a
``Connection``-like object and a plain ``list[int]`` -- no real camera,
shared memory, or subprocess -- so their ``StopCmd``/``ShutdownCmd``
dispatch and priority logic is directly unit-testable with a mocked
connection. ``_drain_live`` is unit-tested too, against a real
``multiprocessing.shared_memory.SharedMemory`` segment and small fake
camera/connection stand-ins (see ``_FakeLiveCore``/``_FakeConn`` below) --
unlike ``_drain_sequence``/``run_camera_worker``, which are left to
bench/integration verification (consistent with this package's existing
convention).
"""

from __future__ import annotations

import time
from multiprocessing.shared_memory import SharedMemory
from typing import TYPE_CHECKING
from unittest.mock import MagicMock

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.camera_worker import (
    _LIVE_MAX_IN_FLIGHT,
    CameraWorkerConfig,
    _drain_incoming,
    _drain_live,
    _wait_for_free_slot,
)
from pymmcore_gui.asi_z_stack.worker_messages import (
    ErrorMsg,
    FrameMsg,
    ShutdownCmd,
    SlotFreeCmd,
    StalledMsg,
    StopCmd,
    StoppedMsg,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

# A tiny frame shape keeps the shared-memory segment and every pixel
# comparison below cheap -- the size/dtype aren't what's under test here.
_SHAPE = (4, 4)
_DTYPE = np.uint16
_SLOT_NBYTES = int(np.prod(_SHAPE)) * np.dtype(_DTYPE).itemsize


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

    assert isinstance(_drain_incoming(conn, free_slots), StopCmd)
    assert free_slots == [0]


def test_drain_incoming_shutdown_wins_over_stop_regardless_of_order() -> None:
    """``ShutdownCmd`` always outranks a ``StopCmd`` seen in the same drain pass."""
    assert isinstance(
        _drain_incoming(_mock_conn([StopCmd(), ShutdownCmd()]), []), ShutdownCmd
    )
    assert isinstance(
        _drain_incoming(_mock_conn([ShutdownCmd(), StopCmd()]), []), ShutdownCmd
    )


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
    assert isinstance(
        _wait_for_free_slot(_mock_conn([ShutdownCmd()]), [], "cam0"), ShutdownCmd
    )


def test_wait_for_free_slot_returns_stop_cmd() -> None:
    """A ``StopCmd`` while waiting for a slot is reported."""
    assert isinstance(_wait_for_free_slot(_mock_conn([StopCmd()]), [], "cam0"), StopCmd)


def test_drain_incoming_keeps_latest_stop_token() -> None:
    stop = _drain_incoming(_mock_conn([StopCmd(1), StopCmd(2)]), [])
    assert isinstance(stop, StopCmd)
    assert stop.token == 2


def _trigger_core(current: str, allowed: tuple[str, ...]) -> MagicMock:
    core = MagicMock()
    core.hasProperty.return_value = True
    core.getAllowedPropertyValues.return_value = allowed
    core.getProperty.return_value = current
    return core


def test_internal_trigger_mode_prefers_pre_handoff_mode() -> None:
    from pymmcore_gui.asi_z_stack.camera_worker import _internal_trigger_mode

    allowed = ("Internal Trigger", "Timed", "Edge Trigger", "Level Trigger")
    core = _trigger_core("Level Trigger", allowed)
    assert _internal_trigger_mode(core, "Camera-1", "Timed") == "Timed"
    # An external snapshot value never counts as the free-running mode.
    assert _internal_trigger_mode(core, "Camera-1", "Level Trigger") == (
        "Internal Trigger"
    )
    assert _internal_trigger_mode(core, "Camera-1", None) == "Internal Trigger"
    assert (
        _internal_trigger_mode(_trigger_core("x", ("Edge Trigger",)), "c", None) is None
    )


def test_set_trigger_mode_skips_noop() -> None:
    from pymmcore_gui.asi_z_stack.camera_worker import _set_trigger_mode

    core = _trigger_core("Internal Trigger", ())
    _set_trigger_mode(core, "Camera-1", "Internal Trigger")
    core.setProperty.assert_not_called()
    _set_trigger_mode(core, "Camera-1", "Level Trigger")
    core.setProperty.assert_called_once_with("Camera-1", "TriggerMode", "Level Trigger")
    core.setProperty.reset_mock()
    _set_trigger_mode(core, "Camera-1", None)
    core.setProperty.assert_not_called()


# ---------------------------------------------------------------------------
# _drain_live
# ---------------------------------------------------------------------------


def _frame(value: int) -> np.ndarray:
    """A small, distinctly-valued frame so a slot's contents reveal which one landed."""
    return np.full(_SHAPE, value, dtype=_DTYPE)


def _read_slot(shm: SharedMemory, slot_index: int) -> np.ndarray:
    """Copy one shared-memory slot back out as a frame-shaped array."""
    offset = slot_index * _SLOT_NBYTES
    arr: np.ndarray = np.ndarray(_SHAPE, dtype=_DTYPE, buffer=shm.buf, offset=offset)
    return arr.copy()


class _FakeLiveCore:
    """Minimal camera stand-in for ``_drain_live`` -- no real MMCore involved.

    ``frames`` models the circular buffer's current backlog, oldest first
    (``frames[-1]`` is the newest, matching real PVCAM/MMCore order). When
    *infinite* is set, the backlog never empties -- ``getRemainingImageCount``
    always reports a frame available and ``clearCircularBuffer`` is a no-op
    on it -- which is what the rate-cap/in-flight-cap tests need ("frames
    always available") without caring about a specific count.
    """

    def __init__(self, *, infinite: bool = False) -> None:
        self.frames: list[np.ndarray] = []
        self.infinite = infinite
        self._next_value = 0
        self.running = True
        self.stopped = False
        self.cleared_count = 0

    def push_frame(self, frame: np.ndarray) -> None:
        self.frames.append(frame)

    def getRemainingImageCount(self) -> int:
        if self.infinite:
            return 1
        return len(self.frames)

    def getLastImageAndMD(self) -> tuple[np.ndarray, dict[str, int]]:
        if self.infinite:
            frame = _frame(self._next_value)
            self._next_value += 1
            return frame, {"seq": self._next_value}
        return self.frames[-1], {"seq": len(self.frames)}

    def clearCircularBuffer(self) -> None:
        self.cleared_count += 1
        if not self.infinite:
            self.frames.clear()

    def isSequenceRunning(self, *_: object) -> bool:
        return self.running

    def stopSequenceAcquisition(self, *_: object) -> None:
        self.stopped = True
        self.running = False

    def popNextImageAndMD(
        self, *args: object, **kwargs: object
    ) -> tuple[np.ndarray, dict[str, int]]:
        raise AssertionError("popNextImageAndMD must not be called by _drain_live")


class _FakeConn:
    """Fake ``Connection``: a list ``inbox`` feeds ``poll``/``recv``; sends recorded.

    ``on_send`` (if given) runs after every ``send()``, letting a test react
    to whatever ``_drain_live`` just sent -- e.g. pushing a
    ``SlotFreeCmd``/``StopCmd`` into ``inbox`` to simulate the main process,
    or to end what would otherwise be ``_drain_live``'s infinite loop.
    """

    def __init__(self, on_send: Callable[[object], None] | None = None) -> None:
        self.inbox: list[object] = []
        self.sent: list[object] = []
        self._on_send = on_send

    def poll(self, timeout: float = 0.0) -> bool:
        return bool(self.inbox)

    def recv(self) -> object:
        return self.inbox.pop(0)

    def send(self, msg: object) -> None:
        self.sent.append(msg)
        if self._on_send is not None:
            self._on_send(msg)


@pytest.fixture
def shm() -> Iterator[SharedMemory]:
    """A real shared-memory segment, sized for a handful of ``_SLOT_NBYTES`` slots."""
    mem = SharedMemory(create=True, size=_SLOT_NBYTES * 8)
    try:
        yield mem
    finally:
        mem.close()
        mem.unlink()


def _live_config(
    shm: SharedMemory, *, n_slots: int = 4, live_max_fps: float = 0
) -> CameraWorkerConfig:
    return CameraWorkerConfig(
        camera_label="Camera-1",
        adapter_device_name="Camera-1",
        shm_name=shm.name,
        slot_nbytes=_SLOT_NBYTES,
        n_slots=n_slots,
        live_max_fps=live_max_fps,
    )


def test_drain_live_sends_newest_frame_and_drops_backlog(shm: SharedMemory) -> None:
    """A 50-frame backlog: only the newest frame is sent, the rest dropped."""
    core = _FakeLiveCore()
    for value in range(50):
        core.push_frame(_frame(value))

    def _stop_after_one_frame(msg: object) -> None:
        if isinstance(msg, FrameMsg):
            conn.inbox.append(StopCmd())

    conn = _FakeConn(on_send=_stop_after_one_frame)

    shutdown = _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
    )

    assert shutdown is False
    frame_msgs = [m for m in conn.sent if isinstance(m, FrameMsg)]
    assert len(frame_msgs) == 1
    msg = frame_msgs[0]
    assert msg.images_remaining == 49  # 50 were queued; 1 sent, 49 dropped
    assert np.all(_read_slot(shm, msg.slot_index) == 49)  # newest frame's value
    # One clear from the send itself, plus one from the stop that follows it.
    assert core.cleared_count == 2


def test_drain_live_caps_frames_in_flight(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """With no ``SlotFreeCmd`` replies, at most ``_LIVE_MAX_IN_FLIGHT`` are sent."""
    core = _FakeLiveCore(infinite=True)
    conn = _FakeConn()
    sleeps = {"n": 0}

    def _fake_sleep(_seconds: float) -> None:
        # Once capped, _drain_live idles here every iteration instead of
        # touching the camera -- bound the test by stopping it after a few.
        sleeps["n"] += 1
        if sleeps["n"] >= 5:
            conn.inbox.append(StopCmd())

    monkeypatch.setattr(time, "sleep", _fake_sleep)

    _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm, live_max_fps=0),
    )

    frame_msgs = [m for m in conn.sent if isinstance(m, FrameMsg)]
    assert len(frame_msgs) == _LIVE_MAX_IN_FLIGHT


def test_drain_live_sends_one_more_after_slot_freed(shm: SharedMemory) -> None:
    """Freeing exactly one in-flight slot lets exactly one more send through."""
    core = _FakeLiveCore(infinite=True)
    sent_frames: list[FrameMsg] = []

    def _on_send(msg: object) -> None:
        if not isinstance(msg, FrameMsg):
            return
        sent_frames.append(msg)
        if len(sent_frames) == _LIVE_MAX_IN_FLIGHT:
            conn.inbox.append(SlotFreeCmd(msg.slot_index))
        elif len(sent_frames) == _LIVE_MAX_IN_FLIGHT + 1:
            conn.inbox.append(StopCmd())

    conn = _FakeConn(on_send=_on_send)

    _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm, live_max_fps=0),
    )

    frame_msgs = [m for m in conn.sent if isinstance(m, FrameMsg)]
    assert len(frame_msgs) == _LIVE_MAX_IN_FLIGHT + 1


def test_drain_live_rate_cap_blocks_second_send_before_interval(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """``live_max_fps=10`` means no second send before 0.1s of (fake) time."""
    core = _FakeLiveCore(infinite=True)
    conn = _FakeConn()
    fake_now = [0.0]

    def _fake_monotonic() -> float:
        return fake_now[0]

    def _fake_sleep(_seconds: float) -> None:
        # Advance well short of the fps=10 interval (0.1s), so a second
        # send never becomes due before the test ends itself.
        fake_now[0] += 0.01
        if fake_now[0] >= 0.09:
            conn.inbox.append(StopCmd())

    monkeypatch.setattr(time, "monotonic", _fake_monotonic)
    monkeypatch.setattr(time, "sleep", _fake_sleep)

    _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm, live_max_fps=10),
    )

    frame_msgs = [m for m in conn.sent if isinstance(m, FrameMsg)]
    assert len(frame_msgs) == 1


def test_drain_live_uncapped_rate_sends_every_available_frame(
    shm: SharedMemory,
) -> None:
    """``live_max_fps=0`` sends every iteration, bounded only by the in-flight cap."""
    core = _FakeLiveCore(infinite=True)
    sent_frames: list[FrameMsg] = []

    def _on_send(msg: object) -> None:
        if not isinstance(msg, FrameMsg):
            return
        sent_frames.append(msg)
        if len(sent_frames) >= 10:
            conn.inbox.append(StopCmd())
        else:
            # Keep in-flight headroom open every time, so the rate cap
            # (not the in-flight cap) is the only thing being tested.
            conn.inbox.append(SlotFreeCmd(msg.slot_index))

    conn = _FakeConn(on_send=_on_send)

    _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm, live_max_fps=0),
    )

    frame_msgs = [m for m in conn.sent if isinstance(m, FrameMsg)]
    assert len(frame_msgs) == 10


def test_drain_live_throttles_stalled_messages(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """``StalledMsg`` is capped at ~1/s, not one per 5ms poll iteration."""
    core = _FakeLiveCore()  # no frames pushed; sequence stays "running"
    conn = _FakeConn()
    fake_now = [0.0]

    def _fake_monotonic() -> float:
        return fake_now[0]

    def _fake_sleep(_seconds: float) -> None:
        fake_now[0] += 0.2
        if fake_now[0] >= 10.0:  # well past _STALL_TIMEOUT_S (5s)
            conn.inbox.append(StopCmd())

    monkeypatch.setattr(time, "monotonic", _fake_monotonic)
    monkeypatch.setattr(time, "sleep", _fake_sleep)

    _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm, live_max_fps=0),
    )

    stalls = [m for m in conn.sent if isinstance(m, StalledMsg)]
    # ~5s of eligible stall time (10s run - 5s timeout) at <= 1/s is a
    # handful of messages -- nowhere near the hundreds an unthrottled
    # 5ms-per-iteration loop would produce.
    assert 1 <= len(stalls) <= 12


def test_drain_live_reports_error_when_sequence_stops_unexpectedly(
    shm: SharedMemory,
) -> None:
    """No frames queued and the sequence already isn't running -> ``ErrorMsg``."""
    core = _FakeLiveCore()
    core.running = False
    conn = _FakeConn()

    shutdown = _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
    )

    assert shutdown is False
    errors = [m for m in conn.sent if isinstance(m, ErrorMsg)]
    assert len(errors) == 1
    assert "unexpectedly" in errors[0].message


def test_drain_live_stop_cmd_acknowledged_with_token(shm: SharedMemory) -> None:
    """``StopCmd(token=...)`` is acked with a matching ``StoppedMsg``."""
    core = _FakeLiveCore(infinite=True)
    conn = _FakeConn()
    conn.inbox.append(StopCmd(token=7))

    shutdown = _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
    )

    assert shutdown is False
    assert core.stopped is True
    assert core.cleared_count >= 1
    stopped_msgs = [m for m in conn.sent if isinstance(m, StoppedMsg)]
    assert len(stopped_msgs) == 1
    assert stopped_msgs[0].token == 7


def test_drain_live_shutdown_cmd_returns_true(shm: SharedMemory) -> None:
    """``ShutdownCmd`` tells the caller to exit the worker process."""
    core = _FakeLiveCore(infinite=True)
    conn = _FakeConn()
    conn.inbox.append(ShutdownCmd())

    shutdown = _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
    )

    assert shutdown is True
