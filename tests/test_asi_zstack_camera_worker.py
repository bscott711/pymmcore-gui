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

import re
import sys
import time
from multiprocessing.shared_memory import SharedMemory
from typing import TYPE_CHECKING
from unittest.mock import MagicMock

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack import camera_worker as cw_module
from pymmcore_gui.asi_z_stack.camera_worker import (
    _LIVE_MAX_IN_FLIGHT,
    CameraWorkerConfig,
    _drain_incoming,
    _drain_live,
    _log,
    _wait_for_free_slot,
    run_camera_worker,
)
from pymmcore_gui.asi_z_stack.worker_messages import (
    ArmCmd,
    ArmLiveCmd,
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
    from pathlib import Path

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


def test_read_camera_timing_keeps_timing_props_and_skips_failures() -> None:
    from pymmcore_gui.asi_z_stack.camera_worker import _read_camera_timing

    core = MagicMock()
    core.getDevicePropertyNames.return_value = [
        "Timing-ReadoutTimeNs",
        "Timing-ClearingTimeNs",
        "TriggerMode",
        "ExposeOutMode",
        "Exposure",
        "PortName",
        "Gain",
    ]
    values = {
        "Timing-ReadoutTimeNs": "13500000",
        "TriggerMode": "Edge Trigger",
        "ExposeOutMode": "All Rows",
        "Exposure": "10.0",
        "PortName": "Sensitivity",
    }

    def _get(label: str, prop: str) -> str:
        if prop not in values:
            raise RuntimeError("PL_ERR_ACQUISITION_SETUP_REQUIRED")
        return values[prop]

    core.getProperty.side_effect = _get
    core.getROI.return_value = (485, 644, 1458, 1143)

    timing = _read_camera_timing(core, "Camera-1")

    assert timing == {**values, "ROI": "485,644,1458,1143"}


def test_set_expose_out_matches_case_insensitively_and_skips_noop() -> None:
    from pymmcore_gui.asi_z_stack.camera_worker import (
        _expose_out_property,
        _set_expose_out,
    )

    core = MagicMock()
    core.getDevicePropertyNames.return_value = ["Exposure", "ExposeOutMode", "Port"]
    core.getAllowedPropertyValues.return_value = ("First Row", "All Rows", "Any Row")
    core.getProperty.return_value = "First Row"

    prop = _expose_out_property(core, "Camera-1")
    assert prop == "ExposeOutMode"
    _set_expose_out(core, "Camera-1", prop, "all rows")
    core.setProperty.assert_called_once_with("Camera-1", "ExposeOutMode", "All Rows")

    core.setProperty.reset_mock()
    core.getProperty.return_value = "All Rows"
    _set_expose_out(core, "Camera-1", prop, "All Rows")
    core.setProperty.assert_not_called()


def test_set_expose_out_refuses_unknown_mode() -> None:
    from pymmcore_gui.asi_z_stack.camera_worker import _set_expose_out

    core = MagicMock()
    core.getAllowedPropertyValues.return_value = ("First Row",)
    with pytest.raises(RuntimeError, match="no 'All Rows' mode"):
        _set_expose_out(core, "Camera-1", "ExposeOutMode", "All Rows")
    with pytest.raises(RuntimeError, match="no expose-out mode property"):
        _set_expose_out(core, "Camera-1", None, "All Rows")


def test_require_trigger_mode_and_set_exposure() -> None:
    from pymmcore_gui.asi_z_stack.camera_worker import (
        _require_trigger_mode,
        _set_exposure,
    )

    core = MagicMock()
    core.getAllowedPropertyValues.return_value = (
        "Internal Trigger",
        "Level Trigger",
        "Level Trigger-Overlap",
    )
    assert _require_trigger_mode(core, "Camera-1", "Level Trigger") == "Level Trigger"
    # matched loosely: the camera's own spelling comes back
    assert (
        _require_trigger_mode(core, "Camera-1", "Level Trigger Overlap")
        == "Level Trigger-Overlap"
    )
    with pytest.raises(RuntimeError, match="no 'Edge Trigger'"):
        _require_trigger_mode(core, "Camera-1", "Edge Trigger")

    core.getExposure.return_value = 10.0
    _set_exposure(core, "Camera-1", 10.0)
    core.setExposure.assert_not_called()
    _set_exposure(core, "Camera-1", 20.0)
    core.setExposure.assert_called_once_with("Camera-1", 20.0)


def test_log_line_has_timestamp_and_worker_prefix(
    capsys: pytest.CaptureFixture[str],
) -> None:
    _log("camA", "hello")
    line = capsys.readouterr().err.rstrip("\n")
    assert re.fullmatch(
        r"\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3} \[worker:camA\] hello", line
    )


class _ScriptedConn:
    """Hands back *cmds*, then ``EOFError`` (the parent closing the pipe)."""

    def __init__(self, cmds: list[object]) -> None:
        self._cmds = list(cmds)
        self.sent: list[object] = []

    def recv(self) -> object:
        if not self._cmds:
            raise EOFError
        return self._cmds.pop(0)

    def send(self, msg: object) -> None:
        self.sent.append(msg)


def test_run_camera_worker_logs_commands_and_enables_faulthandler(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    # sys.stderr/stdout are reassigned by the worker; restore them afterwards
    monkeypatch.setattr(sys, "stderr", sys.stderr)
    monkeypatch.setattr(sys, "stdout", sys.stdout)
    fh_enable = MagicMock()
    monkeypatch.setattr("faulthandler.enable", fh_enable)
    monkeypatch.setattr(cw_module, "CMMCorePlus", MagicMock())
    for name in (
        "_apply_snapshot",
        "preferred_external_trigger_mode",
        "_internal_trigger_mode",
        "_expose_out_property",
        "_set_trigger_mode",
        "_set_expose_out",
        "_set_exposure",
        "_report_timing",
    ):
        monkeypatch.setattr(cw_module, name, MagicMock(return_value=None))
    monkeypatch.setattr(cw_module, "_report_timing", MagicMock(return_value={}))
    monkeypatch.setattr(cw_module, "_drain_sequence", MagicMock(return_value=False))
    monkeypatch.setattr(cw_module, "_drain_live", MagicMock(return_value=False))

    shm = SharedMemory(create=True, size=16)
    log_path = tmp_path / "worker-stderr.log"
    try:
        config = CameraWorkerConfig(
            camera_label="camA",
            adapter_device_name="Fake",
            shm_name=shm.name,
            slot_nbytes=16,
            n_slots=1,
            stderr_log_file=str(log_path),
        )
        conn = _ScriptedConn(
            [
                ArmCmd(
                    5,
                    external_trigger=True,
                    trigger_mode="Edge Trigger",
                    exposure_ms=12.5,
                ),
                ArmLiveCmd(exposure_ms=7.0),
                StopCmd(),
            ]
        )
        run_camera_worker(config, conn)  # type: ignore[arg-type]
        fh_enable.assert_called_once()
        log_file = fh_enable.call_args.kwargs["file"]
        assert fh_enable.call_args.kwargs["all_threads"] is True
        text = log_path.read_text()
        log_file.close()
    finally:
        shm.close()
        shm.unlink()

    assert re.search(
        r"\[worker:camA\] ArmCmd n_images=5 external_trigger=True "
        r"trigger_mode='Edge Trigger' exposure_ms=12.5",
        text,
    )
    assert re.search(r"\[worker:camA\] ArmLiveCmd exposure_ms=7.0", text)
    assert re.search(r"\[worker:camA\] StopCmd sequence_running=", text)
    assert re.match(r"\d{4}-\d\d-\d\d \d\d:", text)


# ---------------------------------------------------------------------------
# Commit-headroom guard (pymmcore's image-getter typemap crashes the process
# when the allocation of the returned numpy array fails; see commit_headroom.py)
# ---------------------------------------------------------------------------

_GIB = 1024**3
_LOW_COMMIT = 34 * 1024**2
_OK_COMMIT = 64 * _GIB
_CRASH = "allocating getter called with commit exhausted: native crash in pymmcore"


class _Commit:
    """The simulated system commit; both the probe and the fake core read it."""

    def __init__(self, free: int | None) -> None:
        self.free = free
        self.calls = 0

    def probe(self) -> int | None:
        self.calls += 1
        return self.free


class _FakeSeqCore(_FakeLiveCore):
    """``_FakeLiveCore`` plus a FIFO ``popNextImageAndMD`` that "crashes" when
    the simulated commit is below the 2 GiB margin, like pymmcore would."""

    def __init__(self, commit: _Commit, n_frames: int = 3) -> None:
        super().__init__()
        self.commit = commit
        for value in range(n_frames):
            self.push_frame(_frame(value))
        self.getter_calls: list[str] = []
        self.stop_calls = 0

    def _check(self, name: str) -> None:
        self.getter_calls.append(name)
        if self.commit.free is not None and self.commit.free < 2 * _GIB:
            raise AssertionError(_CRASH)

    def popNextImageAndMD(
        self, *args: object, **kwargs: object
    ) -> tuple[np.ndarray, dict[str, int]]:
        self._check("popNextImageAndMD")
        return self.frames.pop(0), {}

    def getLastImageAndMD(self) -> tuple[np.ndarray, dict[str, int]]:
        self._check("getLastImageAndMD")
        return super().getLastImageAndMD()

    def stopSequenceAcquisition(self, *_: object) -> None:
        self.stop_calls += 1
        super().stopSequenceAcquisition()


def _guard_setup(
    monkeypatch: pytest.MonkeyPatch, commit: _Commit, wait_s: float = 0.2
) -> None:
    # raising=False: on code without the guard these names do not exist, which
    # lets the negative-control run reach the (fake) crashing getter.
    monkeypatch.setattr(
        cw_module, "available_commit_bytes", commit.probe, raising=False
    )
    monkeypatch.setattr(cw_module, "COMMIT_HEADROOM_WAIT_S", wait_s, raising=False)


def _sent(conn: _FakeConn, kind: type) -> list:
    return [m for m in conn.sent if isinstance(m, kind)]


def test_drain_sequence_never_pops_with_commit_exhausted(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """NEGATIVE CONTROL: frames pending, commit gone: no pop, stop, typed error."""
    commit = _Commit(_LOW_COMMIT)
    _guard_setup(monkeypatch, commit)
    core = _FakeSeqCore(commit)
    conn = _FakeConn()

    shutdown = cw_module._drain_sequence(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
        3,
    )

    assert core.getter_calls == []
    assert core.stop_calls == 1
    assert not core.running
    assert shutdown is False  # worker is back at its idle command loop
    (err,) = _sent(conn, ErrorMsg)
    assert err.exc_type == "CommitHeadroomError"
    assert "34 MiB free" in err.message
    assert not _sent(conn, FrameMsg)


def test_drain_live_never_gets_last_image_with_commit_exhausted(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """NEGATIVE CONTROL: same for the Live getter."""
    commit = _Commit(_LOW_COMMIT)
    _guard_setup(monkeypatch, commit)
    core = _FakeSeqCore(commit)
    conn = _FakeConn()

    shutdown = _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
    )

    assert core.getter_calls == []
    assert core.stop_calls == 1
    assert shutdown is False
    (err,) = _sent(conn, ErrorMsg)
    assert err.exc_type == "CommitHeadroomError"
    assert not _sent(conn, FrameMsg)


def test_drain_sequence_waits_and_continues_when_commit_recovers(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    commit = _Commit(_LOW_COMMIT)
    _guard_setup(monkeypatch, commit, wait_s=5.0)
    real_sleep = time.sleep

    def _recover(seconds: float) -> None:
        commit.free = _OK_COMMIT  # the page file grew while we waited
        real_sleep(0)

    monkeypatch.setattr(time, "sleep", _recover)
    core = _FakeSeqCore(commit, n_frames=2)
    conn = _FakeConn()

    cw_module._drain_sequence(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
        2,
    )

    assert core.getter_calls == ["popNextImageAndMD"] * 2
    assert len(_sent(conn, FrameMsg)) == 2
    assert not _sent(conn, ErrorMsg)


def test_drain_live_waits_and_continues_when_commit_recovers(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    commit = _Commit(_LOW_COMMIT)
    _guard_setup(monkeypatch, commit, wait_s=5.0)

    def _recover(seconds: float) -> None:
        commit.free = _OK_COMMIT

    monkeypatch.setattr(time, "sleep", _recover)
    core = _FakeSeqCore(commit)
    conn = _FakeConn(
        on_send=lambda m: conn.inbox.append(StopCmd())
        if isinstance(m, FrameMsg)
        else None
    )

    _drain_live(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
    )

    assert core.getter_calls == ["getLastImageAndMD"]
    assert len(_sent(conn, FrameMsg)) == 1
    assert not _sent(conn, ErrorMsg)


def test_drain_unchanged_when_probe_is_unavailable(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Probe returns None (non-Windows): behaves exactly as before the guard."""
    commit = _Commit(None)
    _guard_setup(monkeypatch, commit)
    core = _FakeSeqCore(commit, n_frames=2)
    conn = _FakeConn()

    cw_module._drain_sequence(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
        2,
    )

    assert core.getter_calls == ["popNextImageAndMD"] * 2
    assert len(_sent(conn, FrameMsg)) == 2
    assert not _sent(conn, ErrorMsg)


def test_stop_during_commit_wait_is_honoured(
    shm: SharedMemory, monkeypatch: pytest.MonkeyPatch
) -> None:
    commit = _Commit(_LOW_COMMIT)
    _guard_setup(monkeypatch, commit, wait_s=5.0)
    core = _FakeSeqCore(commit)
    conn = _FakeConn()
    conn.inbox.append(StopCmd(token=7))
    # the drain loop's own top-of-loop poll would eat the StopCmd; deliver it
    # only once the guard is waiting
    inbox, conn.inbox = list(conn.inbox), []
    real_probe = commit.probe

    def _probe() -> int | None:
        if commit.calls == 1:  # first call is the guard's; the wait loop polls next
            conn.inbox.extend(inbox)
        return real_probe()

    monkeypatch.setattr(cw_module, "available_commit_bytes", _probe, raising=False)

    shutdown = cw_module._drain_sequence(
        core,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        conn,  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
        shm,
        _live_config(shm),
        3,
    )

    assert shutdown is False
    assert core.getter_calls == []
    assert core.stop_calls == 1
    (stopped,) = _sent(conn, StoppedMsg)
    assert stopped.token == 7
    assert not _sent(conn, ErrorMsg)


_ALLOCATING_GETTER_ATTR = re.compile(
    r"\.(popNextImageAndMD|getLastImageAndMD|getImage|popNextImage|getLastImage"
    r"|getNBeforeLastImage|getNBeforeLastImageMD|getNBeforeLastImageAndMD"
    r"|getTaggedImage|popNextTaggedImage|getLastTaggedImage|getLastImageMD"
    r"|popNextImageMD)\b"
)


def test_allocating_getter_matcher_catches_calls_and_bare_attributes() -> None:
    """Negative control for the structural test below: the matcher must fire."""
    for bad in (
        "mmc.popNextImageAndMD()",
        "f = mmc.getLastImageAndMD",  # attribute taken without a call
        "core.getTaggedImage(0)",
        "x = self._mmc.popNextTaggedImage",
        "mmc.getLastTaggedImage()",
        "mmc.getNBeforeLastImageMD(1)",
        "mmc.getNBeforeLastImageAndMD(1)",
    ):
        assert _ALLOCATING_GETTER_ATTR.search(bad), bad
    assert not _ALLOCATING_GETTER_ATTR.search('getattr(mmc, "popNextImageAndMD")()')


def test_every_allocating_getter_call_goes_through_fetch_image() -> None:
    """Structural: the getters are reached in exactly one place, behind the guard."""
    import inspect

    src = inspect.getsource(cw_module)
    # No attribute access at all (called or not): only via getattr(mmc, "<name>").
    assert _ALLOCATING_GETTER_ATTR.findall(src) == []
    for getter in ("popNextImageAndMD", "getLastImageAndMD"):
        assert (
            len(re.findall(rf'"{getter}"', src)) == 2
        )  # the tuple + its one call site
    fetch_src = inspect.getsource(cw_module._fetch_image)
    assert "_wait_for_commit_headroom(" in fetch_src
    assert fetch_src.index("_wait_for_commit_headroom(") < fetch_src.index(
        "getattr(mmc"
    )


class _ArmCore:
    """Fake core for the Arm handlers: ``isSequenceRunning`` follows start/stop."""

    def __init__(self) -> None:
        self.running = False
        self.calls: list[str] = []

    def startSequenceAcquisition(self, *args: object) -> None:
        self.calls.append("startSequenceAcquisition")
        if self.running:
            raise RuntimeError(
                "can not be executed while sequence acquisition is running"
            )
        self.running = True

    def startContinuousSequenceAcquisition(self, *args: object) -> None:
        self.calls.append("startContinuousSequenceAcquisition")
        if self.running:
            raise RuntimeError(
                "can not be executed while sequence acquisition is running"
            )
        self.running = True

    def isSequenceRunning(self, *_: object) -> bool:
        return self.running

    def stopSequenceAcquisition(self, *_: object) -> None:
        self.calls.append("stopSequenceAcquisition")
        self.running = False

    def clearCircularBuffer(self) -> None:
        self.calls.append("clearCircularBuffer")

    def getProperty(self, *_: object) -> str:
        return ""

    def setCircularBufferMemoryFootprint(self, *_: object) -> None:
        pass

    def loadDevice(self, *_: object) -> None:
        pass

    def initializeDevice(self, *_: object) -> None:
        pass

    def setCameraDevice(self, *_: object) -> None:
        pass

    def unloadDevice(self, *_: object) -> None:
        pass


def _run_worker_with(
    monkeypatch: pytest.MonkeyPatch,
    core: object,
    cmds: list[object],
    *,
    report_timing: object = None,
) -> _ScriptedConn:
    monkeypatch.setattr(sys, "stderr", sys.stderr)
    monkeypatch.setattr(sys, "stdout", sys.stdout)
    monkeypatch.setattr("faulthandler.enable", MagicMock())
    monkeypatch.setattr(cw_module, "CMMCorePlus", MagicMock(return_value=core))
    for name in (
        "_apply_snapshot",
        "preferred_external_trigger_mode",
        "_internal_trigger_mode",
        "_expose_out_property",
        "_set_trigger_mode",
        "_set_expose_out",
        "_set_exposure",
    ):
        monkeypatch.setattr(cw_module, name, MagicMock(return_value=None))
    monkeypatch.setattr(
        cw_module,
        "_report_timing",
        report_timing if report_timing is not None else MagicMock(return_value={}),
    )
    monkeypatch.setattr(cw_module, "_drain_sequence", MagicMock(return_value=False))
    monkeypatch.setattr(cw_module, "_drain_live", MagicMock(return_value=False))
    shm = SharedMemory(create=True, size=16)
    try:
        config = CameraWorkerConfig(
            camera_label="camA",
            adapter_device_name="Fake",
            shm_name=shm.name,
            slot_nbytes=16,
            n_slots=1,
        )
        conn = _ScriptedConn(cmds)
        # The worker's shutdown path stops a still-running camera, which would
        # hide a missing stop; record the camera state when each ErrorMsg goes out.
        conn.running_at_error = []  # type: ignore[attr-defined]
        plain_send = conn.send

        def _send(msg: object) -> None:
            if isinstance(msg, ErrorMsg):
                conn.running_at_error.append(core.isSequenceRunning())  # type: ignore[attr-defined]
            plain_send(msg)

        conn.send = _send  # type: ignore[method-assign]
        run_camera_worker(config, conn)  # type: ignore[arg-type]
    finally:
        shm.close()
        shm.unlink()
    return conn


def test_arm_refused_below_margin_never_starts_the_camera(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commit = _Commit(_LOW_COMMIT)
    _guard_setup(monkeypatch, commit)
    core = _ArmCore()
    conn = _run_worker_with(monkeypatch, core, [ArmCmd(5), ArmLiveCmd()])

    assert "startSequenceAcquisition" not in core.calls
    assert "startContinuousSequenceAcquisition" not in core.calls
    errors = [m for m in conn.sent if isinstance(m, ErrorMsg)]
    assert [e.exc_type for e in errors] == ["CommitHeadroomError"] * 2
    assert not [m for m in conn.sent if type(m).__name__ == "ArmedMsg"]
    cw_module._drain_sequence.assert_not_called()  # type: ignore[attr-defined]
    cw_module._drain_live.assert_not_called()  # type: ignore[attr-defined]


def test_arm_proceeds_when_probe_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    _guard_setup(monkeypatch, _Commit(None))
    core = _ArmCore()
    conn = _run_worker_with(monkeypatch, core, [ArmCmd(5), StopCmd()])

    assert core.calls[0] == "startSequenceAcquisition"
    assert not [m for m in conn.sent if isinstance(m, ErrorMsg)]


def test_arm_logs_available_commit(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    _guard_setup(monkeypatch, _Commit(8 * _GIB))
    _run_worker_with(monkeypatch, _ArmCore(), [ArmCmd(5), ArmLiveCmd()])
    err = capsys.readouterr().err
    assert re.search(r"ArmCmd .*commit_avail=8\.0 GiB", err)
    assert re.search(r"ArmLiveCmd .*commit_avail=8\.0 GiB", err)


def test_arm_failure_after_start_stops_the_camera(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """09-28: an Arm that fails after the camera started used to leave it running,
    so the next arm failed with "can not be executed while sequence acquisition
    is running". NEGATIVE CONTROL (fails on code without the stop)."""
    _guard_setup(monkeypatch, _Commit(None))
    core = _ArmCore()
    boom = MagicMock(side_effect=ValueError("timing read failed"))
    conn = _run_worker_with(
        monkeypatch, core, [ArmCmd(5), ArmCmd(5)], report_timing=boom
    )

    assert conn.running_at_error == [False, False]  # type: ignore[attr-defined]
    errors = [m for m in conn.sent if isinstance(m, ErrorMsg)]
    # both arms reached "start" (the second did not hit "already running")
    assert [e.exc_type for e in errors] == ["AcquisitionError"] * 2
    assert all("is running" not in e.traceback_text for e in errors)
    assert core.calls.count("startSequenceAcquisition") == 2


def test_arm_live_failure_after_start_stops_the_camera(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _guard_setup(monkeypatch, _Commit(None))
    core = _ArmCore()
    boom = MagicMock(side_effect=ValueError("timing read failed"))
    conn = _run_worker_with(
        monkeypatch, core, [ArmLiveCmd(), ArmLiveCmd()], report_timing=boom
    )

    assert conn.running_at_error == [False, False]  # type: ignore[attr-defined]
    errors = [m for m in conn.sent if isinstance(m, ErrorMsg)]
    assert [e.exc_type for e in errors] == ["AcquisitionError"] * 2
    assert all("is running" not in e.traceback_text for e in errors)
    assert core.calls.count("startContinuousSequenceAcquisition") == 2


def test_failure_while_stopping_does_not_mask_the_original_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _guard_setup(monkeypatch, _Commit(None))
    core = _ArmCore()
    core.stopSequenceAcquisition = MagicMock(side_effect=OSError("stop failed"))  # type: ignore[method-assign]
    boom = MagicMock(side_effect=ValueError("timing read failed"))
    conn = _run_worker_with(monkeypatch, core, [ArmCmd(5)], report_timing=boom)

    (err,) = [m for m in conn.sent if isinstance(m, ErrorMsg)]
    assert err.exc_type == "AcquisitionError"
    assert "timing read failed" in err.traceback_text
