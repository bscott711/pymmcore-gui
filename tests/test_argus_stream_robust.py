"""The stream when things go wrong, and that it always says so.

Rig 2026-10-24: from 12:23 on, ten runs reached Argus with no frames. The
label said "reconnecting…", runs paused on their first pass behind earlier
stuck runs, and a run Argus had closed resynced every 30 s. These tests pin
what the stream now does instead.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, cast

import msgpack
import numpy as np
import pytest
from psygnal import Signal

from pymmcore_gui._argus_stream import StreamState
from pymmcore_gui._argus_stream import _log as stream_log
from pymmcore_gui._argus_stream import _session as session_mod
from pymmcore_gui._argus_stream._protocol import (
    MSG_ACK,
    MSG_CLIENT_LOG,
    MSG_FRAME,
    MSG_HEARTBEAT,
    MSG_SESSION_END,
    MSG_SESSION_START,
)
from pymmcore_gui._argus_stream._session import _BufferBudget
from pymmcore_gui._argus_stream._tunnel import port_busy

# The main window's priority for the stream's slots.
from pymmcore_gui._main_window import _ARGUS_SLOT_PRIORITY
from test_argus_stream import (
    _FakeReceiver,
    _links_session,
    _timelapse,
    _wait,
)

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

    import useq

    from pymmcore_gui._argus_stream import ArgusStreamSession
    from pymmcore_gui._argus_stream._session import _RunWorker


@pytest.fixture
def fake() -> Iterator[_FakeReceiver]:
    receiver = _FakeReceiver()
    yield receiver
    receiver.close()


def _of(receiver: _FakeReceiver, msg_type: bytes) -> list[dict]:
    return [h for t, _s, h, _p in receiver.messages if t == msg_type]


def _recording(session: ArgusStreamSession) -> list[tuple[StreamState, str]]:
    states: list[tuple[StreamState, str]] = []
    session._on_state_changed = lambda s, d: states.append((s, d))
    return states


def _plane() -> np.ndarray:
    return np.zeros((4, 4), dtype="uint16")


# --- slots: a viewer error can't starve the stream -------------------------


class _Events:
    sequenceStarted = Signal(object, object)
    frameReady = Signal(object, object, object)
    sequenceFinished = Signal(object)


def _wire(session: ArgusStreamSession, events: _Events, priority: int) -> None:
    """Connect a raising viewer slot first, then the stream's.

    That is the order the main window connects them in.
    """

    def viewer(*_args: object) -> None:
        raise RuntimeError("viewer broke")

    events.frameReady.connect(viewer)
    events.sequenceFinished.connect(viewer)
    events.sequenceStarted.connect(session.sequenceStarted, priority=priority)
    events.frameReady.connect(session.frameReady, priority=priority)
    events.sequenceFinished.connect(session.sequenceFinished, priority=priority)


def _emit_run(events: _Events, seq: useq.MDASequence) -> None:
    events.sequenceStarted.emit(seq, {})
    for event in seq:
        try:
            events.frameReady.emit(_plane(), event, {})
        except Exception:
            pass  # psygnal re-raises the viewer's error
    try:
        events.sequenceFinished.emit(seq)
    except Exception:
        pass


def test_a_raising_viewer_slot_no_longer_starves_the_stream(
    fake: _FakeReceiver,
) -> None:
    session = _links_session(fake.port, links=1)
    seq = _timelapse(2)
    events = _Events()
    _wire(session, events, _ARGUS_SLOT_PRIORITY)
    _emit_run(events, seq)
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_END)))
    assert len(_of(fake, MSG_FRAME)) == 2
    assert _of(fake, MSG_SESSION_END)[0]["reason"] == "complete"
    session.shutdown(timeout=1)


def test_negative_control_default_priority_loses_every_frame(
    fake: _FakeReceiver,
) -> None:
    """What the rig had: the viewer connected first, so psygnal stopped each
    emit at its error and the stream got SESSION_START but no frame."""
    session = _links_session(fake.port, links=1)
    seq = _timelapse(2)
    events = _Events()
    _wire(session, events, 0)  # psygnal's default
    _emit_run(events, seq)
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_START)))
    time.sleep(0.5)
    assert _of(fake, MSG_FRAME) == []
    assert _of(fake, MSG_SESSION_END) == []  # never got sequenceFinished
    session.shutdown(timeout=1)


def test_a_failing_stream_slot_logs_and_never_raises(
    fake: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    session = _links_session(fake.port, links=1)
    seq = _timelapse(2)
    session.sequenceStarted(seq, {})

    def boom(*_a: object, **_k: object) -> list:
        raise ValueError("assembler broke")

    monkeypatch.setattr(session._assembler, "add_plane", boom)
    for event in seq:
        session.frameReady(_plane(), event, {})  # must not raise
    worker = session._worker
    assert worker is not None and "assembler broke" in worker.last_error
    session.shutdown(timeout=1)


# --- the sender: failures end the run with a reason ------------------------


def test_a_sender_failure_ends_the_run_as_client_error(fake: _FakeReceiver) -> None:
    """Before: run() had no except; the thread died silently, sent no
    SESSION_END, and the label went back to "idle"."""
    session = _links_session(fake.port, links=1)
    states = _recording(session)
    fake.auto_ack = False
    seq = _timelapse(3)
    events = list(seq)
    session.sequenceStarted(seq, {})
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_START)))
    fake.send(MSG_ACK, {"through_frame_index": "not a number"})  # breaks on_ack
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_END)))
    (end,) = _of(fake, MSG_SESSION_END)
    assert end["reason"] == "client_error" and "TypeError" in end["detail"]
    assert _wait(lambda: any(s == StreamState.ERROR for s, _d in states))
    worker = session._all_workers[0]
    assert worker.paused.is_set() and "TypeError" in worker.error
    session.frameReady(_plane(), events[0], {})  # nothing is assembled
    assert not session._assembler._pending
    assert states[-1][0] == StreamState.ERROR  # not "idle"
    session.shutdown(timeout=1)


@pytest.mark.parametrize(
    "reply",
    [
        {"closed": True, "rejected": "closed on Argus after 10 min ..."},
        # The receiver before the "closed" flag: only its reason says so.
        {"rejected": "closed on Argus after 10 min with no new data; not resuming it"},
    ],
)
def test_a_run_argus_closed_stops_instead_of_resyncing_forever(
    fake: _FakeReceiver, reply: dict
) -> None:
    fake.features = ["resume"]
    session = _links_session(fake.port, links=1)
    states = _recording(session)
    seq = _timelapse(3)
    events = list(seq)
    session.sequenceStarted(seq, {})
    session.frameReady(_plane(), events[0], {})
    assert _wait(lambda: len(_of(fake, MSG_FRAME)) == 1)
    fake.auto_ack = False
    fake.send(
        MSG_ACK,
        {"through_frame_index": -1, "unknown_session": True, "features": ["resume"]}
        | reply,
    )
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_END)))
    assert _of(fake, MSG_SESSION_END)[0]["reason"] == "paused"
    starts = len(_of(fake, MSG_SESSION_START))
    session.frameReady(_plane(), events[1], {})
    time.sleep(0.6)
    assert len(_of(fake, MSG_SESSION_START)) == starts  # no resync loop
    paused = [d for s, d in states if s == StreamState.PAUSED]
    assert paused and "Argus closed this run" in paused[-1]
    session.shutdown(timeout=1)


def test_the_oldest_run_holding_data_pays_the_cap_not_a_new_one() -> None:
    """Before: the run that checked paused, and a finishing run never did,
    so old stuck runs paused every new run before it sent anything."""
    budget = _BufferBudget(cap_bytes=100)
    old, new = object(), object()
    budget.update(old, 150)  # stuck behind a dead link
    budget.update(new, 10)  # just started
    assert budget.must_pause(old)
    assert not budget.must_pause(new)
    budget.forget(old)
    assert not budget.must_pause(new)
    budget.update(new, 120)  # a single run over the cap still pauses
    assert budget.must_pause(new)


# --- reporting in: heartbeat, log, stall alarm -----------------------------


def test_heartbeats_go_once_argus_advertises_them(
    fake: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(session_mod, "_HEARTBEAT_EVERY_S", 0.1)
    fake.features = ["resume", "heartbeat"]
    session = _links_session(fake.port, links=1)
    seq = _timelapse(3)
    session.sequenceStarted(seq, {})
    assert _wait(lambda: len(_of(fake, MSG_HEARTBEAT)) >= 2)
    beat = _of(fake, MSG_HEARTBEAT)[-1]
    assert beat["state"] == "acquiring"
    assert beat["links_total"] == 1 and beat["links_up"] == 1
    assert beat["submitted"] == 0 and beat["unacked_bytes"] == 0
    session.shutdown(timeout=1)


def test_no_heartbeat_to_an_argus_that_does_not_know_them(
    fake: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(session_mod, "_HEARTBEAT_EVERY_S", 0.1)
    fake.features = ["resume"]
    session = _links_session(fake.port, links=1)
    session.sequenceStarted(_timelapse(3), {})
    time.sleep(0.5)
    assert _of(fake, MSG_HEARTBEAT) == []
    assert _of(fake, MSG_CLIENT_LOG) == []
    session.shutdown(timeout=1)


def test_the_stream_log_goes_to_argus(
    fake: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(session_mod, "_CLIENT_LOG_EVERY_S", 0.1)
    fake.features = ["resume", "client_log"]
    session = _links_session(fake.port, links=1)
    logging.getLogger("pymmcore_gui._argus_stream._tunnel").warning(
        "ssh 5555: Timeout, server not responding (marker-7f3)"
    )
    session.sequenceStarted(_timelapse(3), {})

    def shipped() -> str:
        return "\n".join("\n".join(h["lines"]) for h in _of(fake, MSG_CLIENT_LOG))

    assert _wait(lambda: "marker-7f3" in shipped())
    assert "WARNING" in shipped() and _of(fake, MSG_CLIENT_LOG)[0]["host"]
    session.shutdown(timeout=1)


def test_the_label_says_argus_is_not_receiving_within_seconds(
    fake: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Before: "reconnecting…" for 10 min, until Argus closed the run."""
    monkeypatch.setattr(session_mod, "_STALL_ALARM_S", 0.3)
    session = _links_session(fake.port, links=1)
    states = _recording(session)
    seq = _timelapse(3)
    session.sequenceStarted(seq, {})
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_START)))
    fake.auto_ack = False
    session.frameReady(_plane(), next(iter(seq)), {})
    assert _wait(lambda: any(s == StreamState.STALLED for s, _d in states))
    detail = next(d for s, d in states if s == StreamState.STALLED)
    assert "nothing confirmed for" in detail and "local save continues" in detail
    session.shutdown(timeout=1)


def test_the_label_says_when_no_camera_frames_reach_the_stream(
    fake: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(session_mod, "_NO_FRAMES_ALARM_S", 0.3)
    session = _links_session(fake.port, links=1)
    states = _recording(session)
    session.sequenceStarted(_timelapse(3), {})
    assert _wait(lambda: any(s == StreamState.STALLED for s, _d in states))
    detail = next(d for s, d in states if s == StreamState.STALLED)
    assert "no camera frames reached the stream" in detail
    session.shutdown(timeout=1)


def test_an_unknown_message_from_argus_is_skipped(fake: _FakeReceiver) -> None:
    session = _links_session(fake.port, links=1)
    seq = _timelapse(1)
    session.sequenceStarted(seq, {})
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_START)))
    sid = fake._session_id
    assert sid is not None and fake._identity is not None
    fake.sock.send_multipart(
        [fake._identity, b"SOMETHING_NEW", sid.encode(), msgpack.packb({})]
    )
    for event in seq:
        session.frameReady(_plane(), event, {})
    session.sequenceFinished(seq)
    assert _wait(lambda: bool(_of(fake, MSG_SESSION_END)))
    assert _of(fake, MSG_SESSION_END)[0]["reason"] == "complete"
    session.shutdown(timeout=1)


def test_an_older_runs_idle_does_not_overwrite_the_current_runs_label(
    fake: _FakeReceiver,
) -> None:
    session = _links_session(fake.port, links=1)
    states = _recording(session)
    old, current = cast("_RunWorker", object()), cast("_RunWorker", object())
    session._worker = current
    session._emit_worker_state(old, StreamState.IDLE, "")
    session._emit_worker_state(old, StreamState.STREAMING, "SSH tunnel")
    assert states == []
    session._emit_worker_state(old, StreamState.PAUSED, "Argus stopped answering")
    assert states == [(StreamState.PAUSED, "previous run: Argus stopped answering")]
    session._emit_worker_state(current, StreamState.STREAMING, "SSH tunnel")
    assert states[-1] == (StreamState.STREAMING, "SSH tunnel")
    session._worker = None


# --- log and tunnel helpers --------------------------------------------------


def test_the_log_buffer_hands_out_each_line_once(tmp_path: Path) -> None:
    buf = stream_log.StreamLogBuffer(maxlen=10)
    logger = logging.getLogger("test_argus_stream_robust.buffer")
    logger.addHandler(buf)
    logger.setLevel(logging.INFO)
    try:
        logger.info("one")
        logger.info("two")
        through, lines = buf.unsent()
        assert [ln.rsplit(" ", 1)[-1] for ln in lines] == ["one", "two"]
        buf.mark_sent(through)
        assert buf.unsent() == (0, [])
        logger.warning("three")
        assert buf.unsent()[1][0].endswith("three")
    finally:
        logger.removeHandler(buf)
    stream_log.install(tmp_path)
    stream_log.install(tmp_path)  # idempotent
    root = logging.getLogger(stream_log.LOGGER_NAME)
    assert root.handlers.count(stream_log.BUFFER) == 1
    root.info("to the file")
    for handler in root.handlers:
        handler.flush()
    assert "to the file" in (tmp_path / stream_log.LOG_FILE_NAME).read_text()


def test_a_busy_local_port_is_recognized() -> None:
    assert port_busy(["bind [127.0.0.1]:5555: Address already in use"])
    assert port_busy(["channel_setup_fwd_listener_tcpip: cannot listen to port: 5556"])
    assert not port_busy(["Timeout, server not responding."])
