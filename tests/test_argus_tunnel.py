from __future__ import annotations

import logging
import struct
import sys
import threading
import time
from typing import TYPE_CHECKING

import pytest
import zmq

from pymmcore_gui._argus_stream import _tunnel, linkbench
from pymmcore_gui._argus_stream._tunnel import ArgusTunnelManager, link_endpoints

if TYPE_CHECKING:
    from collections.abc import Iterator


def _forwards(cmd: list[str]) -> list[str]:
    return [cmd[i + 1] for i, arg in enumerate(cmd) if arg == "-L"]


def test_each_link_gets_its_own_local_port() -> None:
    tunnel = ArgusTunnelManager("Argus", 5555, 5555, links=4)
    assert tunnel.endpoints == [f"tcp://127.0.0.1:{p}" for p in range(5555, 5559)]
    assert tunnel.endpoints == link_endpoints(5555, 4)
    # one ssh process per link by default
    assert [_forwards(c) for c in tunnel.commands()] == [
        [f"{p}:127.0.0.1:5555"] for p in range(5555, 5559)
    ]


def test_links_are_spread_over_fewer_ssh_processes() -> None:
    tunnel = ArgusTunnelManager("Argus", 6000, 5555, links=4, ssh_processes=2)
    assert [_forwards(c) for c in tunnel.commands()] == [
        ["6000:127.0.0.1:5555", "6002:127.0.0.1:5555"],
        ["6001:127.0.0.1:5555", "6003:127.0.0.1:5555"],
    ]


def test_one_link_is_the_old_single_tunnel() -> None:
    (cmd,) = ArgusTunnelManager("Argus", 5555, 5555).commands()
    assert _forwards(cmd) == ["5555:127.0.0.1:5555"]


def test_ssh_notices_a_dead_peer_quickly_and_extra_args_precede_the_host() -> None:
    (cmd,) = ArgusTunnelManager(
        "Argus", 5555, 5555, ssh_args=["-c", "aes128-gcm@openssh.com"]
    ).commands()
    assert cmd[0] == "ssh"
    assert cmd[-3:] == ["-c", "aes128-gcm@openssh.com", "Argus"]
    for opt in (
        "BatchMode=yes",
        "ExitOnForwardFailure=yes",
        "ServerAliveInterval=5",
        "ServerAliveCountMax=3",
        "ConnectTimeout=10",
    ):
        assert opt in cmd


def test_a_dead_ssh_is_respawned_and_its_stderr_logged(
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    monkeypatch.setattr(_tunnel, "_BACKOFF_START_S", 0.05)
    monkeypatch.setattr(_tunnel, "_POLL_INTERVAL_S", 0.05)
    monkeypatch.setattr(
        _tunnel._SshProcess,
        "command",
        lambda self: [
            sys.executable,
            "-c",
            "import sys; sys.stderr.write('channel 0: open failed\\n'); sys.exit(255)",
        ],
    )
    tunnel = ArgusTunnelManager("Argus", 5555, 5555, links=2)
    with caplog.at_level(logging.WARNING, logger=_tunnel.__name__):
        tunnel.start()
        deadline = time.monotonic() + 20
        while tunnel.restarts < 4 and time.monotonic() < deadline:
            time.sleep(0.05)
        tunnel.stop()
    assert tunnel.restarts >= 4
    assert "channel 0: open failed" in caplog.text
    assert not tunnel.is_running


def test_stop_ends_a_running_ssh(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        _tunnel._SshProcess,
        "command",
        lambda self: [sys.executable, "-c", "import time; time.sleep(60)"],
    )
    tunnel = ArgusTunnelManager("Argus", 5555, 5555, links=1)
    tunnel.start()
    deadline = time.monotonic() + 10
    while not tunnel.is_running and time.monotonic() < deadline:
        time.sleep(0.05)
    assert tunnel.is_running
    tunnel.stop()
    assert not tunnel.is_running


# ----------------------------------------------------------------------------
# linkbench against an in-process sink (same protocol as opym.stream.linkbench)
# ----------------------------------------------------------------------------


class _Sink:
    def __init__(self) -> None:
        self.sock = zmq.Context.instance().socket(zmq.ROUTER)
        self.ports = []
        for _ in range(4):
            self.ports.append(self.sock.bind_to_random_port("tcp://127.0.0.1"))
        self.results: list[bytes] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.sock.poll(50):
                continue
            ident, kind, *rest = self.sock.recv_multipart()
            if kind == b"DATA":
                reply = [b"RCPT", rest[0], struct.pack("<q", len(rest[1]))]
            elif kind == b"PING":
                reply = [b"PONG", rest[0]]
            else:
                self.results.append(rest[0])
                reply = [b"OK"]
            self.sock.send_multipart([ident, *reply])

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.sock.close(linger=0)


@pytest.fixture
def sink() -> Iterator[_Sink]:
    s = _Sink()
    yield s
    s.close()


def test_linkbench_measures_every_link_and_reports_to_the_sink(
    sink: _Sink, monkeypatch: pytest.MonkeyPatch
) -> None:
    class _FakeTunnel:
        def __init__(self, *_a: object, links: int = 1, **_k: object) -> None:
            self.endpoints = [f"tcp://127.0.0.1:{p}" for p in sink.ports[:links]]

        def start(self) -> None:
            pass

        def stop(self) -> None:
            pass

    monkeypatch.setattr(linkbench, "ArgusTunnelManager", _FakeTunnel)
    monkeypatch.setattr(linkbench, "_WARMUP_S", 0.2)
    case = linkbench.Case("4 ssh x 1 link", 4, 4)
    result = linkbench.run_case(case, "Argus", 0, 0, 1.0, b"x" * 65536, ())
    assert result.error == ""
    assert result.links_up == 4
    assert len(result.per_link_mb_s) == 4
    assert all(rate > 0 for rate in result.per_link_mb_s)
    assert result.rtt_ms_p50 is not None
    assert len(sink.results) == 1
    assert b'"kind": "case"' in sink.results[0]


def test_linkbench_default_cases_cover_scaling_sharing_and_cipher() -> None:
    names = [c.name for c in linkbench.default_cases([1, 2, 4, 8], True, True)]
    assert names == [
        "1 ssh x 1 link",
        "2 ssh x 1 link",
        "4 ssh x 1 link",
        "8 ssh x 1 link",
        "1 ssh x 4 links",
        "1 ssh x 1 link, aes128-gcm",
        "4 ssh x 1 link, aes128-gcm",
    ]
