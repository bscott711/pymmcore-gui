"""The backfill sender's wrapper in this app (``_argus_stream.backfill_send``).

The sender itself (scanning, path mapping, the wire exchange, the summary) is
``opym.stream.backfill_client``; its tests live in the opym repo, against the real
receiver. These pin what is left here: the module path the rig's scheduled task runs,
the Settings-based defaults, and the ``ArgusTunnelManager`` hand-over.
"""

from __future__ import annotations

import os
import subprocess
import sys
from types import SimpleNamespace
from typing import TYPE_CHECKING

from opym.stream import backfill_client

from pymmcore_gui._argus_stream import backfill_send as bs

if TYPE_CHECKING:
    from pathlib import Path

    import pytest


def _settings(monkeypatch: pytest.MonkeyPatch, host: str, links: int) -> None:
    from pymmcore_gui._settings import Settings

    stream = SimpleNamespace(ssh_host=host, stream_links=links)
    monkeypatch.setattr(
        Settings, "instance", lambda: SimpleNamespace(argus_stream=stream)
    )


def test_the_defaults_are_the_streams_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    _settings(monkeypatch, "RigArgus", 3)
    assert bs.sender_defaults() == backfill_client.SenderDefaults("RigArgus", 3)
    _settings(monkeypatch, "RigArgus", 0)  # at least one link
    assert bs.sender_defaults().links == 1


def test_unreadable_settings_fall_back_to_opyms_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from pymmcore_gui._settings import Settings

    def _boom() -> None:
        raise RuntimeError("settings file is broken")

    monkeypatch.setattr(Settings, "instance", _boom)
    assert bs.sender_defaults() == backfill_client.SenderDefaults()
    assert bs.sender_defaults() == backfill_client.SenderDefaults("Argus", 4)


def test_no_tunnel_means_a_plain_message_not_a_hang(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """The app's ``ArgusTunnelManager`` is the factory, with the stream's links."""
    started: list[tuple[str, int, int, int]] = []

    class _DeadTunnel:
        def __init__(self, host: str, local: int, remote: int, links: int = 1) -> None:
            started.append((host, local, remote, links))
            self.endpoints = [f"tcp://127.0.0.1:{local + k}" for k in range(links)]

        def start(self) -> None:
            pass

        def stop(self) -> None:
            started.append(("stopped", 0, 0, 0))

    from pymmcore_gui._argus_stream import _tunnel

    monkeypatch.setattr(_tunnel, "ArgusTunnelManager", _DeadTunnel)
    monkeypatch.setattr(backfill_client, "TUNNEL_UP_TIMEOUT_S", 0.3)
    _settings(monkeypatch, "RigArgus", 4)
    code = bs.main(["--map", f"{tmp_path}=/remote", "--local-port", "1"])
    assert code == 2
    # The Settings host and as many links as the stream; opym's remote port.
    assert started[0] == ("RigArgus", 1, backfill_client.REMOTE_PORT, 4)
    assert started[-1][0] == "stopped"
    assert "Could not open an SSH tunnel to 'RigArgus'" in capsys.readouterr().out


def test_the_scheduled_tasks_module_path_runs() -> None:
    """The rig runs ``python -m pymmcore_gui._argus_stream.backfill_send --map ...``."""
    # The child sees the same import path as this process (a checkout, not an install).
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    done = subprocess.run(
        [sys.executable, "-m", "pymmcore_gui._argus_stream.backfill_send", "--help"],
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    assert "--map" in done.stdout
    assert "--connect" in done.stdout
