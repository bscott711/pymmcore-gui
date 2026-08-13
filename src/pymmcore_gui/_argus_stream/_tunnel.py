"""Manage a persistent SSH local port-forward to the Argus streaming receiver.

The receiver binds to ``127.0.0.1`` on Argus (never a public interface) and
is reachable only through an SSH tunnel over the connection already
configured for ``ssh <Host>`` (see ``~/.ssh/config``). This is an
app-lifetime service, not scoped to a single MDA run, so back-to-back
acquisitions never pay tunnel-setup latency -- see
:class:`~pymmcore_gui._argus_stream._session.ArgusStreamSession`, which
mints a fresh ZMQ session per run but shares one tunnel across all of them.

ZMQ's ``DEALER`` socket auto-reconnects to the same address on its own once
the local port starts listening again, so this manager's only job is to
keep that local port listening: if the ``ssh`` subprocess dies (network
blip, Argus reboot, etc.), it's respawned with a short backoff.
"""

from __future__ import annotations

import logging
import subprocess
import sys
import threading

logger = logging.getLogger(__name__)

_POLL_INTERVAL_S = 2.0
_RESTART_BACKOFF_S = 5.0


class ArgusTunnelManager:
    """Spawns and supervises ``ssh -N -L <local_port>:127.0.0.1:<remote_port> <host>``.

    Parameters
    ----------
    ssh_host : str
        The ``Host`` alias from ``~/.ssh/config`` to tunnel through.
    local_port : int
        Local end of the forward -- what the ZMQ ``DEALER`` socket connects to.
    remote_port : int
        Port the Argus-side receiver binds to on ``127.0.0.1``.
    """

    def __init__(self, ssh_host: str, local_port: int, remote_port: int) -> None:
        self._ssh_host = ssh_host
        self._local_port = local_port
        self._remote_port = remote_port
        self._proc: subprocess.Popen[bytes] | None = None
        self._monitor_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @property
    def is_running(self) -> bool:
        """Whether the tunnel subprocess is currently alive.

        This only reflects the local ``ssh`` process's liveness, not
        end-to-end reachability of the Argus-side receiver -- the ZMQ layer
        is what actually discovers that.
        """
        return self._proc is not None and self._proc.poll() is None

    def start(self) -> None:
        """Start the tunnel and its auto-restart monitor, if not already running."""
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            return
        self._stop_event.clear()
        self._monitor_thread = threading.Thread(
            target=self._run, name="ArgusTunnelManager", daemon=True
        )
        self._monitor_thread.start()

    def stop(self) -> None:
        """Stop the tunnel subprocess and its monitor thread."""
        self._stop_event.set()
        if self._proc is not None:
            self._proc.terminate()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=5)
        self._proc = None

    def _spawn(self) -> subprocess.Popen[bytes]:
        cmd = [
            "ssh",
            "-N",
            "-o",
            "BatchMode=yes",
            "-o",
            "ExitOnForwardFailure=yes",
            "-o",
            "ServerAliveInterval=15",
            "-o",
            "ServerAliveCountMax=3",
            "-L",
            f"{self._local_port}:127.0.0.1:{self._remote_port}",
            self._ssh_host,
        ]
        creationflags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
        return subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            creationflags=creationflags,
        )

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._proc = self._spawn()
            except OSError:
                logger.exception("Failed to spawn SSH tunnel to %s", self._ssh_host)
                self._stop_event.wait(_RESTART_BACKOFF_S)
                continue

            logger.info(
                "Argus SSH tunnel started (pid=%s, %s -> 127.0.0.1:%d via "
                "127.0.0.1:%d)",
                self._proc.pid,
                self._ssh_host,
                self._remote_port,
                self._local_port,
            )
            while not self._stop_event.is_set():
                if self._proc.poll() is not None:
                    stderr = self._proc.stderr.read() if self._proc.stderr else b""
                    logger.warning(
                        "Argus SSH tunnel exited (code=%s): %s",
                        self._proc.returncode,
                        stderr.decode(errors="replace"),
                    )
                    break
                self._stop_event.wait(_POLL_INTERVAL_S)

            if self._stop_event.is_set():
                break
            self._stop_event.wait(_RESTART_BACKOFF_S)
