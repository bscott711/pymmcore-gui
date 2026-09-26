"""Manage persistent SSH local port-forwards to the Argus streaming receiver.

The receiver binds to ``127.0.0.1`` on Argus (never a public interface) and
is reachable only through SSH tunnels over the connection already configured
for ``ssh <Host>`` (see ``~/.ssh/config``). This is an app-lifetime service,
not scoped to a single MDA run, so back-to-back acquisitions never pay
tunnel-setup latency -- see
:class:`~pymmcore_gui._argus_stream._session.ArgusStreamSession`, which mints
a fresh ZMQ session per run but shares the tunnels across all of them.

Links: one SSH connection moves about 33 MB/s to Argus (2026-09-25), far
below what the network carries (Globus bursts fill the PC's 10 Gbps NIC).
The cap is per connection: sshd's fixed 2 MB channel window allows at most
2 MB per round trip, a single TCP connection has a single congestion window,
and one ``ssh`` process encrypts on one thread. So the manager forwards
``links`` local ports, ``local_port`` .. ``local_port + links - 1``, all to
the same receiver port, and a run sends over all of them at once (the same
trick Globus uses). The links are spread over ``ssh_processes`` separate
``ssh`` processes (separate TCP connections); several links in one process
are separate SSH channels sharing that connection.

ZMQ's ``DEALER`` sockets auto-reconnect to their local port on their own
once it listens again, so this manager's only job is to keep every local
port listening: if an ``ssh`` process dies (network blip, Argus reboot,
etc.), it's respawned with exponential backoff.
"""

from __future__ import annotations

import logging
import subprocess
import sys
import threading
import time
from collections import deque
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

logger = logging.getLogger(__name__)

_POLL_INTERVAL_S = 1.0
_BACKOFF_START_S = 1.0
_BACKOFF_MAX_S = 30.0
# A process that stayed up this long resets the backoff.
_HEALTHY_AFTER_S = 60.0
# Between process starts, so a burst of logins stays under sshd's MaxStartups.
_STAGGER_S = 0.5
# Keepalive probes every 5 s, three unanswered = dead: a black-holed TCP
# connection is noticed in ~15 s (was 15 s x 3 = 45 s).
_SERVER_ALIVE_INTERVAL_S = 5
_SERVER_ALIVE_COUNT_MAX = 3
_CONNECT_TIMEOUT_S = 10
_STDERR_TAIL_LINES = 20


def link_endpoints(local_port: int, links: int) -> list[str]:
    """The local ZMQ endpoint of each link, in link order."""
    return [f"tcp://127.0.0.1:{local_port + k}" for k in range(max(1, links))]


def _creationflags() -> int:
    if sys.platform != "win32":
        return 0
    # Below-normal priority: encrypting the stream must never take CPU from
    # the acquisition threads.
    return subprocess.CREATE_NO_WINDOW | subprocess.BELOW_NORMAL_PRIORITY_CLASS


def _terminate(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)
    if proc.stderr is not None:
        proc.stderr.close()


class _SshProcess:
    """One supervised ``ssh -N`` process carrying one or more forwards."""

    def __init__(
        self,
        ssh_host: str,
        forwards: list[tuple[int, int]],
        ssh_args: Sequence[str],
        start_delay_s: float,
    ) -> None:
        self.ssh_host = ssh_host
        self.forwards = forwards
        self.ssh_args = list(ssh_args)
        self.start_delay_s = start_delay_s
        self.proc: subprocess.Popen[bytes] | None = None
        self.restarts = 0
        self._stderr_tail: deque[str] = deque(maxlen=_STDERR_TAIL_LINES)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def command(self) -> list[str]:
        cmd = [
            "ssh",
            "-N",
            "-o",
            "BatchMode=yes",
            "-o",
            "ExitOnForwardFailure=yes",
            "-o",
            f"ServerAliveInterval={_SERVER_ALIVE_INTERVAL_S}",
            "-o",
            f"ServerAliveCountMax={_SERVER_ALIVE_COUNT_MAX}",
            "-o",
            f"ConnectTimeout={_CONNECT_TIMEOUT_S}",
        ]
        for local, remote in self.forwards:
            cmd += ["-L", f"{local}:127.0.0.1:{remote}"]
        return [*cmd, *self.ssh_args, self.ssh_host]

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=f"ArgusTunnel-{self.forwards[0][0]}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
        if self.proc is not None:  # the thread didn't get to it
            _terminate(self.proc)
        self.proc = None

    def _drain_stderr(self, proc: subprocess.Popen[bytes]) -> None:
        # Read continuously: a full, unread stderr pipe would block ssh.
        assert proc.stderr is not None
        for raw in proc.stderr:
            line = raw.decode(errors="replace").rstrip()
            if line:
                self._stderr_tail.append(line)
                logger.debug("ssh %s: %s", self.forwards[0][0], line)

    def _run(self) -> None:
        if self._stop.wait(self.start_delay_s):
            return
        backoff = _BACKOFF_START_S
        while not self._stop.is_set():
            self._stderr_tail.clear()
            try:
                proc = subprocess.Popen(
                    self.command(),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    creationflags=_creationflags(),
                )
            except OSError:
                logger.exception("Failed to spawn SSH tunnel to %s", self.ssh_host)
                self._stop.wait(backoff)
                backoff = min(backoff * 2, _BACKOFF_MAX_S)
                continue
            self.proc = proc
            started = time.monotonic()
            reader = threading.Thread(
                target=self._drain_stderr, args=(proc,), daemon=True
            )
            reader.start()
            logger.info(
                "Argus SSH tunnel started (pid=%s, local %s -> %s:%d)",
                proc.pid,
                ",".join(str(local) for local, _ in self.forwards),
                self.ssh_host,
                self.forwards[0][1],
            )
            while not self._stop.is_set() and proc.poll() is None:
                self._stop.wait(_POLL_INTERVAL_S)
            if self._stop.is_set():
                _terminate(proc)
                reader.join(timeout=1)
                self.proc = None
                break
            reader.join(timeout=1)
            if proc.stderr is not None:
                proc.stderr.close()
            logger.warning(
                "Argus SSH tunnel on local %s exited (code=%s): %s",
                ",".join(str(local) for local, _ in self.forwards),
                proc.returncode,
                " | ".join(self._stderr_tail) or "no output",
            )
            self.restarts += 1
            if time.monotonic() - started >= _HEALTHY_AFTER_S:
                backoff = _BACKOFF_START_S
            self._stop.wait(backoff)
            backoff = min(backoff * 2, _BACKOFF_MAX_S)


class ArgusTunnelManager:
    """Spawns and supervises the ``ssh -N -L ...`` processes behind the links.

    Parameters
    ----------
    ssh_host : str
        The ``Host`` alias from ``~/.ssh/config`` to tunnel through.
    local_port : int
        Local end of link 0's forward; link k listens on ``local_port + k``.
        These are what the run's ZMQ ``DEALER`` sockets connect to.
    remote_port : int
        Port the Argus-side receiver binds to on ``127.0.0.1``. Every link
        forwards to it.
    links : int
        How many forwards (parallel connections) to keep up.
    ssh_processes : int
        How many ``ssh`` processes carry them; 0 means one per link.
        Link k is carried by process ``k % ssh_processes``.
    ssh_args : Sequence[str]
        Extra ``ssh`` arguments, placed before the host (e.g. ``-c`` or
        ``-F``).
    """

    def __init__(
        self,
        ssh_host: str,
        local_port: int,
        remote_port: int,
        links: int = 1,
        ssh_processes: int = 0,
        ssh_args: Sequence[str] = (),
    ) -> None:
        self._ssh_host = ssh_host
        self._local_port = local_port
        self._remote_port = remote_port
        self._links = max(1, links)
        n_procs = self._links if ssh_processes <= 0 else min(ssh_processes, self._links)
        self._procs = [
            _SshProcess(
                ssh_host,
                [
                    (local_port + k, remote_port)
                    for k in range(self._links)
                    if k % n_procs == p
                ],
                ssh_args,
                start_delay_s=p * _STAGGER_S,
            )
            for p in range(n_procs)
        ]

    @property
    def endpoints(self) -> list[str]:
        """The local ZMQ endpoint of each link, in link order."""
        return link_endpoints(self._local_port, self._links)

    @property
    def is_running(self) -> bool:
        """Whether any tunnel subprocess is currently alive.

        This only reflects the local ``ssh`` processes' liveness, not
        end-to-end reachability of the Argus-side receiver -- the ZMQ layer
        is what actually discovers that.
        """
        return any(p.alive for p in self._procs)

    @property
    def processes_alive(self) -> int:
        return sum(p.alive for p in self._procs)

    @property
    def restarts(self) -> int:
        """Total respawns across all processes since start."""
        return sum(p.restarts for p in self._procs)

    def commands(self) -> list[list[str]]:
        """The ``ssh`` command line of each process (for logs and tests)."""
        return [p.command() for p in self._procs]

    def start(self) -> None:
        """Start every tunnel and its auto-restart monitor, if not running."""
        for proc in self._procs:
            proc.start()

    def stop(self) -> None:
        """Stop every tunnel subprocess and its monitor thread."""
        for proc in self._procs:  # all at once, then wait for each
            proc._stop.set()
        for proc in self._procs:
            proc.stop()
