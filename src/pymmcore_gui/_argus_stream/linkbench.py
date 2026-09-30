"""Measure how fast the acquisition PC can push data to Argus through SSH.

Run on the acquisition PC, between acquisitions::

    uv run python -m pymmcore_gui._argus_stream.linkbench

It needs ``opym-linkbench-sink`` listening on Argus (``127.0.0.1:5599``).
For each case it opens the tunnels with the same
:class:`~pymmcore_gui._argus_stream._tunnel.ArgusTunnelManager` the app
uses, pushes incompressible 6 MB messages (about one compressed slab)
through every link for a fixed time, and counts what Argus confirms it
received. The cases answer three questions:

- Does throughput scale with parallel SSH connections (1, 2, 4, 8)?
- Several links in ONE ssh process: if that scales too, the cap is the SSH
  channel window (2 MB per round trip); if not, it's per connection or per
  process.
- Is AES-GCM faster than the default cipher here?

It also times blosc compression (what the stream does before sending) on
a real saved stack (``--stack``) or a synthetic one. Results are printed and
sent to the sink, which records them on Argus.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import statistics
import struct
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import zmq
from zmq.utils.monitor import recv_monitor_message

from ._tunnel import ArgusTunnelManager

_PAYLOAD_BYTES = 6 * 1024 * 1024
_IN_FLIGHT_PER_LINK = 3
_CONNECT_TIMEOUT_S = 45.0
_PINGS = 10
_WARMUP_S = 3.0
_AES = ("-c", "aes128-gcm@openssh.com")


@dataclass
class Case:
    name: str
    links: int
    ssh_processes: int
    ssh_args: tuple[str, ...] = ()


@dataclass
class CaseResult:
    name: str
    links: int
    ssh_processes: int
    ssh_args: list[str]
    links_up: int = 0
    connect_s: float | None = None
    rtt_ms_p50: float | None = None
    mb_s: float = 0.0
    steady_mb_s: float = 0.0
    per_link_mb_s: list[float] = field(default_factory=list)
    cpu_cores: float | None = None
    error: str = ""


def default_cases(link_counts: list[int], shared: bool, cipher: bool) -> list[Case]:
    cases = [Case(f"{n} ssh x 1 link", n, n) for n in link_counts]
    if shared:
        cases.append(Case("1 ssh x 4 links", 4, 1))
    if cipher:
        cases += [
            Case(f"{n} ssh x 1 link, aes128-gcm", n, n, _AES)
            for n in (1, 4)
            if n in link_counts
        ]
    return cases


def _cpu_sampler() -> Any:
    """Return ``sample() -> cpu seconds`` of this process and its children."""
    try:
        import psutil
    except ImportError:
        return lambda: None

    me = psutil.Process()

    def sample() -> float:
        total = sum(me.cpu_times()[:2])
        for child in me.children(recursive=True):
            try:
                total += sum(child.cpu_times()[:2])
            except psutil.Error:
                pass
        return float(total)

    return sample


def _open_link(ctx: zmq.Context, endpoint: str, identity: bytes) -> tuple:
    sock = ctx.socket(zmq.DEALER)
    sock.setsockopt(zmq.IDENTITY, identity)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.IMMEDIATE, 1)
    mon = sock.get_monitor_socket(zmq.EVENT_HANDSHAKE_SUCCEEDED)
    sock.connect(endpoint)
    return sock, mon


def _wait_up(links: list[tuple], timeout_s: float) -> tuple[set[int], float | None]:
    """Wait for every link's handshake; return the ones up and how long."""
    start = time.monotonic()
    up: set[int] = set()
    poller = zmq.Poller()
    for _sock, mon in links:
        poller.register(mon, zmq.POLLIN)
    while len(up) < len(links) and time.monotonic() - start < timeout_s:
        for mon, _ in poller.poll(200):
            recv_monitor_message(mon)
            up.add(next(i for i, (_s, m) in enumerate(links) if m is mon))
    elapsed = time.monotonic() - start if len(up) == len(links) else None
    return up, elapsed


def _rtt_ms(sock: zmq.Socket) -> float | None:
    samples = []
    for seq in range(_PINGS):
        t0 = time.perf_counter()
        sock.send_multipart([b"PING", struct.pack("<q", seq)])
        if not sock.poll(5000):
            return None
        sock.recv_multipart()
        samples.append((time.perf_counter() - t0) * 1000)
    return statistics.median(samples)


def _push(links: list[tuple], seconds: float, payload: bytes) -> tuple:
    """Keep _IN_FLIGHT_PER_LINK messages outstanding on every link."""
    socks = [s for s, _ in links]
    in_flight = [0] * len(socks)
    confirmed = [0] * len(socks)
    steady_start: list[int] | None = None
    steady_t0 = 0.0
    poller = zmq.Poller()
    for sock in socks:
        poller.register(sock, zmq.POLLIN)
    seq = 0
    start = time.monotonic()
    end = start + seconds
    while time.monotonic() < end:
        for i, sock in enumerate(socks):
            while in_flight[i] < _IN_FLIGHT_PER_LINK:
                try:
                    sock.send_multipart(
                        [b"DATA", struct.pack("<q", seq), payload],
                        flags=zmq.NOBLOCK,
                        copy=False,
                    )
                except zmq.Again:
                    break
                in_flight[i] += 1
                seq += 1
        for sock, _ in poller.poll(100):
            i = socks.index(sock)
            while sock.poll(0):
                parts = sock.recv_multipart()
                if parts[0] == b"RCPT":
                    in_flight[i] -= 1
                    confirmed[i] += struct.unpack("<q", parts[2])[0]
        if steady_start is None and time.monotonic() - start >= _WARMUP_S:
            steady_start = list(confirmed)
            steady_t0 = time.monotonic()
    elapsed = time.monotonic() - start
    steady = 0.0
    if steady_start is not None:
        steady = (sum(confirmed) - sum(steady_start)) / (time.monotonic() - steady_t0)
    return confirmed, elapsed, steady


def run_case(
    case: Case,
    host: str,
    remote_port: int,
    local_port: int,
    seconds: float,
    payload: bytes,
    ssh_args: tuple[str, ...],
) -> CaseResult:
    result = CaseResult(
        case.name, case.links, case.ssh_processes, [*ssh_args, *case.ssh_args]
    )
    tunnel = ArgusTunnelManager(
        host,
        local_port,
        remote_port,
        links=case.links,
        ssh_processes=case.ssh_processes,
        ssh_args=[*ssh_args, *case.ssh_args],
    )
    ctx = zmq.Context.instance()
    tag = f"lb-{os.getpid()}-{local_port}"
    links: list[tuple] = []
    tunnel.start()
    try:
        links = [
            _open_link(ctx, ep, f"{tag}-{k}".encode())
            for k, ep in enumerate(tunnel.endpoints)
        ]
        up, result.connect_s = _wait_up(links, _CONNECT_TIMEOUT_S)
        result.links_up = len(up)
        if not up:
            result.error = "no link came up (is opym-linkbench-sink running?)"
            return result
        live = [link for k, link in enumerate(links) if k in up]
        result.rtt_ms_p50 = _rtt_ms(live[0][0])
        cpu = _cpu_sampler()
        cpu0 = cpu()
        confirmed, elapsed, steady = _push(live, seconds, payload)
        cpu1 = cpu()
        result.mb_s = round(sum(confirmed) / elapsed / 1e6, 1)
        result.steady_mb_s = round(steady / 1e6, 1)
        result.per_link_mb_s = [round(b / elapsed / 1e6, 1) for b in confirmed]
        if cpu0 is not None and cpu1 is not None:
            result.cpu_cores = round((cpu1 - cpu0) / elapsed, 2)
        _report(live[0][0], {"kind": "case", **asdict(result)})
    except Exception as exc:
        result.error = repr(exc)
    finally:
        for sock, mon in links:
            sock.disable_monitor()
            mon.close(linger=0)
            sock.close(linger=0)
        tunnel.stop()
    return result


def _report(sock: zmq.Socket, record: dict) -> bool:
    """Send one record to the sink; True once it confirms.

    Receipts for data still in flight arrive first, so skip past them.
    """
    sock.send_multipart([b"RESULT", json.dumps(record).encode()])
    deadline = time.monotonic() + 30
    while (left := deadline - time.monotonic()) > 0:
        if not sock.poll(int(left * 1000)):
            break
        if sock.recv_multipart()[0] == b"OK":
            return True
    return False


def _load_stack(path: Path | None) -> tuple[np.ndarray, str]:
    if path is None:
        rng = np.random.default_rng(0)
        planes, ny, nx = 32, 490, 1458
        yy, xx = np.mgrid[0:ny, 0:nx]
        cell = 400 * np.exp(-(((yy - 245) / 80) ** 2 + ((xx - 729) / 200) ** 2))
        stack = rng.poisson(100 + cell, size=(planes, ny, nx)).astype("uint16")
        return stack, "synthetic (ratio is not representative)"
    arr: Any
    if path.suffix == ".npy":
        arr = np.load(path, mmap_mode="r")
    elif path.suffix in (".tif", ".tiff"):
        import tifffile

        arr = tifffile.imread(path)
    else:
        import zarr

        node: Any = zarr.open(str(path), mode="r")
        while not hasattr(node, "shape"):  # descend into the first array
            node = node[sorted(node.array_keys() or node.group_keys())[0]]
        arr = node
    while arr.ndim > 3:
        arr = arr[0]
    return np.ascontiguousarray(arr[: min(arr.shape[0], 64)]), str(path)


def compression_bench(stack: np.ndarray) -> list[dict]:
    """Single-threaded, off the main thread: how the stream compresses."""
    from numcodecs import blosc  # pyright: ignore[reportAttributeAccessIssue]

    results: list[dict] = []

    def work() -> None:
        for cname, level in (("lz4", 5), ("zstd", 1), ("zstd", 3)):
            t0 = time.perf_counter()
            out = blosc.compress(stack, cname.encode(), level, blosc.BITSHUFFLE)
            dt = time.perf_counter() - t0
            results.append(
                {
                    "codec": f"{cname} {level} bitshuffle",
                    "ratio": round(stack.nbytes / len(out), 2),
                    "compress_mb_s": round(stack.nbytes / dt / 1e6),
                }
            )

    thread = threading.Thread(target=work)
    thread.start()
    thread.join()
    return results


def _ssh_version() -> str:
    try:
        out = subprocess.run(
            ["ssh", "-V"], capture_output=True, text=True, timeout=10, check=False
        )
        return (out.stderr or out.stdout).strip()
    except OSError as exc:
        return repr(exc)


def _default_host() -> str:
    try:
        from pymmcore_gui._settings import Settings

        return Settings.instance().argus_stream.ssh_host
    except Exception:
        return "Argus"


def _print_table(results: list[CaseResult]) -> None:
    print()
    print(f"{'case':34} {'up':>4} {'MB/s':>7} {'steady':>7} {'rtt ms':>7} {'cpu':>5}")
    for r in results:
        rtt = f"{r.rtt_ms_p50:.1f}" if r.rtt_ms_p50 is not None else "-"
        cpu = f"{r.cpu_cores:.2f}" if r.cpu_cores is not None else "-"
        print(
            f"{r.name:34} {r.links_up:>2}/{r.links:<1} {r.mb_s:>7.1f} "
            f"{r.steady_mb_s:>7.1f} {rtt:>7} {cpu:>5}"
            + (f"  ERROR {r.error}" if r.error else "")
        )
        if r.per_link_mb_s and len(r.per_link_mb_s) > 1:
            print(f"{'':34}   per link: {r.per_link_mb_s}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("--host", default=None, help="ssh Host alias (settings)")
    parser.add_argument("--remote-port", type=int, default=5599)
    parser.add_argument("--local-port", type=int, default=5600)
    parser.add_argument("--links", default="1,2,4,8")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--no-shared", action="store_true")
    parser.add_argument("--no-cipher", action="store_true")
    parser.add_argument("--stack", type=Path, default=None)
    parser.add_argument(
        "--ssh-arg", action="append", default=[], help="extra ssh argument"
    )
    args = parser.parse_args(argv)
    host = args.host or _default_host()
    counts = [int(n) for n in args.links.split(",") if n.strip()]
    cases = default_cases(counts, not args.no_shared, not args.no_cipher)
    payload = os.urandom(_PAYLOAD_BYTES)
    ssh_args = tuple(args.ssh_arg)

    print(f"Argus link benchmark: host {host!r}, {len(cases)} cases")
    print(f"{_ssh_version()} on {platform.platform()}, {os.cpu_count()} CPUs")
    results = []
    for i, case in enumerate(cases):
        print(f"  {case.name} ...", flush=True)
        results.append(
            run_case(
                case,
                host,
                args.remote_port,
                args.local_port + 10 * i,
                args.seconds,
                payload,
                ssh_args,
            )
        )
    _print_table(results)

    stack, source = _load_stack(args.stack)
    comp = compression_bench(stack)
    print(f"\nCompression on {source}, {stack.nbytes / 1e6:.0f} MB:")
    for c in comp:
        print(f"  {c['codec']:22} ratio {c['ratio']:>5}  {c['compress_mb_s']:>5} MB/s")

    summary = {
        "kind": "summary",
        "host": host,
        "ssh": _ssh_version(),
        "platform": platform.platform(),
        "cpus": os.cpu_count(),
        "compression": comp,
        "compression_source": source,
        "cases": [asdict(r) for r in results],
    }
    tunnel = ArgusTunnelManager(
        host, args.local_port - 1, args.remote_port, ssh_args=ssh_args
    )
    tunnel.start()
    ctx = zmq.Context.instance()
    sock, mon = _open_link(ctx, tunnel.endpoints[0], f"lb-{os.getpid()}-sum".encode())
    try:
        if _wait_up([(sock, mon)], _CONNECT_TIMEOUT_S)[0] and _report(sock, summary):
            print("\nResults recorded on Argus.")
        else:
            print("\nCould not reach Argus to record the results; paste them instead.")
    finally:
        sock.disable_monitor()
        mon.close(linger=0)
        sock.close(linger=0)
        tunnel.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
