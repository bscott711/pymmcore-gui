r"""Send finished acquisitions from this PC to Argus over SSH (``opym-backfill-send``).

For runs the live stream didn't bring in whole (Argus closed or the PC
paused them, or streaming was off). Scans local acquisition folders for
finished ``*.ome.zarr`` stores, asks Argus's backfill receiver
(``opym-receive-backfill``) what it already holds, and sends only what is
missing: each store's small files verbatim, then pixel volumes as slabs. A
store is finished with a per-volume SHA-256 manifest that Argus checks by
reading every volume back. Argus never overwrites: a store that already holds
different data is reported as a ``conflict`` and left alone.

It runs in this app's own environment, so the PC needs nothing else::

    uv run --no-sync python -m pymmcore_gui._argus_stream.backfill_send ^
        --map S:\\=/mmfs1/scratch/jacks.local/microscopy --dry-run

``--map LOCAL=REMOTE`` keeps the path below LOCAL below REMOTE, the same
mirroring the live stream uses (``ArgusStreamSettingsV1.gpfs_scratch_root``).
It opens its own SSH tunnel to Argus (the ``ssh_host`` the stream uses) unless
``--connect`` names an endpoint. It pauses while Argus reports a live
acquisition, and skips stores written to in the last 15 minutes.

A port of ``opym.stream.backfill_client`` (the opym repo); the receiver and
the wire format (``_bfill.py``) live there.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import socket
import time
import uuid
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import zarr
import zmq

from . import _bfill as bfill

if TYPE_CHECKING:
    from collections.abc import Iterator

logger = logging.getLogger(__name__)

REMOTE_PORT = 5556
# Local end of this tool's own tunnel: clear of the live stream's 5555+.
DEFAULT_LOCAL_PORT = 5566
REPLY_TIMEOUT_S = 180.0
PAUSE_POLL_S = 10.0
MAX_RETRIES = 3
TUNNEL_UP_TIMEOUT_S = 30.0


class BackfillError(RuntimeError):
    pass


@dataclass
class LocalStore:
    name: str
    path: Path
    main_rel: str
    shape: list[int]
    chunks: list[int]
    dtype: str
    sep: str
    written_t: list[int]

    @classmethod
    def load(cls, path: Path) -> LocalStore | None:
        main_rel = bfill.main_array_rel(path)
        if main_rel is None:
            return None
        meta = json.loads((bfill.array_dir(path, main_rel) / ".zarray").read_text())
        shape = list(meta["shape"])
        return cls(
            name=path.name,
            path=path,
            main_rel=main_rel,
            shape=shape,
            chunks=list(meta["chunks"]),
            dtype=meta["dtype"],
            sep=meta.get("dimension_separator") or ".",
            written_t=bfill.written_timepoints(path, main_rel, len(shape)),
        )

    def spec(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "shape": self.shape,
            "chunks": self.chunks,
            "dtype": self.dtype,
            "written_t": self.written_t,
        }

    def array(self) -> zarr.Array:
        return zarr.open_array(str(bfill.array_dir(self.path, self.main_rel)), mode="r")

    def newest_mtime(self) -> float:
        return bfill.newest_mtime(self.path, self.main_rel)

    def planes_present(self, t: int, nz: int) -> list[int]:
        """Return the z indices whose chunk exists on disk.

        Only ``/``-separated per-plane layouts (the acquisition writer's) are
        checked; anything else sends every plane.
        """
        per_plane = self.chunks[1] == 1 and self.chunks[2:] == self.shape[2:]
        if len(self.shape) < 4 or self.sep != "/" or not per_plane:
            return list(range(nz))
        d = bfill.array_dir(self.path, self.main_rel) / str(t)
        try:
            return sorted(int(e.name) for e in os.scandir(d) if e.name.isdigit())
        except OSError:
            return []


@dataclass
class LocalLeaf:
    path: Path
    stores: list[LocalStore] = field(default_factory=list)


_TIFF_SUFFIXES = (".tif", ".tiff")


def scan_local_root(
    root: Path, max_depth: int = 6, other: Counter[str] | None = None
) -> list[LocalLeaf]:
    """Every directory under ``root`` that directly holds ``*.ome.zarr`` stores.

    TIFF files met on the way are counted in ``other`` (by folder): this tool
    sends OME-Zarr only, so they would otherwise be left behind unsaid.
    """
    leaves: list[LocalLeaf] = []

    def walk(d: Path, depth: int) -> None:
        try:
            entries = sorted(os.scandir(d), key=lambda e: e.name)
        except OSError:
            return
        stores = []
        if other is not None:
            n_tiff = sum(1 for e in entries if e.name.lower().endswith(_TIFF_SUFFIXES))
            if n_tiff:
                other[str(d)] += n_tiff
        for e in entries:
            name_ok = e.name.endswith(".ome.zarr") and bfill.valid_store_name(e.name)
            if e.is_dir() and name_ok:
                s = LocalStore.load(Path(e.path))
                if s is not None:
                    stores.append(s)
        if stores:
            leaves.append(LocalLeaf(d, stores))
        if depth < max_depth:
            for e in entries:
                hidden = e.name.startswith(".") or e.name.endswith(".ome.zarr")
                if e.is_dir() and not hidden:
                    walk(Path(e.path), depth + 1)

    walk(Path(root), 0)
    return leaves


class _Link:
    """DEALER sockets to the receiver: one per tunnel, so parallel tunnels add up."""

    def __init__(self, addrs: list[str], client_id: str) -> None:
        self.ctx = zmq.Context.instance()
        self.socks: list[zmq.Socket[bytes]] = []
        for k, addr in enumerate(addrs):
            s = self.ctx.socket(zmq.DEALER)
            s.setsockopt(zmq.IDENTITY, f"{client_id}#{k}".encode())
            s.setsockopt(zmq.LINGER, 0)
            s.connect(addr)
            self.socks.append(s)
        self.poller = zmq.Poller()
        for s in self.socks:
            self.poller.register(s, zmq.POLLIN)
        self._rr = 0

    def send(self, parts: list[bytes]) -> None:
        s = self.socks[self._rr % len(self.socks)]
        self._rr += 1
        s.send_multipart(parts)

    def recv_any(self, timeout_ms: int) -> dict[str, Any] | None:
        ready = dict(self.poller.poll(timeout_ms))
        for s in self.socks:
            if s in ready:
                msg_type, _sid, header, _payload = bfill.unpack(s.recv_multipart())
                if msg_type == bfill.BF_ACK:
                    return header
        return None

    def close(self) -> None:
        for s in self.socks:
            s.close(0)


_Inflight = dict[tuple[int, int], tuple[dict[str, Any], bytes, float, int]]


class BackfillClient:
    def __init__(
        self,
        connect: list[str],
        root_map: list[tuple[Path, str]],
        *,
        client_id: str | None = None,
        settle_s: float = 900.0,
        window: int = 8,
        slab_bytes: int = 16 << 20,
        compress: bool = True,
        max_mbps: float | None = None,
        pause_file: Path | None = None,
        order: str = "newest",
        reply_timeout_s: float = REPLY_TIMEOUT_S,
    ) -> None:
        self.root_map = [(Path(a), b.rstrip("/")) for a, b in root_map]
        # `client_id` owns the per-store lock, so it is stable across restarts
        # of this PC's sender (a crashed run resumes at once); the ZMQ
        # identity is unique per process.
        self.client_id = client_id or socket.gethostname()
        ident = f"{self.client_id}-{os.getpid()}-{uuid.uuid4().hex[:6]}"
        self.link = _Link(connect, ident)
        self.settle_s, self.window, self.slab_bytes = settle_s, window, slab_bytes
        self.compress, self.max_mbps, self.pause_file = compress, max_mbps, pause_file
        self.order, self.reply_timeout_s = order, reply_timeout_s
        self.features: list[str] = []
        self.tiff_folders: Counter[str] = Counter()
        self.live_active = False
        self.sent_bytes = 0
        self._t0 = time.monotonic()
        # Volume ACKs that arrived while a control request was waiting.
        self._stash: list[dict[str, Any]] = []

    def close(self) -> None:
        self.link.close()

    # ---- transport helpers --------------------------------------------------

    def _note(self, reply: dict[str, Any]) -> None:
        self.features = reply.get("features", self.features)
        if "live_active" in reply:
            self.live_active = bool(reply["live_active"])

    def _request(
        self,
        msg_type: bytes,
        header: dict[str, Any],
        expect: str,
        payload: bytes | None = None,
    ) -> dict[str, Any]:
        """Send one control request; wait for its reply (our echo token)."""
        token = uuid.uuid4().hex
        header = {**header, "client": self.client_id, "echo": token}
        deadline = time.monotonic() + self.reply_timeout_s
        self.link.send(bfill.pack(msg_type, self.client_id, header, payload))
        while time.monotonic() < deadline:
            reply = self.link.recv_any(500)
            if reply is None:
                continue
            self._note(reply)
            if reply.get("echo") == token:
                return reply
            if reply.get("kind") == "volume":
                self._stash.append(reply)
        raise BackfillError(
            f"no {expect} reply from Argus within {self.reply_timeout_s:.0f} s "
            "(is the SSH tunnel up, and opym-receive-backfill running?)"
        )

    def ping(self) -> bool:
        self._request(bfill.BF_INVENTORY, {"leaf": "/", "stores": []}, "inventory")
        return self.live_active

    def paused(self) -> str | None:
        if self.pause_file is not None and Path(self.pause_file).exists():
            return f"pause file {self.pause_file} exists"
        if self.live_active:
            return "a live acquisition is streaming to Argus"
        return None

    def wait_if_paused(self) -> None:
        why = self.paused()
        while why:
            logger.info("paused: %s", why)
            time.sleep(PAUSE_POLL_S)
            try:
                self.ping()
            except BackfillError:
                pass
            why = self.paused()

    def _throttle(self, nbytes: int) -> None:
        self.sent_bytes += nbytes
        if self.max_mbps:
            due = self.sent_bytes / (self.max_mbps * 1e6)
            ahead = due - (time.monotonic() - self._t0)
            if ahead > 0:
                time.sleep(ahead)

    # ---- mapping ------------------------------------------------------------

    def remote_leaf(self, local_leaf: Path) -> str:
        for local_root, remote_root in self.root_map:
            try:
                rel = Path(local_leaf).relative_to(local_root)
            except ValueError:
                continue
            return "/".join([remote_root, *rel.parts]) if rel.parts else remote_root
        raise BackfillError(f"{local_leaf} is under no --map root")

    # ---- one leaf -----------------------------------------------------------

    def sync_leaf(self, leaf: LocalLeaf, dry_run: bool = False) -> dict[str, Any]:
        report: dict[str, Any] = {"leaf": str(leaf.path), "stores": {}}
        remote = self.remote_leaf(leaf.path)
        report["remote"] = remote
        ready = []
        for s in leaf.stores:
            age = time.time() - s.newest_mtime()
            if age < self.settle_s:
                report["stores"][s.name] = {"result": "settling", "idle_s": int(age)}
            elif not s.written_t:
                report["stores"][s.name] = {"result": "empty"}
            else:
                ready.append(s)
        if not ready:
            return report
        inv = self._request(
            bfill.BF_INVENTORY,
            {"leaf": remote, "stores": [s.spec() for s in ready]},
            "inventory",
        )
        if not inv.get("ok"):
            raise BackfillError(f"inventory refused for {remote}: {inv.get('error')}")
        self._probe_elsewhere(remote, ready, inv["stores"])
        for s in ready:
            info = inv["stores"].get(s.name, {})
            state = info.get("state")
            if state == bfill.MARKED:
                report["stores"][s.name] = {"result": "already complete"}
            elif state in (bfill.BUSY, bfill.CONFLICT):
                report["stores"][s.name] = {
                    "result": state,
                    "reason": info.get("reason"),
                }
            elif info.get("duplicate_of"):
                # The same data is already on Argus in another folder (e.g. it
                # came by Globus): never a second copy.
                dup_have = info.get("duplicate_have_t", [])
                whole = set(s.written_t) <= set(dup_have)
                report["stores"][s.name] = {
                    "result": "already on Argus" if whole else "partial copy elsewhere",
                    "at": info["duplicate_of"],
                    "has": len(dup_have),
                    "of": len(s.written_t),
                }
            elif set(s.written_t) <= set(info.get("have_t", [])):
                # Every timepoint is there: the live stream brought it in
                # whole (its stores carry no backfill marker). Nothing to send,
                # and its stream-tagged metadata would only be refused.
                report["stores"][s.name] = {"result": "already on Argus"}
            elif dry_run:
                have = set(info.get("have_t", []))
                missing = [t for t in s.written_t if t not in have]
                report["stores"][s.name] = {
                    "result": "would send",
                    "missing_timepoints": len(missing),
                    "of": len(s.written_t),
                }
                if info.get("stream_copy"):
                    report["stores"][s.name]["completes_stream_copy"] = True
            else:
                have = set(info.get("have_t", []))
                try:
                    report["stores"][s.name] = self._sync_store(remote, s, have)
                except BackfillError as exc:
                    # One refused store must not hide what the others did.
                    logger.error("%s: %s", s.name, exc)
                    report["stores"][s.name] = {"result": "refused", "reason": str(exc)}
        return report

    def _probe_elsewhere(
        self, remote: str, stores: list[LocalStore], infos: dict[str, Any]
    ) -> None:
        """Tell the same data elsewhere on Argus from a run that shares the name.

        For each store Argus reports under another folder, send one volume's
        digest; Argus compares it with its copy. Updates ``infos`` in place.
        """
        probes = []
        for s in stores:
            cands = infos.get(s.name, {}).get("elsewhere") or []
            t = next(
                (
                    t
                    for t in s.written_t
                    if any(t in c.get("have_t", []) for c in cands)
                ),
                None,
            )
            if t is None:
                continue
            arr = s.array()
            vol = np.asarray(arr[t] if arr.ndim >= 4 else arr[...])
            probe = {"t": t, "sha256": bfill.volume_digest(vol)}
            probes.append({**s.spec(), "probe": probe})
        if not probes:
            return
        header = {"leaf": remote, "stores": probes}
        inv = self._request(bfill.BF_INVENTORY, header, "inventory")
        if inv.get("ok"):
            for name, info in inv["stores"].items():
                if info.get("duplicate_of"):
                    infos[name] = info

    def _sync_store(self, remote: str, s: LocalStore, have: set[int]) -> dict[str, Any]:
        t_start = time.monotonic()
        before = self.sent_bytes
        self._send_meta(remote, s)
        arr = s.array()
        digests: dict[int, str] = {}
        todo = [t for t in s.written_t if t not in have]
        self._pump(self._slab_stream(remote, s, arr, todo, digests))
        for t in s.written_t:  # volumes Argus already held: hash locally
            if t not in digests:
                vol = np.asarray(arr[t] if arr.ndim >= 4 else arr[...])
                digests[t] = bfill.volume_digest(vol)
        zattrs = s.path / ".zattrs"
        root_zattrs = zattrs.read_bytes() if zattrs.exists() else None
        while True:
            self.wait_if_paused()
            commit = {
                "leaf": remote,
                "store": s.name,
                "digests": {str(t): d for t, d in digests.items()},
                "declared_t": s.shape[0] if len(s.shape) >= 4 else 1,
                "root_zattrs": root_zattrs,
                "source": f"{socket.gethostname()}:{s.path}",
            }
            reply = self._request(bfill.BF_COMMIT, commit, "commit")
            if reply.get("hold"):
                self.live_active = True
                continue
            break
        if not reply.get("ok"):
            raise BackfillError(
                f"{s.name}: commit failed: {reply.get('error')} "
                f"{reply.get('mismatch', '')}"
            )
        sent = self.sent_bytes - before
        secs = max(time.monotonic() - t_start, 1e-6)
        return {
            "result": "complete",
            "timepoints": len(s.written_t),
            "sent_timepoints": len(todo),
            "sent_gb": round(sent / 1e9, 3),
            "mb_per_s": round(sent / secs / 1e6, 1),
            "verified": reply.get("verified"),
        }

    def _send_meta(self, remote: str, s: LocalStore) -> None:
        files = [
            {"rel": rel, "data": (s.path / rel).read_bytes()}
            for rel in bfill.meta_files(s.path, s.main_rel)
        ]
        header = {"leaf": remote, "store": s.name, "files": files}
        reply = self._request(bfill.BF_META, header, "meta")
        if not reply.get("ok"):
            raise BackfillError(f"{s.name}: metadata refused: {reply.get('error')}")

    # ---- volumes ------------------------------------------------------------

    def _slab_runs(self, s: LocalStore, t: int, nz: int) -> list[tuple[int, int]]:
        present = s.planes_present(t, nz) if len(s.shape) >= 4 else list(range(nz))
        plane_bytes = int(np.prod(s.shape[-2:])) * np.dtype(s.dtype).itemsize
        per_slab = max(1, self.slab_bytes // max(plane_bytes, 1))
        runs: list[tuple[int, int]] = []
        start: int | None = None
        prev = 0
        for z in present:
            if start is None:
                start = prev = z
            elif z == prev + 1 and (z - start) < per_slab:
                prev = z
            else:
                runs.append((start, prev - start + 1))
                start = prev = z
        if start is not None:
            runs.append((start, prev - start + 1))
        return runs

    def _slab_stream(
        self,
        remote: str,
        s: LocalStore,
        arr: zarr.Array,
        todo: list[int],
        digests: dict[int, str],
    ) -> Iterator[tuple[tuple[int, int], dict[str, Any], bytes]]:
        for t in todo:
            vol = np.asarray(arr[t] if arr.ndim >= 4 else arr[...])
            digests[t] = bfill.volume_digest(vol)
            for z0, nz in self._slab_runs(s, t, vol.shape[0]):
                slab = vol[z0 : z0 + nz]
                header: dict[str, Any] = {
                    "leaf": remote,
                    "store": s.name,
                    "t": t,
                    "z0": z0,
                    "nz": nz,
                    "shape_yx": list(vol.shape[1:]),
                    "sha256": bfill.volume_digest(slab),
                }
                if self.compress and "blosc" in self.features:
                    from numcodecs import (
                        blosc,  # pyright: ignore[reportAttributeAccessIssue]
                    )

                    payload = blosc.compress(
                        np.ascontiguousarray(slab), b"lz4", 5, blosc.BITSHUFFLE
                    )
                    header["codec"] = "blosc"
                else:
                    payload = np.ascontiguousarray(slab).tobytes()
                yield (t, z0), header, payload

    def _pump(
        self, stream: Iterator[tuple[tuple[int, int], dict[str, Any], bytes]]
    ) -> None:
        """Send slabs, at most ``window`` unacknowledged, resending failures."""
        inflight: _Inflight = {}
        exhausted = False
        while True:
            while not exhausted and len(inflight) < self.window and not self.paused():
                try:
                    key, header, payload = next(stream)
                except StopIteration:
                    exhausted = True
                    break
                self._send_slab(key, header, payload, inflight, attempts=0)
            if exhausted and not inflight:
                return
            if not inflight:  # paused with nothing outstanding
                self.wait_if_paused()
                continue
            reply = self._stash.pop(0) if self._stash else self.link.recv_any(500)
            now = time.monotonic()
            if reply is not None:
                self._note(reply)
                if reply.get("kind") == "volume":
                    key = (int(reply.get("t", -1)), int(reply.get("z0", -1)))
                    entry = inflight.get(key)
                    if entry is None:
                        continue
                    header, payload, _sent, attempts = entry
                    if reply.get("ok"):
                        del inflight[key]
                    elif reply.get("hold"):
                        del inflight[key]
                        self.live_active = True
                        self.wait_if_paused()
                        self._send_slab(key, header, payload, inflight, attempts)
                    elif reply.get("retry") and attempts < MAX_RETRIES:
                        del inflight[key]
                        self._send_slab(key, header, payload, inflight, attempts + 1)
                    else:
                        raise BackfillError(
                            f"volume t={key[0]} z0={key[1]} refused: "
                            f"{reply.get('error')}"
                        )
            for key, (header, payload, sent, attempts) in list(inflight.items()):
                if now - sent > self.reply_timeout_s:
                    if attempts >= MAX_RETRIES:
                        raise BackfillError(
                            f"volume t={key[0]} z0={key[1]} got no reply "
                            "(is the tunnel up?)"
                        )
                    del inflight[key]
                    self._send_slab(key, header, payload, inflight, attempts + 1)

    def _send_slab(
        self,
        key: tuple[int, int],
        header: dict[str, Any],
        payload: bytes,
        inflight: _Inflight,
        attempts: int,
    ) -> None:
        header = {**header, "client": self.client_id}
        self.link.send(bfill.pack(bfill.BF_VOLUME, self.client_id, header, payload))
        inflight[key] = (header, payload, time.monotonic(), attempts)
        self._throttle(len(payload))

    # ---- everything ---------------------------------------------------------

    def run_once(self, dry_run: bool = False) -> list[dict[str, Any]]:
        self.ping()
        reports = []
        leaves: list[LocalLeaf] = []
        self.tiff_folders = Counter()
        for local_root, _ in self.root_map:
            leaves += scan_local_root(local_root, other=self.tiff_folders)
        leaves.sort(
            key=lambda lf: max(s.newest_mtime() for s in lf.stores),
            reverse=self.order == "newest",
        )
        for leaf in leaves:
            self.wait_if_paused()
            try:
                rep = self.sync_leaf(leaf, dry_run=dry_run)
            except BackfillError as exc:
                logger.error("%s: %s", leaf.path, exc)
                rep = {"leaf": str(leaf.path), "error": str(exc)}
            logger.info("%s", json.dumps(rep))
            reports.append(rep)
        return reports


def summarize(
    reports: list[dict[str, Any]],
    dry_run: bool,
    tiff_folders: Counter[str] | None = None,
) -> str:
    """What happened, for people: one line per store that needs attention."""
    counts: Counter[str] = Counter()
    lines = []
    for rep in reports:
        if "error" in rep:
            counts["error"] += 1
            lines.append(f"  ERROR    {rep['leaf']}: {rep['error']}")
            continue
        for name, st in rep["stores"].items():
            result = st["result"]
            counts[result] += 1
            if result == "would send":
                what = f"{st['missing_timepoints']} of {st['of']} timepoints"
                if st.get("completes_stream_copy"):
                    what += ", completing the live stream's partial copy"
                lines.append(f"  SEND     {name}: {what} -> {rep['remote']}")
            elif result == "already on Argus" and st.get("at"):
                lines.append(f"  SKIP     {name}: already on Argus at {st['at']}")
            elif result == "partial copy elsewhere":
                lines.append(
                    f"  CHECK    {name}: Argus has {st['has']} of {st['of']} "
                    f"timepoints at {st['at']}; not sent, so there is no second copy"
                )
            elif result == "complete":
                what = f"{st['sent_timepoints']} timepoints, {st['sent_gb']} GB"
                lines.append(f"  SENT     {name}: {what}, verified")
            elif result in (bfill.CONFLICT, bfill.BUSY, "refused"):
                reason = str(st.get("reason"))
                if "exists and differs" in reason:
                    # Argus never overwrites. A receiver from before 2026-10-24
                    # refuses even the live stream's own partial copy, which a
                    # current one completes in place.
                    reason += (
                        " -- Argus holds a different copy of this store and never "
                        "overwrites; if it is the live stream's partial copy, the "
                        "backfill receiver on Argus needs updating"
                    )
                lines.append(f"  {result.upper():8} {name}: {reason}")
    total = (
        ", ".join(f"{n} {k}" for k, n in sorted(counts.items())) or "no stores found"
    )
    if tiff_folders:
        n = sum(tiff_folders.values())
        lines.append(
            f"  Not handled: {n} TIFF file(s) in {len(tiff_folders)} folder(s) "
            "(this tool sends OME-Zarr only; send those by Globus), e.g. "
            f"{next(iter(sorted(tiff_folders)))}"
        )
    head = "Dry run -- nothing was sent." if dry_run else "Done."
    return "\n".join([head, f"Stores: {total}.", *lines])


def _parse_map(text: str) -> tuple[Path, str]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("--map expects LOCAL=REMOTE")
    local, remote = text.split("=", 1)
    return Path(local), remote


def _ssh_host_from_settings() -> str:
    try:
        from pymmcore_gui._settings import Settings

        return Settings.instance().argus_stream.ssh_host
    except Exception:  # settings unreadable: the stream's default alias
        return "Argus"


def _wait_for_port(port: int, timeout_s: float) -> bool:
    """Whether the tunnel's local port accepts a connection within the timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    p = argparse.ArgumentParser(
        prog="opym-backfill-send",
        description=(__doc__ or "").split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--map",
        dest="maps",
        action="append",
        type=_parse_map,
        required=True,
        help="LOCAL=REMOTE root mapping (repeatable); the path below LOCAL is "
        "kept below REMOTE",
    )
    p.add_argument(
        "--connect",
        action="append",
        default=None,
        help="receiver endpoint(s); skips opening a tunnel",
    )
    p.add_argument(
        "--ssh-host", default=None, help="ssh Host alias (default: the stream's)"
    )
    p.add_argument("--local-port", type=int, default=DEFAULT_LOCAL_PORT)
    p.add_argument("--links", type=int, default=2, help="parallel SSH tunnels")
    p.add_argument(
        "--settle",
        type=float,
        default=900.0,
        help="skip stores written to within this many seconds",
    )
    p.add_argument("--window", type=int, default=8, help="slabs in flight")
    p.add_argument(
        "--max-mbps", type=float, default=None, help="cap the send rate (MB/s)"
    )
    p.add_argument(
        "--pause-file", type=Path, default=None, help="send nothing while it exists"
    )
    p.add_argument("--order", choices=("newest", "oldest"), default="newest")
    p.add_argument("--dry-run", action="store_true", help="report what would be sent")
    p.add_argument("--watch", type=float, default=None, metavar="SECONDS")
    a = p.parse_args(argv)

    tunnel = None
    connect = a.connect
    if not connect:
        from ._tunnel import ArgusTunnelManager

        host = a.ssh_host or _ssh_host_from_settings()
        tunnel = ArgusTunnelManager(host, a.local_port, REMOTE_PORT, links=a.links)
        tunnel.start()
        connect = tunnel.endpoints
        logger.info(
            "SSH tunnel to %s: local %s -> Argus:%d", host, connect, REMOTE_PORT
        )
        if not _wait_for_port(a.local_port, TUNNEL_UP_TIMEOUT_S):
            tunnel.stop()
            print(
                f"Could not open an SSH tunnel to {host!r} within "
                f"{TUNNEL_UP_TIMEOUT_S:.0f} s (see the ssh lines above). Does "
                f"'ssh {host}' work from this PC?"
            )
            return 2
    client = BackfillClient(
        connect,
        a.maps,
        settle_s=a.settle,
        window=a.window,
        max_mbps=a.max_mbps,
        pause_file=a.pause_file,
        order=a.order,
    )
    try:
        while True:
            try:
                reports = client.run_once(dry_run=a.dry_run)
                print(summarize(reports, a.dry_run, client.tiff_folders))
            except BackfillError as exc:
                logger.error("%s", exc)
                print(f"Could not reach Argus's backfill receiver: {exc}")
                if a.watch is None:
                    return 1
            if a.watch is None:
                return 0
            time.sleep(a.watch)
    finally:
        client.close()
        if tunnel is not None:
            tunnel.stop()


if __name__ == "__main__":
    raise SystemExit(main())
