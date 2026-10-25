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
import math
import os
import re
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
from . import _bfill_run as bfill_run

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

logger = logging.getLogger(__name__)

REMOTE_PORT = 5556
# Local end of this tool's own tunnel: clear of the live stream's 5555+.
DEFAULT_LOCAL_PORT = 5566
REPLY_TIMEOUT_S = 180.0
PAUSE_POLL_S = 10.0
MAX_RETRIES = 3
TUNNEL_UP_TIMEOUT_S = 30.0
SCAN_PROGRESS_S = 5.0
# Commit waits for Argus to read the whole store back; allow it this rate at worst.
COMMIT_READBACK_MBPS = 50.0
# `<prefix>_<Channel>_<wavelength>[nm]`, as opym's discovery groups a run's stores.
_CHANNEL_SUFFIX = re.compile(
    r"^(?P<prefix>.+)_(?P<channel>[A-Za-z]+)_(?P<wavelength>\d+(?:nm)?)$"
)
MAX_RENAME = 9


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
# Never walked: the Windows recycle bin (deleted runs must not be sent) and
# other system folders at a drive's root.
_SKIP_DIRS = {"$recycle.bin", "system volume information", "recycler", "found.000"}


def scan_local_root(
    root: Path,
    max_depth: int = 6,
    other: Counter[str] | None = None,
    progress: Callable[[Path, int], None] | None = None,
) -> list[LocalLeaf]:
    """Every directory under ``root`` that directly holds ``*.ome.zarr`` stores.

    TIFF files met on the way are counted in ``other`` (by folder): this tool
    sends OME-Zarr only, so they would otherwise be left behind unsaid.
    """
    leaves: list[LocalLeaf] = []

    def walk(d: Path, depth: int) -> None:
        if progress is not None:
            progress(d, sum(len(lf.stores) for lf in leaves))
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
                hidden = (
                    e.name.startswith((".", "$"))
                    or e.name.endswith(".ome.zarr")
                    or e.name.lower() in _SKIP_DIRS
                )
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
        runs_dir: Path | None = None,
    ) -> None:
        self.runs_dir = runs_dir
        self.runlog: bfill_run.RunLog | None = None
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
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        """Send one control request; wait for its reply (our echo token)."""
        token = uuid.uuid4().hex
        header = {**header, "client": self.client_id, "echo": token}
        if self.runlog is not None:
            header["run_id"] = self.runlog.run_id
        timeout_s = max(timeout_s or 0.0, self.reply_timeout_s)
        deadline = time.monotonic() + timeout_s
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
            f"no {expect} reply from Argus within {timeout_s:.0f} s "
            "(is the SSH tunnel up, and opym-receive-backfill running?)"
        )

    def _send_run(self, header: dict[str, Any]) -> None:
        """Mirror the run record to Argus.

        Its start and end wait for the reply (the end is the last thing sent
        before the sockets close, which drops anything unsent); heartbeats and
        per-store results don't.
        """
        if "run_log" not in self.features:
            return
        if header.get("op") in ("start", "end"):
            try:
                self._request(bfill.BF_RUN, header, "run")
            except BackfillError as exc:
                logger.warning("the run record did not reach Argus: %s", exc)
        else:
            msg = {**header, "client": self.client_id}
            self.link.send(bfill.pack(bfill.BF_RUN, self.client_id, msg))

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
        """Plan and carry out one folder (what ``run_once`` does for each)."""
        remote, decisions = self.plan_leaf(leaf)
        report: dict[str, Any] = {"leaf": str(leaf.path), "remote": remote}
        report["stores"] = {
            d["store"].name: self._carry_out(d, dry_run) for d in decisions
        }
        return report

    # ---- planning -----------------------------------------------------------

    def plan_leaf(
        self, leaf: LocalLeaf, skip: frozenset[str] | set[str] = frozenset()
    ) -> tuple[str, list[dict[str, Any]]]:
        """Decide what to do with each store of ``leaf`` from Argus's inventory.

        Sends nothing but questions.
        """
        remote = self.remote_leaf(leaf.path)
        decisions: list[dict[str, Any]] = []
        ready = []
        for s in leaf.stores:
            d: dict[str, Any] = {"store": s, "remote": remote, "name": s.name}
            age = time.time() - s.newest_mtime()
            if str(s.path) in skip:
                d.update(
                    action="skip", result="already complete", reason="previous run"
                )
            elif age < self.settle_s:
                d.update(action="wait", result="settling", idle_s=int(age))
            elif not s.written_t:
                d.update(action="wait", result="empty")
            elif len(s.shape) < 3:
                d.update(
                    action="report",
                    result="no pixel data",
                    reason=f"no image array on this PC (largest: {s.shape} {s.dtype})",
                )
            else:
                ready.append(s)
                continue
            decisions.append(d)
        if not ready:
            return remote, decisions
        inv = self._request(
            bfill.BF_INVENTORY,
            {"leaf": remote, "stores": [s.spec() for s in ready]},
            "inventory",
        )
        if not inv.get("ok"):
            raise BackfillError(f"inventory refused for {remote}: {inv.get('error')}")
        infos = inv["stores"]
        self._probe_elsewhere(remote, ready, infos)
        for s in ready:
            decisions.append(self._decide(remote, s, infos.get(s.name, {})))
        self._resolve_renames(remote, decisions)
        return remote, decisions

    def _decide(
        self, remote: str, s: LocalStore, info: dict[str, Any]
    ) -> dict[str, Any]:
        d: dict[str, Any] = {"store": s, "remote": remote, "name": s.name}
        state = info.get("state")
        have = set(info.get("have_t", []))
        if state == bfill.MARKED:
            return {**d, "action": "skip", "result": "already complete"}
        if state == bfill.BUSY:
            return {
                **d,
                "action": "report",
                "result": state,
                "reason": info.get("reason"),
            }
        if state == bfill.CONFLICT:
            return {
                **d,
                "action": "report",
                "result": state,
                "reason": str(info.get("reason") or ""),
                "different_run": _different_run(s, info),
            }
        if info.get("duplicate_of"):
            # The same data is already on Argus in another folder (e.g. it
            # came by Globus): never a second copy. A partial copy there is
            # completed where it is.
            at = info["duplicate_of"]
            dup_have = set(info.get("duplicate_have_t", []))
            if set(s.written_t) <= dup_have:
                return {**d, "action": "skip", "result": "already on Argus", "at": at}
            return {
                **d,
                "action": "complete elsewhere",
                "remote": str(Path(at).parent),
                "have": dup_have,
                "at": at,
            }
        if set(s.written_t) <= have and not info.get("backfill_started"):
            # Every timepoint is there: the live stream (or Globus) brought it
            # in whole. Nothing to send, and its stream-tagged metadata would
            # only be refused.
            return {**d, "action": "skip", "result": "already on Argus"}
        action = "complete stream copy" if info.get("stream_copy") else "send"
        return {**d, "action": action, "have": have}

    def _resolve_renames(self, remote: str, decisions: list[dict[str, Any]]) -> None:
        """Send a different run under a name Argus holds as ``<run>_pc<N>_...``.

        The acquisition software has reused run names. Every channel of the run
        gets the same N, so they stay one dataset; N is the first that is free,
        or that already holds this very store from an earlier run.
        """
        groups: dict[str, list[dict[str, Any]]] = {}
        for d in decisions:
            m = _CHANNEL_SUFFIX.match(d["name"].removesuffix(".ome.zarr"))
            groups.setdefault(m.group("prefix") if m else d["name"], []).append(d)
        source = f"{socket.gethostname()}:"
        for prefix, members in groups.items():
            differ = [d for d in members if d.get("different_run")]
            if not differ:
                continue
            if len(differ) != len(members):
                for d in differ:
                    d["reason"] += (
                        "; not renamed: other channels of this run are not "
                        "conflicts, so a person has to decide"
                    )
                continue
            if "marker_from" not in self.features:
                continue  # an older receiver can't tell a renamed copy is this run
            for n in range(2, MAX_RENAME + 1):
                names = {d["name"]: _renamed(prefix, d["name"], n) for d in differ}
                specs = [
                    {**d["store"].spec(), "name": names[d["name"]]} for d in differ
                ]
                inv = self._request(
                    bfill.BF_INVENTORY, {"leaf": remote, "stores": specs}, "inventory"
                )
                if not inv.get("ok"):
                    break
                infos = {
                    d["name"]: inv["stores"].get(names[d["name"]], {}) for d in differ
                }

                def usable(d: dict[str, Any], i: dict[str, Any]) -> bool:
                    state = i.get("state")
                    if state == bfill.ABSENT:
                        return True
                    if state == bfill.PARTIAL:
                        return not i.get("stream_copy") and not i.get("elsewhere")
                    mine = f"{source}{d['store'].path}"
                    return bool(state == bfill.MARKED and i.get("from") == mine)

                if not all(usable(d, infos[d["name"]]) for d in differ):
                    continue
                for d in differ:
                    i, new = infos[d["name"]], names[d["name"]]
                    d.update(
                        name=new,
                        renamed_from=d["name"],
                        reason=None,
                        different_run=False,
                    )
                    if i.get("state") == bfill.MARKED:
                        d.update(
                            action="skip",
                            result="already on Argus",
                            at=f"{remote}/{new}",
                        )
                    else:
                        d.update(
                            action="send renamed",
                            result=None,
                            have=set(i.get("have_t", [])),
                        )
                break

    def plan_entry(self, d: dict[str, Any]) -> dict[str, Any]:
        """The decision as the run record lists it."""
        s: LocalStore = d["store"]
        per_t = 0
        if len(s.shape) >= 3:
            per_t = math.prod(s.shape[-3:]) * np.dtype(s.dtype).itemsize
        have = d.get("have", set(s.written_t))
        send_t = len([t for t in s.written_t if t not in have])
        entry: dict[str, Any] = {
            "key": f"{d['remote']}/{d['name']}",
            "leaf": d["remote"],
            "name": d["name"],
            "local": str(s.path),
            "action": d["action"],
            "timepoints": len(s.written_t),
            "send_t": send_t,
            "bytes": send_t * per_t,
        }
        for k in ("result", "reason", "renamed_from", "at"):
            if d.get(k):
                entry[k] = d[k]
        return entry

    # ---- carrying it out ----------------------------------------------------

    def _carry_out(self, d: dict[str, Any], dry_run: bool) -> dict[str, Any]:
        """Return the store's report entry, sending it unless ``dry_run``."""
        s: LocalStore = d["store"]
        action = d["action"]
        out: dict[str, Any]
        if action in ("skip", "wait", "report"):
            keys = ("result", "reason", "at", "idle_s")
            out = {k: d[k] for k in keys if d.get(k) is not None}
        elif dry_run:
            have = d.get("have", set())
            results = {
                "send": "would send",
                "complete stream copy": "would send",
                "complete elsewhere": "would complete elsewhere",
                "send renamed": "would send renamed",
            }
            out = {
                "result": results[action],
                "missing_timepoints": len([t for t in s.written_t if t not in have]),
                "of": len(s.written_t),
            }
            if action == "complete stream copy":
                out["completes_stream_copy"] = True
        else:
            try:
                out = self._sync_store(
                    d["remote"],
                    s,
                    d.get("have", set()),
                    name=d["name"],
                    renamed_from=d.get("renamed_from"),
                )
            except BackfillError as exc:
                # One refused store must not hide what the others did.
                logger.error("%s: %s", s.name, exc)
                out = {"result": "refused", "reason": str(exc)}
        if d.get("at") and action != "skip":
            out["at"] = d["at"]
        if d.get("renamed_from"):
            out["as"] = d["name"]
        if self.runlog is not None and not dry_run:
            self.runlog.store(f"{d['remote']}/{d['name']}", out)
        return out

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
            if infos.get(s.name, {}).get("needs_probe") and s.written_t:
                # e.g. the live stream's 4-D copy of a single 3-D volume
                t = s.written_t[0]
            if t is None:
                continue
            probes.append({**s.spec(), "probe": self._probe_for(s, t)})
        if not probes:
            return
        header = {"leaf": remote, "stores": probes}
        inv = self._request(bfill.BF_INVENTORY, header, "inventory")
        if inv.get("ok"):
            infos.update(inv["stores"])

    def _probe_for(self, s: LocalStore, t: int) -> dict[str, Any]:
        """Return the SHA-256 Argus compares with its copy.

        Of one plane that is on disk here (the middle one) when Argus takes
        plane probes, else of the whole volume t. Whole-volume probes read
        ~190 MB per store here and again on Argus (2026-10-24).
        """
        arr = s.array()
        if "plane_probe" in self.features:
            nz = s.shape[1] if len(s.shape) >= 4 else s.shape[0]
            present = s.planes_present(t, nz) if len(s.shape) >= 4 else list(range(nz))
            if present:
                z = present[len(present) // 2]
                plane = np.asarray(arr[t, z] if arr.ndim >= 4 else arr[z])
                return {"t": t, "z": z, "sha256": bfill.volume_digest(plane)}
        vol = np.asarray(arr[t] if arr.ndim >= 4 else arr[...])
        return {"t": t, "sha256": bfill.volume_digest(vol)}

    def _sync_store(
        self,
        remote: str,
        s: LocalStore,
        have: set[int],
        name: str | None = None,
        renamed_from: str | None = None,
    ) -> dict[str, Any]:
        name = name or s.name
        t_start = time.monotonic()
        before = self.sent_bytes
        self._send_meta(remote, s, name)
        arr = s.array()
        digests: dict[int, str] = {}
        todo = [t for t in s.written_t if t not in have]
        self._pump(self._slab_stream(remote, s, arr, todo, digests, name))
        for t in s.written_t:  # volumes Argus already held: hash locally
            if t not in digests:
                vol = np.asarray(arr[t] if arr.ndim >= 4 else arr[...])
                digests[t] = bfill.volume_digest(vol)
        zattrs = s.path / ".zattrs"
        root_zattrs = zattrs.read_bytes() if zattrs.exists() else None
        # Argus reads every volume back before it answers (~19 GB for a
        # 100-timepoint run), so the wait scales with the store.
        nbytes = len(digests) * math.prod(s.shape[-3:]) * np.dtype(s.dtype).itemsize
        readback_s = nbytes / (COMMIT_READBACK_MBPS * 1e6)
        commit: dict[str, Any] = {
            "leaf": remote,
            "store": name,
            "digests": {str(t): d for t, d in digests.items()},
            "declared_t": s.shape[0] if len(s.shape) >= 4 else 1,
            "root_zattrs": root_zattrs,
            "source": f"{socket.gethostname()}:{s.path}",
        }
        if renamed_from:
            commit["renamed_from"] = renamed_from
        while True:
            self.wait_if_paused()
            if self.runlog is not None:
                self.runlog.heartbeat(f"{remote}/{name}", None, force=True)
            reply = self._request(
                bfill.BF_COMMIT, commit, "commit", timeout_s=readback_s
            )
            if reply.get("hold"):
                self.live_active = True
                continue
            break
        if not reply.get("ok"):
            raise BackfillError(
                f"{name}: commit failed: {reply.get('error')} "
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

    def _send_meta(self, remote: str, s: LocalStore, name: str) -> None:
        files = [
            {"rel": rel, "data": (s.path / rel).read_bytes()}
            for rel in bfill.meta_files(s.path, s.main_rel)
        ]
        header = {"leaf": remote, "store": name, "files": files}
        reply = self._request(bfill.BF_META, header, "meta")
        if not reply.get("ok"):
            raise BackfillError(f"{name}: metadata refused: {reply.get('error')}")

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
        name: str,
    ) -> Iterator[tuple[tuple[int, int], dict[str, Any], bytes]]:
        for t in todo:
            if self.runlog is not None:
                self.runlog.heartbeat(f"{remote}/{name}", t)
            vol = np.asarray(arr[t] if arr.ndim >= 4 else arr[...])
            digests[t] = bfill.volume_digest(vol)
            for z0, nz in self._slab_runs(s, t, vol.shape[0]):
                slab = vol[z0 : z0 + nz]
                header: dict[str, Any] = {
                    "leaf": remote,
                    "store": name,
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

    def run_once(
        self, dry_run: bool = False, resume: dict[str, Any] | None = None
    ) -> list[dict[str, Any]]:
        """Scan, plan every folder, record the plan, then carry it out.

        ``resume`` is a previous run's record (``_bfill_run.load_run``); the
        stores it finished are not looked at again.
        """
        self.ping()
        leaves: list[LocalLeaf] = []
        self.tiff_folders = Counter()
        folders = 0
        last = time.monotonic()

        def progress(d: Path, stores: int) -> None:
            nonlocal folders, last
            folders += 1
            if time.monotonic() - last >= SCAN_PROGRESS_S:
                last = time.monotonic()
                print(
                    f"Scanning: {folders} folders, {stores} stores so far ({d})",
                    flush=True,
                )

        for local_root, _ in self.root_map:
            print(f"Scanning {local_root} for OME-Zarr stores...", flush=True)
            leaves += scan_local_root(
                local_root, other=self.tiff_folders, progress=progress
            )
        n_stores = sum(len(lf.stores) for lf in leaves)
        print(
            f"Found {n_stores} stores in {len(leaves)} folders; checking each "
            "with Argus.",
            flush=True,
        )
        leaves.sort(
            key=lambda lf: max(s.newest_mtime() for s in lf.stores),
            reverse=self.order == "newest",
        )
        skip = bfill_run.done_locals(resume)
        runlog = bfill_run.RunLog(
            self.runs_dir,
            client=self.client_id,
            dry_run=dry_run,
            maps=[(str(a), b) for a, b in self.root_map],
            send=self._send_run,
        )
        self.runlog = runlog
        try:
            plans = self._plan_all(leaves, skip)
            entries = [self.plan_entry(d) for _lf, _r, ds, _e in plans for d in ds]
            runlog.plan(entries)
            n_send = sum(
                1
                for e in entries
                if e["send_t"] and e["action"] not in ("skip", "wait", "report")
            )
            gb = (
                sum(
                    e["bytes"]
                    for e in entries
                    if e["action"] not in ("skip", "wait", "report")
                )
                / 1e9
            )
            where = f" -- recorded in {runlog.path}" if runlog.path else ""
            print(
                f"Plan: {n_send} store(s) to send, {gb:.1f} GB{where}"
                + (" (Argus keeps a copy)" if "run_log" in self.features else ""),
                flush=True,
            )
            reports: list[dict[str, Any]] = []
            for i, (leaf, remote, decisions, error) in enumerate(plans, 1):
                if error is not None:
                    rep: dict[str, Any] = {"leaf": str(leaf.path), "error": error}
                else:
                    rep = {"leaf": str(leaf.path), "remote": remote, "stores": {}}
                    for d in decisions:
                        self.wait_if_paused()
                        rep["stores"][d["store"].name] = self._carry_out(d, dry_run)
                logger.debug("%s", json.dumps(rep, default=str))
                results = Counter(st["result"] for st in rep.get("stores", {}).values())
                what = ", ".join(f"{n} {k}" for k, n in sorted(results.items()))
                print(
                    f"[{i}/{len(plans)}] {leaf.path}: {what or rep.get('error', '')}",
                    flush=True,
                )
                reports.append(rep)
            if not dry_run:
                self._retry_refused(plans, reports)
            runlog.end("complete")
            return reports
        except BaseException as exc:
            runlog.end("failed", error=repr(exc))
            raise
        finally:
            self.runlog = None

    def _plan_all(
        self, leaves: list[LocalLeaf], skip: set[str]
    ) -> list[tuple[LocalLeaf, str | None, list[dict[str, Any]], str | None]]:
        plans: list[tuple[LocalLeaf, str | None, list[dict[str, Any]], str | None]] = []
        for i, leaf in enumerate(leaves, 1):
            self.wait_if_paused()
            try:
                remote, decisions = self.plan_leaf(leaf, skip)
                plans.append((leaf, remote, decisions, None))
            except BackfillError as exc:
                logger.error("%s: %s", leaf.path, exc)
                plans.append((leaf, None, [], str(exc)))
            print(f"Planning [{i}/{len(leaves)}] {leaf.path}", flush=True)
        return plans

    def _retry_refused(
        self,
        plans: list[tuple[LocalLeaf, str | None, list[dict[str, Any]], str | None]],
        reports: list[dict[str, Any]],
    ) -> None:
        """Try each refused store once more, re-planned from what Argus holds now."""
        for (leaf, _remote, decisions, error), rep in zip(plans, reports, strict=True):
            if error is not None:
                continue
            failed = {
                d["store"].name
                for d in decisions
                if rep["stores"][d["store"].name].get("result") == "refused"
            }
            if not failed:
                continue
            print(f"Retrying {len(failed)} refused store(s) in {leaf.path}", flush=True)
            retry = LocalLeaf(leaf.path, [s for s in leaf.stores if s.name in failed])
            try:
                _remote, again = self.plan_leaf(retry)
            except BackfillError as exc:
                logger.error("%s: retry: %s", leaf.path, exc)
                continue
            for d in again:
                self.wait_if_paused()
                out = self._carry_out(d, dry_run=False)
                out["retried"] = True
                rep["stores"][d["store"].name] = out


def _different_run(s: LocalStore, info: dict[str, Any]) -> bool:
    """Whether Argus's store of this name is certainly another acquisition.

    Its volumes have another z/y/x size or pixel type, which one run can't
    change. Differing only in the declared number of timepoints may be the same
    run, so that stays a conflict.
    """
    shape, dtype = info.get("argus_shape"), info.get("argus_dtype")
    if not shape or len(shape) < 3 or len(s.shape) < 3:
        return False
    return list(shape[-3:]) != list(s.shape[-3:]) or dtype != s.dtype


def _renamed(prefix: str, name: str, n: int) -> str:
    """``<prefix>_pc<n>_<channel>.ome.zarr`` for a run sent under a new name."""
    m = _CHANNEL_SUFFIX.match(name.removesuffix(".ome.zarr"))
    if m is None:
        return f"{name.removesuffix('.ome.zarr')}_pc{n}.ome.zarr"
    return f"{prefix}_pc{n}_{m.group('channel')}_{m.group('wavelength')}.ome.zarr"


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
            elif result == "would send renamed":
                what = f"{st['missing_timepoints']} of {st['of']} timepoints"
                lines.append(
                    f"  SEND     {name}: {what} as {st['as']} (Argus holds a "
                    f"different run under this name) -> {rep['remote']}"
                )
            elif result == "already on Argus" and st.get("at"):
                lines.append(f"  SKIP     {name}: already on Argus at {st['at']}")
            elif result == "no pixel data":
                lines.append(f"  NO DATA  {name}: {st.get('reason')}")
            elif result == "would complete elsewhere":
                what = f"{st['missing_timepoints']} of {st['of']} timepoints"
                lines.append(
                    f"  SEND     {name}: {what}, completing the copy at {st['at']}"
                )
            elif result == "complete":
                what = f"{st['sent_timepoints']} timepoints, {st['sent_gb']} GB"
                where = f" (completed the copy at {st['at']})" if st.get("at") else ""
                if st.get("as"):
                    where += f" as {st['as']}"
                lines.append(f"  SENT     {name}: {what}, verified{where}")
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


def _links_from_settings() -> int:
    """Return as many tunnels as the live stream uses.

    Each SSH connection is capped (~63 MB/s measured, 2026-09-26), so N of
    them carry about N times that.
    """
    try:
        from pymmcore_gui._settings import Settings

        return max(1, Settings.instance().argus_stream.stream_links)
    except Exception:
        return 4


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
    p.add_argument(
        "--links",
        type=int,
        default=None,
        help="parallel SSH tunnels (default: the stream's stream_links, 4)",
    )
    p.add_argument(
        "--settle",
        type=float,
        default=900.0,
        help="skip stores written to within this many seconds",
    )
    p.add_argument(
        "--window",
        type=int,
        default=None,
        help="slabs in flight (default: 4 per link)",
    )
    p.add_argument(
        "--max-mbps", type=float, default=None, help="cap the send rate (MB/s)"
    )
    p.add_argument(
        "--pause-file", type=Path, default=None, help="send nothing while it exists"
    )
    p.add_argument("--order", choices=("newest", "oldest"), default="newest")
    p.add_argument("--dry-run", action="store_true", help="report what would be sent")
    p.add_argument("--watch", type=float, default=None, metavar="SECONDS")
    p.add_argument(
        "--runs-dir",
        type=Path,
        default=bfill_run.default_runs_dir(),
        help="where this run's record is kept (default %(default)s)",
    )
    p.add_argument(
        "--resume",
        nargs="?",
        const="latest",
        default=None,
        metavar="RUN_ID",
        help="skip what the latest (or the named) run already finished",
    )
    a = p.parse_args(argv)

    tunnel = None
    connect = a.connect
    links = a.links or (len(connect) if connect else _links_from_settings())
    window = a.window or 4 * links
    if not connect:
        from ._tunnel import ArgusTunnelManager

        host = a.ssh_host or _ssh_host_from_settings()
        tunnel = ArgusTunnelManager(host, a.local_port, REMOTE_PORT, links=links)
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
        window=window,
        max_mbps=a.max_mbps,
        pause_file=a.pause_file,
        order=a.order,
        runs_dir=a.runs_dir,
    )
    try:
        while True:
            try:
                resume = None
                if a.resume:
                    resume = bfill_run.load_run(a.runs_dir, a.resume)
                    if resume is None:
                        print(f"No earlier run record in {a.runs_dir}; sending it all.")
                reports = client.run_once(dry_run=a.dry_run, resume=resume)
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
