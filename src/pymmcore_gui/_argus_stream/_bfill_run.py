"""The backfill sender's run record (sidecar): what one run plans to send, then how it went.

On 2026-10-24 a whole-drive run stalled at 22:54 and never finished, and neither end
kept a trace: the sender only printed its summary at the very end, and the receiver
logged nothing per store. A run now writes its plan before the first byte goes out,
records each store's result as it happens, and says how it ended. One JSON document:

    {"format": 1, "run_id": ..., "client": ..., "dry_run": false,
     "state": "planning" | "running" | "complete" | "failed" (| "interrupted", set by
              the receiver when a run stops reporting),
     "started_at", "updated_at", "ended_at": epoch s,
     "plan": [{"key", "leaf", "name", "local", "action", "timepoints", "send_t", "bytes",
               "reason"?, "renamed_from"?}, ...],
     "stores": {key: {"result", "attempts", "finished_at", ...}},
     "current": {"key", "t"} | null,
     "counts": {result: n}}

`key` is `<remote leaf>/<remote store name>`: where the store lands on Argus.

The sender keeps it in a local file (rewritten atomically after every change, so a
crash still leaves it) and, when the receiver advertises the `run_log` feature, mirrors
every change to it (`BF_RUN`); the receiver keeps Argus's copy under
`<raw root>/.opym_backfill/runs/`, where the dashboard reads it.

This file is copied verbatim into pymmcore-gui (`_argus_stream/_bfill_run.py`); keep the
two identical.
"""
# ruff: noqa: D205, E501 (a verbatim copy of opym.stream.bfill_run)

from __future__ import annotations

import json
import os
import socket
import time
import uuid
from collections import Counter
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable

RUN_FORMAT = 1
HEARTBEAT_S = 60.0
# Results after which a store needs nothing more (a resumed run skips them).
DONE_RESULTS = frozenset({"complete", "already complete", "already on Argus"})


def default_runs_dir() -> Path:
    r"""`%LOCALAPPDATA%\\opym-backfill\\runs` on Windows, else under XDG state."""
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("XDG_STATE_HOME")
    if not base:
        base = str(Path.home() / ".local" / "state")
    return Path(base) / "opym-backfill" / "runs"


def new_run_id(client: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in client)
    return f"{time.strftime('%Y%m%dT%H%M%S')}-{safe}-{uuid.uuid4().hex[:6]}"


def _atomic_write_json(path: Path, doc: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.part")
    tmp.write_text(json.dumps(doc, indent=1))
    os.replace(tmp, path)


class RunLog:
    """One run's record. `send(header)` mirrors each change to the receiver
    (fire-and-forget); None keeps it local only.
    """

    def __init__(
        self,
        runs_dir: Path | None,
        *,
        client: str,
        dry_run: bool,
        maps: list[tuple[str, str]],
        send: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.runs_dir = Path(runs_dir) if runs_dir is not None else None
        self.send = send
        now = time.time()
        self.doc: dict[str, Any] = {
            "format": RUN_FORMAT,
            "run_id": new_run_id(client),
            "client": client,
            "host": socket.gethostname(),
            "dry_run": dry_run,
            "maps": [list(m) for m in maps],
            "state": "planning",
            "started_at": now,
            "updated_at": now,
            "ended_at": None,
            "plan": [],
            "stores": {},
            "current": None,
            "counts": {},
        }
        self._last_beat = 0.0

    @property
    def run_id(self) -> str:
        return str(self.doc["run_id"])

    @property
    def path(self) -> Path | None:
        if self.runs_dir is None:
            return None
        return self.runs_dir / f"{self.run_id}.json"

    def _save(self) -> None:
        self.doc["updated_at"] = time.time()
        if self.path is not None:
            try:
                _atomic_write_json(self.path, self.doc)
            except OSError:
                pass  # the record must never stop the transfer

    def _mirror(self, op: str, **fields: Any) -> None:
        if self.send is None:
            return
        try:
            self.send({"op": op, "run_id": self.run_id, **fields})
        except Exception:  # the record must never stop the transfer
            pass

    # ---- the run's phases ---------------------------------------------------

    def plan(self, entries: list[dict[str, Any]]) -> None:
        """What this run will do, before it does any of it."""
        self.doc["plan"] = entries
        self.doc["state"] = "running"
        self._save()
        self._mirror("start", doc=self.doc)

    def store(self, key: str, result: dict[str, Any]) -> None:
        prev = self.doc["stores"].get(key, {})
        entry = {
            **result,
            "attempts": int(prev.get("attempts", 0)) + 1,
            "finished_at": time.time(),
        }
        self.doc["stores"][key] = entry
        self.doc["current"] = None
        self._save()
        self._mirror("store", key=key, entry=entry)

    def heartbeat(
        self, key: str | None, t: int | None = None, force: bool = False
    ) -> None:
        """Rate-limited 'still alive, working on key/t'."""
        now = time.monotonic()
        if not force and now - self._last_beat < HEARTBEAT_S:
            return
        self._last_beat = now
        self.doc["current"] = None if key is None else {"key": key, "t": t}
        self._save()
        self._mirror("heartbeat", current=self.doc["current"])

    def end(self, state: str = "complete", error: str | None = None) -> None:
        counts = Counter(str(s.get("result")) for s in self.doc["stores"].values())
        self.doc["counts"] = dict(counts)
        self.doc["state"] = state
        self.doc["ended_at"] = time.time()
        self.doc["current"] = None
        if error:
            self.doc["error"] = error
        self._save()
        self._mirror("end", doc=self.doc)


def load_run(runs_dir: Path, which: str = "latest") -> dict[str, Any] | None:
    """A previous run's record: `which` is a run id, or "latest" (newest real run,
    not a dry run).
    """
    runs_dir = Path(runs_dir)
    if which != "latest":
        try:
            named: dict[str, Any] = json.loads((runs_dir / f"{which}.json").read_text())
        except (OSError, ValueError):
            return None
        return named
    best: tuple[float, dict[str, Any]] | None = None
    for p in runs_dir.glob("*.json"):
        try:
            doc: dict[str, Any] = json.loads(p.read_text())
        except (OSError, ValueError):
            continue
        if doc.get("dry_run"):
            continue
        started = float(doc.get("started_at") or 0)
        if best is None or started > best[0]:
            best = (started, doc)
    return best[1] if best else None


def done_locals(doc: dict[str, Any] | None) -> set[str]:
    """Local store paths a previous run finished (or found already on Argus)."""
    if not doc:
        return set()
    by_key = {e["key"]: e for e in doc.get("plan", []) if "key" in e}
    out = set()
    for key, st in doc.get("stores", {}).items():
        if st.get("result") in DONE_RESULTS and key in by_key:
            out.add(str(by_key[key].get("local")))
    return out
