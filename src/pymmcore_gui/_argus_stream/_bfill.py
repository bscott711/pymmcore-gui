"""Wire protocol and store helpers for the SSH backfill (sender side).

A mirror of ``opym.stream.bfill`` in the opym repo -- the receiver,
``opym-receive-backfill``, runs there -- kept here so the acquisition PC can
send without installing opym, whose package imports its whole processing
stack. Same rule as ``_protocol.py``: change both together.
"""

# ruff: noqa: D205, E501  (kept close to the opym original for diffing)

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from pathlib import Path
from typing import Any

import msgpack
import numpy as np

BF_INVENTORY = b"BF_INVENTORY"
BF_META = b"BF_META"
BF_VOLUME = b"BF_VOLUME"
BF_COMMIT = b"BF_COMMIT"
BF_RUN = b"BF_RUN"
BF_ACK = b"BF_ACK"
_VALID = frozenset({BF_INVENTORY, BF_META, BF_VOLUME, BF_COMMIT, BF_RUN, BF_ACK})

MARKER_NAME = ".opym_complete.json"
LOCK_NAME = ".opym_backfill.lock"
MARKER_FORMAT = 1
FEATURES = ["blosc", "links", "plane_probe", "run_log", "marker_from"]

# Store state words the server returns in an inventory reply.
ABSENT, PARTIAL, MARKED, BUSY, CONFLICT = (
    "absent",
    "partial",
    "marked",
    "busy",
    "conflict",
)

_STORE_NAME = re.compile(r"^[\w.+\- ]+\.ome\.zarr$")
_SAFE_PART = re.compile(r"^[\w.+\- ]+$")


def pack(
    msg_type: bytes,
    session_id: str,
    header: dict[str, Any],
    payload: bytes | None = None,
) -> list[bytes]:
    if msg_type not in _VALID:
        raise ValueError(f"unknown backfill message type {msg_type!r}")
    parts = [
        msg_type,
        session_id.encode("utf-8"),
        msgpack.packb(header, use_bin_type=True),
    ]
    if payload is not None:
        parts.append(payload)
    return parts


def unpack(parts: list[bytes]) -> tuple[bytes, str, dict[str, Any], bytes | None]:
    if len(parts) not in (3, 4):
        raise ValueError(f"expected 3 or 4 message parts, got {len(parts)}")
    msg_type, sid, header, *rest = parts
    if msg_type not in _VALID:
        raise ValueError(f"unknown backfill message type {msg_type!r}")
    return (
        msg_type,
        sid.decode("utf-8"),
        msgpack.unpackb(header, raw=False),
        (rest[0] if rest else None),
    )


def volume_digest(volume: np.ndarray) -> str:
    """SHA-256 of a volume's bytes in C order. Both ends compute it over the decoded
    array, so it is independent of how each side's chunks are compressed on disk.
    """
    return hashlib.sha256(
        memoryview(np.ascontiguousarray(volume)).cast("B")  # pyright: ignore[reportArgumentType]
    ).hexdigest()


def manifest_digest(digests: dict[int, str]) -> str:
    """One digest over the whole per-timepoint manifest (recorded in the marker)."""
    h = hashlib.sha256()
    for t in sorted(digests):
        h.update(f"{t}:{digests[t]}\n".encode())
    return h.hexdigest()


def valid_store_name(name: str) -> bool:
    return bool(_STORE_NAME.match(name)) and ".." not in name


def safe_leaf(leaf: str, raw_roots: list[Path]) -> Path | None:
    """`leaf` resolved, only if it lies under one of the allowed raw roots (and
    names no `..`). None otherwise.
    """
    if not leaf or ".." in Path(leaf).parts or not Path(leaf).is_absolute():
        return None
    p = Path(leaf)
    resolved = p.resolve()
    for root in raw_roots:
        if resolved == root or resolved.is_relative_to(root):
            return resolved
    return None


def read_marker(store: Path) -> dict | None:
    try:
        marker = json.loads((Path(store) / MARKER_NAME).read_text())
    except (OSError, ValueError):
        return None
    if marker.get("format") == MARKER_FORMAT and marker.get("state") == "complete":
        return marker  # type: ignore[no-any-return]
    return None


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".part")
    tmp.write_bytes(data)
    os.replace(tmp, path)


# ---- store layout helpers ---------------------------------------------------------------


def main_array_rel(store: Path) -> str | None:
    """Relative path of the store's pixel array: the `.zarray` with the most dimensions
    (then the largest). Everything else in the store is small metadata.
    """
    best: tuple[tuple[int, int], str] | None = None
    store = Path(store)
    for dirpath, dirnames, filenames in os.walk(store):
        if ".zarray" not in filenames:
            continue
        # An array's subdirectories are its chunks: never walk them (a 200-timepoint
        # store holds ~30,000 chunk files; rglob read them all, store after store).
        dirnames[:] = []
        try:
            meta = json.loads((Path(dirpath) / ".zarray").read_text())
        except (OSError, ValueError):
            continue
        shape = meta.get("shape", [])
        key = (len(shape), math.prod(shape) if shape else 0)
        rel = Path(dirpath).relative_to(store).as_posix()
        if best is None or key > best[0]:
            best = (key, "." if rel in ("", ".") else rel)
    return best[1] if best else None


def array_dir(store: Path, rel: str) -> Path:
    return Path(store) if rel in (".", "") else Path(store) / rel


def meta_files(store: Path, main_rel: str) -> list[str]:
    """Posix relpaths of every file in the store except the pixel array's chunk files
    (its own `.zarray`/`.zattrs` count as metadata), and except our marker and lock.
    """
    store = Path(store)
    skip_dir = None if main_rel in (".", "") else (store / main_rel)
    out = []
    for dirpath, dirnames, filenames in os.walk(store):
        d = Path(dirpath)
        if skip_dir is not None and d == skip_dir:
            out += [
                (d / n).relative_to(store).as_posix()
                for n in filenames
                if n in (".zarray", ".zattrs")
            ]
            dirnames[:] = []  # chunk directories
            continue
        for n in filenames:
            if n in (MARKER_NAME, LOCK_NAME) or n.endswith(".part"):
                continue
            rel = (d / n).relative_to(store).as_posix()
            # a flat (`.`-separated) main array keeps its chunks beside its metadata
            if (
                main_rel in (".", "")
                and n not in (".zarray", ".zattrs", ".zgroup")
                and not n.startswith(".")
            ):
                if re.fullmatch(r"\d+([./]\d+)*", n):
                    continue
            out.append(rel)
    return sorted(out)


def written_timepoints(store: Path, main_rel: str, ndim: int) -> list[int]:
    """Timepoints with a chunk directory on disk (4-D, `/`-separated stores), else [0]."""
    if ndim < 4:
        return [0]
    d = array_dir(store, main_rel)
    return sorted(int(e.name) for e in os.scandir(d) if e.name.isdigit() and e.is_dir())


def chunk_count_for_t(store: Path, main_rel: str, t: int) -> tuple[int, int]:
    """(chunk files present, smallest chunk file) for timepoint `t` of a `/`-separated 4-D store."""
    d = array_dir(store, main_rel) / str(t)
    n, smallest = 0, None
    if not d.is_dir():
        return 0, 0
    for root, _dirs, files in os.walk(d):
        for f in files:
            n += 1
            sz = os.stat(os.path.join(root, f)).st_size
            smallest = sz if smallest is None else min(smallest, sz)
    return n, smallest or 0


def expected_chunks_per_t(shape: list[int], chunks: list[int]) -> int:
    return math.prod(
        math.ceil(s / c) for s, c in zip(shape[1:], chunks[1:], strict=True)
    )


def newest_mtime(store: Path, main_rel: str) -> float:
    """Newest modification time among the store, its pixel array dir and that dir's
    direct entries (one per timepoint). A directory's mtime moves when a child is
    created, so this follows a writer that is still adding timepoints.
    """
    d = array_dir(store, main_rel)
    newest = max(Path(store).stat().st_mtime, d.stat().st_mtime)
    with os.scandir(d) as it:
        for e in it:
            try:
                newest = max(newest, e.stat().st_mtime)
            except OSError:
                continue
    return newest
