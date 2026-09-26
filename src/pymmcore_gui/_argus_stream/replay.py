"""Stream a saved acquisition to Argus exactly as a live run would.

Run on the acquisition PC, no microscope needed::

    uv run python -m pymmcore_gui._argus_stream.replay S:/path/Cell_001_GFP_488.ome.zarr

Give it one channel's store; the run's other channels
(``<base>_<Channel>_<wavelength>.ome.zarr`` next to it, as the MDA save
writes them) come along. Each volume is fed plane by plane through the same
:class:`~pymmcore_gui._argus_stream._session.ArgusStreamSession` and tunnels
the app uses, to a TEST receiver on Argus (``--remote-port``, default 5602),
never the production one. A SHA-1 of every (t, c) volume is sent to Argus
afterwards, so the copy there can be checked bit for bit.

Pacing (``--pace``): by default the planes go out when the original run
exposed them. The MDA save records every frame's ``runner_time_ms`` in each
channel store's ``frame_meta``: the run's setup before its first plane, each
channel's stack in turn (the 488 stack, then the 561 stack), the gap between
them, and the time from one timepoint to the next. Those are replayed, from
a template of the recorded timepoints (median per plane), for as many
timepoints as asked for. The run's own z step comes from its recorded
``useq_MDASequence``. A store with no frame times (or ``--pace fixed``) is
paced by ``--stack-s`` per channel, channels one after the other, every
``--interval``.

A reader thread keeps the next few volumes (``--readahead`` timepoints) in
RAM, so a long run never needs all of it at once; any time the send waits
on the disk is reported as ``read_stall_s``, so a slow disk can't pass for
network or processing lag.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import logging
import queue
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import useq
import zarr

from ._session import ArgusStreamSession
from ._tunnel import ArgusTunnelManager

log = logging.getLogger("pymmcore_gui.replay")


def _split(store: Path) -> tuple[str, str]:
    """``<base>_<Channel>_<wavelength>.ome.zarr`` -> (base, "Channel_wavelength")."""
    parts = store.name.removesuffix(".ome.zarr").split("_")
    if len(parts) < 3:
        raise SystemExit(f"{store.name}: expected <base>_<Channel>_<wavelength>")
    return "_".join(parts[:-2]), "_".join(parts[-2:])


def _channel_stores(path: Path) -> list[tuple[str, str, Path]]:
    """``(base_name, channel_name, store)`` for each channel of one run.

    ``path`` is one channel's store (its sibling channels are found next to
    it) or a folder holding a single run's channel stores.
    """
    if path.name.endswith(".ome.zarr"):
        base = _split(path)[0]
        stores = [s for s in path.parent.glob("*.ome.zarr") if _split(s)[0] == base]
    else:
        stores = list(path.glob("*.ome.zarr"))
        bases = sorted({_split(s)[0] for s in stores})
        if len(bases) > 1:
            raise SystemExit(
                f"{path} holds several runs ({', '.join(bases)}); "
                "pass one channel's .ome.zarr instead"
            )
    if not stores:
        raise SystemExit(f"no *.ome.zarr stores at {path}")
    return [(*_split(s), s) for s in sorted(stores)]


def _array(store: Path) -> Any:
    node: Any = zarr.open(str(store), mode="r")
    return node["p0"] if hasattr(node, "array_keys") else node


def _as_tzyx(arr: Any, name: str) -> Any:
    if arr.ndim == 5:  # (t, c, z, y, x): the MDA save writes one channel per store
        if arr.shape[1] != 1:
            raise SystemExit(f"{name} holds {arr.shape[1]} channels; expected one")
        arr = arr[:, 0]
    if arr.ndim not in (3, 4):
        raise SystemExit(f"{name}: can't read a {arr.ndim}-D array as (t, z, y, x)")
    if arr.ndim == 3:
        arr = arr[np.newaxis]
    return arr


# --- pacing -----------------------------------------------------------------


@dataclass(frozen=True)
class Schedule:
    """When each plane goes out.

    Timepoint ``t``'s plane ``(c, z)`` at ``setup_s + t * interval_s +
    offsets[(c, z)]`` seconds after the run starts.
    """

    source: str  # "recorded" or "fixed"
    setup_s: float
    interval_s: float
    offsets: dict[tuple[int, int], float]

    def at(self, t: int, c: int, z: int) -> float:
        return self.setup_s + t * self.interval_s + self.offsets[(c, z)]

    def order(self, n_t: int) -> list[tuple[float, int, int, int]]:
        """Every plane of ``n_t`` timepoints as ``(time, t, c, z)``, in order."""
        return sorted(
            (self.at(t, c, z), t, c, z) for t in range(n_t) for c, z in self.offsets
        )

    def summary(self) -> dict[str, Any]:
        """Per channel, its stack's length and start within a timepoint."""
        chans: dict[int, list[float]] = {}
        for (c, _z), s in self.offsets.items():
            chans.setdefault(c, []).append(s)
        return {
            "pace": self.source,
            "setup_s": round(self.setup_s, 3),
            "interval_s": round(self.interval_s, 3),
            "channel_start_s": {c: round(min(v), 3) for c, v in sorted(chans.items())},
            "channel_stack_s": {
                c: round(max(v) - min(v), 3) for c, v in sorted(chans.items())
            },
        }


def fixed_schedule(
    n_c: int, nz: int, *, stack_s: float, interval_s: float, setup_s: float = 1.0
) -> Schedule:
    """Channels one after the other, ``stack_s`` each, evenly paced planes."""
    plane_s = stack_s / nz
    offsets = {(c, z): (c * nz + z) * plane_s for c in range(n_c) for z in range(nz)}
    return Schedule("fixed", setup_s, interval_s, offsets)


def recorded_schedule(frame_meta: list[dict], n_c: int, nz: int) -> Schedule | None:
    """The original run's pacing, from its frames' ``runner_time_ms``.

    ``frame_meta``: every channel store's frame records together. Uses the
    timepoints recorded in full (every channel, every plane): each plane's
    offset from its timepoint's first plane (the median over them), the
    median time between consecutive timepoints, and the first plane's time
    after the run started (the MDA's own setup). None if no timepoint was
    recorded in full.
    """
    per_t: dict[int, dict[tuple[int, int], float]] = {}
    for m in frame_meta:
        try:
            idx = m["mda_event"]["index"]
            ms = float(m["runner_time_ms"])
        except (KeyError, TypeError, ValueError):
            continue
        per_t.setdefault(int(idx.get("t", 0)), {})[
            (int(idx.get("c", 0)), int(idx.get("z", 0)))
        ] = ms / 1000.0
    want = {(c, z) for c in range(n_c) for z in range(nz)}
    full = sorted(t for t, planes in per_t.items() if want <= planes.keys())
    if not full:
        return None
    starts = {t: min(per_t[t][k] for k in want) for t in full}
    offsets = {
        k: statistics.median(per_t[t][k] - starts[t] for t in full) for k in want
    }
    steps = [(starts[b] - starts[a]) / (b - a) for a, b in itertools.pairwise(full)]
    last = max(offsets.values())
    interval = statistics.median(steps) if steps else last + 1.0
    setup = starts[full[0]] - full[0] * interval
    return Schedule("recorded", max(0.0, setup), interval, offsets)


def recorded_z_step(arrays: list[Any]) -> float | None:
    """The run's z step, from the ``useq_MDASequence`` the MDA save records."""
    for arr in arrays:
        seq = dict(arr.attrs).get("useq_MDASequence")
        if isinstance(seq, str):
            try:
                seq = json.loads(seq)
            except ValueError:
                continue
        step = ((seq or {}).get("z_plan") or {}).get("step")
        if step:
            return float(step)
    return None


# --- reading ahead ------------------------------------------------------------


class VolumeReader(threading.Thread):
    """Reads (t, c) volumes in send order into a bounded queue.

    Each is hashed as it's read. ``get`` hands them out and counts the time
    spent waiting.
    """

    def __init__(self, arrays: list[Any], order: list[tuple[int, int]], depth: int):
        super().__init__(daemon=True, name="replay-reader")
        self._arrays = arrays
        self._order = order
        self._q: queue.Queue = queue.Queue(maxsize=max(1, depth))
        self._ready: dict[tuple[int, int], np.ndarray] = {}
        self.sha1: dict[str, str] = {}
        self.stalls: list[float] = []
        self.error: BaseException | None = None

    def run(self) -> None:
        try:
            for t, c in self._order:
                vol = np.ascontiguousarray(self._arrays[c][t])
                self.sha1[f"{t},{c}"] = hashlib.sha1(vol.tobytes()).hexdigest()
                self._q.put(((t, c), vol))
        except BaseException as exc:  # reported by get()
            self.error = exc
            self._q.put(None)

    def get(self, t: int, c: int) -> np.ndarray:
        waited = time.monotonic()
        while (t, c) not in self._ready:
            item = self._q.get()
            if item is None:
                raise RuntimeError(f"reading volume t={t} c={c} failed") from self.error
            self._ready[item[0]] = item[1]
        self.stalls.append(time.monotonic() - waited)
        return self._ready.pop((t, c))


# --- the run ------------------------------------------------------------------


class _Core:
    def getPixelSizeUm(self) -> float:
        return 0.0

    def getCameraDevice(self) -> str:
        return "Replay"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    ap.add_argument("path", type=Path, help="one channel's .ome.zarr (or its folder)")
    ap.add_argument("--links", type=int, default=4)
    ap.add_argument("--timepoints", type=int, default=10)
    ap.add_argument(
        "--pace",
        choices=("auto", "recorded", "fixed"),
        default="auto",
        help="recorded frame times (auto: when the store has them) or fixed",
    )
    ap.add_argument(
        "--interval", type=float, default=None, help="s between timepoints (override)"
    )
    ap.add_argument(
        "--stack-s", type=float, default=1.6, help="s per channel's stack (fixed pace)"
    )
    ap.add_argument(
        "--setup-s",
        type=float,
        default=None,
        help="s from the run's start to its first plane (override)",
    )
    ap.add_argument(
        "--readahead", type=int, default=3, help="timepoints read ahead into RAM"
    )
    ap.add_argument("--host", default=None, help="ssh Host alias (settings)")
    ap.add_argument("--remote-port", type=int, default=5602)
    ap.add_argument("--local-port", type=int, default=5800)
    ap.add_argument("--argus-root", default="/dev/shm/opym_links_pc/raw")
    ap.add_argument("--name", default=time.strftime("replay_%H%M%S"))
    ap.add_argument("--ssh-arg", action="append", default=[], help="extra ssh argument")
    args = ap.parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s"
    )
    from pymmcore_gui._settings import (
        ArgusStreamSettingsV1,
        Settings,
        SettingsV1,
        SpectralChannelSettingsV1,
    )

    host = args.host or Settings.instance().argus_stream.ssh_host
    channels = _channel_stores(args.path)
    base_name = channels[0][0]
    raw = [_array(store) for _b, _c, store in channels]
    arrays = [
        _as_tzyx(a, store.name)
        for a, (_b, _c, store) in zip(raw, channels, strict=True)
    ]
    n_c = len(arrays)
    n_t = min(args.timepoints, *(a.shape[0] for a in arrays))
    nz, ny, nx = arrays[0].shape[1:]

    schedule = None
    if args.pace != "fixed":
        metas = [m for a in raw for m in (dict(a.attrs).get("frame_meta") or [])]
        schedule = recorded_schedule(metas, n_c, nz)
        if schedule is None and args.pace == "recorded":
            raise SystemExit(f"{args.path}: no complete recorded timepoint to pace by")
    if schedule is None:
        schedule = fixed_schedule(
            n_c, nz, stack_s=args.stack_s, interval_s=args.interval or 5.0
        )
    if args.interval is not None or args.setup_s is not None:
        schedule = Schedule(
            schedule.source,
            schedule.setup_s if args.setup_s is None else args.setup_s,
            schedule.interval_s if args.interval is None else args.interval,
            schedule.offsets,
        )
    dz = recorded_z_step(raw) or 0.5
    log.info(
        "Replaying %d timepoint(s) x %d channel(s) of %s from %s, %s",
        n_t,
        n_c,
        (nz, ny, nx),
        args.path,
        json.dumps(schedule.summary()),
    )

    order = schedule.order(n_t)
    volume_order = list(dict.fromkeys((t, c) for _s, t, c, _z in order))
    reader = VolumeReader(arrays, volume_order, depth=args.readahead * n_c)
    reader.start()

    tunnel = ArgusTunnelManager(
        host, args.local_port, args.remote_port, links=args.links, ssh_args=args.ssh_arg
    )
    tunnel.start()
    time.sleep(1.0 + 0.5 * args.links)
    settings = SettingsV1(
        argus_stream=ArgusStreamSettingsV1(
            enabled=True,
            local_port=args.local_port,
            stream_links=args.links,
            gpfs_scratch_root=args.argus_root,
        ),
        spectral=SpectralChannelSettingsV1(enabled=False),
    )
    states: list[tuple[str, str]] = []
    session = ArgusStreamSession(
        _Core(),  # type: ignore[arg-type]
        tunnel,
        get_settings=lambda: settings,
        on_state_changed=lambda s, d: states.append((s.value, d)),
    )
    seq = useq.MDASequence(
        channels=[name for _b, name, _s in channels],
        z_plan=useq.ZRangeAround(range=(nz - 1) * dz, step=dz),
        time_plan={"interval": schedule.interval_s, "loops": n_t},
        axis_order="tcz",
        metadata={
            "pymmcore_widgets": {
                "save_dir": f"S:/{args.name}",
                "save_name": f"{base_name}.ome.zarr",
            }
        },
    )
    events = {(e.index["t"], e.index["c"], e.index["z"]): e for e in seq}
    summary = {
        "image_infos": [
            {
                "camera_label": "Replay",
                "dtype": str(arrays[0].dtype),
                "height": ny,
                "width": nx,
                "pixel_size_um": 0.136,
            }
        ]
    }
    captured: list[str] = []

    class _Capture(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            captured.append(self.format(record))

    capture = _Capture(logging.INFO)
    capture.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger("pymmcore_gui._argus_stream").addHandler(capture)

    # The run starts here, as the MDA's sequenceStarted does; its first plane
    # comes setup_s later.
    start = time.monotonic()
    session.sequenceStarted(seq, summary)  # type: ignore[arg-type]
    worker = session._worker
    late: list[float] = []
    current: tuple[int, int] | None = None
    vol: np.ndarray | None = None
    for due, t, c, z in order:
        if (t, c) != current:
            current, vol = (t, c), reader.get(t, c)
        ahead = start + due - time.monotonic()
        if ahead > 0:
            time.sleep(ahead)
        else:
            late.append(-ahead)
        meta = {"camera_device": "Replay", "runner_time_ms": due * 1000.0}
        session.frameReady(vol[z], events[(t, c, z)], meta)  # type: ignore[arg-type,index]
    session.sequenceFinished(seq)
    last_plane = time.monotonic()
    while any(w.is_alive() for w in session._all_workers):
        time.sleep(0.1)
        if time.monotonic() - last_plane > 600:
            log.error("gave up waiting for Argus")
            break
    drained_s = time.monotonic() - last_plane
    session.shutdown(timeout=2)
    tunnel.stop()

    stalls = reader.stalls
    report = {
        "name": args.name,
        "base_name": base_name,
        "links": args.links,
        "timepoints": n_t,
        "channels": [name for _b, name, _s in channels],
        "shape_zyx": [nz, ny, nx],
        "z_step_um": dz,
        **schedule.summary(),
        "read_stall_s": {
            "total": round(sum(stalls), 3),
            "max": round(max(stalls, default=0.0), 3),
            "n_over_10ms": sum(s > 0.01 for s in stalls),
        },
        "planes_late_s": {
            "n_over_10ms": sum(s > 0.01 for s in late),
            "max": round(max(late, default=0.0), 3),
        },
        "all_sent_after_last_plane_s": round(drained_s, 2),
        "states": sorted({s for s, _ in states}),
        "stats": worker.stats if worker is not None else {},
        "log": captured[-300:],
        "sha1": reader.sha1,
    }
    print(
        json.dumps(
            {k: v for k, v in report.items() if k not in ("sha1", "log")}, indent=1
        )
    )
    try:
        subprocess.run(
            ["ssh", *args.ssh_arg, host, f"cat > {args.name}.replay.json"],
            input=json.dumps(report).encode(),
            check=True,
            timeout=60,
        )
        print(f"Report sent to Argus (~/{args.name}.replay.json).")
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"Could not send the report to Argus ({exc}); paste it instead.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
