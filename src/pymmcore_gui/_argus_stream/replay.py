"""Stream a saved acquisition to Argus exactly as a live run would.

Run on the acquisition PC, no microscope needed::

    uv run python -m pymmcore_gui._argus_stream.replay S:/path/Cell_001_GFP_488.ome.zarr

Give it one channel's store; the run's other channels
(``<base>_<Channel>_<wavelength>.ome.zarr`` next to it, as the MDA save
writes them) come along. The first ``--timepoints`` volumes are read into
RAM first (so disk reads can't distort the timing), then fed plane by plane
through the same :class:`~pymmcore_gui._argus_stream._session.ArgusStreamSession`
and tunnels the app uses, paced like the acquisition (``--stack-s`` per
volume, ``--interval`` between timepoints), to a TEST receiver on Argus
(``--remote-port``, default 5602), never the production one. A SHA-1 of every
(t, c) volume is sent to Argus afterwards, so the copy there can be checked
bit for bit.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import subprocess
import sys
import time
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


def _as_tzyx(store: Path) -> Any:
    node: Any = zarr.open(str(store), mode="r")
    arr = node["p0"] if hasattr(node, "array_keys") else node
    while arr.ndim > 4:  # (t, c, z, y, x) with one channel per store
        arr = arr[:, 0] if arr.shape[1] == 1 else arr[0]
    if arr.ndim == 3:
        arr = arr[np.newaxis]
    return arr


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
    ap.add_argument("--interval", type=float, default=5.0)
    ap.add_argument("--stack-s", type=float, default=1.6)
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
    arrays = [_as_tzyx(store) for _b, _c, store in channels]
    n_t = min(args.timepoints, *(a.shape[0] for a in arrays))
    log.info(
        "Reading %d timepoint(s) x %d channel(s) of %s from %s",
        n_t,
        len(arrays),
        arrays[0].shape[1:],
        args.path,
    )
    vols = [[np.ascontiguousarray(a[t]) for a in arrays] for t in range(n_t)]
    nz, ny, nx = vols[0][0].shape
    manifest = {
        f"{t},{c}": hashlib.sha1(v.tobytes()).hexdigest()
        for t in range(n_t)
        for c, v in enumerate(vols[t])
    }

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
        z_plan=useq.ZRangeAround(range=(nz - 1) * 0.5, step=0.5),
        time_plan={"interval": args.interval, "loops": n_t},
        axis_order="tcz",
        metadata={
            "pymmcore_widgets": {
                "save_dir": f"S:/{args.name}",
                "save_name": f"{base_name}.ome.zarr",
            }
        },
    )
    summary = {
        "image_infos": [
            {
                "camera_label": "Replay",
                "dtype": str(vols[0][0].dtype),
                "height": ny,
                "width": nx,
                "pixel_size_um": 0.136,
            }
        ]
    }
    session.sequenceStarted(seq, summary)  # type: ignore[arg-type]
    time.sleep(1.0)
    plane_s = args.stack_s / (nz * len(arrays))
    start = time.monotonic()
    t_prev, t_start, k = -1, start, 0
    for event in seq:
        t, c, z = event.index["t"], event.index["c"], event.index["z"]
        if t != t_prev:
            t_prev, k = t, 0
            while time.monotonic() < start + t * args.interval:
                time.sleep(0.002)
            t_start = time.monotonic()
        meta = {"camera_device": "Replay", "runner_time_ms": 0.0}
        session.frameReady(vols[t][c][z], event, meta)  # type: ignore[arg-type]
        k += 1
        ahead = t_start + k * plane_s - time.monotonic()
        if ahead > 0:
            time.sleep(ahead)
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

    report = {
        "name": args.name,
        "base_name": base_name,
        "links": args.links,
        "timepoints": n_t,
        "channels": [name for _b, name, _s in channels],
        "shape_zyx": [nz, ny, nx],
        "interval_s": args.interval,
        "stack_s": args.stack_s,
        "all_sent_after_last_plane_s": round(drained_s, 2),
        "states": sorted({s for s, _ in states}),
        "sha1": manifest,
    }
    print(json.dumps({k: v for k, v in report.items() if k != "sha1"}, indent=1))
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
