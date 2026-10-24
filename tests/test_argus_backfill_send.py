"""The backfill sender that ships with the app (``opym-backfill-send``).

On 2026-10-24 the rig had no way to run it: it lived in opym, which the rig
doesn't install. The end-to-end path against the real receiver
(``opym-receive-backfill``) was exercised on Argus; these tests pin the parts
that live here: scanning, path mapping, the summary and the tunnel wiring.
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path, PureWindowsPath
from typing import TYPE_CHECKING, Any

import numpy as np
import zarr

from pymmcore_gui._argus_stream import _bfill as bfill
from pymmcore_gui._argus_stream import backfill_send as bs

if TYPE_CHECKING:
    import pytest


def _store(path: Path, written: tuple[int, ...] = (0, 1)) -> Path:
    """Like the acquisition writer's: per-plane chunks, ``/`` separator."""
    root = zarr.open_group(str(path), mode="w")
    arr = root.create_dataset(
        "p0",
        shape=(3, 4, 5, 6),
        chunks=(1, 1, 5, 6),
        dtype="<u2",
        dimension_separator="/",
    )
    for t in written:
        arr[t] = np.full((4, 5, 6), t + 1, dtype=np.uint16)
    root.create_dataset("z", data=np.arange(4) * 0.5)
    return path


def test_stores_are_found_with_what_is_written(tmp_path: Path) -> None:
    exp = tmp_path / "S" / "20261023_2xfyve"
    _store(exp / "WT_Stim_005_mNG_488nm.ome.zarr", written=(0, 1))
    _store(exp / "WT_Stim_005_mScarlet_561nm.ome.zarr", written=(0,))
    (exp / "notes.txt").write_text("not a store")
    (leaf,) = bs.scan_local_root(tmp_path / "S")
    assert leaf.path == exp
    by_name = {s.name: s for s in leaf.stores}
    assert by_name["WT_Stim_005_mNG_488nm.ome.zarr"].written_t == [0, 1]
    assert by_name["WT_Stim_005_mScarlet_561nm.ome.zarr"].written_t == [0]
    assert by_name["WT_Stim_005_mNG_488nm.ome.zarr"].main_rel == "p0"


def test_the_local_path_is_mirrored_below_the_remote_root(tmp_path: Path) -> None:
    client = bs.BackfillClient(
        ["tcp://127.0.0.1:1"],
        [(tmp_path / "S", "/mmfs1/scratch/jacks.local/microscopy/")],
    )
    try:
        leaf = tmp_path / "S" / "20261023_LCK-NG_JB"
        assert client.remote_leaf(leaf) == (
            "/mmfs1/scratch/jacks.local/microscopy/20261023_LCK-NG_JB"
        )
    finally:
        client.close()


def test_a_windows_drive_map_parses() -> None:
    local, remote = bs._parse_map(r"S:\=/mmfs1/scratch/jacks.local/microscopy")
    assert PureWindowsPath(str(local)).drive in ("S:", "")  # "S:" on Windows
    assert remote == "/mmfs1/scratch/jacks.local/microscopy"


def test_the_summary_says_what_to_do() -> None:
    reports: list[dict[str, Any]] = [
        {
            "leaf": "S:/20261023",
            "remote": "/mmfs1/x/20261023",
            "stores": {
                "A_mNG_488nm.ome.zarr": {
                    "result": "would send",
                    "missing_timepoints": 3,
                    "of": 3,
                },
                "B_mNG_488nm.ome.zarr": {"result": "already complete"},
                "E_mNG_488nm.ome.zarr": {"result": "already on Argus"},
                "C_mNG_488nm.ome.zarr": {
                    "result": "refused",
                    "reason": "metadata refused: .zattrs exists and differs",
                },
                "D_mNG_488nm.ome.zarr": {"result": "settling", "idle_s": 30},
            },
        },
        {"leaf": "S:/elsewhere", "error": "S:/elsewhere is under no --map root"},
    ]
    text = bs.summarize(reports, dry_run=True)
    assert text.startswith("Dry run -- nothing was sent.")
    assert "1 already complete" in text and "1 would send" in text and "1 error" in text
    assert "1 already on Argus" in text
    assert (
        "SEND     A_mNG_488nm.ome.zarr: 3 of 3 timepoints -> /mmfs1/x/20261023" in text
    )
    assert "never overwrites" in text
    assert "under no --map root" in text


def test_no_tunnel_means_a_plain_message_not_a_hang(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
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
    monkeypatch.setattr(bs, "TUNNEL_UP_TIMEOUT_S", 0.3)
    code = bs.main(
        ["--map", f"{tmp_path}=/remote", "--ssh-host", "Argus", "--local-port", "1"]
    )
    assert code == 2
    assert started[0] == ("Argus", 1, bs.REMOTE_PORT, 2)
    assert started[-1][0] == "stopped"
    assert "Could not open an SSH tunnel to 'Argus'" in capsys.readouterr().out


def test_the_wire_helpers_match_the_receivers_contract() -> None:
    parts = bfill.pack(bfill.BF_INVENTORY, "rig", {"leaf": "/", "stores": []})
    msg_type, sid, header, payload = bfill.unpack(parts)
    assert (msg_type, sid, header, payload) == (
        bfill.BF_INVENTORY,
        "rig",
        {"leaf": "/", "stores": []},
        None,
    )
    vol = np.arange(24, dtype=np.uint16).reshape(2, 3, 4)
    assert bfill.volume_digest(vol) == bfill.volume_digest(np.asfortranarray(vol))
    assert bfill.valid_store_name("WT_Stim_005_mNG_488nm.ome.zarr")
    assert not bfill.valid_store_name("../x.ome.zarr")


def test_duplicates_and_stream_copies_read_plainly() -> None:
    reports: list[dict[str, Any]] = [
        {
            "leaf": "S:/exp",
            "remote": "/mmfs1/x/exp",
            "stores": {
                "A.ome.zarr": {
                    "result": "already on Argus",
                    "at": "/mmfs1/DataUpload/exp/A.ome.zarr",
                },
                "B.ome.zarr": {
                    "result": "partial copy elsewhere",
                    "at": "/mmfs1/y/B.ome.zarr",
                    "has": 1,
                    "of": 10,
                },
                "C.ome.zarr": {
                    "result": "would send",
                    "missing_timepoints": 9,
                    "of": 10,
                    "completes_stream_copy": True,
                },
            },
        }
    ]
    tiffs: Counter[str] = Counter({"S:/old/Cell_001": 300})
    text = bs.summarize(reports, dry_run=True, tiff_folders=tiffs)
    assert (
        "SKIP     A.ome.zarr: already on Argus at /mmfs1/DataUpload/exp/A.ome.zarr"
        in text
    )
    assert (
        "CHECK    B.ome.zarr: Argus has 1 of 10 timepoints" in text
        and "no second copy" in text
    )
    assert "9 of 10 timepoints, completing the live stream's partial copy" in text
    assert "Not handled: 300 TIFF file(s) in 1 folder(s)" in text


def test_tiff_files_on_the_way_are_counted(tmp_path: Path) -> None:
    old = tmp_path / "S" / "2025_old" / "Cell_001"
    old.mkdir(parents=True)
    for i in range(3):
        (old / f"img_{i}.tif").write_bytes(b"")
    _store(tmp_path / "S" / "2026_new" / "A_mNG_488nm.ome.zarr")
    tiffs: Counter[str] = Counter()
    leaves = bs.scan_local_root(tmp_path / "S", other=tiffs)
    assert [lf.path.name for lf in leaves] == ["2026_new"]
    assert tiffs == Counter({str(old): 3})
