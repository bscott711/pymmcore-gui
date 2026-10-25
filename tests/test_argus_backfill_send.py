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
    monkeypatch.setattr(bs, "_links_from_settings", lambda: 4)
    code = bs.main(
        ["--map", f"{tmp_path}=/remote", "--ssh-host", "Argus", "--local-port", "1"]
    )
    assert code == 2
    assert started[0] == ("Argus", 1, bs.REMOTE_PORT, 4)  # as many links as the stream
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
                    "result": "would complete elsewhere",
                    "at": "/mmfs1/y/B.ome.zarr",
                    "missing_timepoints": 9,
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
        "SEND     B.ome.zarr: 9 of 10 timepoints, completing the copy at "
        "/mmfs1/y/B.ome.zarr" in text
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


def test_the_recycle_bin_and_system_folders_are_never_walked(tmp_path: Path) -> None:
    """A whole-drive map must not send runs someone deleted."""
    drive = tmp_path / "S"
    _store(drive / "$RECYCLE.BIN" / "S-1-5-21" / "$R0X1" / "Deleted_mNG_488nm.ome.zarr")
    _store(drive / "System Volume Information" / "X_mNG_488nm.ome.zarr")
    _store(drive / "20261023_exp" / "Kept_mNG_488nm.ome.zarr")
    leaves = bs.scan_local_root(drive)
    assert [s.name for lf in leaves for s in lf.stores] == ["Kept_mNG_488nm.ome.zarr"]


def test_the_scan_reports_progress_and_skips_chunks(tmp_path: Path) -> None:
    for k in range(3):
        _store(tmp_path / "S" / f"exp_{k}" / f"R{k}_mNG_488nm.ome.zarr")
    seen: list[tuple[Path, int]] = []
    leaves = bs.scan_local_root(
        tmp_path / "S", progress=lambda d, n: seen.append((d, n))
    )
    assert len(leaves) == 3 and seen[0] == (tmp_path / "S", 0)
    assert all("p0" not in d.parts for d, _ in seen)  # never inside a store
    assert bfill.main_array_rel(leaves[0].stores[0].path) == "p0"


def test_a_renamed_run_keeps_its_channel_suffix() -> None:
    assert (
        bs._renamed("Cell_005", "Cell_005_GFP_488.ome.zarr", 2)
        == "Cell_005_pc2_GFP_488.ome.zarr"
    )
    assert (
        bs._renamed("WT_004", "WT_004_mNG_488nm.ome.zarr", 3)
        == "WT_004_pc3_mNG_488nm.ome.zarr"
    )
    assert bs._renamed("odd", "odd.ome.zarr", 2) == "odd_pc2.ome.zarr"


def test_only_another_volume_size_or_type_counts_as_a_different_run(
    tmp_path: Path,
) -> None:
    store = bs.LocalStore.load(_store(tmp_path / "c_GFP_488.ome.zarr"))
    assert store is not None
    other_z = {"argus_shape": [3, 9, 5, 6], "argus_dtype": "<u2"}
    other_t = {"argus_shape": [7, 4, 5, 6], "argus_dtype": "<u2"}
    assert bs._different_run(store, other_z)
    assert not bs._different_run(store, other_t)  # may be the same run
    assert not bs._different_run(store, {})  # an older receiver says nothing


def test_the_summary_names_a_renamed_send_and_a_store_without_pixels() -> None:
    reports: list[dict[str, Any]] = [
        {
            "leaf": "S:\\x",
            "remote": "/r/x",
            "stores": {
                "Cell_005_GFP_488.ome.zarr": {
                    "result": "would send renamed",
                    "missing_timepoints": 150,
                    "of": 150,
                    "as": "Cell_005_pc2_GFP_488.ome.zarr",
                },
                "Cell_004_GFP_488.ome.zarr": {
                    "result": "no pixel data",
                    "reason": "no image array on this PC (largest: [111] <f8)",
                },
            },
        }
    ]
    text = bs.summarize(reports, dry_run=True)
    assert "as Cell_005_pc2_GFP_488.ome.zarr" in text
    assert "NO DATA  Cell_004_GFP_488.ome.zarr" in text


def test_the_run_record_is_written_before_and_after(tmp_path: Path) -> None:
    from pymmcore_gui._argus_stream import _bfill_run as bfill_run

    sent: list[dict[str, Any]] = []
    log = bfill_run.RunLog(
        tmp_path, client="EEP230-OPM", dry_run=False, maps=[], send=sent.append
    )
    log.plan([{"key": "/r/x/a.ome.zarr", "local": "S:\\x\\a.ome.zarr"}])
    assert log.path is not None and log.path.exists()
    log.store("/r/x/a.ome.zarr", {"result": "complete"})
    log.end()
    assert [h["op"] for h in sent] == ["start", "store", "end"]
    doc = bfill_run.load_run(tmp_path)
    assert doc is not None and doc["counts"] == {"complete": 1}
    assert bfill_run.done_locals(doc) == {"S:\\x\\a.ome.zarr"}


def test_a_whole_copy_made_another_way_is_checked_not_trusted(tmp_path: Path) -> None:
    """2026-10-25: a faithful copy of S:\\ is the point; copies the live stream or
    Globus made had only ever been counted, never compared with the PC."""
    store = bs.LocalStore.load(_store(tmp_path / "c_GFP_488.ome.zarr"))
    assert store is not None
    whole = {"state": bfill.PARTIAL, "have_t": [0, 1], "stream_copy": True}
    client = bs.BackfillClient(["tcp://127.0.0.1:1"], [(tmp_path, "/r")])
    try:
        d = client._decide("/r", store, whole)
        assert d["action"] == "verify" and not d["pixels_only"]
        single = {**whole, "single_volume_as_4d": True}
        assert client._decide("/r", store, single)["pixels_only"]
        client.verify = False
        assert client._decide("/r", store, whole)["result"] == "already on Argus"
    finally:
        client.close()


def test_a_store_whose_non_pixel_files_are_huge_is_not_sent(tmp_path: Path) -> None:
    """2026-10-25: the rig locked up right as a store's 'metadata' was gathered.
    Gigabytes of unrecognised array chunks must never go out as metadata."""
    path = _store(tmp_path / "c_GFP_488.ome.zarr")
    rels = bfill.meta_files(path, "p0")
    assert bs._meta_too_big(path, rels) is None
    zattrs = path / "p0" / ".zattrs"  # real: per-frame metadata, up to 110 MB on S:
    with zattrs.open("ab") as f:
        f.truncate(110 << 20)
    assert bs._meta_too_big(path, [*rels, "p0/.zattrs"]) is None
    big = path / "extra.bin"
    with big.open("wb") as f:
        f.truncate(bs.META_MAX_BYTES + 1)
    reason = bs._meta_too_big(path, [*rels, "extra.bin"])
    assert reason is not None and "nothing was sent" in reason
    leaked = [f"p1/0/{z}" for z in range(bs.META_MAX_FILES + 1)]  # chunks as "meta"
    for rel in leaked:
        (path / rel).parent.mkdir(parents=True, exist_ok=True)
        (path / rel).write_bytes(b"x")
    assert bs._meta_too_big(path, [*rels, *leaked]) is not None


def test_a_dead_tunnel_is_skipped_and_the_last_one_is_never_given_up_silently() -> None:
    link = bs._Link(["tcp://127.0.0.1:1", "tcp://127.0.0.1:2"], "t")
    try:
        link.mark_dead(0, "test")
        assert {link.pick() for _ in range(4)} == {1}
        link.mark_dead(1, "test")
        try:
            link.pick()
        except bs.BackfillError as exc:
            assert "no tunnel to Argus answers" in str(exc)
        else:
            raise AssertionError("picked a dead tunnel")
    finally:
        link.close()
