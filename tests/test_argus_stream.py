from __future__ import annotations

import contextlib
import random
import socket
import threading
import time
from typing import TYPE_CHECKING, cast

import numpy as np
import pytest
import useq
import zmq

from pymmcore_gui._argus_stream import ArgusStreamSession, StreamState
from pymmcore_gui._argus_stream import _session as session_mod
from pymmcore_gui._argus_stream._protocol import (
    MSG_ACK,
    MSG_FRAME,
    MSG_QC,
    MSG_RESUME,
    MSG_SESSION_END,
    MSG_SESSION_START,
    AckHeader,
    MessageHeader,
    pack_message,
    unpack_message,
)
from pymmcore_gui._argus_stream._session import (
    CameraGeometry,
    _active_spectral_channels,
    _build_session_header,
    _camera_geometry,
)
from pymmcore_gui._argus_stream._volume_assembler import VolumeAssembler
from pymmcore_gui._settings import (
    ArgusStreamSettingsV1,
    SettingsV1,
    SpectralChannelConfig,
    SpectralChannelSettingsV1,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

PYMMCW_KEY = "pymmcore_widgets"
LASER_GROUP = "Lasers"
ALL_LASERS = "AllLasers"


class _StubCore:
    """Minimal stand-in exposing only what ``_build_session_header`` /
    ``_active_spectral_channels`` need."""

    def __init__(
        self,
        num_channels: int = 1,
        width: int = 8,
        height: int = 6,
        bytes_per_pixel: int = 2,
        pixel_size_um: float = 0.11,
        physical_cameras: list[str] | None = None,
    ) -> None:
        self._num_channels = num_channels
        self._width = width
        self._height = height
        self._bytes_per_pixel = bytes_per_pixel
        self._pixel_size_um = pixel_size_um
        self._physical_cameras = physical_cameras or ["Camera"]

    def getNumberOfCameraChannels(self) -> int:
        return self._num_channels

    def getCameraDevice(self) -> str:
        return self._physical_cameras[0]

    def getPhysicalCameraDevice(self, i: int) -> str:
        return self._physical_cameras[i]

    def getImageWidth(self) -> int:
        return self._width

    def getImageHeight(self) -> int:
        return self._height

    def getBytesPerPixel(self) -> int:
        return self._bytes_per_pixel

    def getPixelSizeUm(self) -> float:
        return self._pixel_size_um


def _geom(core: _StubCore) -> CameraGeometry:
    return _camera_geometry({}, core)  # pyright: ignore[reportArgumentType]


def test_camera_geometry_prefers_summary_meta_over_unloaded_core() -> None:
    """Rig failure 2026-09-23: by sequenceStarted the ASI engine had already
    handed the cameras to worker processes, so the core reported no camera
    and 0 bytes/pixel. The summary meta, built in setup_sequence, still has
    the real geometry."""
    unloaded = _StubCore(width=0, height=0, bytes_per_pixel=0, physical_cameras=[""])
    meta = {
        "image_infos": (
            {
                "camera_label": "Multi Camera",
                "dtype": "uint16",
                "height": 2400,
                "width": 2400,
                "pixel_size_um": 0.136,
                "num_camera_adapter_channels": 2,
            },
            {"camera_label": "Camera-1", "dtype": "uint16", "height": 2400},
            {"camera_label": "Camera-2", "dtype": "uint16", "height": 2400},
        )
    }
    geom = _camera_geometry(meta, unloaded)  # pyright: ignore[reportArgumentType]
    assert geom == CameraGeometry(
        labels=["Camera-1", "Camera-2"],
        dtype="uint16",
        height=2400,
        width=2400,
        pixel_size_um=0.136,
    )


def test_build_session_header_skips_unknown_dtype() -> None:
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(
        _save_seq(), _geom(_StubCore(bytes_per_pixel=0)), settings, []
    )
    assert header is None
    assert "pixel type" in reason


def _spectral_channels() -> list[SpectralChannelConfig]:
    """Two cameras, two vertically-stacked regions each, on a 4x4 test frame."""
    return [
        SpectralChannelConfig(
            name="GFP_488", camera="Camera-1", laser_preset="488nm", rect=(0, 0, 4, 2)
        ),
        SpectralChannelConfig(
            name="CalceinViolet_405",
            camera="Camera-1",
            laser_preset="405nm",
            rect=(0, 2, 4, 2),
        ),
        SpectralChannelConfig(
            name="mScarlet_561",
            camera="Camera-2",
            laser_preset="561nm",
            rect=(0, 0, 4, 2),
        ),
        SpectralChannelConfig(
            name="CF647_638", camera="Camera-2", laser_preset="638nm", rect=(0, 2, 4, 2)
        ),
    ]


def _spectral_settings(**overrides: object) -> SpectralChannelSettingsV1:
    kwargs: dict = {
        "enabled": True,
        "channels": _spectral_channels(),
        "laser_config_group": LASER_GROUP,
        "all_lasers_preset": ALL_LASERS,
    }
    kwargs.update(overrides)
    return SpectralChannelSettingsV1(**kwargs)  # pyright: ignore[reportArgumentType]


def _save_seq(**overrides: object) -> useq.MDASequence:
    """A single-position, two-channel, three-z-plane sequence with save metadata."""
    save_meta = {"save_dir": "S:/exp", "save_name": "cooked_001.ome.zarr"}
    kwargs: dict = {
        "channels": ["488nm", "561nm"],
        "z_plan": useq.ZRangeAround(range=2, step=1),
        "metadata": {PYMMCW_KEY: save_meta},
    }
    kwargs.update(overrides)
    return useq.MDASequence(**kwargs)  # pyright: ignore[reportArgumentType]


# ----------------------------------------------------------------------------
# protocol.py
# ----------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("msg_type", "header", "payload"),
    [
        (MSG_SESSION_START, {"base_name": "x", "num_timepoints": 3}, None),
        (MSG_FRAME, {"t": 1, "c": 0, "frame_index": 4}, b"\x00\x01\x02\x03"),
        (MSG_SESSION_END, {"reason": "complete"}, None),
        (MSG_ACK, {"through_frame_index": 7}, None),
        (MSG_RESUME, {}, None),
    ],
)
def test_pack_unpack_round_trip(
    msg_type: bytes, header: dict, payload: bytes | None
) -> None:
    parts = pack_message(msg_type, "session-1", cast("MessageHeader", header), payload)
    assert all(isinstance(p, bytes) for p in parts)
    assert len(parts) == (4 if payload is not None else 3)

    out_type, out_sid, out_header, out_payload = unpack_message(parts)
    assert out_type == msg_type
    assert out_sid == "session-1"
    assert out_header == header
    assert out_payload == payload


def test_pack_message_rejects_unknown_type() -> None:
    with pytest.raises(ValueError, match="Unknown message type"):
        pack_message(b"NOT_A_TYPE", "s", {})


def test_unpack_message_rejects_bad_part_count() -> None:
    with pytest.raises(ValueError, match="Expected 3 or 4"):
        unpack_message([b"only", b"two"])


# ----------------------------------------------------------------------------
# _volume_assembler.py
# ----------------------------------------------------------------------------


def test_volume_assembler_completes_one_volume_per_t_c() -> None:
    seq = useq.MDASequence(
        channels=["DAPI", "FITC"],  # pyright: ignore[reportArgumentType]
        z_plan=useq.ZRangeAround(range=2, step=1),  # 3 z steps
        time_plan=useq.TIntervalLoops(interval=0, loops=2),  # pyright: ignore
    )
    asm = VolumeAssembler()
    asm.reset(seq)

    completed = []
    for event in seq:
        frame = np.full((4, 4), event.index.get("z", 0), dtype="uint16")
        vol = asm.add_frame(frame, event, {})  # pyright: ignore[reportArgumentType]
        if vol is not None:
            completed.append(vol)

    assert len(completed) == 4  # 2 timepoints x 2 channels
    assert {(v.t, v.c) for v in completed} == {(0, 0), (0, 1), (1, 0), (1, 1)}
    for vol in completed:
        assert vol.array.shape == (3, 4, 4)
        # z-plane content should land at the correct z-index regardless of
        # the sequence's axis order (see next test for an explicit check).
        assert list(vol.array[:, 0, 0]) == [0, 1, 2]


def test_volume_assembler_stamps_first_and_last_plane_times() -> None:
    seq = useq.MDASequence(z_plan=useq.ZRangeAround(range=2, step=1))
    asm = VolumeAssembler()
    asm.reset(seq)
    before = time.time()
    vols = [
        v
        for event in seq
        if (v := asm.add_frame(np.zeros((4, 4), "uint16"), event, {}))  # pyright: ignore[reportArgumentType]
    ]
    assert len(vols) == 1
    assert before <= vols[0].acq_first_s <= vols[0].acq_last_s <= time.time()


def test_volume_assembler_slab_mode_hands_out_consecutive_planes() -> None:
    seq = useq.MDASequence(z_plan=useq.ZRangeAround(range=4, step=1))  # 5 planes
    asm = VolumeAssembler()
    asm.reset(seq)
    events = list(seq)
    got = []
    for event in events:
        frame = np.full((4, 4), event.index["z"], dtype="uint16")
        got.append(asm.add_plane(frame, event, {}, slab_planes=2))  # pyright: ignore[reportArgumentType]
    # A 2-plane slab after planes 1 and 3; the 1-plane remainder at the end.
    assert [len(g) for g in got] == [0, 1, 0, 1, 1]
    slabs = [v for g in got for v in g]
    assert [(v.z0, v.array.shape[0], v.nz) for v in slabs] == [
        (0, 2, 5),
        (2, 2, 5),
        (4, 1, 5),
    ]
    assert [list(v.array[:, 0, 0]) for v in slabs] == [[0, 1], [2, 3], [4]]
    assert all(v.acq_first_s <= v.acq_last_s for v in slabs)


def test_volume_assembler_slab_waits_for_a_gap_to_fill() -> None:
    seq = useq.MDASequence(z_plan=useq.ZRangeAround(range=2, step=1))  # 3 planes
    asm = VolumeAssembler()
    asm.reset(seq)
    ev = {e.index["z"]: e for e in seq}
    zero = np.zeros((4, 4), "uint16")
    assert asm.add_plane(zero, ev[1], {}, slab_planes=1) == []  # pyright: ignore[reportArgumentType]
    assert asm.add_plane(zero, ev[2], {}, slab_planes=1) == []  # pyright: ignore[reportArgumentType]
    out = asm.add_plane(zero, ev[0], {}, slab_planes=1)  # pyright: ignore[reportArgumentType]
    assert [v.z0 for v in out] == [0, 1, 2]


def test_volume_assembler_is_axis_order_agnostic() -> None:
    """z-planes delivered in reverse order still land at the right index."""
    seq = useq.MDASequence(z_plan=useq.ZRangeAround(range=2, step=1))
    asm = VolumeAssembler()
    asm.reset(seq)

    events = list(seq)[::-1]  # z=2,1,0 instead of 0,1,2
    vol = None
    for event in events:
        frame = np.full((3, 3), event.index["z"], dtype="uint8")
        result = asm.add_frame(frame, event, {})  # pyright: ignore[reportArgumentType]
        if result is not None:
            vol = result
    assert vol is not None
    assert list(vol.array[:, 0, 0]) == [0, 1, 2]


def test_volume_assembler_no_z_plan_is_single_plane_volume() -> None:
    seq = useq.MDASequence(channels=["DAPI"])  # pyright: ignore[reportArgumentType]
    asm = VolumeAssembler()
    asm.reset(seq)
    event = next(iter(seq))
    vol = asm.add_frame(np.zeros((2, 2), dtype="uint8"), event, {})  # pyright: ignore[reportArgumentType]
    assert vol is not None
    assert vol.array.shape == (1, 2, 2)


# ----------------------------------------------------------------------------
# _build_session_header
# ----------------------------------------------------------------------------


def test_build_session_header_happy_path() -> None:
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/mmfs2/scratch/lab")
    header, reason = _build_session_header(
        _save_seq(), _geom(_StubCore()), settings, []
    )
    assert reason == ""
    assert header is not None
    assert header["base_name"] == "cooked_001"
    # raw_root mirrors the local save_dir's structure under
    # gpfs_scratch_root, drive letter stripped (_save_seq's save_dir is
    # "S:/exp") -- channel stores are named "<base_name>_<channel>.ome.zarr"
    # directly under THAT (see opym.stream.rawmirror.store_path_for_channel
    # on Argus), not nested under a further base_name/Decon subdirectory.
    assert header["raw_root"] == "/mmfs2/scratch/lab/exp"
    assert header["dtype"] == "uint16"
    assert header["shape_zyx"] == [3, 6, 8]
    assert header["num_timepoints"] == 1
    assert header["channels"] == [0, 1]
    assert header["channel_names"] == ["488nm", "561nm"]


def test_build_session_header_raw_root_mirrors_nested_save_dir_structure() -> None:
    """A multi-segment local path is preserved in full, not just its last part."""
    settings = ArgusStreamSettingsV1(
        gpfs_scratch_root="/mmfs1/scratch/jacks.local/microscopy"
    )
    seq = _save_seq(
        metadata={
            PYMMCW_KEY: {
                "save_dir": "S:/20260922-SVO-YG_0.1umBead_PSF",
                "save_name": "Bead_001.ome.zarr",
            }
        }
    )
    header, reason = _build_session_header(seq, _geom(_StubCore()), settings, [])
    assert reason == ""
    assert header is not None
    assert (
        header["raw_root"]
        == "/mmfs1/scratch/jacks.local/microscopy/20260922-SVO-YG_0.1umBead_PSF"
    )
    assert header["base_name"] == "Bead_001"


def test_build_session_header_requires_save_dir() -> None:
    seq = _save_seq(metadata={PYMMCW_KEY: {"save_name": "cooked_001.ome.zarr"}})
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(seq, _geom(_StubCore()), settings, [])
    assert header is None
    assert "save directory" in reason


def test_build_session_header_rejects_multi_position() -> None:
    seq = _save_seq(stage_positions=[(0, 0), (1, 1)])
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(seq, _geom(_StubCore()), settings, [])
    assert header is None
    assert "multi-position" in reason


def test_build_session_header_rejects_uncropped_multi_camera() -> None:
    """Without spectral cropping, multi-camera has no channel-axis resolution."""
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(
        _save_seq(),
        _geom(_StubCore(num_channels=2, physical_cameras=["Camera-1", "Camera-2"])),
        settings,
        [],
    )
    assert header is None
    assert "multi-camera" in reason


def test_build_session_header_requires_save_name() -> None:
    seq = useq.MDASequence(channels=["488nm"])  # pyright: ignore[reportArgumentType]
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(seq, _geom(_StubCore()), settings, [])
    assert header is None
    assert "experiment name" in reason


def test_build_session_header_requires_gpfs_scratch_root() -> None:
    settings = ArgusStreamSettingsV1()  # gpfs_scratch_root unset
    header, reason = _build_session_header(
        _save_seq(), _geom(_StubCore()), settings, []
    )
    assert header is None
    assert "gpfs_scratch_root" in reason


def test_build_session_header_contains_every_field_the_receiver_requires() -> None:
    """Contract test against `opym.stream.receiver.StreamReceiver.
    _handle_session_start` (a DIFFERENT repo, on Argus -- not importable
    here, so this hardcodes the required-key list rather than a live
    schema check). This is exactly the class of bug found on the rig: an
    earlier header shape had `output_dir` instead of `raw_root`, which the
    receiver's `header["raw_root"]` lookup raised `KeyError` on, silently
    rejecting the whole session (every frame dropped as unknown). If this
    test starts failing, opym_local's receiver.py's own required-key set
    has likely changed and this client needs updating to match -- check
    `~/projects/opym_local/src/opym/stream/receiver.py` on Argus, not just
    this repo, before "fixing" it here.
    """
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/mmfs2/scratch/lab")
    header, reason = _build_session_header(
        _save_seq(), _geom(_StubCore()), settings, []
    )
    assert reason == ""
    assert header is not None
    required = {
        "raw_root",
        "base_name",
        "channels",
        "channel_names",
        "dtype",
        "shape_zyx",
        "num_timepoints",
        "z_step_um",
    }
    assert required <= header.keys()


def test_build_session_header_asks_for_live_qc() -> None:
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/mmfs2/scratch/lab")
    header, _ = _build_session_header(_save_seq(), _geom(_StubCore()), settings, [])
    assert header is not None
    assert header["accepts"] == ["qc"]


# ----------------------------------------------------------------------------
# spectral cropping + multi-camera resolution
# ----------------------------------------------------------------------------


def test_active_spectral_channels_disabled_returns_empty() -> None:
    seq = _save_seq()
    spectral = _spectral_settings(enabled=False)
    assert _active_spectral_channels(seq, ["Camera"], spectral) == []


def test_active_spectral_channels_resolves_dual_camera() -> None:
    seq = _save_seq(channels=[{"config": ALL_LASERS, "group": LASER_GROUP}])
    spectral = _spectral_settings()
    active = _active_spectral_channels(seq, ["Camera-1", "Camera-2"], spectral)
    assert [c.name for c in active] == [
        "GFP_488",
        "CalceinViolet_405",
        "mScarlet_561",
        "CF647_638",
    ]


def test_build_session_header_spectral_allows_multi_camera() -> None:
    """Spectral cropping resolves camera identity into the channel axis, so
    multi-camera acquisitions are eligible here (unlike the raw path)."""
    seq = _save_seq(channels=[{"config": ALL_LASERS, "group": LASER_GROUP}])
    argus_settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    mmcore = _StubCore(num_channels=2, physical_cameras=["Camera-1", "Camera-2"])
    active = _spectral_channels()

    header, reason = _build_session_header(seq, _geom(mmcore), argus_settings, active)
    assert reason == ""
    assert header is not None
    assert header["channels"] == [0, 1, 2, 3]
    assert header["channel_names"] == [
        "GFP_488",
        "CalceinViolet_405",
        "mScarlet_561",
        "CF647_638",
    ]
    # shape_zyx's Y/X come from the (shared) region rect, not the full chip
    assert header["shape_zyx"][1:] == [2, 4]  # rect h=2, w=4


# ----------------------------------------------------------------------------
# ArgusStreamSession against a fake in-process ZMQ ROUTER
# ----------------------------------------------------------------------------


class _FakeReceiver:
    """A minimal stand-in for opym-receive's ROUTER side.

    Records every message it gets and lets a test control exactly when (and
    whether) it ACKs, so reconnect/backlog behavior can be driven precisely
    without a real network or the real receiver process.
    """

    def __init__(self) -> None:
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.ROUTER)
        self.port = self.sock.bind_to_random_port("tcp://127.0.0.1")
        self.messages: list[tuple[bytes, str, dict, bytes | None]] = []
        # The link identity each message came from, parallel to messages.
        self.identities: list[bytes] = []
        self.auto_ack = True
        # When set, ACKs carry server_time_s = time.time() + this skew, like
        # the real receiver (whose clock the client must correct for).
        self.clock_skew_s: float | None = None
        # Advertised in every ACK when set (the real receiver sends ["slabs"]).
        self.features: list[str] | None = None
        # Answer SESSION_END with the final ACK ({"ended": true}).
        self.confirm_end = True
        self._stop = threading.Event()
        self._identity: bytes | None = None
        self._session_id: str | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        poller = zmq.Poller()
        poller.register(self.sock, zmq.POLLIN)
        while not self._stop.is_set():
            events = dict(poller.poll(timeout=100))
            if self.sock not in events:
                continue
            identity, *rest = self.sock.recv_multipart()
            msg_type, session_id, header, payload = unpack_message(rest)
            self._identity = identity
            self._session_id = session_id
            self.identities.append(identity)
            self.messages.append((msg_type, session_id, header, payload))
            if not self.auto_ack:
                continue
            if msg_type == MSG_SESSION_END and self.confirm_end:
                # The real receiver's final ACK.
                self.send(MSG_ACK, {"through_frame_index": -1, "ended": True})
            if msg_type in (MSG_SESSION_START, MSG_FRAME, MSG_RESUME):
                frame_indices = (
                    h["frame_index"] for t, _s, h, _p in self.messages if t == MSG_FRAME
                )
                self.ack(max(frame_indices, default=-1))

    def ack(self, through_frame_index: int) -> None:
        assert self._identity is not None and self._session_id is not None
        header: AckHeader = {"through_frame_index": through_frame_index}
        if self.clock_skew_s is not None:
            header["server_time_s"] = time.time() + self.clock_skew_s
        if self.features is not None:
            header["features"] = self.features
        self.sock.send_multipart(
            [self._identity, *pack_message(MSG_ACK, self._session_id, header)]
        )

    def send(self, msg_type: bytes, header: dict) -> None:
        assert self._identity is not None and self._session_id is not None
        self.sock.send_multipart(
            [self._identity, *pack_message(msg_type, self._session_id, header)]
        )

    def frame_messages(self) -> list[tuple[dict, bytes | None]]:
        return [(h, p) for t, _s, h, p in self.messages if t == MSG_FRAME]

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self.sock.close(linger=0)


@pytest.fixture
def fake_receiver() -> Iterator[_FakeReceiver]:
    receiver = _FakeReceiver()
    yield receiver
    receiver.close()


class _NoopTunnel:
    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def _full_settings(
    argus: ArgusStreamSettingsV1, spectral: SpectralChannelSettingsV1 | None = None
) -> SettingsV1:
    spectral = spectral or SpectralChannelSettingsV1(enabled=False)
    return SettingsV1(argus_stream=argus, spectral=spectral)


def _feed_sequence(session: ArgusStreamSession, seq: useq.MDASequence) -> None:
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    for event in seq:
        frame = np.zeros((4, 4), dtype="uint16")
        session.frameReady(frame, event, {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)


def test_session_streams_all_volumes_and_ends_session(
    fake_receiver: _FakeReceiver,
) -> None:
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    states: list[StreamState] = []
    session = ArgusStreamSession(
        _StubCore(),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
        on_state_changed=lambda s, _d: states.append(s),
    )

    seq = _save_seq(z_plan=useq.ZRangeAround(range=1, step=1))  # 2 z steps
    _feed_sequence(session, seq)

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        types = [m[0] for m in fake_receiver.messages]
        if MSG_SESSION_END in types:
            break
        time.sleep(0.05)

    types = [m[0] for m in fake_receiver.messages]
    assert types[0] == MSG_SESSION_START
    assert types[-1] == MSG_SESSION_END
    frames = fake_receiver.frame_messages()
    assert {(h["t"], h["c"]) for h, _p in frames} == {(0, 0), (0, 1)}
    assert StreamState.STREAMING in states
    session.shutdown(timeout=1)


def test_session_skips_ineligible_run_without_touching_network(
    fake_receiver: _FakeReceiver,
) -> None:
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    states: list[StreamState] = []
    session = ArgusStreamSession(
        # uncropped multi-camera -> ineligible
        _StubCore(num_channels=2, physical_cameras=["Camera-1", "Camera-2"]),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
        on_state_changed=lambda s, _d: states.append(s),
    )
    _feed_sequence(session, _save_seq())
    time.sleep(0.2)
    assert fake_receiver.messages == []
    assert StreamState.SKIPPED in states


def test_session_spectral_crops_and_routes_dual_camera(
    fake_receiver: _FakeReceiver,
) -> None:
    """A raw frame from each physical camera gets cropped into its lit
    regions and streamed as independent channels -- matching exactly what
    ``SpectralChannelHandler`` would save locally for the same run.
    """
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        ),
        _spectral_settings(),
    )
    mmcore = _StubCore(num_channels=2, physical_cameras=["Camera-1", "Camera-2"])
    session = ArgusStreamSession(
        mmcore,
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )

    seq = _save_seq(
        channels=[{"config": ALL_LASERS, "group": LASER_GROUP}], z_plan=None
    )
    event = next(iter(seq))

    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    # The stock multi-camera engine yields one frame per physical camera for
    # every event, sharing the same event.index -- see MultiCameraHandler's
    # module docstring for the real pattern this mirrors.
    frame_cam1 = np.arange(16, dtype="uint16").reshape(4, 4)
    frame_cam2 = frame_cam1 + 100
    session.frameReady(frame_cam1, event, {"camera_device": "Camera-1"})  # pyright: ignore[reportArgumentType]
    session.frameReady(frame_cam2, event, {"camera_device": "Camera-2"})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if any(m[0] == MSG_SESSION_END for m in fake_receiver.messages):
            break
        time.sleep(0.05)

    frames = fake_receiver.frame_messages()
    assert {h["c"] for h, _p in frames} == {0, 1, 2, 3}
    by_channel = {h["c"]: p for h, p in frames}
    # GFP_488 (c=0) is Camera-1's top half; CF647_638 (c=3) is Camera-2's
    # bottom half -- confirm actual crop content, not just message count.
    assert by_channel[0] == frame_cam1[0:2, 0:4].tobytes()
    assert by_channel[3] == frame_cam2[2:4, 0:4].tobytes()
    session.shutdown(timeout=1)


def test_session_resends_unacked_frames_after_resume(
    fake_receiver: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A receiver that goes silent, then replies to RESUME with -1, gets
    every unacked frame resent -- exercising the fix for the resend-on-resume
    step the wire protocol's worked example requires.
    """
    monkeypatch.setattr(session_mod, "_STALE_ACK_S", 0.2)
    fake_receiver.auto_ack = False  # go silent immediately

    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True,
            local_port=fake_receiver.port,
            gpfs_scratch_root="/scratch",
            buffer_budget_mb=4096,
        )
    )
    session = ArgusStreamSession(
        _StubCore(),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )
    seq = _save_seq(z_plan=None, channels=["488nm"])

    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    event = next(iter(seq))
    session.frameReady(np.zeros((4, 4), dtype="uint16"), event, {})  # pyright: ignore[reportArgumentType]

    # wait for the client to notice staleness and send RESUME
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if any(m[0] == MSG_RESUME for m in fake_receiver.messages):
            break
        time.sleep(0.05)
    assert any(m[0] == MSG_RESUME for m in fake_receiver.messages)

    # simulate "receiver forgot everything" and let the client resend
    fake_receiver.ack(-1)

    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if len(fake_receiver.frame_messages()) >= 2:
            break
        time.sleep(0.05)

    frames = fake_receiver.frame_messages()
    assert len(frames) == 2  # original send + resend after RESUME's ACK
    assert frames[0][0]["frame_index"] == frames[1][0]["frame_index"] == 0

    session.sequenceCanceled(seq)
    session.shutdown(timeout=1)


def test_session_hands_qc_verdicts_to_the_callback_and_keeps_streaming(
    fake_receiver: _FakeReceiver,
) -> None:
    """MSG_QC from Argus reaches on_qc; a consumer that raises can't stall
    the send/ACK loop."""
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    got: list[dict] = []

    def on_qc(header: dict) -> None:
        got.append(header)
        if len(got) == 1:
            raise RuntimeError("consumer bug")

    session = ArgusStreamSession(
        _StubCore(),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
        on_qc=on_qc,  # pyright: ignore[reportArgumentType]
    )
    seq = _save_seq(z_plan=useq.ZRangeAround(range=1, step=1))
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not fake_receiver.messages:
        time.sleep(0.02)
    assert fake_receiver.messages[0][0] == MSG_SESSION_START
    assert fake_receiver.messages[0][2]["accepts"] == ["qc"]
    for t, verdict in ((0, "act"), (1, "ok")):
        fake_receiver.send(
            MSG_QC,
            {"seq": t, "t": t, "stage": "raw", "verdict": verdict, "flags": []},
        )
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and len(got) < 2:
        time.sleep(0.02)
    assert [(h["t"], h["verdict"]) for h in got] == [(0, "act"), (1, "ok")]

    for event in seq:
        frame = np.zeros((4, 4), dtype="uint16")
        session.frameReady(frame, event, {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if MSG_SESSION_END in [m[0] for m in fake_receiver.messages]:
            break
        time.sleep(0.05)
    assert fake_receiver.messages[-1][0] == MSG_SESSION_END
    session.shutdown(timeout=1)


def test_session_no_spurious_resume_after_long_idle_before_first_volume(
    fake_receiver: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Rig 2026-09-23: nothing is sent while a z-stack acquires, so the first
    volume went out with the last ACK already long past _STALE_ACK_S and was
    instantly treated as stale -- RESUME + resend every run, even though the
    receiver ACKed promptly."""
    monkeypatch.setattr(session_mod, "_STALE_ACK_S", 0.2)
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    states: list[StreamState] = []
    session = ArgusStreamSession(
        _StubCore(),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
        on_state_changed=lambda s, _d: states.append(s),
    )
    seq = _save_seq(z_plan=None, channels=["488nm"])
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.6)  # "acquisition" idle, well past _STALE_ACK_S
    session.frameReady(np.zeros((4, 4), dtype="uint16"), next(iter(seq)), {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if any(m[0] == MSG_SESSION_END for m in fake_receiver.messages):
            break
        time.sleep(0.05)

    assert not any(m[0] == MSG_RESUME for m in fake_receiver.messages)
    assert StreamState.RECONNECTING not in states
    assert len(fake_receiver.frame_messages()) == 1
    session.shutdown(timeout=1)


@pytest.mark.parametrize("fmt", ["both", "ome-zarr", "tiff"])
def test_build_session_header_carries_output_format(fmt: str) -> None:
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch", output_format=fmt)  # pyright: ignore[reportArgumentType]
    header, reason = _build_session_header(
        _save_seq(), _geom(_StubCore()), settings, []
    )
    assert reason == ""
    assert header is not None
    assert header["output_format"] == fmt


def test_output_format_defaults_to_both_and_rejects_unknown() -> None:
    assert ArgusStreamSettingsV1().output_format == "both"
    with pytest.raises(ValueError):
        ArgusStreamSettingsV1(output_format="png")  # pyright: ignore[reportArgumentType]


def test_frames_carry_trace_timestamps_and_the_measured_clock_offset(
    fake_receiver: _FakeReceiver,
) -> None:
    """opym-live-trace on Argus needs each volume's acquisition and send
    times, plus how far this machine's clock is from Argus's (estimated from
    the SESSION_START -> first ACK round trip)."""
    fake_receiver.clock_skew_s = 1000.0
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    session = ArgusStreamSession(
        _StubCore(),  # pyright: ignore[reportArgumentType]
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )
    seq = _save_seq(z_plan=useq.ZRangeAround(range=1, step=1))
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.3)  # the first ACK lands before any volume completes
    for event in seq:
        session.frameReady(np.zeros((4, 4), dtype="uint16"), event, {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if any(m[0] == MSG_SESSION_END for m in fake_receiver.messages):
            break
        time.sleep(0.05)

    frames = fake_receiver.frame_messages()
    assert len(frames) == 2
    for header, _payload in frames:
        assert (
            header["acq_first_s"]
            <= header["acq_last_s"]
            <= header["queued_s"]
            <= header["sent_s"]
        )
        assert header["clock_offset_s"] == pytest.approx(1000.0, abs=0.5)
    session.shutdown(timeout=1)


def _wait_for_session_end(receiver: _FakeReceiver) -> None:
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if any(m[0] == MSG_SESSION_END for m in receiver.messages):
            return
        time.sleep(0.05)


def test_volumes_stream_as_slabs_once_argus_advertises_them(
    fake_receiver: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake_receiver.features = ["slabs"]
    monkeypatch.setattr(session_mod, "_SLAB_TARGET_BYTES", 1)  # 1 plane/slab
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    session = ArgusStreamSession(
        _StubCore(),  # pyright: ignore[reportArgumentType]
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )
    seq = _save_seq(z_plan=useq.ZRangeAround(range=1, step=1), channels=["488nm"])
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.3)  # the first ACK (with features) lands before any plane
    for event in seq:
        frame = np.full((4, 4), event.index["z"] + 1, dtype="uint16")
        session.frameReady(frame, event, {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)
    _wait_for_session_end(fake_receiver)

    frames = fake_receiver.frame_messages()
    assert [(h["z0"], h["nz"], h["shape_zyx"]) for h, _p in frames] == [
        (0, 2, [1, 4, 4]),
        (1, 2, [1, 4, 4]),
    ]
    assert [np.frombuffer(p or b"", "uint16")[0] for _h, p in frames] == [1, 2]
    assert [h["frame_index"] for h, _p in frames] == [0, 1]
    session.shutdown(timeout=1)


def test_whole_volumes_without_the_slabs_feature(fake_receiver: _FakeReceiver) -> None:
    """An older receiver never advertises slabs, so it only ever gets whole
    volumes -- no z0 field it wouldn't understand."""
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True, local_port=fake_receiver.port, gpfs_scratch_root="/scratch"
        )
    )
    session = ArgusStreamSession(
        _StubCore(),  # pyright: ignore[reportArgumentType]
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )
    seq = _save_seq(z_plan=useq.ZRangeAround(range=1, step=1), channels=["488nm"])
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.3)
    _feed_frames(session, seq)
    _wait_for_session_end(fake_receiver)
    [(header, _payload)] = fake_receiver.frame_messages()
    assert "z0" not in header and header["shape_zyx"] == [2, 4, 4]
    session.shutdown(timeout=1)


def _feed_frames(session: ArgusStreamSession, seq: useq.MDASequence) -> None:
    for event in seq:
        session.frameReady(np.zeros((4, 4), dtype="uint16"), event, {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)


def test_runs_go_direct_when_argus_answers_there() -> None:
    direct, tunnel = _FakeReceiver(), _FakeReceiver()
    try:
        settings = _full_settings(
            ArgusStreamSettingsV1(
                enabled=True,
                local_port=tunnel.port,
                direct_endpoint=f"tcp://127.0.0.1:{direct.port}",
                gpfs_scratch_root="/scratch",
            )
        )
        states: list[tuple[StreamState, str]] = []
        session = ArgusStreamSession(
            _StubCore(),  # pyright: ignore[reportArgumentType]
            _NoopTunnel(),  # pyright: ignore[reportArgumentType]
            get_settings=lambda: settings,
            on_state_changed=lambda s, d: states.append((s, d)),
        )
        seq = _save_seq(z_plan=None, channels=["488nm"])
        session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
        _feed_frames(session, seq)
        _wait_for_session_end(direct)
        assert next(m[0] for m in direct.messages) == MSG_SESSION_START
        assert len(direct.frame_messages()) == 1
        assert tunnel.messages == []
        assert (StreamState.STREAMING, "direct") in states
        session.shutdown(timeout=1)
    finally:
        direct.close()
        tunnel.close()


def test_runs_fall_back_to_the_tunnel_when_direct_is_unreachable(
    fake_receiver: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(session_mod, "_DIRECT_HANDSHAKE_S", 0.3)
    dead = zmq.Context.instance().socket(zmq.ROUTER)
    dead_port = dead.bind_to_random_port("tcp://127.0.0.1")
    dead.close(linger=0)  # nothing listens there any more
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True,
            local_port=fake_receiver.port,
            direct_endpoint=f"tcp://127.0.0.1:{dead_port}",
            gpfs_scratch_root="/scratch",
        )
    )
    states: list[tuple[StreamState, str]] = []
    session = ArgusStreamSession(
        _StubCore(),  # pyright: ignore[reportArgumentType]
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
        on_state_changed=lambda s, d: states.append((s, d)),
    )
    seq = _save_seq(z_plan=None, channels=["488nm"])
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    _feed_frames(session, seq)
    _wait_for_session_end(fake_receiver)
    types = [m[0] for m in fake_receiver.messages]
    assert types[0] == MSG_SESSION_START and types[-1] == MSG_SESSION_END
    assert len(fake_receiver.frame_messages()) == 1
    assert (StreamState.STREAMING, "SSH tunnel") in states
    session.shutdown(timeout=1)


def _stream_one_volume(
    receiver: _FakeReceiver, compress: bool = True, direct: bool = False
) -> list[tuple[dict, bytes | None]]:
    endpoint = f"tcp://127.0.0.1:{receiver.port}"
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True,
            local_port=1 if direct else receiver.port,
            direct_endpoint=endpoint if direct else "",
            compress_over_tunnel=compress,
            gpfs_scratch_root="/scratch",
        )
    )
    session = ArgusStreamSession(
        _StubCore(),  # pyright: ignore[reportArgumentType]
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )
    seq = _save_seq(z_plan=useq.ZRangeAround(range=1, step=1), channels=["488nm"])
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.3)  # the first ACK (with features) lands before any plane
    for event in seq:
        frame = np.full((4, 4), 100 + event.index["z"], dtype="uint16")
        session.frameReady(frame, event, {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)
    _wait_for_session_end(receiver)
    session.shutdown(timeout=1)
    return receiver.frame_messages()


def test_frames_go_compressed_through_the_tunnel_once_argus_accepts_them(
    fake_receiver: _FakeReceiver,
) -> None:
    from numcodecs import blosc

    fake_receiver.features = ["blosc"]
    [(header, payload)] = _stream_one_volume(fake_receiver)
    assert header["codec"] == "blosc" and header["shape_zyx"] == [2, 4, 4]
    vol = np.frombuffer(blosc.decompress(payload or b""), "uint16").reshape(2, 4, 4)
    assert vol[0, 0, 0] == 100 and vol[1, 0, 0] == 101


def test_frames_go_raw_unless_argus_accepts_compression(
    fake_receiver: _FakeReceiver,
) -> None:
    [(header, payload)] = _stream_one_volume(fake_receiver)
    assert "codec" not in header and len(payload or b"") == 2 * 4 * 4 * 2


def test_compression_can_be_turned_off(fake_receiver: _FakeReceiver) -> None:
    fake_receiver.features = ["blosc"]
    [(header, _p)] = _stream_one_volume(fake_receiver, compress=False)
    assert "codec" not in header


def test_the_direct_link_always_gets_raw_frames() -> None:
    """One thread compresses slower than 10 GbE moves raw bytes."""
    direct = _FakeReceiver()
    try:
        direct.features = ["blosc"]
        [(header, _p)] = _stream_one_volume(direct, direct=True)
        assert "codec" not in header
    finally:
        direct.close()


# ----------------------------------------------------------------------------
# Links: one run over several connections (one SSH tunnel each)
# ----------------------------------------------------------------------------


class _LinkProxy:
    """A TCP forwarder standing in for one SSH tunnel.

    A test can cut it (connections reset, port closed) or silence it
    (connections stay open, nothing flows): the two ways a tunnel fails.
    """

    def __init__(self, listener: socket.socket, target_port: int) -> None:
        self.listener = listener
        self.target_port = target_port
        self.silenced = threading.Event()
        self._conns: list[socket.socket] = []
        self._stop = threading.Event()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self) -> None:
        self.listener.settimeout(0.05)
        while not self._stop.is_set():
            try:
                client, _ = self.listener.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            upstream = socket.create_connection(("127.0.0.1", self.target_port))
            self._conns += [client, upstream]
            for src, dst in ((client, upstream), (upstream, client)):
                threading.Thread(
                    target=self._pipe, args=(src, dst), daemon=True
                ).start()

    def _pipe(self, src: socket.socket, dst: socket.socket) -> None:
        try:
            while not self._stop.is_set():
                data = src.recv(1 << 16)
                if not data:
                    break
                while self.silenced.is_set() and not self._stop.is_set():
                    time.sleep(0.01)
                dst.sendall(data)
        except OSError:
            pass
        for sock in (src, dst):
            with contextlib.suppress(OSError):
                sock.shutdown(socket.SHUT_RDWR)

    def cut(self) -> None:
        self._stop.set()
        self.listener.close()
        for sock in self._conns:
            with contextlib.suppress(OSError):
                sock.shutdown(socket.SHUT_RDWR)
            sock.close()


@pytest.fixture
def link_proxies(
    fake_receiver: _FakeReceiver,
) -> Iterator[tuple[int, list[_LinkProxy]]]:
    """Three proxies on consecutive ports in front of the fake receiver."""
    for _ in range(100):
        base = random.randint(20000, 60000)
        listeners: list[socket.socket] = []
        try:
            for k in range(3):
                sock = socket.socket()
                sock.bind(("127.0.0.1", base + k))
                sock.listen()
                listeners.append(sock)
        except OSError:
            for sock in listeners:
                sock.close()
            continue
        proxies = [_LinkProxy(sock, fake_receiver.port) for sock in listeners]
        yield base, proxies
        for proxy in proxies:
            proxy.cut()
        return
    raise RuntimeError("no three consecutive free ports")


def _links_session(
    local_port: int, links: int = 3, **settings_kw: object
) -> ArgusStreamSession:
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True,
            local_port=local_port,
            stream_links=links,
            gpfs_scratch_root="/scratch",
            **settings_kw,  # pyright: ignore[reportArgumentType]
        )
    )
    return ArgusStreamSession(
        _StubCore(),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
    )


def _timelapse(n: int) -> useq.MDASequence:
    return _save_seq(
        z_plan=None, channels=["488nm"], time_plan={"interval": 0, "loops": n}
    )


def _wait(predicate: Callable[[], bool], timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _frames_by_link(receiver: _FakeReceiver) -> list[tuple[int, bytes]]:
    return [
        (h["frame_index"], ident)
        for (t, _s, h, _p), ident in zip(
            receiver.messages, receiver.identities, strict=False
        )
        if t == MSG_FRAME
    ]


def test_links_open_once_argus_offers_them_and_share_the_volumes(
    fake_receiver: _FakeReceiver, link_proxies: tuple[int, list[_LinkProxy]]
) -> None:
    fake_receiver.features = ["links"]
    base, _proxies = link_proxies
    session = _links_session(base)
    seq = _timelapse(12)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.5)  # the first ACK opens links 1 and 2
    _feed_frames(session, seq)
    _wait_for_session_end(fake_receiver)

    sent = _frames_by_link(fake_receiver)
    assert {fi for fi, _ in sent} == set(range(12))
    sid = fake_receiver.messages[0][1]
    assert {ident for _, ident in sent} == {
        sid.encode(),
        f"{sid}#1".encode(),
        f"{sid}#2".encode(),
    }
    assert fake_receiver.messages[-1][0] == MSG_SESSION_END
    session.shutdown(timeout=1)


def test_an_older_receiver_without_links_gets_one_connection(
    fake_receiver: _FakeReceiver, link_proxies: tuple[int, list[_LinkProxy]]
) -> None:
    fake_receiver.features = ["slabs"]
    base, _proxies = link_proxies
    session = _links_session(base)
    seq = _timelapse(6)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.3)
    _feed_frames(session, seq)
    _wait_for_session_end(fake_receiver)
    sid = fake_receiver.messages[0][1].encode()
    assert set(fake_receiver.identities) == {sid}
    session.shutdown(timeout=1)


def _stream_then_break_link_1(
    receiver: _FakeReceiver,
    base: int,
    break_link: Callable[[], None],
    break_first: bool,
) -> tuple[ArgusStreamSession, useq.MDASequence]:
    receiver.features = ["links"]
    session = _links_session(base)
    seq = _timelapse(12)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    time.sleep(0.5)  # the first ACK opens links 1 and 2
    receiver.auto_ack = False  # hold every volume unACKed
    if break_first:
        break_link()
    for event in seq:
        session.frameReady(np.zeros((4, 4), dtype="uint16"), event, {})  # pyright: ignore[reportArgumentType]
    if not break_first:
        assert _wait(lambda: len(_frames_by_link(receiver)) >= 12)
        break_link()
    return session, seq


def _assert_link_1_volumes_resent_elsewhere(
    receiver: _FakeReceiver, link_1_delivered: bool
) -> None:
    """Every volume arrives on links 0 and 2. A cut link delivered its
    volumes before it broke; a silenced one never delivers them at all."""
    sid = receiver.messages[0][1]
    link1 = f"{sid}#1".encode()

    def resent() -> bool:
        sent = _frames_by_link(receiver)
        on_link1 = {fi for fi, ident in sent if ident == link1}
        elsewhere = {fi for fi, ident in sent if ident != link1}
        if link_1_delivered and not on_link1:
            return False
        return elsewhere >= set(range(12))

    assert _wait(resent, timeout=4)
    # Resent because the link dropped, not by the slow stale-ACK backstop.
    assert not any(m[0] == MSG_RESUME for m in receiver.messages)


def test_a_cut_link_s_volumes_are_resent_on_the_others_at_once(
    fake_receiver: _FakeReceiver, link_proxies: tuple[int, list[_LinkProxy]]
) -> None:
    base, proxies = link_proxies
    session, seq = _stream_then_break_link_1(
        fake_receiver, base, proxies[1].cut, break_first=False
    )
    _assert_link_1_volumes_resent_elsewhere(fake_receiver, link_1_delivered=True)
    fake_receiver.auto_ack = True
    fake_receiver.ack(11)
    session.sequenceFinished(seq)
    _wait_for_session_end(fake_receiver)
    assert fake_receiver.messages[-1][0] == MSG_SESSION_END
    session.shutdown(timeout=1)


def test_a_silent_link_is_found_by_heartbeat_and_its_volumes_resent(
    fake_receiver: _FakeReceiver,
    link_proxies: tuple[int, list[_LinkProxy]],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The nastiest failure: the tunnel's TCP connection stays open but
    nothing flows (its ssh still looks alive)."""
    monkeypatch.setattr(session_mod, "_HEARTBEAT_IVL_MS", 100)
    monkeypatch.setattr(session_mod, "_HEARTBEAT_TIMEOUT_MS", 500)
    monkeypatch.setattr(session_mod, "_HEARTBEAT_TTL_MS", 1000)
    base, proxies = link_proxies
    session, seq = _stream_then_break_link_1(
        fake_receiver, base, proxies[1].silenced.set, break_first=True
    )
    _assert_link_1_volumes_resent_elsewhere(fake_receiver, link_1_delivered=False)
    sid = fake_receiver.messages[0][1]
    assert f"{sid}#1".encode() not in {i for _, i in _frames_by_link(fake_receiver)}
    fake_receiver.auto_ack = True
    fake_receiver.ack(11)
    session.sequenceFinished(seq)
    _wait_for_session_end(fake_receiver)
    session.shutdown(timeout=1)


def test_an_unknown_session_is_resumed_where_argus_last_acked(
    fake_receiver: _FakeReceiver,
) -> None:
    """Argus restarted mid-run: SESSION_START again, with resume_through,
    then everything unACKed."""
    fake_receiver.features = ["resume"]
    session = _links_session(fake_receiver.port, links=1)
    seq = _timelapse(2)
    events = list(seq)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    session.frameReady(np.zeros((4, 4), dtype="uint16"), events[0], {})  # pyright: ignore[reportArgumentType]
    assert _wait(lambda: len(fake_receiver.frame_messages()) == 1)
    time.sleep(0.2)  # frame 0 ACKed
    fake_receiver.auto_ack = False
    session.frameReady(np.zeros((4, 4), dtype="uint16"), events[1], {})  # pyright: ignore[reportArgumentType]
    assert _wait(lambda: len(fake_receiver.frame_messages()) == 2)

    fake_receiver.send(
        MSG_ACK,
        {"through_frame_index": -1, "unknown_session": True, "features": ["resume"]},
    )

    def starts():
        return [h for t, _s, h, _p in fake_receiver.messages if t == MSG_SESSION_START]

    assert _wait(lambda: len(starts()) == 2)
    assert starts()[1]["resume_through"] == 0
    assert starts()[1]["base_name"] == starts()[0]["base_name"]

    fake_receiver.ack(0)  # the resumed session's first ACK
    resent = lambda: [h["frame_index"] for h, _p in fake_receiver.frame_messages()]  # noqa: E731
    assert _wait(lambda: resent() == [0, 1, 1])
    fake_receiver.auto_ack = True
    fake_receiver.ack(1)
    session.sequenceFinished(seq)
    _wait_for_session_end(fake_receiver)
    session.shutdown(timeout=1)


def test_the_run_pauses_at_the_hard_cap_without_touching_acquisition(
    fake_receiver: _FakeReceiver,
) -> None:
    fake_receiver.auto_ack = False  # Argus never confirms anything
    states: list[StreamState] = []
    settings = _full_settings(
        ArgusStreamSettingsV1(
            enabled=True,
            local_port=fake_receiver.port,
            gpfs_scratch_root="/scratch",
            buffer_hard_cap_mb=1,
        )
    )
    session = ArgusStreamSession(
        _StubCore(),
        _NoopTunnel(),  # pyright: ignore[reportArgumentType]
        get_settings=lambda: settings,
        on_state_changed=lambda s, _d: states.append(s),
    )
    seq = _timelapse(3)
    events = list(seq)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    big = np.zeros((1024, 1024), dtype="uint16")  # 2 MiB > the 1 MiB cap
    session.frameReady(big, events[0], {})  # pyright: ignore[reportArgumentType]

    def ended() -> list[dict]:
        return [h for t, _s, h, _p in fake_receiver.messages if t == MSG_SESSION_END]

    assert _wait(lambda: bool(ended()))
    assert ended()[0]["reason"] == "paused"
    assert StreamState.PAUSED in states
    # Later frames aren't even assembled; the run's sender is gone.
    session.frameReady(big, events[1], {})  # pyright: ignore[reportArgumentType]
    session.sequenceFinished(seq)
    time.sleep(0.3)
    assert len(fake_receiver.frame_messages()) <= 1
    assert states[-1] == StreamState.PAUSED
    session.shutdown(timeout=1)


def test_the_hard_cap_is_the_setting_or_a_quarter_of_the_ram() -> None:
    assert session_mod.hard_cap_bytes(5) == 5 * 1024 * 1024
    ram = session_mod._physical_ram_bytes()
    assert ram > 0
    assert session_mod.hard_cap_bytes(0) == ram // 4


def test_session_end_is_resent_until_argus_confirms_it(
    fake_receiver: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A SESSION_END lost with its link would leave Argus waiting out its
    idle timeout (holding the GPUs), so it goes again until confirmed."""
    monkeypatch.setattr(session_mod, "_END_RETRY_S", 0.2)
    fake_receiver.features = ["resume"]
    fake_receiver.confirm_end = False
    session = _links_session(fake_receiver.port, links=1)
    seq = _timelapse(1)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    _feed_frames(session, seq)

    def ends() -> int:
        return sum(m[0] == MSG_SESSION_END for m in fake_receiver.messages)

    assert _wait(lambda: ends() >= 2)
    fake_receiver.send(MSG_ACK, {"through_frame_index": 0, "ended": True})
    worker = session._all_workers[0]
    worker.join(timeout=3)
    assert not worker.is_alive()
    session.shutdown(timeout=1)


def test_a_plain_ack_after_session_end_is_not_its_confirmation(
    fake_receiver: _FakeReceiver, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(session_mod, "_END_RETRY_S", 0.2)
    fake_receiver.features = ["resume"]
    fake_receiver.confirm_end = False
    session = _links_session(fake_receiver.port, links=1)
    seq = _timelapse(1)
    session.sequenceStarted(seq, {})  # pyright: ignore[reportArgumentType]
    _feed_frames(session, seq)
    assert _wait(lambda: any(m[0] == MSG_SESSION_END for m in fake_receiver.messages))
    fake_receiver.ack(0)  # a stray periodic ACK
    time.sleep(0.5)
    assert session._all_workers[0].is_alive()
    fake_receiver.send(MSG_ACK, {"through_frame_index": -1, "unknown_session": True})
    session._all_workers[0].join(timeout=3)
    assert not session._all_workers[0].is_alive()
    session.shutdown(timeout=1)
