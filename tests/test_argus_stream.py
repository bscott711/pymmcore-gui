from __future__ import annotations

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
    MSG_RESUME,
    MSG_SESSION_END,
    MSG_SESSION_START,
    MessageHeader,
    pack_message,
    unpack_message,
)
from pymmcore_gui._argus_stream._session import (
    _active_spectral_channels,
    _build_session_header,
)
from pymmcore_gui._argus_stream._volume_assembler import VolumeAssembler
from pymmcore_gui._settings import (
    ArgusStreamSettingsV1,
    SettingsV1,
    SpectralChannelConfig,
    SpectralChannelSettingsV1,
)

if TYPE_CHECKING:
    from collections.abc import Iterator

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
    header, reason = _build_session_header(_save_seq(), _StubCore(), settings, [])
    assert reason == ""
    assert header is not None
    assert header["base_name"] == "cooked_001"
    assert header["output_dir"] == "/mmfs2/scratch/lab/cooked_001/Decon"
    assert header["dtype"] == "uint16"
    assert header["shape_zyx"] == [3, 6, 8]
    assert header["num_timepoints"] == 1
    assert header["channels"] == [0, 1]
    assert header["channel_names"] == ["488nm", "561nm"]
    assert header["psf_paths"] is None
    assert header["iterations"] is None
    assert header["sheet_angle_deg"] == 60.0  # validated production default


def test_build_session_header_rejects_multi_position() -> None:
    seq = _save_seq(stage_positions=[(0, 0), (1, 1)])
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(seq, _StubCore(), settings, [])
    assert header is None
    assert "multi-position" in reason


def test_build_session_header_rejects_uncropped_multi_camera() -> None:
    """Without spectral cropping, multi-camera has no channel-axis resolution."""
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(
        _save_seq(), _StubCore(num_channels=2), settings, []
    )
    assert header is None
    assert "multi-camera" in reason


def test_build_session_header_requires_save_name() -> None:
    seq = useq.MDASequence(channels=["488nm"])  # pyright: ignore[reportArgumentType]
    settings = ArgusStreamSettingsV1(gpfs_scratch_root="/scratch")
    header, reason = _build_session_header(seq, _StubCore(), settings, [])
    assert header is None
    assert "experiment name" in reason


def test_build_session_header_requires_gpfs_scratch_root() -> None:
    settings = ArgusStreamSettingsV1()  # gpfs_scratch_root unset
    header, reason = _build_session_header(_save_seq(), _StubCore(), settings, [])
    assert header is None
    assert "gpfs_scratch_root" in reason


def test_build_session_header_psf_paths_require_full_channel_coverage(
    caplog: pytest.LogCaptureFixture,
) -> None:
    settings = ArgusStreamSettingsV1(
        gpfs_scratch_root="/scratch",
        psf_paths={"488nm": "/scratch/psf/488.tif"},  # missing 561nm
        dz_psf=0.1,
        iterations=10,
    )
    header, _reason = _build_session_header(_save_seq(), _StubCore(), settings, [])
    assert header is not None
    assert header["psf_paths"] is None
    assert header["dz_psf"] is None
    assert header["iterations"] is None
    assert "not every channel" in caplog.text


def test_build_session_header_psf_paths_full_coverage_preserves_order() -> None:
    settings = ArgusStreamSettingsV1(
        gpfs_scratch_root="/scratch",
        psf_paths={"561nm": "/scratch/psf/561.tif", "488nm": "/scratch/psf/488.tif"},
        dz_psf=0.1,
        iterations=10,
    )
    header, _reason = _build_session_header(_save_seq(), _StubCore(), settings, [])
    assert header is not None
    # order follows channel_names (488nm, 561nm), not the settings dict order
    assert header["psf_paths"] == ["/scratch/psf/488.tif", "/scratch/psf/561.tif"]
    assert header["dz_psf"] == 0.1
    assert header["iterations"] == 10


# ----------------------------------------------------------------------------
# spectral cropping + multi-camera resolution
# ----------------------------------------------------------------------------


def test_active_spectral_channels_disabled_returns_empty() -> None:
    seq = _save_seq()
    spectral = _spectral_settings(enabled=False)
    assert _active_spectral_channels(seq, _StubCore(), spectral) == []


def test_active_spectral_channels_resolves_dual_camera() -> None:
    seq = _save_seq(channels=[{"config": ALL_LASERS, "group": LASER_GROUP}])
    spectral = _spectral_settings()
    mmcore = _StubCore(num_channels=2, physical_cameras=["Camera-1", "Camera-2"])
    active = _active_spectral_channels(seq, mmcore, spectral)
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

    header, reason = _build_session_header(seq, mmcore, argus_settings, active)
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
        self.auto_ack = True
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
            self.messages.append((msg_type, session_id, header, payload))
            if not self.auto_ack:
                continue
            if msg_type in (MSG_SESSION_START, MSG_FRAME, MSG_RESUME):
                frame_indices = (
                    h["frame_index"] for t, _s, h, _p in self.messages if t == MSG_FRAME
                )
                self.ack(max(frame_indices, default=-1))

    def ack(self, through_frame_index: int) -> None:
        assert self._identity is not None and self._session_id is not None
        self.sock.send_multipart(
            [
                self._identity,
                *pack_message(
                    MSG_ACK,
                    self._session_id,
                    {"through_frame_index": through_frame_index},
                ),
            ]
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
        _StubCore(num_channels=2),  # uncropped multi-camera -> ineligible
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
