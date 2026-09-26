"""Stream acquired volumes to the Argus real-time deskew/decon receiver.

:class:`ArgusStreamSession` connects directly to ``core.mda.events`` -- the
same raw signal :class:`~pymmcore_gui._ndv_viewers.NDVViewersManager` uses --
rather than being passed through ``GuiMDAWidget.execute_mda``'s ``output=``
handler composition. That keeps it off the shared ``MDARelayThread`` the
local disk writers run on, so a struggling network can never delay a local
save (and vice versa): see the plan's "Key architecture findings" for the
verified code paths this relies on.

v1 scope: single-position MDA sequences only (Argus's wire protocol has no
position axis -- see ``_protocol.py``). Multi-camera sequences are streamed
*only* when spectral-channel cropping is active and resolves each camera's
regions into distinct channel identities -- exactly what
:class:`~pymmcore_gui._spectral_channel_handler.SpectralChannelHandler`
already does for local saves, reused here directly
(:func:`~pymmcore_gui._spectral_channel_handler.active_channels_for_event`)
so the two paths can never disagree about which pixels belong to which
channel. An uncropped multi-camera acquisition has no such resolution and is
skipped, same as multi-position. Anything skipped is reported via a logged
reason and a status callback, and still saves locally exactly as if
streaming were disabled.

Transport: when ``ArgusStreamSettingsV1.direct_endpoint`` is set, each run
first connects straight to Argus over 10 GbE. If Argus hasn't answered
SESSION_START within ``_DIRECT_HANDSHAKE_S``, the run falls back to the SSH
tunnel, which stays up for exactly that. One tunnel tops out around
33 MB/s, the largest delay in the live view (2026-09-25).

Links: that cap is per SSH connection (sshd allows 2 MB per ~60 ms round
trip), not the network's. So once Argus advertises ``"links"``, each run
sends over ``stream_links`` connections at once -- through the tunnel, one
SSH forward each -- and resends whatever a dropped link was carrying on the
others straight away (see :class:`_RunWorker`).

Slabs: once Argus's first ACK advertises ``"slabs"``, each volume is sent as
runs of about ``_SLAB_TARGET_BYTES`` of consecutive planes while it is still
being acquired, instead of in one piece after its last plane.

Live QC: every session asks for ``MSG_QC`` (``accepts: ["qc"]``). Argus's
verdict on each timepoint -- is the cell cut off by a face of the volume,
drifting out, defocused, bleaching, and what to change -- is handed to the
``on_qc`` callback, from the sender thread (marshal it before touching Qt).
"""

from __future__ import annotations

import logging
import os
import posixpath
import queue
import sys
import threading
import time
from collections import deque
from enum import Enum
from pathlib import PureWindowsPath
from typing import TYPE_CHECKING, Any, ClassVar, Literal, NamedTuple, cast
from uuid import uuid4

import numpy as np
import zmq
from pymmcore_widgets.useq_widgets import PYMMCW_METADATA_KEY
from zmq.utils.monitor import recv_monitor_message

from pymmcore_gui._multi_camera_handler import physical_camera_labels, without_cam_index
from pymmcore_gui._spectral_channel_handler import (
    _strip_known_suffix,
    active_channels_for_event,
    channels_for_sequence,
)
from pymmcore_gui._vendored.mda_handlers._util import position_sizes

from ._protocol import (
    MSG_ACK,
    MSG_FRAME,
    MSG_QC,
    MSG_RESUME,
    MSG_SESSION_END,
    MSG_SESSION_START,
    FrameHeader,
    QCHeader,
    SessionStartHeader,
    pack_message,
    unpack_message,
)
from ._tunnel import link_endpoints
from ._volume_assembler import Volume, VolumeAssembler

if TYPE_CHECKING:
    from collections.abc import Callable

    import useq
    from pymmcore_plus import CMMCorePlus
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1

    from pymmcore_gui._settings import (
        ArgusStreamSettingsV1,
        SettingsV1,
        SpectralChannelConfig,
        SpectralChannelSettingsV1,
    )

    from ._tunnel import ArgusTunnelManager

logger = logging.getLogger(__name__)

_STALE_ACK_S = 5.0
# How long a run waits for Argus to answer SESSION_START over the direct
# endpoint before falling back to the SSH tunnel.
_DIRECT_HANDSHAKE_S = 3.0
# Slab size: big enough that per-message overhead is negligible, small
# enough that the last slab of a stack leaves right after its last plane.
_SLAB_TARGET_BYTES = 16 * 1024 * 1024
# SESSION_END must actually leave before the socket closes: with LINGER=0 it
# was dropped, and both 100-timepoint runs on 2026-09-25 ended only by the
# receiver's 10-minute idle timeout (holding its GPU lease all that time).
_SESSION_END_LINGER_MS = 2000
# The SESSION_START -> first ACK round trip gives the clock offset for the
# FRAME trace fields; a reply slower than this is too blurry to use.
_MAX_OFFSET_RTT_S = 2.0
# ~1/3 of the measured ~60 MB/s tunnel throughput (2026-09-21).
_MIN_LINK_BYTES_PER_S = 20 * 1024 * 1024
_POLL_TIMEOUT_MS = 200
# Per link: a message or two queued in ZMQ is enough to keep a tunnel busy,
# and the next one goes to whichever link has room.
_LINK_SNDHWM = 2
# End-to-end liveness through the tunnel: a link whose path went silent is
# dropped (and its volumes resent elsewhere) within the timeout, even while
# its ssh process still looks alive.
# A link's queue (SNDHWM plus ssh's window) drains in well under a second
# at ~60 MB/s, so a PING that sees no reply in 5 s means a dead path, not a
# busy one. (A black-holed tunnel held its volumes back 10 s at 10 s.)
_HEARTBEAT_IVL_MS = 1000
_HEARTBEAT_TIMEOUT_MS = 5000
_HEARTBEAT_TTL_MS = 10000
_RECONNECT_IVL_MS = 250
_RECONNECT_IVL_MAX_MS = 5000
_RATE_LOG_S = 30.0
_END_RETRY_S = 2.0
_END_TRIES = 5
# If the RAM can't be read; the setting's 0 otherwise means a quarter of it.
_FALLBACK_HARD_CAP_BYTES = 8 * 1024**3
_DTYPE_BY_BYTES_PER_PIXEL = {1: "uint8", 2: "uint16", 4: "uint32"}
_SessionEndReason = Literal["complete", "idle_timeout", "client_abort", "paused"]


class StreamState(str, Enum):
    """Coarse status of the Argus stream, surfaced to the GUI status label."""

    DISABLED = "disabled"
    IDLE = "idle"
    SKIPPED = "skipped"
    CONNECTING = "connecting"
    STREAMING = "streaming"
    RECONNECTING = "reconnecting"
    BACKLOG_ALARM = "backlog_alarm"
    FINISHING = "finishing"
    PAUSED = "paused"


class CameraGeometry(NamedTuple):
    """Camera identity and image format for one MDA run."""

    labels: list[str]
    dtype: str
    height: int
    width: int
    pixel_size_um: float


def _camera_geometry(meta: SummaryMetaV1, mmcore: CMMCorePlus) -> CameraGeometry:
    """Read camera geometry from the run's summary metadata.

    Prefers ``meta["image_infos"]``, which the engine builds in
    ``setup_sequence`` while the cameras are still loaded. The live core is
    not reliable here: ``sequenceStarted`` handlers run concurrently with the
    MDA thread, and the ASI engines unload every camera into worker processes
    on their first ``exec_event`` -- so by the time this runs, the core can
    already report no camera at all (confirmed on the rig 2026-09-23:
    ``camera=''``, 0x0, 0 bytes/pixel). Falls back to the core only when the
    metadata has no image info.

    When a persistent ``CameraWorkerService`` is active, its cached geometry
    wins outright: Camera-1/Camera-2 are then never loaded on *mmcore*, so
    both the summary metadata's image infos and the core itself describe a
    cameraless core.
    """
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

    if (svc := CameraWorkerService.get_active()) is not None and (
        snap := svc.geometry
    ) is not None:
        infos = meta.get("image_infos") or ()
        pixel_size = float(infos[0].get("pixel_size_um") or 0.0) if infos else 0.0
        return CameraGeometry(
            labels=list(snap.camera_labels),
            dtype=snap.dtype_str,
            height=snap.image_height,
            width=snap.image_width,
            pixel_size_um=pixel_size or float(mmcore.getPixelSizeUm() or 0.0),
        )

    infos = meta.get("image_infos") or ()
    if infos:
        first = infos[0]
        if first.get("num_camera_adapter_channels", 1) > 1:
            labels = [info["camera_label"] for info in infos[1:]]
        else:
            labels = [first["camera_label"]]
        return CameraGeometry(
            labels=labels,
            dtype=first["dtype"],
            height=int(first["height"]),
            width=int(first["width"]),
            pixel_size_um=float(first.get("pixel_size_um") or 0.0),
        )
    return CameraGeometry(
        labels=physical_camera_labels(mmcore),
        dtype=_DTYPE_BY_BYTES_PER_PIXEL.get(mmcore.getBytesPerPixel(), "unknown"),
        height=int(mmcore.getImageHeight()),
        width=int(mmcore.getImageWidth()),
        pixel_size_um=float(mmcore.getPixelSizeUm() or 0.0),
    )


def _active_spectral_channels(
    sequence: useq.MDASequence,
    camera_labels: list[str],
    spectral: SpectralChannelSettingsV1,
) -> list[SpectralChannelConfig]:
    """Return the spectral-channel regions this run will crop, if any.

    Empty when the feature is disabled or no configured region matches this
    sequence's lasers -- callers should treat that as "not in spectral
    mode" and fall back to the raw-frame path. Reuses
    :func:`~pymmcore_gui._spectral_channel_handler.channels_for_sequence`
    unchanged, the same function :class:`GuiMDAWidget` uses to decide what
    :class:`SpectralChannelHandler` will save locally.
    """
    if not spectral.enabled:
        return []
    return channels_for_sequence(
        sequence,
        spectral.channels,
        spectral.laser_config_group,
        spectral.all_lasers_preset,
        set(camera_labels),
    )


def _build_session_header(
    sequence: useq.MDASequence,
    geometry: CameraGeometry,
    settings: ArgusStreamSettingsV1,
    active_spectral: list[SpectralChannelConfig],
) -> tuple[SessionStartHeader | None, str]:
    """Build the SESSION_START header, or return ``(None, reason)`` if ineligible.

    Parameters
    ----------
    sequence : useq.MDASequence
        The MDA sequence about to run.
    geometry : CameraGeometry
        This run's camera identity and image format, from
        :func:`_camera_geometry`.
    settings : ArgusStreamSettingsV1
        Argus-streaming configuration.
    active_spectral : list[SpectralChannelConfig]
        The spectral-channel regions this run will crop into, as returned by
        :func:`_active_spectral_channels`. When non-empty, each region
        becomes one channel slot in the header (and multi-camera is allowed,
        since each region already carries its own camera identity). When
        empty, this falls back to the raw, uncropped-frame path, which
        requires a single camera (there's no per-camera channel resolution
        without cropping).
    """
    if len(sequence.stage_positions) > 1:
        return None, "multi-position sequences are not supported in v1"
    if not active_spectral and len(geometry.labels) > 1:
        return None, (
            "multi-camera acquisitions require spectral-channel cropping to "
            "resolve camera identity into the channel axis"
        )
    if geometry.dtype not in _DTYPE_BY_BYTES_PER_PIXEL.values():
        return None, f"unsupported camera pixel type for streaming: {geometry.dtype}"

    meta = sequence.metadata.get(PYMMCW_METADATA_KEY, {})
    save_name = meta.get("save_name")
    save_dir = meta.get("save_dir")
    if not save_name:
        return None, "no experiment name set in the MDA save widget"
    if not save_dir:
        return None, "no save directory set in the MDA save widget"
    if not settings.gpfs_scratch_root:
        return None, "ArgusStreamSettingsV1.gpfs_scratch_root is not configured"

    base_name = _strip_known_suffix(str(save_name))
    # Mirror the local save directory's structure under gpfs_scratch_root,
    # stripping only the drive letter -- e.g. local "S:/2026.../Bead_PSF"
    # streams to "<gpfs_scratch_root>/2026.../Bead_PSF" (see
    # ArgusStreamSettingsV1.gpfs_scratch_root's docstring). Each channel's
    # raw store is then named "<base_name>_<channel_name>.ome.zarr" directly
    # under THAT (see opym.stream.rawmirror.store_path_for_channel on
    # Argus) -- not a base_name subdirectory of it; that nesting is unique
    # to decon_stage/, written server-side, not by this client.
    local_dir = PureWindowsPath(str(save_dir))
    session_parts = local_dir.relative_to(local_dir.anchor).parts
    raw_root = posixpath.join(settings.gpfs_scratch_root, *session_parts)

    if active_spectral:
        channel_names = [c.name for c in active_spectral]
        # All regions share one size (enforced by the spectral-channel config
        # UI -- see SpectralChannelConfig's docstring), so any one's rect
        # gives the cropped Y/X shape every FRAME in this session will use.
        # channels_for_sequence only ever returns regions with a drawn rect.
        rect = active_spectral[0].rect
        assert rect is not None
        _x, _y, crop_w, crop_h = rect
        yx_shape = (crop_h, crop_w)
    else:
        channel_names = [ch.config for ch in sequence.channels] or ["Default"]
        yx_shape = (geometry.height, geometry.width)
    channels = list(range(len(channel_names)))

    # Decon parameters (PSF, iterations, rl_method, ...) are NOT part of
    # this handshake -- the receiver resolves them entirely server-side
    # from OPYM_DECON_PSF, same as a batch/Globus-landed dataset. See
    # SessionStartHeader's docstring.
    z_step_um = getattr(sequence.z_plan, "step", None) or 0.0
    t_interval = getattr(sequence.time_plan, "interval", None)
    t_interval_s = t_interval.total_seconds() if t_interval is not None else 0.0

    pos_sizes = position_sizes(sequence)[0]

    header: SessionStartHeader = {
        "base_name": base_name,
        "raw_root": raw_root,
        "dtype": geometry.dtype,
        "shape_zyx": [pos_sizes.get("z", 1), *yx_shape],
        "num_timepoints": pos_sizes.get("t", 1),
        "channels": channels,
        "channel_names": channel_names,
        "z_step_um": float(z_step_um),
        "xy_pixel_size": geometry.pixel_size_um,
        "t_interval_s": float(t_interval_s),
        "output_format": settings.output_format,
        "accepts": ["qc"],
    }
    return header, ""


def _compress(payload: bytes, dtype: str) -> bytes:
    """Compress one FRAME payload into a blosc frame (``codec: "blosc"``).

    lz4 + bitshuffle over the pixel type: ~2.5x on camera frames.
    """
    from numcodecs import blosc  # pyright: ignore[reportAttributeAccessIssue]

    return cast(
        "bytes",
        blosc.compress(
            np.frombuffer(payload, dtype=dtype), b"lz4", 5, blosc.BITSHUFFLE
        ),
    )


class _BufferBudget:
    """RAM the senders of every run hold, against one hard cap.

    Summed across runs: a run that finished during an outage keeps its
    unsent volumes while the next run starts buffering its own.
    """

    def __init__(self, cap_bytes: int) -> None:
        self.cap_bytes = cap_bytes
        self._lock = threading.Lock()
        self._held: dict[int, int] = {}

    def update(self, owner: object, nbytes: int) -> int:
        """Record what ``owner`` holds now; return the total across owners."""
        with self._lock:
            if nbytes:
                self._held[id(owner)] = nbytes
            else:
                self._held.pop(id(owner), None)
            return sum(self._held.values())


def _physical_ram_bytes() -> int:
    """Total physical RAM, or 0 if it can't be read."""
    if sys.platform == "win32":
        import ctypes

        class _MemoryStatus(ctypes.Structure):
            _fields_: ClassVar = [
                ("dwLength", ctypes.c_ulong),
                ("dwMemoryLoad", ctypes.c_ulong),
                ("ullTotalPhys", ctypes.c_ulonglong),
                ("ullAvailPhys", ctypes.c_ulonglong),
                ("ullTotalPageFile", ctypes.c_ulonglong),
                ("ullAvailPageFile", ctypes.c_ulonglong),
                ("ullTotalVirtual", ctypes.c_ulonglong),
                ("ullAvailVirtual", ctypes.c_ulonglong),
                ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
            ]

        status = _MemoryStatus()
        status.dwLength = ctypes.sizeof(status)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return int(status.ullTotalPhys)
        return 0
    try:
        return int(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES"))
    except (ValueError, OSError, AttributeError):
        return 0


def hard_cap_bytes(setting_mb: int) -> int:
    """The send buffer's hard cap: the setting, or a quarter of the RAM if 0."""
    if setting_mb > 0:
        return setting_mb * 1024 * 1024
    ram = _physical_ram_bytes()
    return ram // 4 if ram else _FALLBACK_HARD_CAP_BYTES


class _Link:
    """One connection of a run: a DEALER socket and its connection monitor.

    ``up`` follows the monitor. It turns true once the ZMTP handshake with
    the receiver completes, which through a tunnel proves the whole path.
    It turns false when the connection drops, including a heartbeat timeout
    on a path that went silent while its ``ssh`` still looks alive.
    """

    def __init__(self, index: int, endpoint: str, identity: str) -> None:
        self.index = index
        self.endpoint = endpoint
        self.up = False
        self.sent_bytes = 0
        sock = zmq.Context.instance().socket(zmq.DEALER)
        sock.setsockopt(zmq.IDENTITY, identity.encode("utf-8"))
        sock.setsockopt(zmq.SNDHWM, _LINK_SNDHWM)
        sock.setsockopt(zmq.LINGER, 0)
        # Link 0 queues SESSION_START while its tunnel is still coming up;
        # the others queue nothing while disconnected.
        sock.setsockopt(zmq.IMMEDIATE, 1 if index else 0)
        sock.setsockopt(zmq.HEARTBEAT_IVL, _HEARTBEAT_IVL_MS)
        sock.setsockopt(zmq.HEARTBEAT_TIMEOUT, _HEARTBEAT_TIMEOUT_MS)
        sock.setsockopt(zmq.HEARTBEAT_TTL, _HEARTBEAT_TTL_MS)
        sock.setsockopt(zmq.RECONNECT_IVL, _RECONNECT_IVL_MS)
        sock.setsockopt(zmq.RECONNECT_IVL_MAX, _RECONNECT_IVL_MAX_MS)
        self.monitor = sock.get_monitor_socket(
            zmq.EVENT_HANDSHAKE_SUCCEEDED | zmq.EVENT_DISCONNECTED
        )
        sock.connect(endpoint)
        self.sock = sock

    def poll_events(self) -> tuple[bool, bool]:
        """Apply queued monitor events; return (came up, went down)."""
        came_up = went_down = False
        while self.monitor.poll(0):
            event = recv_monitor_message(self.monitor)["event"]
            if event == zmq.EVENT_HANDSHAKE_SUCCEEDED:
                self.up, came_up = True, True
            elif event == zmq.EVENT_DISCONNECTED:
                self.up, went_down = False, True
        return came_up, went_down

    def close(self, linger_ms: int = 0) -> None:
        self.sock.disable_monitor()
        self.monitor.close(linger=0)
        self.sock.close(linger=linger_ms)


class _RunWorker(threading.Thread):
    """Owns one MDA run's links and the send/ACK loop.

    All socket I/O and ``unacked`` bookkeeping happens on this thread only.
    :meth:`submit` is the sole entry point called from the acquisition
    thread; it is a cheap, non-blocking queue append.

    Links: link 0 sends SESSION_START. Once an ACK advertises ``"links"``,
    links 1..N-1 open too, and every volume or slab goes out on whichever
    link is up and has room (``_LINK_SNDHWM``), so faster links carry more.
    When a link drops, what was last sent on it and is still unACKed is
    resent on the others at once. The stale-ACK RESUME is only a backstop.

    Resume: if Argus answers ``unknown_session`` (the receiver restarted,
    or timed the session out), SESSION_START is sent again with
    ``resume_through`` and then everything unACKed.

    Pause: if what every run holds unsent passes the hard cap, this run
    stops streaming for good. Its buffer is dropped, SESSION_END "paused"
    tells Argus not to keep its partial copy, and the run goes to Argus by
    Globus instead. Acquisition and local saving never wait on any of this.
    """

    def __init__(
        self,
        session_id: str,
        endpoints: list[str],
        header: SessionStartHeader,
        buffer_budget_bytes: int,
        on_state: Callable[[StreamState, str], None],
        on_qc: Callable[[QCHeader], None] | None = None,
        direct_endpoint: str = "",
        compress_over_tunnel: bool = False,
        budget: _BufferBudget | None = None,
    ) -> None:
        super().__init__(name="ArgusStreamWorker", daemon=True)
        self._session_id = session_id
        self._endpoints = endpoints or ["tcp://127.0.0.1:5555"]
        self._direct_endpoint = direct_endpoint
        self._compress_over_tunnel = compress_over_tunnel
        # Set once an ACK advertises "slabs": the acquisition thread then
        # starts new volumes in slab mode (see ArgusStreamSession).
        self.slabs_enabled = threading.Event()
        # Set when this run stops streaming (see "Pause" above): the
        # acquisition thread then stops assembling volumes for it.
        self.paused = threading.Event()
        # What happened on the wire, logged when the run ends (and read by
        # the replay tool): bytes per link, what went out more than once,
        # and every recovery.
        self.stats: dict[str, Any] = {
            "sent_bytes": [],
            "resent_bytes": 0,
            "link_drops": 0,
            "requeued": 0,
            "resumes": 0,
            "resyncs": 0,
        }
        self._header = header
        self._buffer_budget_bytes = buffer_budget_bytes
        self._budget = budget
        self._on_state = on_state
        self._on_qc = on_qc
        self._last_state: tuple[StreamState, str] | None = None

        self._to_send: queue.Queue[tuple[int, FrameHeader, bytes]] = queue.Queue()
        self._next_frame_index = 0
        self._finish_event = threading.Event()
        self._abort_event = threading.Event()
        self._finish_reason: _SessionEndReason = "complete"

    def submit(self, volume: Volume) -> int:
        """Queue one completed volume (or slab) for sending.

        Returns its ``frame_index``.
        """
        frame_index = self._next_frame_index
        self._next_frame_index += 1
        header: FrameHeader = {
            "t": volume.t,
            "c": volume.c,
            "frame_index": frame_index,
            "timestamp": volume.timestamp,
            "camera_id": volume.camera_id,
            "shape_zyx": list(volume.array.shape),
            "dtype": str(volume.array.dtype),
            "acq_first_s": volume.acq_first_s,
            "acq_last_s": volume.acq_last_s,
            "queued_s": time.time(),
        }
        if volume.nz is not None:
            header["z0"] = volume.z0
            header["nz"] = volume.nz
        payload = volume.array.tobytes()
        self._to_send.put((frame_index, header, payload))
        return frame_index

    def finish(self, reason: _SessionEndReason = "complete") -> None:
        """Request a graceful stop: drain and fully ACK before SESSION_END."""
        if not self.paused.is_set():
            self._finish_reason = reason
        self._finish_event.set()

    def abort(self) -> None:
        """Request an immediate stop, discarding anything still unacked."""
        self._abort_event.set()
        self._finish_event.set()

    def _emit(self, state: StreamState, detail: str) -> None:
        if (state, detail) != self._last_state:
            self._last_state = (state, detail)
            self._on_state(state, detail)

    def _start_session(self) -> tuple[_Link, str, float]:
        """Open link 0 and send SESSION_START, directly if Argus answers.

        Falls back to the SSH tunnel otherwise. Returns link 0, a label for
        the route taken, and when SESSION_START was sent (for the
        clock-offset round trip). A direct answer is left unread for the main
        loop, which also takes the clock offset and features from it.
        """
        start = pack_message(MSG_SESSION_START, self._session_id, self._header)
        if self._direct_endpoint:
            link = _Link(0, self._direct_endpoint, self._session_id)
            sent_s = time.time()
            link.sock.send_multipart(start)
            if link.sock.poll(int(_DIRECT_HANDSHAKE_S * 1000)):
                return link, "direct", sent_s
            link.close()
            logger.warning(
                "Argus direct endpoint %s did not answer within %.0f s; "
                "streaming this run through the SSH tunnel",
                self._direct_endpoint,
                _DIRECT_HANDSHAKE_S,
            )
        link = _Link(0, self._endpoints[0], self._session_id)
        sent_s = time.time()
        link.sock.send_multipart(start)
        return link, "SSH tunnel", sent_s

    def run(self) -> None:
        self._emit(StreamState.CONNECTING, "")
        sid = self._session_id
        link0, route, start_sent_s = self._start_session()
        n_links = len(self._endpoints)
        endpoints = (
            [self._direct_endpoint] * n_links if route == "direct" else self._endpoints
        )
        links = [link0]
        self._emit(StreamState.STREAMING, route)

        unacked: dict[int, tuple[FrameHeader, bytes]] = {}
        unacked_bytes = 0
        # Frame indices to send next, in order; resends go to the front.
        pending: deque[int] = deque()
        queued: set[int] = set()
        sent_on: dict[int, int] = {}  # frame_index -> link it last went on
        acked_through = -1
        features: set[str] = set()
        # Compress (here, on this thread) once Argus accepts it, and only
        # through the tunnel -- see ArgusStreamSettingsV1.compress_over_tunnel.
        may_compress = self._compress_over_tunnel and route == "SSH tunnel"
        last_ack_time = time.monotonic()
        # A RESUME (stale-ACK backstop) or a resync SESSION_START is
        # awaiting its ACK; that ACK makes everything unACKed go again.
        resume_sent = False
        resync_sent = False
        # Argus clock minus ours, from the SESSION_START -> first ACK round
        # trip; sent with every FRAME once known (see _protocol.py).
        clock_offset: float | None = None
        next_link = 0
        rate_mark = (time.monotonic(), [0] * n_links)
        ever_sent: set[int] = set()
        end_link: _Link | None = None
        # SESSION_END is confirmed by the receiver's final ACK; until then it
        # is resent every _END_RETRY_S (on the next link), _END_TRIES times.
        end_sent_at: float | None = None
        end_tries = 0
        end_confirmed = False

        def up_links() -> list[_Link]:
            return [link for link in links if link.up]

        def send(parts: list[bytes]) -> _Link | None:
            """Hand ``parts`` to the next link that is up and has room."""
            nonlocal next_link
            up = up_links()
            for i in range(len(up)):
                link = up[(next_link + i) % len(up)]
                try:
                    link.sock.send_multipart(parts, flags=zmq.NOBLOCK, copy=False)
                except zmq.Again:
                    continue
                next_link = (next_link + i + 1) % len(up)
                return link
            return None

        def requeue(frame_indices: list[int]) -> None:
            for fi in sorted(frame_indices, reverse=True):
                sent_on.pop(fi, None)
                if fi in unacked and fi not in queued:
                    pending.appendleft(fi)
                    queued.add(fi)

        def compress(fi: int) -> None:
            nonlocal unacked_bytes
            header, payload = unacked[fi]
            if may_compress and "blosc" in features and "codec" not in header:
                packed = _compress(payload, header["dtype"])
                header["codec"] = "blosc"
                unacked[fi] = (header, packed)
                unacked_bytes += len(packed) - len(payload)

        def dispatch() -> bool:
            """Send pending frames until every link is full; True if any went."""
            sent_any = False
            while pending and not resync_sent:
                fi = pending[0]
                if fi not in unacked:
                    pending.popleft()
                    queued.discard(fi)
                    continue
                compress(fi)
                header, payload = unacked[fi]
                header["sent_s"] = time.time()
                if clock_offset is not None:
                    header["clock_offset_s"] = clock_offset
                link = send(pack_message(MSG_FRAME, sid, header, payload))
                if link is None:
                    break
                pending.popleft()
                queued.discard(fi)
                sent_on[fi] = link.index
                link.sent_bytes += len(payload)
                if fi in ever_sent:
                    self.stats["resent_bytes"] += len(payload)
                ever_sent.add(fi)
                sent_any = True
            return sent_any

        def on_ack(ack: dict[str, object]) -> None:
            nonlocal acked_through, unacked_bytes, last_ack_time, clock_offset
            nonlocal resume_sent, resync_sent
            features.update(cast("list[str]", ack.get("features") or ()))
            if "slabs" in features:
                self.slabs_enabled.set()
            if ack.get("unknown_session"):
                # Argus lost this session (restart, or idle timeout): start
                # it again where it stood, then resend what's unACKed.
                if "resume" in features and not resync_sent:
                    start = dict(self._header)
                    if acked_through >= 0:
                        start["resume_through"] = acked_through
                    if send(pack_message(MSG_SESSION_START, sid, start)):  # type: ignore[arg-type]
                        resync_sent = True
                        self.stats["resyncs"] += 1
                        logger.warning(
                            "Argus no longer knows this run's session; "
                            "resuming it after frame %d",
                            acked_through,
                        )
                return
            server_time = ack.get("server_time_s")
            if clock_offset is None and server_time is not None:
                now = time.time()
                if now - start_sent_s <= _MAX_OFFSET_RTT_S:
                    clock_offset = cast("float", server_time) - (
                        (start_sent_s + now) / 2
                    )
            through = cast("int", ack.get("through_frame_index", -1))
            acked_through = max(acked_through, through)
            for fi in [fi for fi in unacked if fi <= through]:
                _, payload = unacked.pop(fi)
                unacked_bytes -= len(payload)
                sent_on.pop(fi, None)
            last_ack_time = time.monotonic()
            if resume_sent or resync_sent:
                # The reply to a RESUME or resync: per protocol, resend
                # everything still unACKed, in frame_index order -- the
                # receiver dedupes, so resending staged data is a no-op.
                requeue(list(unacked))
                resume_sent = resync_sent = False
            if "links" in features and len(links) < n_links:
                links.extend(
                    _Link(k, endpoints[k], f"{sid}#{k}")
                    for k in range(len(links), n_links)
                )
                logger.info("Argus stream: %d links over the %s", n_links, route)

        def pause(total: int) -> None:
            nonlocal unacked_bytes
            self.paused.set()
            self._finish_reason = "paused"
            self._finish_event.set()
            unacked.clear()
            pending.clear()
            queued.clear()
            sent_on.clear()
            unacked_bytes = 0
            while True:
                try:
                    self._to_send.get_nowait()
                except queue.Empty:
                    break
            logger.error(
                "Argus stream PAUSED for this run: %d MiB could not be sent "
                "(cap %d MiB). Acquisition and the local save continue; send "
                "this run to Argus by Globus.",
                total // 2**20,
                (self._budget.cap_bytes if self._budget else 0) // 2**20,
            )

        try:
            while True:
                if self._abort_event.is_set():
                    break
                now = time.monotonic()

                # Intake: everything the acquisition thread queued.
                while True:
                    try:
                        frame_index, header, payload = self._to_send.get_nowait()
                    except queue.Empty:
                        break
                    if self.paused.is_set():
                        continue
                    if not unacked:
                        # The ACK clock only runs while something is in
                        # flight -- otherwise a volume sent after a long idle
                        # (e.g. a whole z-stack's acquisition) is instantly
                        # "stale" and triggers a spurious RESUME + resend.
                        last_ack_time = now
                    unacked[frame_index] = (header, payload)
                    unacked_bytes += len(payload)
                    compress(frame_index)
                    pending.append(frame_index)
                    queued.add(frame_index)

                for link in links:
                    came_up, went_down = link.poll_events()
                    if went_down:
                        lost = [fi for fi, k in sent_on.items() if k == link.index]
                        requeue(lost)
                        self.stats["link_drops"] += 1
                        self.stats["requeued"] += len(lost)
                        logger.warning(
                            "Argus link %d (%s) dropped; resending its %d "
                            "unACKed volume(s) on the others",
                            link.index,
                            link.endpoint,
                            len(lost),
                        )
                    if came_up:
                        last_ack_time = now if unacked else last_ack_time
                        logger.info("Argus link %d up (%s)", link.index, link.endpoint)

                ending = self._finish_event.is_set() and not unacked and not pending
                if end_confirmed:
                    break
                if ending and self._to_send.empty():
                    due = end_sent_at is None or now - end_sent_at >= _END_RETRY_S
                    if due and end_tries >= _END_TRIES:
                        logger.warning(
                            "Argus never confirmed this run's SESSION_END; it "
                            "will close the session itself after its idle timeout"
                        )
                        break
                    if due:
                        carrier = send(
                            pack_message(
                                MSG_SESSION_END, sid, {"reason": self._finish_reason}
                            )
                        )
                        if carrier is not None:
                            end_link, end_sent_at = carrier, now
                            end_tries += 1

                drained_any = dispatch()

                poller = zmq.Poller()
                for link in links:
                    flags = zmq.POLLIN
                    if link.up and (pending or ending) and not resync_sent:
                        flags |= zmq.POLLOUT
                    poller.register(link.sock, flags)
                    poller.register(link.monitor, zmq.POLLIN)
                events = dict(
                    poller.poll(timeout=0 if drained_any else _POLL_TIMEOUT_MS)
                )
                for link in links:
                    if not events.get(link.sock, 0) & zmq.POLLIN:
                        continue
                    while link.sock.poll(0):
                        msg_type, _sid, reply, _payload = unpack_message(
                            link.sock.recv_multipart()
                        )
                        if msg_type == MSG_ACK and end_sent_at is not None:
                            # The final ACK ("ended"), or "unknown" (it had
                            # already closed the session), confirms
                            # SESSION_END. A receiver without "resume" marks
                            # neither, so any ACK has to do.
                            end_confirmed = bool(
                                reply.get("ended")
                                or reply.get("unknown_session")
                                or "resume" not in features
                            )
                        elif msg_type == MSG_ACK:
                            on_ack(reply)
                        elif msg_type == MSG_QC and self._on_qc is not None:
                            # Advisory; a failing consumer must never stall
                            # the send/ACK loop.
                            try:
                                self._on_qc(cast("QCHeader", reply))
                            except Exception:
                                logger.exception("Argus QC callback failed")

                up = up_links()
                if not up:
                    # Nothing is in flight while every link is down: that's
                    # handled by resending on reconnect, not by staleness.
                    last_ack_time = time.monotonic()
                # A single volume can be hundreds of MB; allow time to
                # transfer what's outstanding before calling the link stale.
                stale_after_s = _STALE_ACK_S + unacked_bytes / _MIN_LINK_BYTES_PER_S
                stale = time.monotonic() - last_ack_time > stale_after_s
                if stale and unacked and not (resume_sent or resync_sent):
                    resume_sent = send(pack_message(MSG_RESUME, sid, {})) is not None
                    self.stats["resumes"] += resume_sent

                total = unacked_bytes
                if self._budget is not None:
                    total = self._budget.update(self, unacked_bytes)
                    if (
                        not self._finish_event.is_set()
                        and total > self._budget.cap_bytes
                    ):
                        pause(total)
                        self._budget.update(self, 0)
                        self._emit(
                            StreamState.PAUSED,
                            "link down too long; local save is complete, "
                            "send this run by Globus",
                        )
                        continue

                label = route if n_links == 1 else f"{route} x{len(up)}/{n_links}"
                if self.paused.is_set():
                    pass
                elif unacked_bytes > self._buffer_budget_bytes:
                    self._emit(
                        StreamState.BACKLOG_ALARM,
                        f"{unacked_bytes // (1024 * 1024)} MiB unacked",
                    )
                elif (stale or not up) and unacked:
                    self._emit(StreamState.RECONNECTING, label)
                elif self._finish_event.is_set():
                    self._emit(StreamState.FINISHING, f"{len(unacked)} volumes unacked")
                else:
                    self._emit(StreamState.STREAMING, label)

                t_mark, bytes_mark = rate_mark
                if n_links > 1 and time.monotonic() - t_mark >= _RATE_LOG_S:
                    dt = time.monotonic() - t_mark
                    sent_now = [link.sent_bytes for link in links]
                    rates = [
                        (b - (bytes_mark[i] if i < len(bytes_mark) else 0)) / dt / 1e6
                        for i, b in enumerate(sent_now)
                    ]
                    if any(rates):
                        logger.info(
                            "Argus links MB/s: %s",
                            " ".join(f"{r:.0f}" for r in rates),
                        )
                    rate_mark = (time.monotonic(), sent_now)

            if self._abort_event.is_set() and end_link is None:
                reason: _SessionEndReason = (
                    "paused" if self.paused.is_set() else "client_abort"
                )
                end_link = send(pack_message(MSG_SESSION_END, sid, {"reason": reason}))
        finally:
            self.stats["sent_bytes"] = [link.sent_bytes for link in links]
            logger.info(
                "Argus run over %s: sent %s MB by link, %.0f MB of it resent; "
                "%d link drop(s) (%d volumes requeued), %d RESUME(s), "
                "%d resync(s)",
                route,
                [round(b / 1e6) for b in self.stats["sent_bytes"]],
                self.stats["resent_bytes"] / 1e6,
                self.stats["link_drops"],
                self.stats["requeued"],
                self.stats["resumes"],
                self.stats["resyncs"],
            )
            for link in links:
                link.close(_SESSION_END_LINGER_MS if link is end_link else 0)
            if self._budget is not None:
                self._budget.update(self, 0)
            if not self.paused.is_set():
                self._on_state(StreamState.IDLE, "")


class ArgusStreamSession:
    """Long-lived object mediating between one MDA run and the Argus stream.

    Connect this instance's ``sequenceStarted``/``frameReady``/
    ``sequenceFinished``/``sequenceCanceled`` methods directly to
    ``core.mda.events`` (see module docstring for why it must bypass the
    shared disk-writer relay thread).
    """

    def __init__(
        self,
        mmcore: CMMCorePlus,
        tunnel: ArgusTunnelManager,
        get_settings: Callable[[], SettingsV1],
        on_state_changed: Callable[[StreamState, str], None] | None = None,
        on_qc: Callable[[QCHeader], None] | None = None,
    ) -> None:
        self._mmc = mmcore
        self._tunnel = tunnel
        self._get_settings = get_settings
        self._on_state_changed = on_state_changed
        self._on_qc = on_qc
        self._assembler = VolumeAssembler()
        # What every run's sender holds unsent, against one hard cap.
        self._budget = _BufferBudget(_FALLBACK_HARD_CAP_BYTES)
        # The worker for the run currently in progress -- frameReady routes
        # to this one. Cleared at sequenceFinished/sequenceCanceled, but the
        # worker itself may still be alive draining its unacked buffer in
        # the background at that point, so it stays in _all_workers (below)
        # until it actually exits -- shutdown() must be able to reach it
        # regardless of whether a newer run has since replaced _worker.
        self._worker: _RunWorker | None = None
        self._all_workers: list[_RunWorker] = []
        # Spectral-cropping state for the run currently in progress, empty
        # when this run isn't in spectral mode -- see sequenceStarted.
        self._active_spectral: list[SpectralChannelConfig] = []
        self._spectral_index: dict[str, int] = {}
        self._laser_group = ""
        self._all_lasers_preset = ""

    def _emit_state(self, state: StreamState, detail: str) -> None:
        if self._on_state_changed is not None:
            self._on_state_changed(state, detail)

    def sequenceStarted(self, sequence: useq.MDASequence, meta: SummaryMetaV1) -> None:
        """Start streaming this run, if eligible and enabled."""
        settings = self._get_settings()
        argus = settings.argus_stream
        if not argus.enabled:
            self._emit_state(StreamState.DISABLED, "")
            self._worker = None
            return

        geometry = _camera_geometry(meta, self._mmc)
        active_spectral = _active_spectral_channels(
            sequence, geometry.labels, settings.spectral
        )
        header, reason = _build_session_header(
            sequence, geometry, argus, active_spectral
        )
        if header is None:
            logger.info("Argus stream skipped for this run: %s", reason)
            self._emit_state(StreamState.SKIPPED, reason)
            self._worker = None
            return

        self._active_spectral = active_spectral
        self._spectral_index = {c.name: i for i, c in enumerate(active_spectral)}
        self._laser_group = settings.spectral.laser_config_group
        self._all_lasers_preset = settings.spectral.all_lasers_preset

        # Fallback only -- the primary start is app-launch time, in
        # _main_window.py, specifically so the SSH handshake is already
        # warm before any acquisition begins. start() is idempotent (a
        # no-op once already running), so this only does real work if
        # ArgusStreamSettingsV1.enabled was flipped True after this app
        # session launched; that first run then still pays the JIT
        # tunnel-startup cost racing _RunWorker's own connect() below.
        self._tunnel.start()
        self._assembler.reset(sequence)
        self._budget.cap_bytes = hard_cap_bytes(argus.buffer_hard_cap_mb)
        self._worker = _RunWorker(
            session_id=str(uuid4()),
            endpoints=link_endpoints(argus.local_port, argus.stream_links),
            header=header,
            buffer_budget_bytes=argus.buffer_budget_mb * 1024 * 1024,
            on_state=self._emit_state,
            on_qc=self._on_qc,
            direct_endpoint=argus.direct_endpoint,
            compress_over_tunnel=argus.compress_over_tunnel,
            budget=self._budget,
        )
        self._all_workers = [w for w in self._all_workers if w.is_alive()]
        self._all_workers.append(self._worker)
        self._worker.start()

    def _submit_if_complete(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        worker = self._worker
        if worker is None:
            return
        slab_planes = (
            max(1, _SLAB_TARGET_BYTES // max(1, frame.nbytes))
            if worker.slabs_enabled.is_set()
            else 0
        )
        for ready in self._assembler.add_plane(frame, event, meta, slab_planes):
            worker.submit(ready)

    def frameReady(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        """Assemble ``frame`` into its volume; submit to the sender when complete.

        Cheap and non-blocking: only a defensive copy into the assembler's
        buffer (see ``VolumeAssembler.add_frame``) and, at most once per
        volume, a queue append -- no network I/O happens on this thread. In
        spectral mode, crops each lit region first (mirroring
        ``SpectralChannelHandler.frameReady`` exactly) and assembles each
        region as its own channel.
        """
        if self._worker is None:
            return
        if self._worker.paused.is_set():
            # This run stopped streaming (see _RunWorker): don't assemble.
            self._assembler.clear()
            return

        if not self._active_spectral:
            self._submit_if_complete(frame, event, meta)
            return

        label = meta.get("camera_device") or self._mmc.getCameraDevice()
        clean_event = without_cam_index(event)
        lit = active_channels_for_event(
            event, self._active_spectral, self._laser_group, self._all_lasers_preset
        )
        for channel in lit:
            if channel.camera != label or channel.rect is None:
                continue
            x, y, w, h = channel.rect
            crop = np.ascontiguousarray(frame[y : y + h, x : x + w])
            idx = self._spectral_index[channel.name]
            synthetic_event = clean_event.model_copy(
                update={"index": {**clean_event.index, "c": idx}}
            )
            self._submit_if_complete(crop, synthetic_event, meta)

    def sequenceFinished(self, sequence: useq.MDASequence) -> None:
        """Signal a graceful end-of-run; does not block the acquisition thread."""
        if self._worker is not None:
            self._worker.finish("complete")
        self._worker = None

    def sequenceCanceled(self, sequence: useq.MDASequence) -> None:
        """Signal an aborted run; does not block the acquisition thread."""
        if self._worker is not None:
            self._worker.finish("client_abort")
        self._worker = None

    def shutdown(self, timeout: float = 2.0) -> None:
        """Hard-stop every worker; call from app shutdown, not per-run.

        This includes workers from already-finished runs still draining in
        the background. Waits briefly for a graceful drain, then aborts
        unconditionally so application exit is never blocked on an
        unreachable Argus receiver.
        """
        workers = [w for w in self._all_workers if w.is_alive()]
        for worker in workers:
            worker.finish("client_abort")
        deadline = time.monotonic() + timeout
        for worker in workers:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
        for worker in workers:
            if worker.is_alive():
                worker.abort()
        for worker in workers:
            worker.join(timeout=timeout)
        self._worker = None
        self._all_workers = []
