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
"""

from __future__ import annotations

import logging
import posixpath
import queue
import threading
import time
from enum import Enum
from typing import TYPE_CHECKING, Literal, cast
from uuid import uuid4

import numpy as np
import zmq
from pymmcore_widgets.useq_widgets import PYMMCW_METADATA_KEY

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
    MSG_RESUME,
    MSG_SESSION_END,
    MSG_SESSION_START,
    FrameHeader,
    SessionStartHeader,
    pack_message,
    unpack_message,
)
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
_POLL_TIMEOUT_MS = 200
_DTYPE_BY_BYTES_PER_PIXEL = {1: "uint8", 2: "uint16", 4: "uint32"}
_SessionEndReason = Literal["complete", "idle_timeout", "client_abort"]


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


def _dtype_str(mmcore: CMMCorePlus) -> str:
    bpp = mmcore.getBytesPerPixel()
    try:
        return _DTYPE_BY_BYTES_PER_PIXEL[bpp]
    except KeyError:
        raise ValueError(f"Unsupported bytes-per-pixel for streaming: {bpp}") from None


def _active_spectral_channels(
    sequence: useq.MDASequence,
    mmcore: CMMCorePlus,
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
    active_cams = set(physical_camera_labels(mmcore))
    return channels_for_sequence(
        sequence,
        spectral.channels,
        spectral.laser_config_group,
        spectral.all_lasers_preset,
        active_cams,
    )


def _build_session_header(
    sequence: useq.MDASequence,
    mmcore: CMMCorePlus,
    settings: ArgusStreamSettingsV1,
    active_spectral: list[SpectralChannelConfig],
) -> tuple[SessionStartHeader | None, str]:
    """Build the SESSION_START header, or return ``(None, reason)`` if ineligible.

    Parameters
    ----------
    sequence : useq.MDASequence
        The MDA sequence about to run.
    mmcore : CMMCorePlus
        The core instance driving the acquisition.
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
    if not active_spectral and mmcore.getNumberOfCameraChannels() > 1:
        return None, (
            "multi-camera acquisitions require spectral-channel cropping to "
            "resolve camera identity into the channel axis"
        )

    meta = sequence.metadata.get(PYMMCW_METADATA_KEY, {})
    save_name = meta.get("save_name")
    if not save_name:
        return None, "no experiment name set in the MDA save widget"
    if not settings.gpfs_scratch_root:
        return None, "ArgusStreamSettingsV1.gpfs_scratch_root is not configured"

    base_name = _strip_known_suffix(str(save_name))
    output_dir = posixpath.join(settings.gpfs_scratch_root, base_name, "Decon")

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
        yx_shape = (int(mmcore.getImageHeight()), int(mmcore.getImageWidth()))
    channels = list(range(len(channel_names)))

    # Only use psf_paths if *every* channel has a configured PSF -- a partial
    # mapping would silently misalign psf_paths[i] with the wrong channel on
    # the receiver side, so fall back to deskew-only instead of guessing.
    psf_paths: list[str] | None = None
    dz_psf: float | None = None
    iterations: int | None = None
    if all(name in settings.psf_paths for name in channel_names):
        psf_paths = [settings.psf_paths[name] for name in channel_names]
        dz_psf = settings.dz_psf
        iterations = settings.iterations
    elif settings.psf_paths:
        logger.warning(
            "Argus stream: not every channel %s has a configured PSF path "
            "(psf_paths=%s) -- falling back to deskew-only for this run",
            channel_names,
            sorted(settings.psf_paths),
        )

    z_step_um = getattr(sequence.z_plan, "step", None) or 0.0
    t_interval = getattr(sequence.time_plan, "interval", None)
    t_interval_s = t_interval.total_seconds() if t_interval is not None else 0.0

    pos_sizes = position_sizes(sequence)[0]

    header: SessionStartHeader = {
        "base_name": base_name,
        "output_dir": output_dir,
        "dtype": _dtype_str(mmcore),
        "shape_zyx": [pos_sizes.get("z", 1), *yx_shape],
        "num_timepoints": pos_sizes.get("t", 1),
        "channels": channels,
        "channel_names": channel_names,
        "z_step_um": float(z_step_um),
        "xy_pixel_size": float(mmcore.getPixelSizeUm() or 0.0),
        "sheet_angle_deg": settings.sheet_angle_deg,
        "t_interval_s": float(t_interval_s),
        "interp_method": settings.interp_method,
        "rl_method": settings.rl_method,
        "iterations": iterations,
        "psf_paths": psf_paths,
        "dz_psf": dz_psf,
    }
    return header, ""


class _RunWorker(threading.Thread):
    """Owns one MDA run's ZMQ DEALER socket and the send/ACK loop.

    All socket I/O and ``_unacked`` bookkeeping happens on this thread only.
    :meth:`submit` is the sole entry point called from the acquisition
    thread; it is a cheap, non-blocking queue append.
    """

    def __init__(
        self,
        session_id: str,
        local_port: int,
        header: SessionStartHeader,
        buffer_budget_bytes: int,
        on_state: Callable[[StreamState, str], None],
    ) -> None:
        super().__init__(name="ArgusStreamWorker", daemon=True)
        self._session_id = session_id
        self._local_port = local_port
        self._header = header
        self._buffer_budget_bytes = buffer_budget_bytes
        self._on_state = on_state

        self._to_send: queue.Queue[tuple[int, FrameHeader, bytes]] = queue.Queue()
        self._next_frame_index = 0
        self._finish_event = threading.Event()
        self._abort_event = threading.Event()
        self._finish_reason: _SessionEndReason = "complete"

    def submit(self, volume: Volume) -> int:
        """Queue one completed volume for sending; returns its ``frame_index``."""
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
        }
        payload = volume.array.tobytes()
        self._to_send.put((frame_index, header, payload))
        return frame_index

    def finish(self, reason: _SessionEndReason = "complete") -> None:
        """Request a graceful stop: drain and fully ACK before SESSION_END."""
        self._finish_reason = reason
        self._finish_event.set()

    def abort(self) -> None:
        """Request an immediate stop, discarding anything still unacked."""
        self._abort_event.set()
        self._finish_event.set()

    def run(self) -> None:
        self._on_state(StreamState.CONNECTING, "")

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.DEALER)
        sock.setsockopt(zmq.IDENTITY, self._session_id.encode("utf-8"))
        sock.setsockopt(zmq.SNDHWM, 50)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://127.0.0.1:{self._local_port}")

        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)

        unacked: dict[int, tuple[FrameHeader, bytes]] = {}
        unacked_bytes = 0
        last_ack_time = time.monotonic()
        resume_pending = False

        try:
            sock.send_multipart(
                pack_message(MSG_SESSION_START, self._session_id, self._header)
            )
            self._on_state(StreamState.STREAMING, "")

            while True:
                if self._abort_event.is_set():
                    break

                drained_any = False
                while True:
                    try:
                        frame_index, header, payload = self._to_send.get_nowait()
                    except queue.Empty:
                        break
                    unacked[frame_index] = (header, payload)
                    unacked_bytes += len(payload)
                    sock.send_multipart(
                        pack_message(MSG_FRAME, self._session_id, header, payload)
                    )
                    drained_any = True

                events = dict(poller.poll(timeout=_POLL_TIMEOUT_MS))
                if sock in events:
                    parts = sock.recv_multipart()
                    msg_type, _sid, ack_header, _payload = unpack_message(parts)
                    if msg_type == MSG_ACK:
                        through = cast("int", ack_header.get("through_frame_index", -1))
                        for fi in [fi for fi in unacked if fi <= through]:
                            _, payload = unacked.pop(fi)
                            unacked_bytes -= len(payload)
                        last_ack_time = time.monotonic()
                        if resume_pending:
                            # This ACK is the RESUME's reply. Per protocol,
                            # explicitly resend everything still unacked (in
                            # frame_index order) rather than relying on ZMQ's
                            # own outbound buffering having survived the
                            # disconnect -- the receiver dedupes by (t, c),
                            # so resending already-staged data is a safe
                            # no-op, not a duplicate ticket.
                            for frame_index in sorted(unacked):
                                header, payload = unacked[frame_index]
                                sock.send_multipart(
                                    pack_message(
                                        MSG_FRAME, self._session_id, header, payload
                                    )
                                )
                            resume_pending = False

                stale = time.monotonic() - last_ack_time > _STALE_ACK_S
                if stale and unacked and not resume_pending:
                    sock.send_multipart(pack_message(MSG_RESUME, self._session_id, {}))
                    resume_pending = True

                if unacked_bytes > self._buffer_budget_bytes:
                    self._on_state(
                        StreamState.BACKLOG_ALARM,
                        f"{unacked_bytes // (1024 * 1024)} MiB unacked",
                    )
                elif stale and unacked:
                    self._on_state(StreamState.RECONNECTING, "")
                elif drained_any or not unacked:
                    self._on_state(StreamState.STREAMING, "")

                if (
                    self._finish_event.is_set()
                    and self._to_send.empty()
                    and not unacked
                ):
                    break
                if self._finish_event.is_set() and not self._abort_event.is_set():
                    self._on_state(
                        StreamState.FINISHING, f"{len(unacked)} volumes unacked"
                    )

            reason = (
                self._finish_reason
                if not self._abort_event.is_set()
                else ("client_abort")
            )
            sock.send_multipart(
                pack_message(MSG_SESSION_END, self._session_id, {"reason": reason})
            )
        finally:
            sock.close(linger=0)
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
    ) -> None:
        self._mmc = mmcore
        self._tunnel = tunnel
        self._get_settings = get_settings
        self._on_state_changed = on_state_changed
        self._assembler = VolumeAssembler()
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

        active_spectral = _active_spectral_channels(
            sequence, self._mmc, settings.spectral
        )
        header, reason = _build_session_header(
            sequence, self._mmc, argus, active_spectral
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

        self._tunnel.start()
        self._assembler.reset(sequence)
        self._worker = _RunWorker(
            session_id=str(uuid4()),
            local_port=argus.local_port,
            header=header,
            buffer_budget_bytes=argus.buffer_budget_mb * 1024 * 1024,
            on_state=self._emit_state,
        )
        self._all_workers = [w for w in self._all_workers if w.is_alive()]
        self._all_workers.append(self._worker)
        self._worker.start()

    def _submit_if_complete(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        volume = self._assembler.add_frame(frame, event, meta)
        if volume is not None and self._worker is not None:
            self._worker.submit(volume)

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
