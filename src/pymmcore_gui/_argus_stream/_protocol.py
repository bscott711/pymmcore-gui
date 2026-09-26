"""Wire protocol for streaming acquired volumes to the Argus real-time receiver.

Hand-ported to match ``opym_local/src/opym/stream/protocol.py`` (Argus side,
read directly from Argus -- see ``opym_local/docs/STREAMING_PROTOCOL.md`` for
the narrative spec). If the two ever disagree, the Argus copy is canonical;
this file exists purely so the acquisition client has zero dependency on the
``opym`` package itself (which pulls in MATLAB/GPU-pipeline tooling that has
no business on the Windows acquisition workstation).

Transport: ZeroMQ ``DEALER`` (this client) <-> ``ROUTER`` (Argus receiver),
reached through SSH local port-forwards (see ``_tunnel.py``). Link 0's
identity is the session_id; once an ACK advertises ``"links"``, extra links
k >= 1 carry the same session with identity ``"<session_id>#<k>"``. Every
message is a ZMQ multipart message::

    [msg_type, session_id, msgpack_header, raw_payload?]

``raw_payload`` is present only on ``MSG_FRAME``. ZMQ frames each part
itself, so there is no additional length-prefixing to implement.
"""

from __future__ import annotations

from typing import Any, Literal, TypedDict

import msgpack

MSG_SESSION_START = b"SESSION_START"
MSG_FRAME = b"FRAME"
MSG_SESSION_END = b"SESSION_END"
MSG_ACK = b"ACK"
MSG_RESUME = b"RESUME"
MSG_QC = b"QC"

_VALID_TYPES = frozenset(
    {MSG_SESSION_START, MSG_FRAME, MSG_SESSION_END, MSG_ACK, MSG_RESUME, MSG_QC}
)


OutputFormat = Literal["tiff", "ome-zarr", "both"]
"""Format the processed (DSR) result is kept in on Argus.

Matches ``opym.stream.rawmirror.OUTPUT_FORMATS``: ``"tiff"`` keeps the
OME-TIFF frames (ChimeraX), ``"ome-zarr"`` replaces them with one OME-Zarr
(napari), ``"both"`` keeps both.
"""


class _SessionStartOptional(TypedDict, total=False):
    # Server -> client message types this client understands beyond ACK.
    # ["qc"] asks for MSG_QC; a receiver without live QC just never sends it.
    accepts: list[str]
    # Only when re-opening a session Argus answered ``unknown_session`` for
    # (it restarted or timed the session out): every frame_index up to this
    # was already ACKed. Needs the "resume" feature.
    resume_through: int


class SessionStartHeader(_SessionStartOptional):
    """``SESSION_START`` header -- sent once, before any ``FRAME`` message.

    Fields match ``opym.stream.receiver.StreamReceiver._handle_session_start``
    (Argus side, canonical -- read directly from Argus, not from this
    file's own history). The receiver writes each channel straight into a
    raw OME-Zarr mirror store under ``raw_root`` (``opym.stream.rawmirror``);
    it does not submit a processing ticket itself, and does not read decon
    parameters (PSF, wiener_alpha, edge_erosion, rl_method, iterations) from
    this handshake at all -- those are resolved entirely server-side from
    the ``OPYM_DECON_PSF`` env var, same as a batch/Globus-landed dataset.
    An earlier draft of this header targeted a since-rewritten receiver that
    took an ``output_dir`` and consumed PSF paths directly; sending that
    shape to the current receiver has ``raw_root`` missing, which the
    receiver rejects outright (every frame silently dropped as an unknown
    session) -- see ``opym.stream.protocol``'s own module docstring for the
    field-by-field spec this must keep matching.
    """

    base_name: str
    raw_root: str
    dtype: str
    shape_zyx: list[int]
    num_timepoints: int
    channels: list[int]
    channel_names: list[str]
    z_step_um: float
    xy_pixel_size: float
    t_interval_s: float
    output_format: OutputFormat


class _FrameTraceFields(TypedDict, total=False):
    # Latency trace (opym.stream.trace on Argus). Epoch seconds on THIS
    # machine's clock; Argus converts them with clock_offset_s.
    acq_first_s: float
    acq_last_s: float
    queued_s: float
    sent_s: float
    clock_offset_s: float


class _FrameSlabFields(TypedDict, total=False):
    # Only once an ACK advertised "slabs": this FRAME carries planes
    # [z0, z0 + shape_zyx[0]) of an nz-plane volume.
    z0: int
    nz: int


class _FrameCodecFields(TypedDict, total=False):
    # Only once an ACK advertised "blosc": "blosc" means the payload is one
    # blosc frame of the raw bytes (lz4 + bitshuffle). shape_zyx and dtype
    # always describe the decoded data.
    codec: str


class FrameHeader(_FrameTraceFields, _FrameSlabFields, _FrameCodecFields):
    """``FRAME`` header -- one per acquired volume, sent alongside its bytes.

    ``t``/``c`` are this volume's own acquisition indices, not send order --
    frames may arrive in any ``(t, c)`` order and the receiver stages/tickets
    by those indices, not by arrival order. ``frame_index`` is unrelated: a
    monotonic counter assigned once per real acquired volume (not per send
    attempt), used only for ``ACK``/``RESUME`` bookkeeping.
    """

    t: int
    c: int
    frame_index: int
    timestamp: float
    camera_id: str | int | None
    shape_zyx: list[int]
    dtype: str


class QCHeader(TypedDict, total=False):
    """``QC`` header -- server -> client, only if ``accepts`` has ``"qc"``.

    The Argus live QC service's verdict on one timepoint, forwarded as it
    wrote it (``opym.stream.protocol`` documents the fields). Advisory only:
    nothing on Argus waits for a reply. There are two per timepoint:
    ``stage == "raw"`` about a second after the frames land (coverage, drift,
    focus, signal), then ``"dsr"`` once deskewed (adds the cell's box).
    Unknown fields must be ignored; new ones may be added.
    """

    seq: int
    session_id: str
    t: int
    stage: str
    verdict: Literal["ok", "warn", "act", "no_cell"]
    flags: list[str]
    advice: list[dict[str, Any]]
    metrics: dict[str, Any]


class SessionEndHeader(TypedDict):
    """``SESSION_END`` header."""

    # "paused": this run stopped streaming mid-way (Argus unreachable longer
    # than the send buffer holds); Argus won't keep its partial copy.
    reason: Literal["complete", "idle_timeout", "client_abort", "paused"]


class _AckOptional(TypedDict, total=False):
    # Argus's clock when it sent the ACK: lets the client estimate its clock
    # offset for the FRAME trace fields.
    server_time_s: float
    # What this receiver supports beyond the base protocol: "slabs",
    # "blosc", "links", "resume".
    features: list[str]
    # Argus has no open session with this id: re-send SESSION_START with
    # resume_through, then everything unACKed.
    unknown_session: bool
    # The final ACK, sent once Argus has closed the session: it confirms
    # SESSION_END.
    ended: bool


class AckHeader(_AckOptional):
    """``ACK`` header -- server -> client, unsolicited, not per-frame.

    ``through_frame_index`` is the highest ``frame_index`` such that every
    frame in ``[0, through_frame_index]`` has been durably staged *and*
    ticketed on the Argus side. ``-1`` means nothing processed yet.
    """

    through_frame_index: int


class ResumeHeader(TypedDict):
    """``RESUME`` header -- client -> server, right after reconnecting.

    No fields required: ``session_id`` in the message envelope is enough for
    the server to look up (or fail to find) this session's state.
    """


MessageHeader = (
    SessionStartHeader
    | FrameHeader
    | SessionEndHeader
    | AckHeader
    | ResumeHeader
    | QCHeader
)


def pack_message(
    msg_type: bytes,
    session_id: str,
    header: MessageHeader,
    payload: bytes | None = None,
) -> list[bytes]:
    """Build the multipart frame list for one protocol message.

    Parameters
    ----------
    msg_type : bytes
        One of the ``MSG_*`` constants.
    session_id : str
        This run's session id (also set as the DEALER socket's identity).
    header : MessageHeader
        The header matching ``msg_type`` (one of the ``*Header`` TypedDicts
        above).
    payload : bytes | None
        The raw volume bytes. Must be given (and non-``None``) only for
        ``MSG_FRAME`` -- every other message type is header-only.
    """
    if msg_type not in _VALID_TYPES:
        raise ValueError(f"Unknown message type: {msg_type!r}")
    parts = [
        msg_type,
        session_id.encode("utf-8"),
        msgpack.packb(header, use_bin_type=True),
    ]
    if payload is not None:
        parts.append(payload)
    return parts


def unpack_message(
    parts: list[bytes],
) -> tuple[bytes, str, dict[str, object], bytes | None]:
    """Inverse of :func:`pack_message`.

    Parameters
    ----------
    parts : list[bytes]
        The raw multipart message as received from the socket. A ``DEALER``
        socket's ``recv_multipart()`` needs no identity-frame stripping --
        only the server's ``ROUTER`` side does.
    """
    if len(parts) not in (3, 4):
        raise ValueError(f"Expected 3 or 4 message parts, got {len(parts)}")
    msg_type, session_id_bytes, header_bytes, *rest = parts
    if msg_type not in _VALID_TYPES:
        raise ValueError(f"Unknown message type: {msg_type!r}")
    header: dict[str, object] = msgpack.unpackb(header_bytes, raw=False)
    payload = rest[0] if rest else None
    return msg_type, session_id_bytes.decode("utf-8"), header, payload
