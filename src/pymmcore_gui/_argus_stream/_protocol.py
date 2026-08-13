"""Wire protocol for streaming acquired volumes to the Argus real-time receiver.

Hand-ported to match ``opym_local/src/opym/stream/protocol.py`` (Argus side,
read directly from Argus -- see ``opym_local/docs/STREAMING_PROTOCOL.md`` for
the narrative spec). If the two ever disagree, the Argus copy is canonical;
this file exists purely so the acquisition client has zero dependency on the
``opym`` package itself (which pulls in MATLAB/GPU-pipeline tooling that has
no business on the Windows acquisition workstation).

Transport: ZeroMQ ``DEALER`` (this client) <-> ``ROUTER`` (Argus receiver),
one TCP connection reached through an SSH local port-forward (see
``_tunnel.py``). Every message is a ZMQ multipart message::

    [msg_type, session_id, msgpack_header, raw_payload?]

``raw_payload`` is present only on ``MSG_FRAME``. ZMQ frames each part
itself, so there is no additional length-prefixing to implement.
"""

from __future__ import annotations

from typing import Literal, TypedDict

import msgpack

MSG_SESSION_START = b"SESSION_START"
MSG_FRAME = b"FRAME"
MSG_SESSION_END = b"SESSION_END"
MSG_ACK = b"ACK"
MSG_RESUME = b"RESUME"

_VALID_TYPES = frozenset(
    {MSG_SESSION_START, MSG_FRAME, MSG_SESSION_END, MSG_ACK, MSG_RESUME}
)


class SessionStartHeader(TypedDict):
    """``SESSION_START`` header -- sent once, before any ``FRAME`` message.

    Fields map directly onto the Argus side's ``opym.petakit.submit_pipeline_job``
    parameters, since the receiver has no other source for them.
    """

    base_name: str
    output_dir: str
    dtype: str
    shape_zyx: list[int]
    num_timepoints: int
    channels: list[int]
    channel_names: list[str]
    z_step_um: float
    xy_pixel_size: float
    sheet_angle_deg: float
    t_interval_s: float
    interp_method: str
    rl_method: str
    iterations: int | None
    psf_paths: list[str] | None
    dz_psf: float | None


class FrameHeader(TypedDict):
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


class SessionEndHeader(TypedDict):
    """``SESSION_END`` header."""

    reason: Literal["complete", "idle_timeout", "client_abort"]


class AckHeader(TypedDict):
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
    SessionStartHeader | FrameHeader | SessionEndHeader | AckHeader | ResumeHeader
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
