"""Assemble per-z-plane ``frameReady`` frames into complete (t, c) volumes.

Argus's ``FRAME`` message is one complete volume per ``(t, c)``, not a raw 2D
plane (see ``_protocol.py``). This module accumulates the z-planes of a
z-stack into one C-contiguous array and reports completion the instant the
last plane lands, so :class:`~pymmcore_gui._argus_stream._session.ArgusStreamSession`
can stream each volume the moment it's ready rather than waiting for the
whole acquisition.

Scoped to single-position sequences (see
:class:`~pymmcore_gui._settings.ArgusStreamSettingsV1`'s docstring) --
callers are responsible for only using this for eligible sequences.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from pymmcore_gui._vendored.mda_handlers._util import position_sizes

if TYPE_CHECKING:
    import useq
    from pymmcore_plus.metadata import FrameMetaV1


@dataclass
class Volume:
    """One fully-assembled ``(t, c)`` volume, ready to send."""

    t: int
    c: int
    array: np.ndarray
    """``(Z, Y, X)`` C-contiguous array."""
    timestamp: float
    camera_id: str | int | None


@dataclass
class _Pending:
    array: np.ndarray
    seen: set[int] = field(default_factory=set)
    timestamp: float = 0.0
    camera_id: str | int | None = None


class VolumeAssembler:
    """Accumulates 2D z-planes into complete ``(t, c)`` volumes.

    Axis-order agnostic: each plane is written into its array at the
    z-index declared by its event, regardless of the sequence's
    ``axis_order``, so a ``(t, c)`` key is complete the instant every
    expected z-index has been seen once -- not on any particular arrival
    order.
    """

    def __init__(self) -> None:
        self._expected_z: int = 1
        self._pending: dict[tuple[int, int], _Pending] = {}

    def reset(self, sequence: useq.MDASequence) -> None:
        """Reset all state for a new sequence, deriving the expected z-count."""
        sizes = position_sizes(sequence)
        pos_sizes = sizes[0] if sizes else {}
        self._expected_z = pos_sizes.get("z", 1)
        self._pending.clear()

    def add_frame(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> Volume | None:
        """Add one 2D plane; return a completed :class:`Volume` if this finishes one.

        Parameters
        ----------
        frame : np.ndarray
            The 2D plane just acquired.
        event : useq.MDAEvent
            The event that produced ``frame`` -- ``t``/``c``/``z`` indices are
            read from ``event.index``.
        meta : FrameMetaV1
            Frame metadata; used only for ``camera_device`` and
            ``runner_time_ms`` when starting a new volume.
        """
        t = event.index.get("t", 0)
        c = event.index.get("c", 0)
        z = event.index.get("z", 0)
        key = (t, c)

        pending = self._pending.get(key)
        if pending is None:
            pending = _Pending(
                array=np.zeros((self._expected_z, *frame.shape), dtype=frame.dtype),
                camera_id=meta.get("camera_device"),
                timestamp=float(meta.get("runner_time_ms", 0.0)),
            )
            self._pending[key] = pending

        # Assigning into a slice copies the data -- pending.array owns its
        # own memory independent of frame's backing buffer, satisfying the
        # same "always copy defensively" requirement an explicit .copy()
        # would, without an extra allocation.
        pending.array[z] = frame
        pending.seen.add(z)

        if len(pending.seen) < self._expected_z:
            return None

        del self._pending[key]
        return Volume(
            t=t,
            c=c,
            array=pending.array,
            timestamp=pending.timestamp,
            camera_id=pending.camera_id,
        )
