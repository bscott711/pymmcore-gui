"""Assemble per-z-plane ``frameReady`` frames into (t, c) volumes or slabs.

Argus's ``FRAME`` message carries either one complete volume per ``(t, c)``
or, once the receiver advertises ``"slabs"``, a run of consecutive z-planes
of one (see ``_protocol.py``). This module accumulates the z-planes of a
z-stack into one C-contiguous array. In volume mode it reports the volume
the instant its last plane lands. In slab mode it hands out each run of
``slab_planes`` consecutive planes as soon as the run is complete, so the
volume is already on its way while the stack is still being acquired.

Scoped to single-position sequences (see
:class:`~pymmcore_gui._settings.ArgusStreamSettingsV1`'s docstring) --
callers are responsible for only using this for eligible sequences.
"""

from __future__ import annotations

import time
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
    acq_first_s: float = 0.0
    """Wall clock (``time.time()``) when this volume's first plane arrived."""
    acq_last_s: float = 0.0
    """Wall clock when its last plane arrived, i.e. when it completed."""
    z0: int = 0
    """First plane of ``array`` within its volume (slabs only)."""
    nz: int | None = None
    """The whole volume's plane count for a slab; ``None`` for a volume."""


@dataclass
class _Pending:
    array: np.ndarray
    seen: set[int] = field(default_factory=set)
    timestamp: float = 0.0
    camera_id: str | int | None = None
    acq_first_s: float = field(default_factory=time.time)
    slab_planes: int = 0
    """Fixed when the volume's first plane lands; 0 = whole-volume mode."""
    next_z: int = 0
    """Slab mode: the first plane not yet handed out."""


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

    def clear(self) -> None:
        """Drop every partly assembled volume (keeps the expected z-count)."""
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
        ready = self.add_plane(frame, event, meta)
        return ready[0] if ready else None

    def add_plane(
        self,
        frame: np.ndarray,
        event: useq.MDAEvent,
        meta: FrameMetaV1,
        slab_planes: int = 0,
    ) -> list[Volume]:
        """Add one 2D plane; return whatever it made ready to send.

        With ``slab_planes == 0`` that's the whole volume once its last plane
        lands, as :meth:`add_frame`. Otherwise it's every run of
        ``slab_planes`` consecutive planes (the last run may be shorter)
        that is now complete, starting at the first plane not yet handed
        out. Planes arriving out of z order simply hold a slab back until
        the gap fills. ``slab_planes`` is fixed per volume by the volume's
        first plane, so a volume never switches mode halfway.
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
                slab_planes=max(0, slab_planes),
            )
            self._pending[key] = pending

        # Assigning into a slice copies the data -- pending.array owns its
        # own memory independent of frame's backing buffer, satisfying the
        # same "always copy defensively" requirement an explicit .copy()
        # would, without an extra allocation.
        pending.array[z] = frame
        pending.seen.add(z)
        complete = len(pending.seen) >= self._expected_z
        if complete:
            del self._pending[key]

        if not pending.slab_planes:
            if not complete:
                return []
            return [
                Volume(
                    t=t,
                    c=c,
                    array=pending.array,
                    timestamp=pending.timestamp,
                    camera_id=pending.camera_id,
                    acq_first_s=pending.acq_first_s,
                    acq_last_s=time.time(),
                )
            ]

        ready = []
        nz = self._expected_z
        while pending.next_z < nz:
            z0 = pending.next_z
            end = min(z0 + pending.slab_planes, nz)
            if any(k not in pending.seen for k in range(z0, end)):
                break
            ready.append(
                Volume(
                    t=t,
                    c=c,
                    array=pending.array[z0:end],
                    timestamp=pending.timestamp,
                    camera_id=pending.camera_id,
                    acq_first_s=pending.acq_first_s,
                    acq_last_s=time.time(),
                    z0=z0,
                    nz=nz,
                )
            )
            pending.next_z = end
        return ready
