"""Chunked, lazily-allocated, memory-bounded, in-RAM single-array MDA display store.

Purely for backing a live ``ndv.ArrayViewer`` during an MDA -- NOT a save
handler (no disk I/O, no metadata), and independent of the MDA's real output
handler. Intentionally does NOT use tensorstore: a prior in-memory
``TensorStoreHandler`` here caused a native STATUS_STACK_BUFFER_OVERRUN during
sustained dual-camera runs (ndv's background thread does blocking native reads
while frames are asynchronously, natively written -- concurrently, x2
cameras).

Backed by a plain, in-memory ``zarr.Array`` (chunk size = exactly one frame)
rather than a dense ``np.ndarray``: declaring the full ``(t, p, z, c, y, x)``
domain up front must NOT eagerly allocate memory for the whole acquisition --
a real 100-timepoint x 201-slice x 2400x2400 uint16 dual-camera run is ~216
GiB dense, which is exactly what a naive ``np.zeros(full_shape)`` here did (it
raised ``MemoryError`` on the very first frame, which silently broke the live
preview since psygnal swallows exceptions raised in ``frameReady`` signal
handlers). zarr's chunks are only materialized when written, so memory grows
with frames actually acquired so far -- the same lazy-allocation profile
``TensorStoreHandler`` had, just without tensorstore's native code.

Even lazy per-chunk allocation isn't enough on its own for very long
acquisitions: retaining every frame ever acquired still grows without bound.
Confirmed in production: a real 100-timepoint dual-camera run hit a genuine
``MemoryError`` (this time inside ``numcodecs.blosc.compress``, writing a
*new* chunk) at timepoint 11 of 100 -- tens of GB of real, poorly-compressible
camera noise had already accumulated per camera. So the ``"t"`` axis is
additionally capped to a rolling window sized from a memory-byte budget (see
``_DEFAULT_WINDOW_BUDGET_BYTES``): writes land at ``t % window_size``, so the
array only ever holds the most recent ``window_size`` timepoints, evicting
(overwriting) older ones. ndv's ``ArrayLikeWrapper`` auto-wraps any object
with ``.shape``/``__getitem__``/``__array__`` (which ``zarr.Array``
satisfies), and slices *before* materializing to numpy (``self._data[idx]``
then ``np.asarray()`` on the small result), so reads never force the whole
array into memory either.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import zarr

from pymmcore_gui._vendored.mda_handlers._util import position_sizes

if TYPE_CHECKING:
    from collections.abc import Mapping

    import numpy as np
    import useq
    from pymmcore_plus.metadata import FrameMetaV1

FRAME_DIM = "frame"
_SIZE_INCREMENT = 300

# Per-instance (i.e. per physical camera) memory budget for the rolling "t"
# window. A fixed *volume count* wouldn't self-scale to frame size/channel/
# position count; a byte budget does, and is the only approach that
# guarantees the production OOM above can't recur regardless of frame size or
# acquisition length. For that exact 201-slice x 2400x2400 uint16 workload
# (~2.16 GiB/timepoint/camera), this yields a window of ~11 timepoints.
#
# Sized against this rig's real memory budget (224 GiB total), not just an
# arbitrary safe-sounding number: ~30 GiB is already committed at session
# startup to the main-process MMCore circular buffer (HardwareConstants
# .circular_buffer_target_mb, asi_z_stack/common.py) plus up to 4 GiB per
# physical camera worker process during a hardware-triggered run
# (.worker_circular_buffer_mb) -- worst case (4 cameras) that's ~46 GiB
# already resident before this budget adds anything. 24 GiB/camera keeps a
# worst-case 4-camera total (96 GiB) comfortably within the remaining
# headroom, with margin for the OS and everything else.
#
# This does NOT cost any acquisition-start allocation time the way growing
# the circular buffer does (that's eager/dense, hence the startup splash
# screen -- see ensure_circular_buffer_capacity_async): a zarr array with an
# in-memory store only allocates lightweight shape/chunk metadata up front
# (empirically ~0.1-100ms regardless of declared size) and defers all real
# memory to individual chunk writes as frames actually arrive during the
# run, so a larger budget here is free until an acquisition is actually long
# enough to use it.
_DEFAULT_WINDOW_BUDGET_BYTES = 24 * 1024**3


class NumpyDisplayStore:
    """Chunked, lazily-allocated, memory-bounded display store for a live preview."""

    def __init__(
        self, *, window_budget_bytes: int = _DEFAULT_WINDOW_BUDGET_BYTES
    ) -> None:
        self._current_sequence: useq.MDASequence | None = None
        self._array: zarr.Array | None = None
        self._labels: tuple[str, ...] = ()
        # "_nd_storage" mirrors TensorStoreHandler: True means we could build a
        # labeled ND array from `seq.sizes`; False is the growable frame-dim
        # fallback for a sequence with no usable axes.
        self._nd_storage: bool = True
        self._frame_index: int = 0

        self._window_budget_bytes = window_budget_bytes
        # Number of timepoints retained in RAM, or None if there's no "t"
        # axis to window (set once, at first-frame time).
        self._window_size: int | None = None
        # Highest "t" seen so far (-1 before the first frame).
        self._max_t_seen: int = -1
        # (t, p) of the volume currently being written, and how many z-slices
        # it has received so far. Only the newest (t, p) can ever be
        # in-progress -- a frame for a new (t, p) implies the previous one is
        # complete (acquisition order is monotonic).
        self._newest_tp: tuple[int, int] | None = None
        self._newest_tp_z_filled: int = 0

    @property
    def array(self) -> zarr.Array | None:
        """The current backing zarr array (``None`` until the first frame)."""
        return self._array

    @property
    def dims(self) -> tuple[str, ...]:
        """Axis labels for ``.array``, in shape order (e.g. ``('t','z','y','x')``).

        Empty until the first frame allocates ``.array``.
        """
        return self._labels

    @property
    def current_sequence(self) -> useq.MDASequence | None:
        """Return current sequence, or None. Use ``.reset()`` for a new one."""
        return self._current_sequence

    @property
    def max_t_seen(self) -> int:
        """Highest ``t`` index written so far (``-1`` before the first frame)."""
        return self._max_t_seen

    @property
    def window_size(self) -> int | None:
        """Number of timepoints retained in RAM, or ``None`` if not windowed."""
        return self._window_size

    def is_evicted(self, t: int) -> bool:
        """Return True if timepoint *t* has been overwritten (no longer resident)."""
        if self._window_size is None or self._max_t_seen < 0:
            return False
        return t < self._max_t_seen - self._window_size + 1

    def z_progress_for(self, t: int, p: int) -> int | None:
        """Number of z-slices written so far for ``(t, p)``.

        Returns ``None`` if ``(t, p)`` is not the currently-filling volume --
        i.e. it's an older, and therefore guaranteed-complete, volume, and
        callers should use the full declared z size instead.
        """
        if (t, p) != self._newest_tp:
            return None
        return self._newest_tp_z_filled

    def reset(self, sequence: useq.MDASequence) -> None:
        """Reset state to prepare for new *sequence*."""
        self._frame_index = 0
        self._array = None
        self._current_sequence = sequence
        self._window_size = None
        self._max_t_seen = -1
        self._newest_tp = None
        self._newest_tp_z_filled = 0

    def frameReady(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1, /
    ) -> None:
        """Write *frame* into the (lazily-allocated) display array."""
        if self._array is None:
            self._array = self._new_array(frame, event.sequence)

        index: tuple[int | slice, ...]
        if self._nd_storage:
            index = self._event_index_to_array_index(event.index)
        else:
            if self._frame_index >= self._array.shape[0]:
                self._array = self._grow(self._array)
            index = (self._frame_index,)

        # Each frame lands in its own, never-before-written-THIS-CYCLE chunk
        # (chunk size is exactly one frame -- see _new_array), and reads only
        # ever target chunks that a prior, already-completed frameReady call
        # finished writing (_on_frame_ready always calls frameReady
        # synchronously before scheduling any viewer update). So there's no
        # read/write overlap on the same chunk to race on, unlike the old
        # tensorstore store's concurrent native async-write + threaded
        # blocking-read. (A chunk *can* be revisited across window
        # wraparound -- see to_storage_index -- but only after the frame
        # that previously occupied it is long since displayed and gone.)
        self._array[index] = frame
        self._frame_index += 1

        if self._window_size is not None:
            self._update_window_bookkeeping(event.index)

    def sequenceFinished(self, sequence: useq.MDASequence) -> None:
        """No-op.

        Unlike ``TensorStoreHandler`` there is no on-disk artifact or metadata
        file to flush here, and ``self._array`` is discarded wholesale by the
        next ``reset()`` anyway. Kept only so call sites can treat every
        per-camera/own display handler uniformly.
        """

    def to_storage_index(
        self, by_label: Mapping[str, int | slice]
    ) -> dict[str, int | slice]:
        """Map a label-keyed logical index to a label-keyed physical-array index.

        Only ``"t"`` is translated (mod ``window_size``, when windowing is
        active); every other axis passes through unchanged. Used by both the
        write path (``_event_index_to_array_index``) and the read path (the
        ndv wrapper's ``isel``), so they can never drift apart.
        """
        out = dict(by_label)
        n = self._window_size
        if n is not None and "t" in out:
            v = out["t"]
            if isinstance(v, int):
                out["t"] = v % n
            elif isinstance(v, slice) and v.start is not None:
                width = 1 if v.stop is None else v.stop - v.start
                start = v.start % n
                out["t"] = slice(start, start + width)
        return out

    # ------------------------------------------------------------------
    # internal helpers -- mirrored (not reused) from TensorStoreHandler; see
    # module docstring for why these can't be imported/subclassed instead.
    # ------------------------------------------------------------------

    def _new_array(self, frame: np.ndarray, seq: useq.MDASequence | None) -> zarr.Array:
        shape, chunks, labels = self._shape_chunks_labels(frame.shape, seq)
        self._nd_storage = FRAME_DIM not in labels
        self._labels = labels

        if self._nd_storage and "t" in labels:
            t_pos = labels.index("t")
            declared_t = shape[t_pos]
            bytes_per_t = frame.dtype.itemsize
            for i, sz in enumerate(shape):
                if i != t_pos:
                    bytes_per_t *= sz
            window = max(1, min(declared_t, self._window_budget_bytes // bytes_per_t))
            self._window_size = window
            shape = (*shape[:t_pos], window, *shape[t_pos + 1 :])
        else:
            self._window_size = None

        # No `store=` -> zarr's default in-memory (plain dict) store. Chunks
        # are created lazily on write; reading an unwritten chunk returns
        # zeros without allocating anything.
        return zarr.zeros(shape, chunks=chunks, dtype=frame.dtype)

    def _shape_chunks_labels(
        self, frame_shape: tuple[int, ...], seq: useq.MDASequence | None
    ) -> tuple[tuple[int, ...], tuple[int, ...], tuple[str, ...]]:
        """Mirrors TensorStoreHandler.get_shape_chunks_labels."""
        labels: tuple[str, ...]
        if seq is not None and seq.sizes:
            # expand the sizes to include the largest size we encounter for each
            # axis; positions with subsequences still yield a jagged array, but
            # it won't take extra space and we won't get index errors.
            max_sizes = dict(seq.sizes)
            for psize in position_sizes(seq):
                for k, v in psize.items():
                    max_sizes[k] = max(max_sizes.get(k, 0), v)

            labels, sizes = zip(*(x for x in max_sizes.items() if x[1]), strict=False)
            full_shape: tuple[int, ...] = (*sizes, *frame_shape)
        else:
            labels = (FRAME_DIM,)
            full_shape = (_SIZE_INCREMENT, *frame_shape)

        # one frame per chunk: every non-frame axis gets a chunk size of 1.
        chunks = (1,) * (len(full_shape) - len(frame_shape)) + frame_shape
        return full_shape, chunks, (*labels, "y", "x")

    def _event_index_to_array_index(
        self, index: Mapping[str, int]
    ) -> tuple[int | slice, ...]:
        """Convert an event.index mapping into a tuple valid for __setitem__."""
        by_label = {label: index.get(label, slice(None)) for label in self._labels}
        storage = self.to_storage_index(by_label)
        return tuple(storage[label] for label in self._labels)

    def _update_window_bookkeeping(self, index: Mapping[str, int]) -> None:
        """Track ``max_t_seen`` and the newest volume's z-fill progress.

        Assumes z arrives in non-decreasing order within a volume (true for
        every engine in this app), so "highest z seen + 1" is an accurate
        count of z-slices written so far for the in-progress volume.
        """
        t = index.get("t")
        if t is None:
            return
        p = index.get("p", 0)
        self._max_t_seen = max(self._max_t_seen, t)
        tp = (t, p)
        if tp != self._newest_tp:
            self._newest_tp = tp
            self._newest_tp_z_filled = 0
        z = index.get("z")
        if z is not None:
            self._newest_tp_z_filled = max(self._newest_tp_z_filled, z + 1)

    def _grow(self, ary: zarr.Array) -> zarr.Array:
        """Grow the frame-dim fallback array by ``_SIZE_INCREMENT`` frames."""
        ary.resize((self._frame_index + _SIZE_INCREMENT, *ary.shape[1:]))
        return ary
