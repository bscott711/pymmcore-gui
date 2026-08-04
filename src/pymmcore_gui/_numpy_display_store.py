"""Chunked, lazily-allocated, in-RAM single-array MDA display store.

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
``TensorStoreHandler`` had, just without tensorstore's native code. ndv's
``ArrayLikeWrapper`` auto-wraps any object with ``.shape``/``__getitem__``/
``__array__`` (which ``zarr.Array`` satisfies), and slices *before*
materializing to numpy (``self._data[idx]`` then ``np.asarray()`` on the
small result), so reads never force the whole array into memory either.
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


class NumpyDisplayStore:
    """Chunked, lazily-allocated, in-RAM display store for a live MDA preview."""

    def __init__(self) -> None:
        self._current_sequence: useq.MDASequence | None = None
        self._array: zarr.Array | None = None
        self._labels: tuple[str, ...] = ()
        # "_nd_storage" mirrors TensorStoreHandler: True means we could build a
        # labeled ND array from `seq.sizes`; False is the growable frame-dim
        # fallback for a sequence with no usable axes.
        self._nd_storage: bool = True
        self._frame_index: int = 0

    @property
    def array(self) -> zarr.Array | None:
        """The current backing zarr array (``None`` until the first frame)."""
        return self._array

    @property
    def current_sequence(self) -> useq.MDASequence | None:
        """Return current sequence, or None. Use ``.reset()`` for a new one."""
        return self._current_sequence

    def reset(self, sequence: useq.MDASequence) -> None:
        """Reset state to prepare for new *sequence*."""
        self._frame_index = 0
        self._array = None
        self._current_sequence = sequence

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

        # Each frame lands in its own, never-before-written chunk (chunk size
        # is exactly one frame -- see _new_array), and reads only ever target
        # chunks that a prior, already-completed frameReady call finished
        # writing (_on_frame_ready always calls frameReady synchronously
        # before scheduling any viewer update). So there's no read/write
        # overlap on the same chunk to race on, unlike the old tensorstore
        # store's concurrent native async-write + threaded blocking-read.
        self._array[index] = frame
        self._frame_index += 1

    def sequenceFinished(self, sequence: useq.MDASequence) -> None:
        """No-op.

        Unlike ``TensorStoreHandler`` there is no on-disk artifact or metadata
        file to flush here, and ``self._array`` is discarded wholesale by the
        next ``reset()`` anyway. Kept only so call sites can treat every
        per-camera/own display handler uniformly.
        """

    # ------------------------------------------------------------------
    # internal helpers -- mirrored (not reused) from TensorStoreHandler; see
    # module docstring for why these can't be imported/subclassed instead.
    # ------------------------------------------------------------------

    def _new_array(self, frame: np.ndarray, seq: useq.MDASequence | None) -> zarr.Array:
        shape, chunks, labels = self._shape_chunks_labels(frame.shape, seq)
        self._nd_storage = FRAME_DIM not in labels
        self._labels = labels
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
        return tuple(index.get(label, slice(None)) for label in self._labels)

    def _grow(self, ary: zarr.Array) -> zarr.Array:
        """Grow the frame-dim fallback array by ``_SIZE_INCREMENT`` frames."""
        ary.resize((self._frame_index + _SIZE_INCREMENT, *ary.shape[1:]))
        return ary
