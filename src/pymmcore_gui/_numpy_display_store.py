"""Minimal, in-RAM, single-array MDA display store for live ndv previews.

Purely for backing a live ``ndv.ArrayViewer`` during an MDA -- NOT a save
handler (no disk I/O, no metadata), and independent of the MDA's real output
handler. Intentionally does NOT use tensorstore: a prior in-memory
``TensorStoreHandler`` here caused a native STATUS_STACK_BUFFER_OVERRUN during
sustained dual-camera runs (ndv's background thread does blocking native reads
while frames are asynchronously, natively written -- concurrently, x2 cameras).
The preview only ever needed a pre-allocated in-RAM array; ndv's
``ArrayLikeWrapper`` auto-wraps a plain ``np.ndarray``, so no custom
``DataWrapper`` is required.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from pymmcore_gui._vendored.mda_handlers._util import position_sizes

if TYPE_CHECKING:
    from collections.abc import Mapping

    import useq
    from pymmcore_plus.metadata import FrameMetaV1

FRAME_DIM = "frame"
_SIZE_INCREMENT = 300


class NumpyDisplayStore:
    """Pre-allocated, in-RAM, single-array display store for a live MDA preview."""

    def __init__(self) -> None:
        self._current_sequence: useq.MDASequence | None = None
        self._array: np.ndarray | None = None
        self._labels: tuple[str, ...] = ()
        # "_nd_storage" mirrors TensorStoreHandler: True means we could build a
        # labeled ND array from `seq.sizes`; False is the growable frame-dim
        # fallback for a sequence with no usable axes.
        self._nd_storage: bool = True
        self._frame_index: int = 0

    @property
    def array(self) -> np.ndarray | None:
        """The current backing numpy array (``None`` until the first frame)."""
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
        """Write *frame* into the pre-allocated display array."""
        if self._array is None:
            self._array = self._new_array(frame, event.sequence)

        index: tuple[int | slice, ...]
        if self._nd_storage:
            index = self._event_index_to_array_index(event.index)
        else:
            if self._frame_index >= self._array.shape[0]:
                self._array = self._grow(self._array)
            index = (self._frame_index,)

        # single vectorized assignment -- see plan's thread-safety note: this
        # races with ndv's background-thread reads, but numpy __setitem__ is
        # GIL-governed and bounds-checked, so at worst a frame is displayed
        # briefly torn/stale, never a native fault.
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

    def _new_array(self, frame: np.ndarray, seq: useq.MDASequence | None) -> np.ndarray:
        shape, labels = self._shape_and_labels(frame.shape, seq)
        self._nd_storage = FRAME_DIM not in labels
        self._labels = labels
        return np.zeros(shape, dtype=frame.dtype)

    def _shape_and_labels(
        self, frame_shape: tuple[int, ...], seq: useq.MDASequence | None
    ) -> tuple[tuple[int, ...], tuple[str, ...]]:
        """Mirrors TensorStoreHandler.get_shape_chunks_labels, minus chunking."""
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
        return full_shape, (*labels, "y", "x")

    def _event_index_to_array_index(
        self, index: Mapping[str, int]
    ) -> tuple[int | slice, ...]:
        """Convert an event.index mapping into a tuple valid for __setitem__."""
        return tuple(index.get(label, slice(None)) for label in self._labels)

    def _grow(self, ary: np.ndarray) -> np.ndarray:
        """Grow the frame-dim fallback array by ``_SIZE_INCREMENT`` frames."""
        pad = np.zeros((_SIZE_INCREMENT, *ary.shape[1:]), dtype=ary.dtype)
        return np.concatenate([ary, pad], axis=0)
