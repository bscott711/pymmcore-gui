"""A single-slot shared-memory frame buffer for the spot camera's live view.

The camera worker (writer) puts every frame it handles here and the GUI process
(reader) polls it, so full-resolution frames never travel through the control
pipe. One slot only: the reader always sees the newest frame, and a slow reader
costs skipped frames, never a stalled writer.

Layout (little endian, 128-byte header, then the pixels)::

    0   uint64   seq        odd while the writer is in the slot (seqlock)
    8   float64  t          frame time (monotonic seconds, worker clock)
    16  4*uint32 roi        x, y, w, h in sensor pixels
    32  2*uint32 sensor     W, H
    40  uint32   bits
    44  uint32   flags      1 = spot present, 2 = spot ok
    48  2*float64 spot      x, y in sensor pixels
    64  float64  saturated_frac
    72  uint32   max_value
    76  uint32   itemsize   1 (uint8) or 2 (uint16)
    80  2*uint32 shape      image rows, cols
    88  uint64   capacity   pixel bytes the segment holds (set once at create)
    96  uint32   magic
    100 uint32   version
    128 pixels

Seqlock: the writer sets ``seq`` odd, writes metadata and pixels, then sets
``seq`` to the next even number. A reader takes ``seq`` before and after
copying and discards the copy if it was odd or changed. The frame number it
reports (``LiveFrame.seq``) is ``seq // 2``, so it counts frames from 1.

Lifetime: the writer creates the segment and unlinks it on exit. On POSIX the
name disappears at unlink and the memory goes with the last mapping. On Windows
``unlink`` does nothing and the segment lives while any handle is open, so the
writer keeps its handle until the process ends and the reader closes its own.
Neither side registers the segment with the multiprocessing resource tracker
(``track=False``, Python 3.13); on older Pythons the tracker entry is removed by
hand, so a reader exiting does not destroy a segment it does not own.
"""

from __future__ import annotations

import os
import secrets
import struct
import sys
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import Any

import numpy as np

HEADER_SIZE = 128
MAGIC = 0x53504F54  # "SPOT"
VERSION = 1

_SEQ = struct.Struct("<Q")
_META = struct.Struct("<d4I2IIIddd2I2I")  # offset 8 .. 88, see module docstring
_FIXED = struct.Struct("<QII")  # offset 88: capacity, magic, version
_META_OFF = 8
_FIXED_OFF = 88
assert _META_OFF + _META.size == _FIXED_OFF
assert _FIXED_OFF + _FIXED.size <= HEADER_SIZE

_FLAG_SPOT = 1
_FLAG_SPOT_OK = 2
#: Attempts a reader makes to get a consistent copy before giving up.
_READ_TRIES = 64


@dataclass
class LiveFrame:
    """One frame read from the buffer (a private copy)."""

    image: np.ndarray
    """2-D uint8 or uint16, ROI-sized."""
    seq: int
    """Frame number, counting from 1; increases with every written frame."""
    t: float
    roi: tuple[int, int, int, int]
    """``(x, y, w, h)`` of the image in sensor pixels."""
    sensor: tuple[int, int]
    """``(W, H)`` of the full sensor."""
    bits: int
    spot: tuple[float, float] | None
    """Fitted spot ``(x, y)`` in sensor pixels, or None."""
    spot_ok: bool
    saturated_frac: float
    max_value: int


def _open(name: str, *, create: bool, size: int = 0) -> shared_memory.SharedMemory:
    """Open a segment without the resource tracker owning it."""
    if sys.version_info >= (3, 13):
        return shared_memory.SharedMemory(
            name=name, create=create, size=size, track=False
        )
    shm = shared_memory.SharedMemory(name=name, create=create, size=size)
    if os.name == "posix":
        # Python < 3.13 registers every open with the tracker, so a reader that
        # exits would unlink the segment under its owner. Drop the entry; the
        # name is removed with _posix_unlink (which does not unregister).
        try:
            from multiprocessing import resource_tracker

            resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - tracker internals differ
            pass
    return shm


def _posix_unlink(shm: shared_memory.SharedMemory) -> None:
    """Remove the name of *shm* without touching the resource tracker."""
    if sys.version_info >= (3, 13):
        shm.unlink()  # track=False: no tracker call
        return
    import _posixshmem  # type: ignore[import-not-found]

    _posixshmem.shm_unlink(shm._name)  # type: ignore[attr-defined]


def new_name() -> str:
    """A short unique segment name valid on POSIX (<= 31 chars) and Windows."""
    return f"spotlive_{os.getpid() % 100000}_{secrets.token_hex(4)}"


class FrameBuffer:
    """The seqlock frame slot. Build with :meth:`create` or :meth:`attach`."""

    def __init__(
        self, shm: shared_memory.SharedMemory, capacity: int, *, owner: bool
    ) -> None:
        self._shm: shared_memory.SharedMemory | None = shm
        self._buf: memoryview | None = shm.buf
        self._capacity = int(capacity)
        self._owner = owner
        self._seq = 0
        self._name = shm.name

    # -- construction ------------------------------------------------------

    @classmethod
    def create(cls, max_bytes: int, name: str | None = None) -> FrameBuffer:
        """Create the segment (writer side). *max_bytes* is the pixel capacity."""
        max_bytes = int(max_bytes)
        if max_bytes <= 0:
            raise ValueError("max_bytes must be positive")
        shm = _open(name or new_name(), create=True, size=HEADER_SIZE + max_bytes)
        fb = cls(shm, max_bytes, owner=True)
        assert fb._buf is not None
        fb._buf[:HEADER_SIZE] = bytes(HEADER_SIZE)
        _FIXED.pack_into(fb._buf, _FIXED_OFF, max_bytes, MAGIC, VERSION)
        return fb

    @classmethod
    def attach(cls, name: str) -> FrameBuffer:
        """Attach to an existing segment (reader side, never tracked)."""
        shm = _open(name, create=False)
        try:
            capacity, magic, version = _FIXED.unpack_from(shm.buf, _FIXED_OFF)
            if magic != MAGIC or version != VERSION:
                raise ValueError(f"{name!r} is not a spot frame buffer")
            if HEADER_SIZE + capacity > shm.size:
                raise ValueError(f"{name!r} is smaller than its header says")
        except BaseException:
            shm.close()
            raise
        return cls(shm, capacity, owner=False)

    # -- properties --------------------------------------------------------

    @property
    def name(self) -> str:
        """The segment name to pass to :meth:`attach`."""
        return self._name

    @property
    def capacity(self) -> int:
        """Pixel bytes the buffer holds."""
        return self._capacity

    @property
    def closed(self) -> bool:
        """True after :meth:`close`."""
        return self._buf is None

    # -- writer ------------------------------------------------------------

    def write(
        self,
        image: np.ndarray,
        *,
        t: float,
        roi: tuple[int, int, int, int],
        sensor: tuple[int, int],
        bits: int,
        spot: tuple[float, float] | None = None,
        spot_ok: bool = False,
        saturated_frac: float = 0.0,
        max_value: int = 0,
    ) -> int:
        """Publish *image* and its metadata; return the new frame number.

        Raises ``ValueError`` if the image does not fit or is not uint8/uint16.
        """
        buf = self._buf
        if buf is None:
            raise ValueError("frame buffer is closed")
        arr = np.ascontiguousarray(image)
        if arr.ndim != 2 or arr.dtype not in (np.uint8, np.uint16):
            raise ValueError(
                f"need a 2-D uint8/uint16 image, got {arr.dtype} {arr.shape}"
            )
        nbytes = arr.nbytes
        if nbytes > self._capacity:
            raise ValueError(
                f"frame of {nbytes} bytes does not fit in the "
                f"{self._capacity}-byte buffer"
            )
        rows, cols = arr.shape
        x, y, w, h = (int(v) for v in roi)
        flags = (_FLAG_SPOT if spot is not None else 0) | (
            _FLAG_SPOT_OK if spot_ok and spot is not None else 0
        )
        sx, sy = (float(spot[0]), float(spot[1])) if spot is not None else (0.0, 0.0)

        seq = self._seq
        _SEQ.pack_into(buf, 0, seq + 1)  # odd: writing
        _META.pack_into(
            buf,
            _META_OFF,
            float(t),
            x,
            y,
            w,
            h,
            int(sensor[0]),
            int(sensor[1]),
            int(bits),
            flags,
            sx,
            sy,
            float(saturated_frac),
            int(max_value),
            arr.itemsize,
            rows,
            cols,
        )
        dst = np.frombuffer(buf, dtype=arr.dtype, count=arr.size, offset=HEADER_SIZE)
        dst[:] = arr.reshape(-1)
        del dst  # no exported pointer may outlive the call (Windows close)
        self._seq = seq + 2
        _SEQ.pack_into(buf, 0, self._seq)  # even: published
        return self._seq // 2

    # -- reader ------------------------------------------------------------

    def read(self, last_seq: int | None = None) -> LiveFrame | None:
        """Return a copy of the newest frame, or None if nothing newer.

        *last_seq* is the ``seq`` of the last frame the caller already has.
        Also None if no frame was written yet or the writer never left the slot
        long enough for a consistent copy.
        """
        buf = self._buf
        if buf is None:
            return None
        for _ in range(_READ_TRIES):
            (s1,) = _SEQ.unpack_from(buf, 0)
            if s1 == 0:
                return None  # nothing written yet
            if s1 & 1:
                continue  # writer inside the slot
            if last_seq is not None and s1 // 2 <= last_seq:
                return None
            (
                t, x, y, w, h, sw, sh, bits, flags, sx, sy, sat, vmax,
                itemsize, rows, cols,
            ) = _META.unpack_from(buf, _META_OFF)  # fmt: skip
            nbytes = rows * cols * itemsize
            if itemsize not in (1, 2) or nbytes > self._capacity:
                continue  # torn header; the seq check below would also catch it
            dtype = np.uint8 if itemsize == 1 else np.uint16
            src = np.frombuffer(buf, dtype=dtype, count=rows * cols, offset=HEADER_SIZE)
            image = src.reshape(rows, cols).copy()
            del src
            (s2,) = _SEQ.unpack_from(buf, 0)
            if s2 != s1:
                continue  # overwritten while copying
            return LiveFrame(
                image=image,
                seq=s1 // 2,
                t=t,
                roi=(x, y, w, h),
                sensor=(sw, sh),
                bits=bits,
                spot=(sx, sy) if flags & _FLAG_SPOT else None,
                spot_ok=bool(flags & _FLAG_SPOT_OK),
                saturated_frac=sat,
                max_value=vmax,
            )
        return None

    # -- lifetime ----------------------------------------------------------

    def close(self) -> None:
        """Release this handle. Safe to call twice."""
        shm, self._shm = self._shm, None
        self._buf = None
        if shm is None:
            return
        try:
            shm.close()
        except BufferError:  # pragma: no cover - a stray view; the OS reclaims it
            pass
        except OSError:  # pragma: no cover
            pass

    def unlink(self) -> None:
        """Remove the segment's name (POSIX); a no-op on Windows.

        Call after the last reader may attach. The memory itself lives until
        every mapping is closed, so frames in flight stay valid.
        """
        if os.name == "nt":
            return  # the segment dies with its last handle
        shm = self._shm
        if shm is None:
            # Closed already: unlink by name.
            try:
                tmp = _open(self._name, create=False)
            except FileNotFoundError:
                return
            try:
                _posix_unlink(tmp)
            except FileNotFoundError:
                pass
            finally:
                tmp.close()
            return
        try:
            _posix_unlink(shm)
        except FileNotFoundError:
            pass

    def __enter__(self) -> FrameBuffer:
        """Return self."""
        return self

    def __exit__(self, *exc: Any) -> None:
        """Close (and unlink, for the creator)."""
        self.close()
        if self._owner:
            self.unlink()


def unlink_by_name(name: str) -> None:
    """Best-effort removal of a segment whose owner died (POSIX); no-op on Windows."""
    if os.name == "nt":
        return
    try:
        shm = _open(name, create=False)
    except (FileNotFoundError, OSError):
        return
    try:
        _posix_unlink(shm)
    except FileNotFoundError:
        pass
    finally:
        try:
            shm.close()
        except Exception:
            pass


__all__ = ["HEADER_SIZE", "FrameBuffer", "LiveFrame", "new_name", "unlink_by_name"]
