"""System commit-charge probe used to keep a camera worker out of a native crash.

pymmcore's SWIG ``%typemap(out) void*`` (the one behind every allocating image
getter: ``popNextImage*``, ``getLastImage*``, ``getImage``, ...) allocates the
returned numpy array with ``PyArray_New`` and dereferences the result with no
NULL check. When the machine's commit charge is at the commit limit that
allocation fails and the worker dies with an access violation instead of a
``MemoryError`` (seen on the rig 2026-10-23, both workers at the same instant).
Upstream is not ours to patch, so the worker asks :func:`available_commit_bytes`
first and refuses to call such a getter without :func:`required_headroom_bytes`
to spare.

Pure stdlib (ctypes), no Qt, never raises.
"""

from __future__ import annotations

import ctypes
import sys

_MIB = 1024**2
_GIB = 1024**3

#: Never run a camera worker's allocating getter with less commit than this
#: left, whatever the frame size.
MIN_COMMIT_HEADROOM_BYTES = 2 * _GIB

#: ... and never with fewer than this many frames' worth left.
HEADROOM_FRAME_MULTIPLE = 8


def required_headroom_bytes(frame_bytes: int) -> int:
    """Commit that must be free before an image getter may allocate one frame."""
    return max(MIN_COMMIT_HEADROOM_BYTES, HEADROOM_FRAME_MULTIPLE * max(0, frame_bytes))


class CommitHeadroomError(MemoryError):
    """System commit (RAM + page file) is too close to exhausted to take a frame.

    Raised by the camera worker instead of letting pymmcore's image getter crash
    the process; ``exc_type`` of the resulting ``ErrorMsg`` is this class' name.
    """

    def __init__(self, free_bytes: int | None, required_bytes: int) -> None:
        self.free_bytes = free_bytes
        self.required_bytes = required_bytes
        free = "unknown" if free_bytes is None else f"{free_bytes // _MIB}"
        super().__init__(
            "System commit (RAM + page file) is nearly exhausted: "
            f"{free} MiB free, {required_bytes // _MIB} MiB needed. "
            "The camera acquisition was stopped to avoid a native crash in "
            "pymmcore. Close MDA viewer tabs or other memory-heavy programs."
        )


class _MemoryStatusEx(ctypes.Structure):
    _fields_ = (
        ("dwLength", ctypes.c_uint32),
        ("dwMemoryLoad", ctypes.c_uint32),
        ("ullTotalPhys", ctypes.c_uint64),
        ("ullAvailPhys", ctypes.c_uint64),
        ("ullTotalPageFile", ctypes.c_uint64),
        ("ullAvailPageFile", ctypes.c_uint64),
        ("ullTotalVirtual", ctypes.c_uint64),
        ("ullAvailVirtual", ctypes.c_uint64),
        ("ullAvailExtendedVirtual", ctypes.c_uint64),
    )


def available_commit_bytes() -> int | None:
    """Commit this process can still allocate, or ``None`` if unknown.

    On Windows this is ``GlobalMemoryStatusEx().ullAvailPageFile``: commit limit
    minus commit charge (it honours a job object's limits). Elsewhere the guard
    is disabled (``None``): with Linux-style overcommit the number says nothing
    about whether an allocation will fail. Never raises.
    """
    if sys.platform != "win32":
        return None
    try:
        status = _MemoryStatusEx()
        status.dwLength = ctypes.sizeof(_MemoryStatusEx)
        kernel32 = ctypes.windll.kernel32
        if not kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return None
        return int(status.ullAvailPageFile)
    except Exception:
        return None


def process_private_bytes() -> int | None:
    """Private (committed) bytes of the current process, or ``None``.

    Windows only; needs ``psutil`` (already a transitive dependency). Never raises.
    """
    if sys.platform != "win32":
        return None
    try:
        import psutil

        return int(psutil.Process().memory_info().private)
    except Exception:
        return None


def format_gib(value: int | None) -> str:
    """``"12.3 GiB"`` for a byte count, ``"n/a"`` for ``None``."""
    return "n/a" if value is None else f"{value / _GIB:.1f} GiB"
