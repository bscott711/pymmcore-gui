"""MDA output handlers, vendored from ``pymmcore_plus.mda.handlers`` (0.18.1).

That module is deprecated upstream ("moving to ome-writers ... please vendor
them into your own codebase" -- see its own ``FutureWarning``), and
``ome_writers``'s replacement API isn't usable yet in the version this project
depends on. This is an unmodified copy of the four writer classes plus the
``handler_for_path`` dispatcher, kept here until ``ome_writers`` stabilizes.
"""

from __future__ import annotations

from pathlib import Path

from ._img_sequence_writer import ImageSequenceWriter
from ._ome_tiff_writer import OMETiffWriter
from ._ome_zarr_writer import OMEZarrWriter
from ._tensorstore_handler import TensorStoreHandler

__all__ = [
    "ImageSequenceWriter",
    "OMETiffWriter",
    "OMEZarrWriter",
    "TensorStoreHandler",
    "handler_for_path",
]


def handler_for_path(path: str | Path) -> object:
    """Convert a string or Path into a handler object.

    This method picks from the built-in handlers based on the extension of the path.
    """
    if str(path).rstrip("/").rstrip(":").lower() == "memory":
        return TensorStoreHandler(kvstore="memory://")

    path = str(Path(path).expanduser().resolve())
    if path.endswith(".zarr"):
        return OMEZarrWriter(path)

    if path.endswith((".tiff", ".tif")):
        return OMETiffWriter(path)

    # FIXME: ugly hack for the moment to represent a non-existent directory
    # there are many features that ImageSequenceWriter supports, and it's unclear
    # how to infer them all from a single string.
    if not (Path(path).suffix or Path(path).exists()):
        return ImageSequenceWriter(path)

    raise ValueError(f"Could not infer a writer handler for path: '{path}'")
