"""MDA output handler that writes one file per physical camera.

The stock :class:`~pymmcore_plus.mda.MDAEngine` yields one frame per physical
camera for every :class:`~useq.MDAEvent`, but each of those frames carries the
*same* ``event.index`` — only ``meta["camera_device"]`` differs.  A single writer
would therefore overwrite one camera's frame with the next.

``MultiCameraHandler`` solves this by giving each physical camera its own writer,
created from the user-chosen output path with the camera label appended.  Each
per-camera writer only ever sees a single camera's stream, where every
``event.index`` is unique, so no frames collide.

When only one camera is present this handler is transparent: it forwards to a
single writer built directly from the original path.
"""

from __future__ import annotations

import re
from inspect import signature
from typing import TYPE_CHECKING, Any

from pymmcore_plus import CMMCorePlus
from pymmcore_plus.mda.handlers import handler_for_path

if TYPE_CHECKING:
    from pathlib import Path

    import numpy as np
    import useq
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1

# Known (possibly compound) output extensions, longest first so that
# ``.ome.tiff`` is matched before ``.tiff``.
_KNOWN_SUFFIXES = (".ome.tiff", ".ome.tif", ".ome.zarr", ".zarr", ".tiff", ".tif")


def without_cam_index(event: useq.MDAEvent) -> useq.MDAEvent:
    """Return *event* with the engine-injected ``cam`` axis removed.

    The :class:`~pymmcore_plus.mda.MDAEngine` adds a ``cam`` entry to
    ``event.index`` for every multi-camera frame so that a single store *could*
    separate cameras along a ``cam`` dimension.  Because we route each physical
    camera to its own writer / viewer (keyed by ``meta["camera_device"]``), that
    axis is redundant — and the per-camera store's dimensions come from
    ``seq.sizes``, which has no ``cam``, so leaving it in raises an
    ``IndexError``.  Strip it so each per-camera event indexes cleanly into its
    own store.  A no-op when ``cam`` is absent (e.g. single-camera events).

    Parameters
    ----------
    event : useq.MDAEvent
        The event emitted by the MDA engine.
    """
    if "cam" not in event.index:
        return event
    new_index = {k: v for k, v in event.index.items() if k != "cam"}
    return event.model_copy(update={"index": new_index})


def _sanitize(label: str) -> str:
    """Make *label* safe to embed in a filename."""
    return re.sub(r"[^\w.-]+", "_", label).strip("_") or "camera"


def per_camera_path(base: str | Path, label: str) -> str:
    """Return *base* with the sanitized camera *label* inserted before its suffix.

    Examples
    --------
    ``exp.ome.tiff`` + ``Camera-1`` -> ``exp_Camera-1.ome.tiff``
    ``exp.ome.zarr`` + ``Camera-1`` -> ``exp_Camera-1.ome.zarr``
    ``exp`` (tiff-sequence dir)      + ``Camera-1`` -> ``exp_Camera-1``
    """
    text = str(base)
    safe = _sanitize(label)
    lowered = text.lower()
    for suffix in _KNOWN_SUFFIXES:
        if lowered.endswith(suffix):
            stem = text[: -len(suffix)]
            return f"{stem}_{safe}{text[len(stem) :]}"
    # No recognized suffix -> treat as a directory (ImageSequenceWriter); create
    # sibling directories per camera.
    return f"{text.rstrip('/').rstrip(chr(92))}_{safe}"


class MultiCameraHandler:
    """Route MDA frames to one writer per physical camera, keyed by camera label."""

    def __init__(
        self, output: str | Path, *, mmcore: CMMCorePlus | None = None
    ) -> None:
        self._output = output
        self._mmc = mmcore or CMMCorePlus.instance()
        # camera label -> writer
        self._writers: dict[str, Any] = {}
        self._started: set[str] = set()
        self._sequence: useq.MDASequence | None = None
        self._summary_meta: SummaryMetaV1 | dict = {}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _physical_camera_labels(self) -> list[str]:
        """Return physical camera labels matching the engine's metadata."""
        n = self._mmc.getNumberOfCameraChannels()
        if n <= 1:
            return [self._mmc.getCameraDevice()]
        return [self._mmc.getPhysicalCameraDevice(i) for i in range(n)]

    def _get_writer(self, label: str) -> Any:
        """Return (creating + starting if needed) the writer for *label*."""
        if label not in self._writers:
            path = per_camera_path(self._output, label)
            self._writers[label] = handler_for_path(path)
        writer = self._writers[label]
        if label not in self._started:
            self._call_sequence_started(writer)
            self._started.add(label)
        return writer

    def _call_sequence_started(self, writer: Any) -> None:
        """Call ``writer.sequenceStarted`` with the arity it supports."""
        method = getattr(writer, "sequenceStarted", None)
        if method is None:
            return
        try:
            n_params = len(signature(method).parameters)
        except (TypeError, ValueError):  # pragma: no cover - builtins
            n_params = 2
        if n_params >= 2:
            method(self._sequence, self._summary_meta)
        else:
            method(self._sequence)

    # ------------------------------------------------------------------
    # MDA signal handlers (connected by name via mda_listeners_connected)
    # ------------------------------------------------------------------

    def sequenceStarted(
        self, seq: useq.MDASequence, meta: SummaryMetaV1 | dict | None = None
    ) -> None:
        self._sequence = seq
        self._summary_meta = meta or {}
        self._writers.clear()
        self._started.clear()
        # Eagerly create + start a writer for every known physical camera so the
        # files exist even if a camera produces no frames.
        for label in self._physical_camera_labels():
            self._get_writer(label)

    def frameReady(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        label = meta.get("camera_device") or self._mmc.getCameraDevice()
        writer = self._get_writer(label)
        # Each writer only sees one camera, so the engine's ``cam`` axis is
        # redundant here and absent from the writer's store dimensions.
        writer.frameReady(frame, without_cam_index(event), meta)

    def sequenceFinished(self, seq: useq.MDASequence) -> None:
        for writer in self._writers.values():
            if (method := getattr(writer, "sequenceFinished", None)) is not None:
                method(seq)
        self._started.clear()
