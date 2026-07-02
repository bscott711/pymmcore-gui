"""MDA output handler that crops each camera frame into its splitter sub-regions.

Each physical camera has an image splitter in front of it that projects two
fixed sub-regions ("spectral channels") onto the sensor. This handler crops
each frame into its configured sub-regions and routes each spectral channel to
its own writer/file, selecting which channels are active for a given frame
based on the laser preset used for that MDA event (see
:class:`~pymmcore_gui._settings.SpectralChannelConfig`).
"""

from __future__ import annotations

from inspect import signature
from typing import TYPE_CHECKING, Any

import numpy as np
from pymmcore_plus import CMMCorePlus
from pymmcore_plus.mda.handlers import handler_for_path

from pymmcore_gui._multi_camera_handler import (
    _KNOWN_SUFFIXES,
    _sanitize,
    without_cam_index,
)

if TYPE_CHECKING:
    from pathlib import Path

    import useq
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1

    from pymmcore_gui._settings import SpectralChannelConfig

# Output suffix implied by each writer format; ``handler_for_path`` dispatches
# to the concrete writer class based on this suffix.
_FORMAT_SUFFIX = {
    "ome-zarr": ".ome.zarr",
    "ome-tiff": ".ome.tiff",
    "tiff-sequence": "",
}


def _strip_known_suffix(text: str) -> str:
    """Remove any known writer-output suffix from *text*, if present."""
    lowered = text.lower()
    for suffix in _KNOWN_SUFFIXES:
        if lowered.endswith(suffix):
            return text[: -len(suffix)]
    return text.rstrip("/").rstrip("\\")


def channel_output_path(base: str | Path, channel: SpectralChannelConfig) -> str:
    """Return *base* with the sanitized channel name and format suffix appended.

    Examples
    --------
    ``exp.ome.zarr`` + ``GFP_488`` (ome-zarr) -> ``exp_GFP_488.ome.zarr``
    ``exp`` + ``GFP_488`` (ome-tiff) -> ``exp_GFP_488.ome.tiff``
    """
    stem = _strip_known_suffix(str(base))
    return f"{stem}_{_sanitize(channel.name)}{_FORMAT_SUFFIX[channel.writer_format]}"


class SpectralChannelHandler:
    """Route MDA frames to one cropped writer per configured spectral channel.

    Parameters
    ----------
    output : str | Path
        Base output path; each channel's file is derived from this path (see
        :func:`channel_output_path`).
    channels : list[SpectralChannelConfig]
        The channels to save. Callers should pre-filter this list to
        ``is_ready`` channels whose ``camera`` is actually present.
    laser_group : str
        The Micro-Manager config group name used for laser presets (e.g.
        ``"Lasers"``). Only ``event.channel`` entries in this group are used
        to select active channels; anything else falls back to saving all
        configured channels for that event.
    all_lasers_preset : str
        The config preset name that fires all lasers simultaneously (e.g.
        ``"AllLasers"``); when active, all configured channels are saved.
    mmcore : CMMCorePlus | None
        The core instance to use. Defaults to the global singleton.
    """

    def __init__(
        self,
        output: str | Path,
        channels: list[SpectralChannelConfig],
        laser_group: str,
        all_lasers_preset: str,
        *,
        mmcore: CMMCorePlus | None = None,
    ) -> None:
        self._output = output
        self._channels = list(channels)
        self._laser_group = laser_group
        self._all_lasers_preset = all_lasers_preset
        self._mmc = mmcore or CMMCorePlus.instance()
        # channel name -> writer
        self._writers: dict[str, Any] = {}
        self._started: set[str] = set()
        self._sequence: useq.MDASequence | None = None
        # The sequence handed to each per-channel writer, with the multi-preset
        # channel axis collapsed (see _active_channels_for_event / sequenceStarted).
        self._writer_seq: useq.MDASequence | None = None
        self._summary_meta: SummaryMetaV1 | dict = {}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _active_channels_for_event(
        self, event: useq.MDAEvent
    ) -> list[SpectralChannelConfig]:
        """Return the configured channels that are "lit" for *event*.

        Determined from the laser preset active for this event
        (``event.channel.config``), matched against each channel's configured
        ``laser_preset``. ``AllLasers`` (or a missing/foreign channel group)
        activates every configured channel.
        """
        ch = event.channel
        in_group = ch is not None and ch.group == self._laser_group
        preset = ch.config if (ch is not None and in_group) else None
        if preset is None or preset == self._all_lasers_preset:
            return list(self._channels)
        return [c for c in self._channels if c.laser_preset == preset]

    def _get_writer(self, channel: SpectralChannelConfig) -> Any:
        """Return (creating + starting if needed) the writer for *channel*."""
        if channel.name not in self._writers:
            path = channel_output_path(self._output, channel)
            self._writers[channel.name] = handler_for_path(path)
        writer = self._writers[channel.name]
        if channel.name not in self._started:
            self._call_sequence_started(writer)
            self._started.add(channel.name)
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
            method(self._writer_seq, self._summary_meta)
        else:
            method(self._writer_seq)

    # ------------------------------------------------------------------
    # MDA signal handlers (connected by name via mda_listeners_connected)
    # ------------------------------------------------------------------

    def sequenceStarted(
        self, seq: useq.MDASequence, meta: SummaryMetaV1 | dict | None = None
    ) -> None:
        self._sequence = seq
        # Each spectral channel gets its own single-channel store: collapse the
        # sequence's channel axis so a writer fed only one preset's frames isn't
        # shaped to expect frames from every preset (which would leave empty
        # slots — see _spectral_channel_handler module docs / project plan).
        self._writer_seq = seq.model_copy(update={"channels": ()})
        self._summary_meta = meta or {}
        self._writers.clear()
        self._started.clear()
        for channel in self._channels:
            self._get_writer(channel)

    def frameReady(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        label = meta.get("camera_device") or self._mmc.getCameraDevice()
        clean_event = without_cam_index(event)
        for channel in self._active_channels_for_event(event):
            if channel.camera != label or channel.rect is None:
                continue
            x, y, w, h = channel.rect
            crop = np.ascontiguousarray(frame[y : y + h, x : x + w])
            self._get_writer(channel).frameReady(crop, clean_event, meta)

    def sequenceFinished(self, seq: useq.MDASequence) -> None:
        for writer in self._writers.values():
            if (method := getattr(writer, "sequenceFinished", None)) is not None:
                method(seq)
        self._started.clear()
