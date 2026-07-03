"""GUI-specific MDA widget that routes multi-camera output to per-camera files."""

from __future__ import annotations

import re
from pathlib import Path
from typing import TYPE_CHECKING

from pymmcore_widgets import MDAWidget
from pymmcore_widgets.useq_widgets import PYMMCW_METADATA_KEY

from pymmcore_gui._multi_camera_handler import (
    MultiCameraHandler,
    per_camera_path,
    physical_camera_labels,
)
from pymmcore_gui._qt.QtWidgets import QMessageBox
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui._spectral_channel_handler import (
    SpectralChannelHandler,
    _strip_known_suffix,
    channel_output_path,
    channels_for_sequence,
)

if TYPE_CHECKING:
    from collections.abc import Sequence

    import useq
    from pymmcore_plus.mda import SupportsFrameReady

    from pymmcore_gui._settings import (
        SpectralChannelConfig,
        SpectralChannelSettingsV1,
    )

# Trailing ``_<3+ digits>`` generation counter, as inserted by
# :meth:`GuiMDAWidget.get_next_available_path` (mirrors the base widget's format).
_GENERATION = re.compile(r"^(.*?)(?:_(\d{3,}))?$")


def _with_generation(base: Path, n: int) -> Path:
    """Return *base* with an ``_NNN`` generation counter inserted before its suffix.

    ``exp.ome.zarr`` + ``2`` -> ``exp_002.ome.zarr``. A pre-existing generation
    counter on *base* is stripped first so counters don't stack across re-runs.
    """
    text = str(base)
    stem = _strip_known_suffix(text)
    extension = text[len(stem) :]
    if match := _GENERATION.match(stem):
        stem = match.group(1)
    return Path(f"{stem}_{n:03d}{extension}")


class GuiMDAWidget(MDAWidget):
    """:class:`pymmcore_widgets.MDAWidget` with multi-camera aware saving.

    When the active camera is a *Multi Camera* device (i.e.
    ``getNumberOfCameraChannels() > 1``) and the chosen output is a save path,
    the output is wrapped in a handler so that each physical camera is written
    to its own file. Single-camera acquisitions behave exactly as the base
    widget.

    If the image-splitter spectral-channel feature is enabled (see
    :class:`~pymmcore_gui._settings.SpectralChannelSettingsV1`), the regions
    saved are derived from the laser presets the MDA actually uses (see
    :func:`~pymmcore_gui._spectral_channel_handler.channels_for_sequence`): a
    :class:`SpectralChannelHandler` crops each matching sub-region to its own
    file, in the format chosen in the MDA save widget. If no configured region
    matches the sequence's lasers, the existing :class:`MultiCameraHandler`
    behavior is unchanged (and the user is warned when regions were configured
    but none matched).
    """

    def execute_mda(
        self,
        output: (
            Path
            | str
            | SupportsFrameReady
            | Sequence[Path | str | SupportsFrameReady]
            | None
        ),
    ) -> None:
        sequence = self.value()
        if isinstance(output, str | Path):
            save = self._spectral_channels_to_save(sequence)
            if save:
                meta = sequence.metadata.get(PYMMCW_METADATA_KEY, {})
                output = SpectralChannelHandler(
                    output,
                    save,
                    SettingsV1.instance().spectral.laser_config_group,
                    SettingsV1.instance().spectral.all_lasers_preset,
                    writer_format=meta.get("format", "ome-zarr"),
                    mmcore=self._mmc,
                )
            else:
                spectral = SettingsV1.instance().spectral
                if spectral.enabled:
                    active_cams = set(physical_camera_labels(self._mmc))
                    self._warn_no_spectral_match(spectral, active_cams)
                if self._mmc.getNumberOfCameraChannels() > 1:
                    output = MultiCameraHandler(output, mmcore=self._mmc)
        self._mmc.run_mda(sequence, output=output)

    def get_next_available_path(self, requested_path: Path) -> Path:
        """Return the next available save path, aware of per-output splitting.

        When the output is split per spectral-channel or per-camera, the actual
        files written are ``<base>_<name><ext>`` (e.g. ``exp_CF647_638.ome.zarr``),
        not ``<base><ext>``. The base widget's uniquifier globs every file with
        the extension and treats any trailing ``_<digits>`` as a counter, so a
        laser-wavelength suffix like ``_638`` is misread as a counter and bumped
        (``exp`` -> ``exp_639``). Here we instead advance a generation counter on
        the base name until none of the *derived* per-output paths exist, so the
        counter reflects acquisitions rather than channel names. Falls back to the
        base behavior when the output is not split.
        """
        sequence = self.value()
        if not self._split_output_paths(requested_path, sequence):
            return super().get_next_available_path(requested_path)

        candidate = requested_path
        n = 0
        while any(
            Path(p).exists() for p in self._split_output_paths(candidate, sequence)
        ):
            n += 1
            candidate = _with_generation(requested_path, n)
        return candidate

    def _spectral_channels_to_save(
        self, sequence: useq.MDASequence
    ) -> list[SpectralChannelConfig]:
        """Return the spectral channels that saving *sequence* will split into files.

        Empty when the spectral-channel feature is disabled or no configured
        region matches the sequence's lasers.
        """
        spectral = SettingsV1.instance().spectral
        if not spectral.enabled:
            return []
        active_cams = set(physical_camera_labels(self._mmc))
        return channels_for_sequence(
            sequence,
            spectral.channels,
            spectral.laser_config_group,
            spectral.all_lasers_preset,
            active_cams,
        )

    def _split_output_paths(
        self, base: str | Path, sequence: useq.MDASequence
    ) -> list[str]:
        """Return the concrete files saving *sequence* to *base* will write.

        Mirrors the branching in :meth:`execute_mda`: per-spectral-channel paths
        when the spectral feature matches, else per-camera paths for a multi-camera
        device. Empty when the output is not split (a single, plain save path).
        """
        if save := self._spectral_channels_to_save(sequence):
            meta = sequence.metadata.get(PYMMCW_METADATA_KEY, {})
            writer_format = meta.get("format", "ome-zarr")
            return [channel_output_path(base, ch, writer_format) for ch in save]
        if self._mmc.getNumberOfCameraChannels() > 1:
            return [
                per_camera_path(base, label)
                for label in physical_camera_labels(self._mmc)
            ]
        return []

    def _warn_no_spectral_match(
        self, spectral: SpectralChannelSettingsV1, active_cams: set[str]
    ) -> None:
        """Warn if regions are configured but none matched the MDA's lasers."""
        ready = [
            c
            for c in spectral.channels
            if c.rect is not None and c.camera in active_cams
        ]
        if not ready:
            return
        presets = sorted({c.laser_preset for c in ready})
        QMessageBox.warning(
            self,
            "Spectral cropping skipped",
            "Spectral-channel cropping is enabled, but none of this MDA's "
            f"channels matched a configured region (config group "
            f"'{spectral.laser_config_group}', presets {presets}). Output will "
            "not be cropped per region. Check that the MDA's channels use that "
            "config group and matching preset names.",
        )
