"""GUI-specific MDA widget that routes multi-camera output to per-camera files."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from pymmcore_widgets import MDAWidget
from pymmcore_widgets.useq_widgets import PYMMCW_METADATA_KEY

from pymmcore_gui._multi_camera_handler import (
    MultiCameraHandler,
    physical_camera_labels,
)
from pymmcore_gui._qt.QtWidgets import QMessageBox
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui._spectral_channel_handler import (
    SpectralChannelHandler,
    channels_for_sequence,
)

if TYPE_CHECKING:
    from collections.abc import Sequence

    from pymmcore_plus.mda import SupportsFrameReady

    from pymmcore_gui._settings import SpectralChannelSettingsV1


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
            spectral = SettingsV1.instance().spectral
            active_cams = set(physical_camera_labels(self._mmc))
            save = (
                channels_for_sequence(
                    sequence,
                    spectral.channels,
                    spectral.laser_config_group,
                    spectral.all_lasers_preset,
                    active_cams,
                )
                if spectral.enabled
                else []
            )
            if save:
                meta = sequence.metadata.get(PYMMCW_METADATA_KEY, {})
                output = SpectralChannelHandler(
                    output,
                    save,
                    spectral.laser_config_group,
                    spectral.all_lasers_preset,
                    writer_format=meta.get("format", "ome-zarr"),
                    mmcore=self._mmc,
                )
            else:
                if spectral.enabled:
                    self._warn_no_spectral_match(spectral, active_cams)
                if self._mmc.getNumberOfCameraChannels() > 1:
                    output = MultiCameraHandler(output, mmcore=self._mmc)
        self._mmc.run_mda(sequence, output=output)

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
