"""GUI-specific MDA widget that routes multi-camera output to per-camera files."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from pymmcore_widgets import MDAWidget

from pymmcore_gui._multi_camera_handler import (
    MultiCameraHandler,
    physical_camera_labels,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui._spectral_channel_handler import SpectralChannelHandler

if TYPE_CHECKING:
    from collections.abc import Sequence

    from pymmcore_plus.mda import SupportsFrameReady


class GuiMDAWidget(MDAWidget):
    """:class:`pymmcore_widgets.MDAWidget` with multi-camera aware saving.

    When the active camera is a *Multi Camera* device (i.e.
    ``getNumberOfCameraChannels() > 1``) and the chosen output is a save path,
    the output is wrapped in a handler so that each physical camera is written
    to its own file. Single-camera acquisitions behave exactly as the base
    widget.

    If the image-splitter spectral-channel feature is enabled (see
    :class:`~pymmcore_gui._settings.SpectralChannelSettingsV1`) and at least
    one configured channel is ready (enabled, has a drawn rect, and targets a
    camera that's actually present), a :class:`SpectralChannelHandler` is used
    instead so each splitter sub-region is cropped and saved to its own file.
    Otherwise the existing :class:`MultiCameraHandler` behavior is unchanged.
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
        if isinstance(output, str | Path):
            spectral = SettingsV1.instance().spectral
            active_cams = set(physical_camera_labels(self._mmc))
            ready = [
                c for c in spectral.channels if c.is_ready and c.camera in active_cams
            ]
            if spectral.enabled and ready:
                output = SpectralChannelHandler(
                    output,
                    ready,
                    spectral.laser_config_group,
                    spectral.all_lasers_preset,
                    mmcore=self._mmc,
                )
            elif self._mmc.getNumberOfCameraChannels() > 1:
                output = MultiCameraHandler(output, mmcore=self._mmc)
        sequence = self.value()
        self._mmc.run_mda(sequence, output=output)
