"""GUI-specific MDA widget that routes multi-camera output to per-camera files."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from pymmcore_widgets import MDAWidget

from pymmcore_gui._multi_camera_handler import MultiCameraHandler

if TYPE_CHECKING:
    from collections.abc import Sequence

    from pymmcore_plus.mda import SupportsFrameReady


class GuiMDAWidget(MDAWidget):
    """:class:`pymmcore_widgets.MDAWidget` with multi-camera aware saving.

    When the active camera is a *Multi Camera* device (i.e.
    ``getNumberOfCameraChannels() > 1``) and the chosen output is a save path, the
    output is wrapped in a :class:`MultiCameraHandler` so that each physical camera
    is written to its own file.  Single-camera acquisitions behave exactly as the
    base widget.
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
        if (
            isinstance(output, (str, Path))
            and self._mmc.getNumberOfCameraChannels() > 1
        ):
            output = MultiCameraHandler(output, mmcore=self._mmc)
        sequence = self.value()
        self._mmc.run_mda(sequence, output=output)
