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
    ``getNumberOfCameraChannels() > 1``) **or** multiple ROIs are active
    (``isMultiROIEnabled()``) and the chosen output is a save path, the output is
    wrapped in a :class:`MultiCameraHandler`.  This writes each physical camera — and,
    for multi-ROI readout, each ROI — to its own file.  Plain single-camera,
    single-ROI acquisitions behave exactly as the base widget.
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
        if isinstance(output, str | Path) and self._needs_multi_camera_handler():
            output = MultiCameraHandler(output, mmcore=self._mmc)
        sequence = self.value()
        self._mmc.run_mda(sequence, output=output)

    def _needs_multi_camera_handler(self) -> bool:
        """Whether output should be routed through :class:`MultiCameraHandler`.

        True for multi-camera devices, or when the camera is reading out multiple
        ROIs (so each ROI is saved as its own image).
        """
        if self._mmc.getNumberOfCameraChannels() > 1:
            return True
        try:
            return bool(self._mmc.isMultiROIEnabled())
        except Exception:  # pragma: no cover - camera w/o multi-ROI support
            return False
