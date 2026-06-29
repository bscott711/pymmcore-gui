"""Fast pygfx-backed live preview with optional per-camera "managed" mode.

This is the high-throughput renderer used for the live / snap previews.  It
reuses :class:`PygfxImagePreview` (direct GPU texture upload) and adds the same
"managed" behaviour that :class:`NDVPreview` had, so the
:class:`~pymmcore_gui._ndv_viewers.NDVViewersManager` can drive one preview per
physical camera.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_gui.widgets.image_preview._pygfx_image import PygfxImagePreview

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from PyQt6.QtWidgets import QWidget


class PygfxPreview(PygfxImagePreview):
    """A :class:`PygfxImagePreview` that can be dedicated to one physical camera.

    When *camera_label* is set the preview operates in *managed* mode: it does
    NOT independently respond to ``imageSnapped`` or streaming-start events — the
    ``NDVViewersManager`` dispatches data directly via :meth:`append`.
    """

    def __init__(
        self,
        mmcore: CMMCorePlus,
        parent: QWidget | None = None,
        *,
        use_with_mda: bool = False,
        camera_label: str | None = None,
    ) -> None:
        # Set BEFORE super().__init__() because the base ctor calls self.attach().
        self.camera_label = camera_label
        super().__init__(parent, mmcore, use_with_mda=use_with_mda)

    # ------------------------------------------------------------------
    # Event connection — selective for managed (per-camera) mode
    # ------------------------------------------------------------------

    def attach(self, core: CMMCorePlus) -> None:
        if getattr(self, "camera_label", None) is None:
            super().attach(core)
            return

        if self._mmc is not None:
            self.detach()

        ev = core.events
        # Stop/restart timer + reconfigure events that are safe to handle alone.
        ev.sequenceAcquisitionStopped.connect(self._on_streaming_stop)
        ev.exposureChanged.connect(self._on_exposure_changed)
        ev.systemConfigurationLoaded.connect(self._on_system_config_loaded)
        ev.roiSet.connect(self._on_roi_set)
        ev.propertyChanged.connect(self._on_property_changed)
        core.mda.events.sequenceStarted.connect(
            lambda: setattr(self, "_is_mda_running", True)
        )
        core.mda.events.sequenceFinished.connect(
            lambda: setattr(self, "_is_mda_running", False)
        )
        self._mmc = core
        # NOT connected in managed mode: imageSnapped / streaming-start — the
        # manager owns those.

    def detach(self) -> None:
        if getattr(self, "camera_label", None) is None:
            super().detach()
            return
        if self._mmc is None:
            return
        ev, self._mmc = self._mmc.events, None
        for sig, slot in [
            (ev.sequenceAcquisitionStopped, self._on_streaming_stop),
            (ev.exposureChanged, self._on_exposure_changed),
            (ev.systemConfigurationLoaded, self._on_system_config_loaded),
            (ev.roiSet, self._on_roi_set),
            (ev.propertyChanged, self._on_property_changed),
        ]:
            with suppress(Exception):
                sig.disconnect(slot)

    # ------------------------------------------------------------------
    # Shape/dtype helpers (used by the manager's invalidation logic)
    # ------------------------------------------------------------------

    def _get_core_dtype_shape(self) -> tuple[str, tuple[int, ...]] | None:
        if (core := self._mmc) is None:
            return None
        if not (bits := core.getImageBitDepth()):
            return None
        return (f"uint{bits}", (core.getImageHeight(), core.getImageWidth()))

    @property
    def dtype_shape(self) -> tuple[str, tuple[int, ...]] | None:
        # ``append`` already recreates the GPU texture when the frame shape
        # changes, so report the current core shape to disable the manager's
        # destroy/recreate invalidation (no churn on e.g. exposure changes).
        return self._get_core_dtype_shape()
