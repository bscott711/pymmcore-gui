from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

import ndv
import numpy as np
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from pymmcore_gui.widgets.image_preview._preview_base import ImagePreviewBase

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus


class NDVPreview(ImagePreviewBase):
    """Live image preview backed by an ndv.ArrayViewer.

    Parameters
    ----------
    mmcore :
        The active CMMCorePlus instance.
    parent :
        Optional Qt parent widget.
    use_with_mda :
        If False (default) the preview is suppressed while an MDA is running.
    camera_label :
        Optional label identifying the physical camera this preview is dedicated
        to (e.g. ``"Camera-1"``).  When set the preview operates in *managed*
        mode: it does NOT independently respond to ``imageSnapped`` or streaming-
        start events — the ``NDVViewersManager`` dispatches data directly by
        calling ``append()``.  When *None* the preview handles whatever camera is
        currently active (composite / legacy path).
    """

    def __init__(
        self,
        mmcore: CMMCorePlus,
        parent: QWidget | None = None,
        *,
        use_with_mda: bool = False,
        camera_label: str | None = None,
    ):
        # Set camera_label BEFORE super().__init__() because the base class
        # calls self.attach() which inspects this attribute.
        self.camera_label = camera_label

        super().__init__(parent, mmcore, use_with_mda=use_with_mda)

        self._viewer = ndv.ArrayViewer()
        self._buffer: np.ndarray | None = None

        qwdg = self._viewer.widget()
        qwdg.setParent(self)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(qwdg)

    # ------------------------------------------------------------------
    # Event connection — selective for managed (per-camera) mode
    # ------------------------------------------------------------------

    def attach(self, core: CMMCorePlus) -> None:
        """Attach to core events.

        In *managed* mode (``camera_label`` is not None) we skip
        ``imageSnapped`` and streaming-start signals — the
        ``NDVViewersManager`` owns those.  We still connect stop/exposure/ROI
        events that are safe to handle independently.
        """
        if self.camera_label is None:
            # Composite / unmanaged mode — full event attachment via base class.
            super().attach(core)
            return

        # Managed per-camera mode: disconnect from previous core if any.
        if self._mmc is not None:
            self.detach()

        ev = core.events
        # Timer stop/restart when streaming stops or exposure changes.
        ev.sequenceAcquisitionStopped.connect(self._on_streaming_stop)
        ev.exposureChanged.connect(self._on_exposure_changed)
        # Config / ROI changes still apply.
        ev.systemConfigurationLoaded.connect(self._on_system_config_loaded)
        ev.roiSet.connect(self._on_roi_set)
        ev.propertyChanged.connect(self._on_property_changed)
        # Track MDA state so use_with_mda can be respected.
        core.mda.events.sequenceStarted.connect(
            lambda: setattr(self, "_is_mda_running", True)
        )
        core.mda.events.sequenceFinished.connect(
            lambda: setattr(self, "_is_mda_running", False)
        )
        self._mmc = core

        # NOT connected in managed mode:
        #   imageSnapped              → manager calls append() directly
        #   continuousSequenceAcquisitionStarted → manager calls _on_streaming_start()
        #   sequenceAcquisitionStarted           → same

    def detach(self) -> None:
        """Detach from core events, handling managed and unmanaged modes."""
        if self.camera_label is None:
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
    # Data ingestion
    # ------------------------------------------------------------------

    def append(self, data: np.ndarray) -> None:
        if (
            self._buffer is None
            or self._buffer.shape != data.shape
            or self._buffer.dtype != data.dtype
        ):
            # Shape/dtype changed (or first frame): (re)build the viewer.  This
            # assigns ``self._buffer`` and ``self._viewer.data`` and runs the full
            # dimension/slider synchronization.
            self._setup_viewer(data)
            return

        # Fast path: the dimensions are unchanged, so mutate the existing buffer in
        # place and ask the viewer to redraw only the current slice.  Reassigning
        # ``self._viewer.data`` here would trigger a full view re-synchronization
        # every frame (rebuilding sliders, dims, etc.), which is the main cause of
        # sluggish live updates.  Emitting ``data_changed`` re-renders in place and
        # lets ndv coalesce paints via the normal Qt event loop — no forced
        # ``QApplication.processEvents()`` (which caused re-entrancy under load).
        np.copyto(self._buffer, data)
        if (wrapper := self._viewer.data_wrapper) is not None:
            wrapper.data_changed.emit()
        else:  # pragma: no cover - defensive; wrapper exists after _setup_viewer
            self._viewer.data = self._buffer

    # ------------------------------------------------------------------
    # Shape / dtype helpers
    # ------------------------------------------------------------------

    @property
    def dtype_shape(self) -> tuple[str, tuple[int, ...]] | None:
        """Return the dtype/shape last configured on the viewer buffer."""
        return getattr(self, "_core_dtype", None)

    def _get_core_dtype_shape(self) -> tuple[str, tuple[int, ...]] | None:
        """Query the expected dtype and shape from the core for this camera.

        When a ``camera_label`` is set this preview is dedicated to a single
        physical camera, so we always return a 2-D (H, W) shape.  Without a
        label we fall back to querying the active camera device and handle
        the multicam case (returning (C, H, W) when N > 1).
        """
        if (core := self._mmc) is None:
            return None
        if not (bits := core.getImageBitDepth()):
            return None

        img_width = core.getImageWidth()
        img_height = core.getImageHeight()

        if self.camera_label is not None:
            # Dedicated single-camera preview — always 2-D.
            shape: tuple[int, ...] = (img_height, img_width)
        elif core.getNumberOfComponents() > 1:
            # RGB / colour camera
            shape = (img_height, img_width, 3)
        elif core.getNumberOfCameraChannels() > 1:
            # Composite multicam preview (legacy / fallback path)
            n_cams = core.getNumberOfCameraChannels()
            shape = (n_cams, img_height, img_width)
        else:
            shape = (img_height, img_width)

        return (f"uint{bits}", shape)

    def _setup_viewer(self, data: np.ndarray | None = None) -> None:
        if data is None:
            core_dtype = self._get_core_dtype_shape()
            if core_dtype is None:
                return
            dtype_str, shape = core_dtype
            buffer_dtype = np.uint16 if dtype_str == "uint12" else np.dtype(dtype_str)
            data = np.zeros(shape, dtype=buffer_dtype)

        self._buffer = np.empty_like(data)
        np.copyto(self._buffer, data)

        self._viewer.data = self._buffer

        display = self._viewer.display_model

        if (
            data.ndim == 3
            and data.shape[0] > 1
            and data.shape[0] <= 8
            and data.shape[-1] not in (3, 4)
        ):
            # Composite multicam (C, H, W) — shown as overlaid channels.
            display.visible_axes = (1, 2)
            display.channel_axis = 0
            display.channel_mode = ndv.models.ChannelMode.COMPOSITE
        elif data.ndim == 3 and data.shape[-1] in (3, 4):
            # RGB / RGBA (H, W, C)
            display.visible_axes = (0, 1)
            display.channel_axis = 2
            display.channel_mode = ndv.models.ChannelMode.RGBA
        else:
            # Grayscale (H, W) — the normal case for a per-camera preview.
            display.visible_axes = (0, 1) if data.ndim == 2 else (-2, -1)
            display.channel_axis = None
            display.channel_mode = ndv.models.ChannelMode.GRAYSCALE

    def _on_system_config_loaded(self) -> None:
        self._setup_viewer()

    def _on_roi_set(self) -> None:
        """Reconfigure the viewer when a Camera ROI is set."""
        self._setup_viewer()
