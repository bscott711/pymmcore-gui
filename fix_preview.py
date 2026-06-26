import pathlib

preview_base_code = '''import warnings
from abc import abstractmethod
from contextlib import suppress

import numpy as np
from pymmcore_plus import CMMCorePlus
from PyQt6.QtCore import Qt, QTimerEvent
from PyQt6.QtWidgets import QWidget

_DEFAULT_WAIT = 10

class ImagePreviewBase(QWidget):
    def __init__(
        self,
        parent: QWidget | None,
        mmcore: CMMCorePlus,
        *,
        use_with_mda: bool = False,
    ):
        super().__init__(parent)
        self._timer_id: int | None = None  # timer for streaming
        self.use_with_mda = use_with_mda
        self._is_mda_running: bool = False
        self._mmc: CMMCorePlus | None = mmcore
        self.attach(mmcore)

    def attach(self, core: CMMCorePlus) -> None:
        """Attach this widget to events in `core`."""
        if self._mmc is not None:
            self.detach()
        ev = core.events
        ev.imageSnapped.connect(self._on_image_snapped)
        ev.continuousSequenceAcquisitionStarted.connect(self._on_streaming_start)
        ev.sequenceAcquisitionStarted.connect(self._on_streaming_start)
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

    def detach(self) -> None:
        """Detach this widget from events in `core`."""
        if self._mmc is None:
            return  # pragma: no cover
        with suppress(Exception):
            ev, self._mmc = self._mmc.events, None
            ev.imageSnapped.disconnect(self._on_image_snapped)
            ev.continuousSequenceAcquisitionStarted.disconnect(self._on_streaming_start)
            ev.sequenceAcquisitionStarted.disconnect(self._on_streaming_start)
            ev.sequenceAcquisitionStopped.disconnect(self._on_streaming_stop)
            ev.exposureChanged.disconnect(self._on_exposure_changed)

    @abstractmethod
    def append(self, data: np.ndarray) -> None:
        raise NotImplementedError

    def _get_all_images(self) -> np.ndarray | None:
        """Fetch all available images from the buffer and stack them if multi-camera."""
        if not (core := self._mmc):
            return None

        count = core.getRemainingImageCount()
        if count <= 0:
            with suppress(Exception):
                return core.fixImage(core.getLastImage())
            return None

        imgs = []
        for _ in range(count):
            imgs.append(core.popNextImage())

        if not imgs:
            return None

        if len(imgs) > 1:
            # Multi-camera: stack into (C, H, W)
            if all(img.shape == imgs[0].shape for img in imgs):
                return np.stack(imgs, axis=0)
            else:
                # Fallback if shapes differ
                return imgs[-1]
        return imgs[0]

    # ----------------------------

    def _on_exposure_changed(self, device: str, value: str) -> None:
        if self._timer_id is not None:
            self.killTimer(self._timer_id)
            self._timer_id = self.startTimer(int(value), Qt.TimerType.PreciseTimer)

    def timerEvent(self, a0: QTimerEvent | None) -> None:
        if (core := self._mmc) and core.getRemainingImageCount() > 0:
            try:
                img = self._get_all_images()
                if img is not None:
                    self.append(img)
            except Exception as e:
                warnings.warn(
                    f"Failed to get image from core: {e}", RuntimeWarning, stacklevel=2
                )

    def _on_image_snapped(self) -> None:
        if (core := self._mmc) is None:
            return  # pragma: no cover
        if not self.use_with_mda and self._is_mda_running:
            return  # pragma: no cover

        img = self._get_all_images()
        if img is not None:
            self.append(img)

    def _on_streaming_start(self) -> None:
        if (core := self._mmc) is not None:
            wait = int(core.getExposure()) or _DEFAULT_WAIT
            self._timer_id = self.startTimer(wait, Qt.TimerType.PreciseTimer)

    def _on_streaming_stop(self) -> None:
        if self._timer_id is not None:
            self.killTimer(self._timer_id)
            self._timer_id = None

    def _on_system_config_loaded(self) -> None:
        pass

    def _on_roi_set(self) -> None:
        pass

    def _on_property_changed(self, dev: str, prop: str, value: str) -> None:
        pass
'''

ndv_preview_code = '''from __future__ import annotations

from typing import TYPE_CHECKING

import ndv
import numpy as np
from PyQt6.QtWidgets import QApplication, QVBoxLayout, QWidget

from pymmcore_gui.widgets.image_preview._preview_base import ImagePreviewBase

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus


class NDVPreview(ImagePreviewBase):
    def __init__(
        self,
        mmcore: CMMCorePlus,
        parent: QWidget | None = None,
        *,
        use_with_mda: bool = False,
    ):
        super().__init__(parent, mmcore, use_with_mda=use_with_mda)
        self._viewer = ndv.ArrayViewer()
        self._buffer: np.ndarray | None = None
        self.process_events_on_update = True

        qwdg = self._viewer.widget()
        qwdg.setParent(self)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(qwdg)

    def append(self, data: np.ndarray) -> None:
        if self._buffer is None or self._buffer.shape != data.shape or self._buffer.dtype != data.dtype:
            self._setup_viewer(data)

        if self._buffer is not None:
            np.copyto(self._buffer, data)
            self._viewer.data = self._buffer

        if self.process_events_on_update:
            QApplication.processEvents()

    @property
    def dtype_shape(self) -> tuple[str, tuple[int, ...]] | None:
        return getattr(self, "_core_dtype", None)

    def _get_core_dtype_shape(self) -> tuple[str, tuple[int, ...]] | None:
        if (core := self._mmc) is not None:
            if bits := core.getImageBitDepth():
                img_width = core.getImageWidth()
                img_height = core.getImageHeight()
                n_cams = core.getNumberOfCameraChannels()
                if core.getNumberOfComponents() > 1:
                    shape: tuple[int, ...] = (img_height, img_width, 3)
                elif n_cams > 1:
                    shape = (n_cams, img_height, img_width)
                else:
                    shape = (img_height, img_width)
                return (f"uint{bits}", shape)
        return None

    def _setup_viewer(self, data: np.ndarray | None = None) -> None:
        if data is None:
            core_dtype = self._get_core_dtype_shape()
            if core_dtype is None:
                return
            dtype_str, shape = core_dtype
            if dtype_str == "uint12":
                buffer_dtype = np.uint16
            else:
                buffer_dtype = np.dtype(dtype_str)
            data = np.zeros(shape, dtype=buffer_dtype)

        self._buffer = np.empty_like(data)
        np.copyto(self._buffer, data)

        self._viewer.data = self._buffer

        display = self._viewer.display_model

        if data.ndim == 3 and data.shape[0] > 1 and data.shape[0] <= 8 and data.shape[-1] not in (3, 4):
            # Multi-camera (C, H, W) -> COMPOSITE MODE
            display.visible_axes = (1, 2)
            display.channel_axis = 0
            display.channel_mode = ndv.models.ChannelMode.COMPOSITE
        elif data.ndim == 3 and data.shape[-1] in (3, 4):
            # RGB/RGBA (H, W, C)
            display.visible_axes = (0, 1)
            display.channel_axis = 2
            display.channel_mode = ndv.models.ChannelMode.RGBA
        else:
            # Grayscale (H, W)
            display.visible_axes = (0, 1) if data.ndim == 2 else (-2, -1)
            display.channel_axis = None
            display.channel_mode = ndv.models.ChannelMode.GRAYSCALE

    def _on_system_config_loaded(self) -> None:
        self._setup_viewer()

    def _on_roi_set(self) -> None:
        """Reconfigure the viewer when a Camera ROI is set."""
        self._setup_viewer()
'''

base_path = pathlib.Path("src/pymmcore_gui/widgets/image_preview")
(base_path / "_preview_base.py").write_text(preview_base_code)
(base_path / "_ndv_preview.py").write_text(ndv_preview_code)
print("✅ Overwrote preview logic to drain buffer and display as COMPOSITE.")
