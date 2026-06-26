import warnings
from abc import abstractmethod
from contextlib import suppress
from typing import Callable

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
        # Optional callback for multicam streaming dispatch.
        # If set, timerEvent will call this instead of self.append.
        # Signature: (frames: list[np.ndarray]) -> None
        self._multicam_frame_callback: Callable[[list[np.ndarray]], None] | None = None
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

    def _pop_latest_frame_group(self) -> list[np.ndarray] | None:
        """Pop the latest complete group of frames from the circular buffer.

        For multicamera, a "complete group" is one frame per physical camera
        (n_cams frames total).  Older incomplete groups are discarded so the
        viewer always shows the most recent data.

        Returns a list of 2-D arrays (one per camera), or a single-element list
        for the single-camera case.  Returns None when the buffer is empty.
        """
        if not (core := self._mmc):
            return None

        n_cams = core.getNumberOfCameraChannels()
        count = core.getRemainingImageCount()

        if count <= 0:
            return None

        if n_cams <= 1:
            # Single camera: drain buffer and keep only the most recent frame.
            img = None
            for _ in range(count):
                with suppress(Exception):
                    img = core.popNextImage()
            return [img] if img is not None else None

        # Multi-camera: frames arrive in round-robin order (cam0, cam1, …).
        # Only process the latest complete group.
        complete_groups = count // n_cams
        if complete_groups == 0:
            return None  # Not enough frames yet for a full set.

        # Discard frames from older complete groups.
        frames_to_skip = (complete_groups - 1) * n_cams
        for _ in range(frames_to_skip):
            with suppress(Exception):
                core.popNextImage()

        # Pop the latest complete group.
        frames: list[np.ndarray] = []
        for _ in range(n_cams):
            with suppress(Exception):
                frames.append(core.popNextImage())

        return frames if frames else None

    def _get_all_images(self) -> np.ndarray | None:
        """Fetch latest image(s) from the buffer; stack into (C, H, W) for multicam.

        Kept for backwards compatibility.  New code should prefer
        `_pop_latest_frame_group()` which returns per-camera frames separately.
        """
        if not (core := self._mmc):
            return None

        n_cams = core.getNumberOfCameraChannels()
        count = core.getRemainingImageCount()

        if count <= 0:
            with suppress(Exception):
                return core.fixImage(core.getLastImage())
            return None

        frames = self._pop_latest_frame_group()
        if not frames:
            return None

        if len(frames) > 1:
            if all(f.shape == frames[0].shape for f in frames):
                return np.stack(frames, axis=0)
            else:
                return frames[-1]  # fallback: last frame if shapes differ
        return frames[0]

    # ----------------------------

    def _on_exposure_changed(self, device: str, value: str) -> None:
        if self._timer_id is not None:
            self.killTimer(self._timer_id)
            self._timer_id = self.startTimer(int(value), Qt.TimerType.PreciseTimer)

    def timerEvent(self, a0: QTimerEvent | None) -> None:
        if not (core := self._mmc):
            return
        if core.getRemainingImageCount() <= 0:
            return

        try:
            frames = self._pop_latest_frame_group()
            if not frames:
                return

            if self._multicam_frame_callback is not None and len(frames) > 1:
                # Route per-camera frames to the manager for dispatching.
                self._multicam_frame_callback(frames)
            else:
                # Single camera or composite stacking.
                if len(frames) > 1:
                    if all(f.shape == frames[0].shape for f in frames):
                        img: np.ndarray = np.stack(frames, axis=0)
                    else:
                        img = frames[-1]
                else:
                    img = frames[0]
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
