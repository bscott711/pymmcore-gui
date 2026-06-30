import os
import sys
import time
import warnings
from abc import abstractmethod
from contextlib import suppress
from typing import TYPE_CHECKING

import numpy as np
from pymmcore_plus import CMMCorePlus
from PyQt6.QtCore import Qt, QTimerEvent
from PyQt6.QtWidgets import QWidget

if TYPE_CHECKING:
    from collections.abc import Callable

_DEFAULT_WAIT = 10

# Set the env var MM_PREVIEW_PROFILE=1 to log live-preview throughput while
# streaming (capture vs display FPS and per-stage timing) once every ~2 s.
_PROFILE = os.getenv("MM_PREVIEW_PROFILE", "") not in ("", "0", "false", "False")
_PROFILE_LOG_PERIOD_S = 2.0


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
        # Signature: (frames: dict[str, np.ndarray]) -> None  (keyed by camera label)
        self._multicam_frame_callback: (
            Callable[[dict[str, np.ndarray]], None] | None
        ) = None

        # --- profiling counters (only used when _PROFILE) ---
        self._prof_seen = 0  # frames removed from the circular buffer
        self._prof_displays = 0  # render/dispatch events
        self._prof_fetch_s = 0.0
        self._prof_render_s = 0.0
        self._prof_last_log = time.perf_counter()
        self._prof_logged_labels = False

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

    def _pop_latest_frame_group(self) -> dict[str, np.ndarray] | None:
        """Return the newest frame for each physical camera, keyed by camera label.

        The ``Utilities/Multi Camera`` device inserts **each** physical camera's
        frame as its own circular-buffer entry, tagged with a ``"Camera"``
        metadata tag holding the physical device label.  (The channel-indexed
        ``getLastImageMD`` API does *not* separate these — only channel 0 exists —
        which is why an earlier attempt left one pane stuck and bled the other
        camera into the first.)

        We therefore walk the buffer newest-first, read each frame's ``"Camera"``
        tag, and keep the newest frame per camera.  Routing by the metadata tag
        (rather than by buffer position) is robust to dropped or interleaved
        frames: a frame can only ever go to the camera that actually produced it,
        so the panes can no longer swap.  Only if the tag is entirely absent do we
        fall back to positional ordering.

        We then drop the whole backlog in one cheap ``clearCircularBuffer`` call
        instead of draining (and copying) every queued frame.

        Returns ``{camera_label: 2-D array}`` (one entry per camera), or ``None``
        when a complete latest set is not yet available.
        """
        if not (core := self._mmc):
            return None

        n_cams = max(core.getNumberOfCameraChannels(), 1)
        count = core.getRemainingImageCount()

        if count < n_cams:
            return None  # wait until a full set is available

        if n_cams == 1:
            img = None
            with suppress(Exception):
                img = core.getLastImage()
            with suppress(Exception):
                core.clearCircularBuffer()
            if img is None:
                return None
            if _PROFILE:
                self._prof_seen += count
            return {core.getCameraDevice(): img}

        # Multi-camera: walk newest-first, keeping the newest frame per camera as
        # identified by its "Camera" metadata tag.
        labels = [core.getPhysicalCameraDevice(i) or f"Camera-ch{i}" for i in range(
            n_cams
        )]
        frames: dict[str, np.ndarray] = {}
        max_walk = min(count, n_cams * 8)
        for offset in range(max_walk):
            try:
                img, md = core.getNBeforeLastImageAndMD(offset)
            except Exception:
                break
            label = md.get("Camera", None)
            if not label:
                # Tag absent: fall back to positional order within the set.
                label = labels[offset % n_cams]
            if label not in frames:
                frames[label] = img
                if len(frames) == n_cams:
                    break
        with suppress(Exception):
            core.clearCircularBuffer()

        if not frames:
            return None
        if _PROFILE:
            self._prof_seen += count / n_cams
            if not self._prof_logged_labels:
                self._prof_logged_labels = True
                print(
                    f"[preview profile] physical labels={labels} | "
                    f"frame keys={list(frames)}",
                    file=sys.stderr,
                    flush=True,
                )
        return frames

    def _get_all_images(self) -> np.ndarray | None:
        """Fetch latest image(s) from the buffer; stack into (C, H, W) for multicam.

        Kept for backwards compatibility.  New code should prefer
        `_pop_latest_frame_group()` which returns per-camera frames separately.
        """
        if not (core := self._mmc):
            return None

        count = core.getRemainingImageCount()

        if count <= 0:
            with suppress(Exception):
                return core.fixImage(core.getLastImage())
            return None

        group = self._pop_latest_frame_group()
        if not group:
            return None

        frames = list(group.values())
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

        t0 = time.perf_counter() if _PROFILE else 0.0
        try:
            group = self._pop_latest_frame_group()
            if not group:
                return

            t_fetch = time.perf_counter() if _PROFILE else 0.0

            if self._multicam_frame_callback is not None:
                # Multicam driver: ALWAYS dispatch by camera label, even when this
                # tick only found one camera's frame.  Falling back to
                # ``self.append`` here would push that lone frame into the driver's
                # *own* pane regardless of which camera produced it — which made
                # the driver pane intermittently show the other camera.
                self._multicam_frame_callback(group)
            else:
                # Single camera or composite stacking.
                frames = list(group.values())
                if len(frames) > 1:
                    if all(f.shape == frames[0].shape for f in frames):
                        img: np.ndarray = np.stack(frames, axis=0)
                    else:
                        img = frames[-1]
                else:
                    img = frames[0]
                self.append(img)

            if _PROFILE:
                now = time.perf_counter()
                self._prof_displays += 1
                self._prof_fetch_s += t_fetch - t0
                self._prof_render_s += now - t_fetch
                self._prof_maybe_log(now)
        except Exception as e:
            warnings.warn(
                f"Failed to get image from core: {e}", RuntimeWarning, stacklevel=2
            )

    def _prof_maybe_log(self, now: float) -> None:
        """Log capture/display throughput once per ``_PROFILE_LOG_PERIOD_S``."""
        dt = now - self._prof_last_log
        if dt < _PROFILE_LOG_PERIOD_S:
            return
        displays = max(self._prof_displays, 1)
        capture_fps = self._prof_seen / dt
        display_fps = self._prof_displays / dt
        fetch_ms = 1000.0 * self._prof_fetch_s / displays
        render_ms = 1000.0 * self._prof_render_s / displays
        dropped = 0.0
        if self._prof_seen:
            dropped = 100.0 * (self._prof_seen - self._prof_displays) / self._prof_seen
        cams = ""
        if (core := self._mmc) is not None:
            with suppress(Exception):
                n = core.getNumberOfCameraChannels()
                cams = f"{n}cam " if n > 1 else ""
        print(
            f"[preview profile] {cams}capture ~{capture_fps:.0f} fps | "
            f"display ~{display_fps:.0f} fps | fetch {fetch_ms:.1f} ms | "
            f"render {render_ms:.1f} ms | dropped {max(dropped, 0.0):.0f}%",
            file=sys.stderr,
            flush=True,
        )
        self._prof_seen = 0
        self._prof_displays = 0
        self._prof_fetch_s = 0.0
        self._prof_render_s = 0.0
        self._prof_last_log = now

    def _on_image_snapped(self) -> None:
        if self._mmc is None:
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
