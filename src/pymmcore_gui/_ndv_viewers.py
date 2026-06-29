from __future__ import annotations

import warnings
from typing import TYPE_CHECKING, cast
from weakref import WeakSet, WeakValueDictionary

import ndv
import useq
from pymmcore_plus.mda.handlers import TensorStoreHandler
from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtWidgets import QWidget
from PyQt6Ads import CDockWidget

from pymmcore_gui.widgets.image_preview._ndv_preview import NDVPreview

if TYPE_CHECKING:
    from collections.abc import Iterator

    import numpy as np
    from ndv.models._array_display_model import (
        IndexMap,  # pyright: ignore[reportPrivateImportUsage]
    )
    from pymmcore_plus import CMMCorePlus
    from pymmcore_plus.mda import SupportsFrameReady
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1
    from useq import MDASequence

    from pymmcore_gui.widgets.image_preview._preview_base import ImagePreviewBase


# NOTE: we make this a QObject mostly so that the lifetime of this object is tied to
# the lifetime of the parent QMainWindow.  If inheriting from QObject is removed in
# the future, make sure not to store a strong reference to this main_window
class NDVViewersManager(QObject):
    """Object that mediates a connection between the MDA experiment and ndv viewers.

    Parameters
    ----------
    parent : QWidget
        The parent widget.
    mmcore : CMMCorePlus
        The CMMCorePlus instance.
    """

    mdaViewerCreated = pyqtSignal(ndv.ArrayViewer, useq.MDASequence, str)
    previewViewerCreated = pyqtSignal(CDockWidget)
    viewerDestroyed = pyqtSignal(str)

    def __init__(self, parent: QWidget, mmcore: CMMCorePlus):
        super().__init__(parent)
        self._mmc = mmcore

        # weakref map of {sequence_uid: ndv.ArrayViewer}
        self._seq_viewers = WeakValueDictionary[str, ndv.ArrayViewer]()
        self._preview_dock_widgets = WeakSet[CDockWidget]()
        # currently active viewer
        self._active_mda_viewer: ndv.ArrayViewer | None = None

        # We differentiate between handlers that were created by someone else, and
        # gathered using mda.get_output_handlers(), vs handlers that were created by us.
        # because we need to call frameReady/sequenceFinished manually on the latter.
        self._handler: SupportsFrameReady | None = None
        self._own_handler: TensorStoreHandler | None = None

        # CONNECTIONS ---------------------------------------------------------

        self._is_mda_running = False

        # Per-camera preview dock widgets, keyed by physical camera label.
        # e.g. {"Camera-1": <CDockWidget>, "Camera-2": <CDockWidget>}
        self._camera_previews: dict[str, CDockWidget] = {}

        # Primary "streaming driver" preview — the first camera's NDVPreview owns
        # the Qt timer that polls the circular buffer.  Other cameras' previews
        # receive frames via a callback set on this widget.
        self._streaming_driver: NDVPreview | None = None

        # Per-camera MDA display handlers/viewers, keyed by physical camera label.
        # Populated only for multi-camera acquisitions; the single-camera path
        # continues to use ``_own_handler`` / ``_active_mda_viewer`` below.
        self._mda_camera_handlers: dict[str, TensorStoreHandler] = {}
        self._mda_camera_viewers: dict[str, ndv.ArrayViewer] = {}

        ev = self._mmc.events
        ev.imageSnapped.connect(self._on_image_snapped)
        ev.sequenceAcquisitionStarted.connect(self._on_streaming_started)
        ev.continuousSequenceAcquisitionStarted.connect(self._on_streaming_started)
        ev.propertyChanged.connect(self._on_property_changed)

        mda_ev = self._mmc.mda.events
        mda_ev.sequenceStarted.connect(self._on_sequence_started)
        mda_ev.frameReady.connect(self._on_frame_ready)
        mda_ev.sequenceFinished.connect(self._on_sequence_finished)

        parent.destroyed.connect(self._cleanup)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_physical_camera_labels(self) -> list[str]:
        """Return the list of physical camera labels behind the active camera device.

        For a ``Multi Camera`` device this reads the ``Physical Camera N``
        properties.  For a plain camera device returns a single-element list.
        """
        cam = self._mmc.getCameraDevice()
        n = self._mmc.getNumberOfCameraChannels()
        if n <= 1:
            return [cam]
        try:
            return [
                self._mmc.getProperty(cam, f"Physical Camera {i + 1}")
                for i in range(n)
            ]
        except Exception:
            # Fallback if the property isn't readable (e.g. non-standard device)
            return [f"{cam}-ch{i}" for i in range(n)]

    def _create_camera_preview(self, camera_label: str) -> NDVPreview:
        """Create a new NDVPreview dock widget for *camera_label* and emit the signal."""
        preview = NDVPreview(mmcore=self._mmc, camera_label=camera_label)
        parent = self.parent()
        if not isinstance(parent, QWidget):
            parent = None  # pragma: no cover
        dw = CDockWidget(f"Preview: {camera_label}", parent)
        self._preview_dock_widgets.add(dw)
        dw.setWidget(preview)
        dw.setFeature(dw.DockWidgetFeature.DockWidgetFloatable, False)
        self._camera_previews[camera_label] = dw
        self.previewViewerCreated.emit(dw)
        return preview

    def _get_or_create_camera_preview(
        self, camera_label: str
    ) -> tuple[NDVPreview, bool]:
        """Return ``(NDVPreview, created)`` for *camera_label*.

        If the dock widget already exists its view is toggled on and ``created``
        is *False*.  Otherwise a new preview is created and ``created`` is *True*.
        """
        if camera_label in self._camera_previews:
            dw = self._camera_previews[camera_label]
            dw.toggleView(True)
            return cast("NDVPreview", dw.widget()), False
        return self._create_camera_preview(camera_label), True

    def _dispatch_snap_to_previews(self, images: dict[int, np.ndarray]) -> None:
        """Send each snapped camera image to its dedicated preview widget."""
        labels = self._get_physical_camera_labels()
        for ch_idx, img in images.items():
            label = labels[ch_idx] if ch_idx < len(labels) else f"Camera-ch{ch_idx}"
            preview, _ = self._get_or_create_camera_preview(label)
            preview.append(img)

    def _make_multicam_streaming_callback(
        self, labels: list[str]
    ):
        """Return a callback that dispatches per-camera frames to their previews.

        The callback is installed on the streaming-driver ``NDVPreview``'s
        ``_multicam_frame_callback`` attribute so that its ``timerEvent`` routes
        frames here instead of calling ``append``.
        """

        def _on_frames(frames: list[np.ndarray]) -> None:
            for ch_idx, frame in enumerate(frames):
                label = labels[ch_idx] if ch_idx < len(labels) else f"Camera-ch{ch_idx}"
                dw = self._camera_previews.get(label)
                if dw is None:
                    continue
                preview = cast("NDVPreview", dw.widget())
                preview.append(frame)

        return _on_frames

    # ------------------------------------------------------------------
    # Streaming / Snap handlers
    # ------------------------------------------------------------------

    def _on_streaming_started(self) -> None:
        if self._is_mda_running:
            return

        labels = self._get_physical_camera_labels()

        if len(labels) <= 1:
            # Single camera path — identical to previous behaviour.
            preview, created = self._get_or_create_camera_preview(labels[0])
            if created:
                preview._on_streaming_start()
            else:
                # Already running, just make sure it's visible.
                pass
            self._streaming_driver = preview
        else:
            # Multi-camera path:
            # Create previews for ALL cameras up front, but only the first one
            # owns the Qt timer (streaming driver).  A callback dispatches frames
            # to the others.
            driver_preview: NDVPreview | None = None
            for i, label in enumerate(labels):
                preview, created = self._get_or_create_camera_preview(label)
                if i == 0:
                    driver_preview = preview

            if driver_preview is not None:
                # Install the multicam dispatch callback so that timerEvent
                # routes per-camera frames correctly.
                driver_preview._multicam_frame_callback = (
                    self._make_multicam_streaming_callback(labels)
                )
                driver_preview._on_streaming_start()
                self._streaming_driver = driver_preview

    def _on_image_snapped(self) -> None:
        if self._is_mda_running:
            return

        n_channels = self._mmc.getNumberOfCameraChannels()

        if n_channels > 1:
            # Retrieve each physical camera's image by channel index.
            images: dict[int, np.ndarray] = {}
            for ch in range(n_channels):
                try:
                    images[ch] = self._mmc.getImage(ch)
                except Exception as exc:  # pragma: no cover
                    warnings.warn(
                        f"Failed to get image for channel {ch}: {exc}",
                        RuntimeWarning,
                        stacklevel=2,
                    )
            if images:
                self._dispatch_snap_to_previews(images)
        else:
            # Single-camera path — identical to previous behaviour.
            label = self._mmc.getCameraDevice()
            preview, _ = self._get_or_create_camera_preview(label)
            try:
                preview.append(self._mmc.getImage())
            except Exception as exc:  # pragma: no cover
                warnings.warn(
                    f"Failed to get image: {exc}", RuntimeWarning, stacklevel=2
                )

    # ------------------------------------------------------------------
    # Property change / invalidation
    # ------------------------------------------------------------------

    def _on_property_changed(self, dev: str, prop: str, value: str) -> None:
        if self._mmc is None:
            return  # pragma: no cover

        cam_device = self._mmc.getCameraDevice()
        physical_labels = set(self._get_physical_camera_labels())

        # Determine which camera labels are affected.
        affected: set[str] = set()
        if dev == "Core" and prop == "Camera":
            # Core camera device changed — invalidate everything.
            affected = set(self._camera_previews.keys())
        elif dev == cam_device:
            # MultiCamera (or active single camera) property changed.
            affected = set(self._camera_previews.keys())
        elif dev in physical_labels:
            # A specific physical camera changed.
            affected.add(dev)

        for label in affected:
            dw = self._camera_previews.get(label)
            if dw is None:
                continue
            preview = cast("NDVPreview", dw.widget())
            # Only invalidate if shape / dtype actually changed.
            if preview._get_core_dtype_shape() != preview.dtype_shape:
                preview.detach()
                del self._camera_previews[label]
                if self._streaming_driver is preview:
                    self._streaming_driver = None

    # ------------------------------------------------------------------
    # MDA handlers (unchanged, kept for completeness)
    # ------------------------------------------------------------------

    def _cleanup(self, obj: QObject | None = None) -> None:
        self._active_mda_viewer = None
        self._handler = None
        self._own_handler = None

    def _on_sequence_started(
        self, sequence: useq.MDASequence, meta: SummaryMetaV1
    ) -> None:
        """Called when a new MDA sequence has been started.

        We grab the first handler in the list of output handlers, or create a new
        TensorStoreHandler if none exist. Then we create a new ndv viewer and show it.
        """
        self._is_mda_running = True

        self._own_handler = self._handler = None
        self._mda_camera_handlers.clear()
        self._mda_camera_viewers.clear()

        labels = self._get_physical_camera_labels()
        if len(labels) > 1:
            # Multi-camera: every physical camera frame shares the same event
            # index, so a single store/viewer would overwrite frames.  Give each
            # camera its own in-memory display handler + viewer (independent of any
            # save handler), routing frames by ``meta["camera_device"]``.
            for label in labels:
                handler = TensorStoreHandler(driver="zarr", kvstore="memory://")
                handler.reset(sequence)
                self._mda_camera_handlers[label] = handler
                self._mda_camera_viewers[label] = self._create_ndv_viewer(
                    sequence, label
                )
            self._active_mda_viewer = None
            return

        if handlers := self._mmc.mda.get_output_handlers():
            # someone else has created a handler for this sequence
            self._handler = handlers[0]
        else:
            # if it does not exist, create a new TensorStoreHandler
            self._own_handler = TensorStoreHandler(driver="zarr", kvstore="memory://")
            self._own_handler.reset(sequence)

        # since the handler is empty at this point, create a ndv viewer with no data
        self._active_mda_viewer = self._create_ndv_viewer(sequence)

    def _on_frame_ready(
        self, frame: np.ndarray, event: useq.MDAEvent, meta: FrameMetaV1
    ) -> None:
        """Create a viewer if it does not exist, otherwise update the current index."""
        # Multi-camera: route to the per-camera display handler + viewer.
        if self._mda_camera_handlers:
            label = meta.get("camera_device") or self._mmc.getCameraDevice()
            handler = self._mda_camera_handlers.get(label)
            viewer = self._mda_camera_viewers.get(label)
            if handler is None or viewer is None:
                return  # pragma: no cover
            handler.frameReady(frame, event, meta)
            self._update_mda_viewer(viewer, handler, event)
            return

        # Single-camera path.
        if self._own_handler is not None:
            self._own_handler.frameReady(frame, event, meta)

        if (viewer := self._active_mda_viewer) is None:
            return  # pragma: no cover

        self._update_mda_viewer(viewer, self._handler or self._own_handler, event)

    def _update_mda_viewer(
        self,
        viewer: ndv.ArrayViewer,
        handler: SupportsFrameReady | None,
        event: useq.MDAEvent,
    ) -> None:
        """Point the viewer at the handler store, or update its current index."""
        # if the viewer does not yet have data, it's likely the very first frame
        # so update the viewer's data source to the underlying handlers store
        if viewer.data_wrapper is None:
            if isinstance(handler, TensorStoreHandler):
                # TODO: temporary. maybe create the DataWrapper for the handlers
                viewer.data = handler.store
            else:
                warnings.warn(
                    f"don't know how to show data of type {type(handler)}",
                    stacklevel=2,
                )
        # otherwise update the sliders to the most recently acquired frame
        else:
            # Add a small delay to make sure the data are available in the handler
            # This is a bit of a hack to get around the data handlers can write data
            # asynchronously, so the data may not be available immediately to the viewer
            # after the handler's frameReady method is called.
            current_index = viewer.display_model.current_index

            def _update(_idx: IndexMap = current_index) -> None:
                try:
                    _idx.update(event.index.items())
                except Exception:  # pragma: no cover
                    # this happens if the viewer has been closed in the meantime
                    # usually it's a RuntimeError, but could be an EmitLoopError
                    pass

            QTimer.singleShot(10, _update)

    def _on_sequence_finished(self, sequence: useq.MDASequence) -> None:
        """Called when a sequence has finished."""
        if self._own_handler is not None:
            self._own_handler.sequenceFinished(sequence)
        for handler in self._mda_camera_handlers.values():
            handler.sequenceFinished(sequence)
        # cleanup pointers somehow?
        self._is_mda_running = False

    def _create_ndv_viewer(
        self, sequence: MDASequence, camera_label: str = ""
    ) -> ndv.ArrayViewer:
        """Create a new ndv viewer with no data.

        *camera_label* identifies the physical camera for multi-camera
        acquisitions (empty string for the single-camera case).
        """
        ndv_viewer = ndv.ArrayViewer()
        # Key by uid (single camera) or uid::label (one viewer per camera) so the
        # weak-value map retains a distinct entry per viewer.
        key = str(sequence.uid)
        if camera_label:
            key = f"{key}::{camera_label}"
        self._seq_viewers[key] = ndv_viewer
        self.mdaViewerCreated.emit(ndv_viewer, sequence, camera_label)
        return ndv_viewer

    def __repr__(self) -> str:  # pragma: no cover
        return f"<{self.__class__.__name__} {hex(id(self))} ({len(self)} viewer)>"

    def __len__(self) -> int:
        return len(self._seq_viewers)

    def viewers(self) -> Iterator[ndv.ArrayViewer]:
        yield from (self._seq_viewers.values())
