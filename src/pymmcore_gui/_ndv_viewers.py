from __future__ import annotations

import warnings
from contextlib import suppress
from typing import TYPE_CHECKING, cast
from weakref import WeakSet, WeakValueDictionary

import ndv
import useq
from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtWidgets import QWidget
from PyQt6Ads import CDockWidget

from pymmcore_gui._multi_camera_handler import without_cam_index
from pymmcore_gui._numpy_display_store import NumpyDisplayStore
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator
    from typing import Any, TypeGuard

    import numpy as np
    from pymmcore_plus import CMMCorePlus
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1
    from useq import MDASequence


class _LabeledArrayWrapper(ndv.DataWrapper):
    """Expose a ``NumpyDisplayStore``'s array with its real t/p/z/c/y/x axis labels.

    ndv's built-in fallback wrapper for a plain array (or zarr array) exposes
    bare integer dim positions (0, 1, 2, ...), which silently breaks
    ``_update_mda_viewer``'s ``current_index.update(event.index.items())``
    calls below: ``event.index`` is keyed by string axis names ("t"/"z"/etc),
    and ndv drops any ``current_index`` key that doesn't resolve against the
    wrapper's ``dims`` (see ``ndv.models._resolve._norm_current_index``'s
    ``except (IndexError, KeyError): continue``) -- so those "jump to latest"
    updates were silent no-ops, and the display only ever advanced when a
    user manually dragged a slider (which uses the view's own,
    correctly-integer-keyed index, giving the impression that frames only
    "arrive" once you scrub). This mirrors what ndv's own
    ``TensorstoreWrapper`` did for a tensorstore store (reading real string
    dim labels from the store's domain), just for our zarr-backed
    ``NumpyDisplayStore``.
    """

    def __init__(self, handler: NumpyDisplayStore) -> None:
        array = handler.array
        if array is None:  # pragma: no cover -- only constructed after frameReady
            raise ValueError("NumpyDisplayStore has no data yet")
        self._dims_ = handler.dims
        super().__init__(array)

    @classmethod
    def supports(cls, obj: Any) -> TypeGuard[Any]:
        # Only ever constructed explicitly (see _update_mda_viewer) -- never
        # auto-detected by DataWrapper.create(), so this must never claim
        # ownership of some other, unrelated bare array elsewhere in the app.
        return False

    @property
    def dims(self) -> tuple[str, ...]:
        return self._dims_


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
    previewViewerCreated = pyqtSignal(CDockWidget, str)
    viewerDestroyed = pyqtSignal(str)

    def __init__(self, parent: QWidget, mmcore: CMMCorePlus):
        super().__init__(parent)
        self._mmc = mmcore

        # weakref map of {sequence_uid: ndv.ArrayViewer}
        self._seq_viewers = WeakValueDictionary[str, ndv.ArrayViewer]()
        self._preview_dock_widgets = WeakSet[CDockWidget]()
        # currently active viewer
        self._active_mda_viewer: ndv.ArrayViewer | None = None

        # Private, in-RAM display-only store for the single-camera case --
        # always created fresh per sequence (see _on_sequence_started),
        # independent of whatever the MDA's real output/save handler is
        # doing. We call frameReady/sequenceFinished on it manually.
        self._own_handler: NumpyDisplayStore | None = None

        # CONNECTIONS ---------------------------------------------------------

        self._is_mda_running = False

        # {viewer: latest event} for a coalesced, at-most-one-in-flight
        # QTimer.singleShot per viewer -- see _update_mda_viewer.
        self._pending_viewer_updates: dict[ndv.ArrayViewer, useq.MDAEvent] = {}

        # {viewer: locked z index} for viewers in "locked slice" playback mode
        # (see set_viewer_z_locked / _update_mda_viewer). Absent from this dict
        # means "live" mode -- the default, always-jump-to-latest behavior.
        self._locked_z_axis: dict[ndv.ArrayViewer, int] = {}
        # {viewer: current_index.item_changed listener} for locked viewers, so
        # set_viewer_z_locked(locked=False) / _cleanup can disconnect them.
        self._z_lock_listeners: dict[ndv.ArrayViewer, Callable[..., None]] = {}

        # Per-camera preview dock widgets, keyed by physical camera label.
        # e.g. {"Camera-1": <CDockWidget>, "Camera-2": <CDockWidget>}
        self._camera_previews: dict[str, CDockWidget] = {}

        # Primary "streaming driver" preview — the first camera's PygfxPreview owns
        # the Qt timer that polls the circular buffer.  Other cameras' previews
        # receive frames via a callback set on this widget.
        self._streaming_driver: PygfxPreview | None = None

        # Per-camera MDA display handlers/viewers, keyed by physical camera label.
        # Populated only for multi-camera acquisitions; the single-camera path
        # continues to use ``_own_handler`` / ``_active_mda_viewer`` below.
        self._mda_camera_handlers: dict[str, NumpyDisplayStore] = {}
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
        # Resolve each channel via the same helper the preview fetch uses
        # (getPhysicalCameraDevice -> "Physical Camera N" property) so the dock
        # keys and the frame-dict keys are guaranteed identical.
        labels: list[str] = []
        for i in range(n):
            try:
                labels.append(self._mmc.getPhysicalCameraDevice(i) or f"Camera-ch{i}")
            except Exception:
                labels.append(f"Camera-ch{i}")
        return labels

    def _create_camera_preview(self, camera_label: str) -> PygfxPreview:
        """Create a new PygfxPreview dock for *camera_label* and emit the signal."""
        preview = PygfxPreview(mmcore=self._mmc, camera_label=camera_label)
        parent = self.parent()
        if not isinstance(parent, QWidget):
            parent = None  # pragma: no cover
        dw = CDockWidget(f"Preview: {camera_label}", parent)
        self._preview_dock_widgets.add(dw)
        dw.setWidget(preview)
        dw.setFeature(dw.DockWidgetFeature.DockWidgetFloatable, False)
        self._camera_previews[camera_label] = dw
        rois = self._apply_roi_overlays(preview, camera_label)
        # Default the view -- including future resets from append() recreating
        # the texture (e.g. the first real frame after the placeholder) -- to
        # the union of this camera's defined spectral ROIs (its new "100%")
        # instead of the full sensor. The user can still pan/zoom further in.
        preview.set_default_zoom_rect(self._roi_union_rect(rois))
        self.previewViewerCreated.emit(dw, camera_label)
        return preview

    @staticmethod
    def _roi_union_rect(
        rois: list[tuple[str, tuple[int, int, int, int]]],
    ) -> tuple[int, int, int, int] | None:
        """Return the ``(x, y, w, h)`` union bbox of *rois*, or None if empty."""
        if not rois:
            return None
        xs0 = [r[0] for _, r in rois]
        ys0 = [r[1] for _, r in rois]
        xs1 = [r[0] + r[2] for _, r in rois]
        ys1 = [r[1] + r[3] for _, r in rois]
        x0, y0 = min(xs0), min(ys0)
        return x0, y0, max(xs1) - x0, max(ys1) - y0

    def create_default_camera_previews(self) -> None:
        """Proactively create a preview dock for every configured physical camera.

        Called once at startup (see ``MicroManagerGUI._ensure_camera_previews``)
        so the user doesn't have to Snap before the camera panes exist. This
        also has the side effect of paying the one-time GPU/wgpu
        initialization that the first ``PygfxPreview`` (or ``ndv.ArrayViewer``)
        triggers during this deliberate startup pause, instead of blocking the
        event loop mid-Acquire on the first MDA of a session -- both use the
        same underlying ``pygfx`` renderer singleton.

        No-op if no camera device is configured yet (e.g. no config loaded).
        """
        if not self._mmc.getCameraDevice():
            return
        for label in self._get_physical_camera_labels():
            self._get_or_create_camera_preview(label)

    def _apply_roi_overlays(
        self, preview: PygfxPreview, camera_label: str
    ) -> list[tuple[str, tuple[int, int, int, int]]]:
        """Draw this camera's configured splitter ROIs on *preview*, if any.

        Returns the ``(name, rect)`` pairs drawn, so callers that need the
        raw rectangles (e.g. to compute a zoom target) don't have to
        re-derive them from settings.
        """
        spectral = SettingsV1.instance().spectral
        rois = [
            (c.name, c.rect)
            for c in spectral.channels
            if c.is_ready and c.camera == camera_label and c.rect is not None
        ]
        preview.set_roi_overlays(rois)
        return rois

    def refresh_roi_overlays(self) -> None:
        """Re-apply spectral-channel ROI overlays to every open camera preview.

        Called after the spectral-channel config UI saves changes, so already
        open live/snap panes reflect the new rectangles immediately. Also
        updates each preview's stored zoom-to-ROI default so it stays correct
        across future texture resets, but doesn't force an immediate re-frame
        -- an already-open preview may have been manually panned/zoomed since
        it was created, and this shouldn't yank that away.
        """
        for label, dw in self._camera_previews.items():
            preview = cast("PygfxPreview", dw.widget())
            rois = self._apply_roi_overlays(preview, label)
            preview.set_default_zoom_rect(self._roi_union_rect(rois), apply=False)

    def get_or_create_camera_preview(self, camera_label: str) -> PygfxPreview:
        """Return (creating and showing if needed) the preview for *camera_label*."""
        preview, _created = self._get_or_create_camera_preview(camera_label)
        return preview

    def _get_or_create_camera_preview(
        self, camera_label: str
    ) -> tuple[PygfxPreview, bool]:
        """Return ``(PygfxPreview, created)`` for *camera_label*.

        If the dock widget already exists its view is toggled on and ``created``
        is *False*.  Otherwise a new preview is created and ``created`` is *True*.
        """
        if camera_label in self._camera_previews:
            dw = self._camera_previews[camera_label]
            dw.toggleView(True)
            return cast("PygfxPreview", dw.widget()), False
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
    ) -> Callable[[dict[str, np.ndarray]], None]:
        """Return a callback that dispatches per-camera frames to their previews.

        The callback is installed on the streaming-driver ``PygfxPreview``'s
        ``_multicam_frame_callback`` attribute so that its ``timerEvent`` routes
        frames here instead of calling ``append``.

        Frames arrive keyed by physical-camera label (the same key used for the
        preview docks), so each frame is dispatched to its own pane by an exact
        lookup — there is no positional/parity assumption that could swap panes.
        """

        def _on_frames(frames: dict[str, np.ndarray]) -> None:
            for label, frame in frames.items():
                dw = self._camera_previews.get(label)
                if dw is None:
                    # Label we didn't pre-create a dock for (e.g. the "Camera"
                    # metadata tag differs from the Physical Camera property).
                    # Create it on demand so no camera is silently dropped.
                    preview, _ = self._get_or_create_camera_preview(label)
                else:
                    preview = cast("PygfxPreview", dw.widget())
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
            driver_preview: PygfxPreview | None = None
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
            preview = cast("PygfxPreview", dw.widget())
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
        self._own_handler = None
        self._pending_viewer_updates.clear()
        for viewer, listener in self._z_lock_listeners.items():
            with suppress(Exception):  # viewer may already be gone/destroyed
                viewer.display_model.current_index.item_changed.disconnect(listener)
        self._z_lock_listeners.clear()
        self._locked_z_axis.clear()

    def _make_z_lock_listener(self, viewer: ndv.ArrayViewer) -> Callable[..., None]:
        """Return a listener that keeps ``_locked_z_axis[viewer]`` in sync.

        Connected to ``current_index.item_changed`` while *viewer* is locked,
        so that manually dragging to a new slice re-locks to it (rather than
        the slice originally captured at lock time). Safe against the
        manager's own programmatic writes in ``_update_mda_viewer``: those
        only ever set "z" to the value already recorded in
        ``_locked_z_axis`` (frames at any other z are dropped before that
        call), so this listener re-recording the same value is a no-op.
        """

        def _on_item_changed(key: str, new_value: object, old_value: object) -> None:
            if key == "z" and viewer in self._locked_z_axis:
                self._locked_z_axis[viewer] = cast("int", new_value)

        return _on_item_changed

    def set_viewer_z_locked(self, viewer: ndv.ArrayViewer, locked: bool) -> None:
        """Toggle "locked slice" playback mode for a live-MDA *viewer*.

        In the default "live" mode, the viewer jumps to show every newly
        acquired frame. In "locked" mode, it captures whichever z index the
        viewer is showing right now and stops jumping around -- it only
        updates when a new frame arrives at that same z index (see
        ``_update_mda_viewer``), so watching one plane over time isn't
        interrupted by frames from other z planes. Manually dragging to a
        different slice while locked re-locks to that new slice for
        subsequent frames. A no-op if the sequence has no z axis. Unlocking
        (or re-locking) returns to normal behavior.
        """
        if locked:
            current_z = dict(viewer.display_model.current_index).get("z")
            if current_z is not None:
                self._locked_z_axis[viewer] = cast("int", current_z)
                listener = self._make_z_lock_listener(viewer)
                self._z_lock_listeners[viewer] = listener
                viewer.display_model.current_index.item_changed.connect(listener)
        else:
            self._locked_z_axis.pop(viewer, None)
            if viewer in self._z_lock_listeners:
                listener = self._z_lock_listeners.pop(viewer)
                viewer.display_model.current_index.item_changed.disconnect(listener)

    def _on_sequence_started(
        self, sequence: useq.MDASequence, meta: SummaryMetaV1
    ) -> None:
        """Called when a new MDA sequence has been started.

        Every camera gets its own private, in-RAM ``NumpyDisplayStore``
        purely for display, regardless of whatever the MDA's real output/save
        handler is doing (pymmcore-plus 0.18 routes a str/Path output through
        a sink that isn't discoverable via the now-deprecated
        ``mda.get_output_handlers()``, so there's no reliable way to reuse the
        real save handler as a display source here). This intentionally does
        NOT use tensorstore -- see ``NumpyDisplayStore`` docstring for why.
        Then we create a new ndv viewer and show it.
        """
        self._is_mda_running = True

        self._own_handler = None
        self._mda_camera_handlers.clear()
        self._mda_camera_viewers.clear()

        labels = self._get_physical_camera_labels()
        if len(labels) > 1:
            # Multi-camera: every physical camera frame shares the same event
            # index, so a single store/viewer would overwrite frames.  Give each
            # camera its own in-memory display handler + viewer (independent of any
            # save handler), routing frames by ``meta["camera_device"]``.
            for label in labels:
                handler = NumpyDisplayStore()
                handler.reset(sequence)
                self._mda_camera_handlers[label] = handler
                self._mda_camera_viewers[label] = self._create_ndv_viewer(
                    sequence, label
                )
            self._active_mda_viewer = None
            return

        self._own_handler = NumpyDisplayStore()
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
            # Each per-camera store is built from ``seq.sizes`` (no ``cam`` axis),
            # so drop the engine's redundant ``cam`` index before writing/indexing.
            event = without_cam_index(event)
            handler.frameReady(frame, event, meta)
            self._update_mda_viewer(viewer, handler, event)
            return

        # Single-camera path.
        if self._own_handler is not None:
            self._own_handler.frameReady(frame, event, meta)

        if (viewer := self._active_mda_viewer) is None:
            return  # pragma: no cover

        self._update_mda_viewer(viewer, self._own_handler, event)

    def _update_mda_viewer(
        self,
        viewer: ndv.ArrayViewer,
        handler: NumpyDisplayStore | None,
        event: useq.MDAEvent,
    ) -> None:
        """Point the viewer at the handler array, or update its current index."""
        if handler is None:
            return  # pragma: no cover

        # if the viewer does not yet have data, it's likely the very first frame
        # so update the viewer's data source to the underlying handler's array,
        # wrapped so its axes carry the real t/p/z/c labels event.index uses
        # (see _LabeledArrayWrapper docstring for why this can't just be a
        # bare `viewer.data = handler.array`).
        if viewer.data_wrapper is None:
            viewer.data = _LabeledArrayWrapper(handler)
            return

        # Otherwise, move the viewer's slider to the most recently acquired
        # frame. At real acquisition frame rates, scheduling a brand-new
        # QTimer.singleShot per frame (as before) piles up callbacks on the Qt
        # event loop faster than they can run -- only the *latest* event
        # actually matters (each update just moves the slider to "wherever we
        # are now"), so coalesce: at most one deferred update in flight per
        # viewer, always reflecting the latest event. The 10ms delay (kept
        # from the original implementation) works around data handlers
        # writing asynchronously, so the frame may not be available to the
        # viewer immediately after the handler's frameReady method is called.
        already_pending = viewer in self._pending_viewer_updates
        self._pending_viewer_updates[viewer] = event
        if already_pending:
            return  # already scheduled -- it will pick up this latest event

        def _update(v: ndv.ArrayViewer = viewer) -> None:
            latest = self._pending_viewer_updates.pop(v, None)
            if latest is None:
                return  # pragma: no cover
            locked_z = self._locked_z_axis.get(v)
            if locked_z is not None and latest.index.get("z") != locked_z:
                return  # locked to a different z-slice -- skip this frame
            try:
                v.display_model.current_index.update(latest.index.items())
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
