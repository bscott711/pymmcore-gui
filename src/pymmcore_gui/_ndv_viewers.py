from __future__ import annotations

import warnings
from contextlib import suppress
from datetime import datetime
from typing import TYPE_CHECKING, cast
from weakref import WeakSet, WeakValueDictionary

import ndv
import numpy as np
import useq
from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtWidgets import QWidget
from PyQt6Ads import CDockWidget

from pymmcore_gui._multi_camera_handler import physical_camera_labels, without_cam_index
from pymmcore_gui._numpy_display_store import NumpyDisplayStore
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui._snapshot_save import LatestFrame
from pymmcore_gui._spectral_channel_handler import roi_origin
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
)
from pymmcore_gui.asi_z_stack.image_focus_events import CALIBRATION_METADATA_KEY
from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

if TYPE_CHECKING:
    from collections.abc import Callable, Hashable, Iterator, Mapping, Sequence
    from typing import Any, Literal, TypeGuard

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

    Also bounds the ``"t"`` slider to timepoints that have actually been
    collected (``coords``), and translates the handler's rolling-window
    storage positions transparently on read (``isel``) -- see
    ``NumpyDisplayStore``'s module docstring for why the store only retains a
    bounded window of recent timepoints. This doesn't need an offset/
    non-zero-based ``coords`` range (which ``_norm_current_index`` mishandles,
    see above): ``useq.MDAEvent.index["t"]`` is a permanent, monotonic,
    0-based counter that's never renumbered as older timepoints are evicted,
    so ``coords["t"]`` can just report ``range(0, max_t_seen + 1)`` forever.

    ``"z"`` deliberately keeps its full declared range. Bounding it to the
    slices written so far for the filling volume made the newest frame
    always the slider's maximum, so in live mode the handle sat pinned at
    the end of the stack while the range grew underneath it. Planes not yet
    written in that volume are blanked on read instead (see
    ``NumpyDisplayStore.is_unwritten``).
    """

    def __init__(self, handler: NumpyDisplayStore) -> None:
        array = handler.array
        if array is None:  # pragma: no cover -- only constructed after frameReady
            raise ValueError("NumpyDisplayStore has no data yet")
        self._dims_ = handler.dims
        self._handler = handler
        # last-reported "t" bound, so dims_changed is only emitted when the
        # valid range has actually changed (avoids redundant slider rebuilds).
        self._last_t_bound = 0
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

    @property
    def coords(self) -> Mapping[Hashable, Sequence]:
        array = self._handler.array
        assert array is not None  # guaranteed by __init__ / handler lifecycle
        coords: dict[Hashable, Sequence] = {
            label: range(size)
            for label, size in zip(self._dims_, array.shape, strict=False)
        }
        if "t" in coords:
            # collected-data bound, not the (possibly much larger) declared
            # domain -- always 0-based since t is never renumbered.
            coords["t"] = range(self._handler.max_t_seen + 1)
        return coords

    def isel(self, index: Mapping[int, int | slice]) -> np.ndarray:
        by_label = {
            self._dims_[k]: index.get(k, slice(None)) for k in range(len(self._dims_))
        }
        t_val = by_label.get("t")
        if isinstance(t_val, slice):
            t_val = t_val.start
        evicted = t_val is not None and self._handler.is_evicted(t_val)

        storage = self._handler.to_storage_index(by_label)
        idx = tuple(storage[label] for label in self._dims_)
        array = self._handler.array
        assert array is not None
        result = self._asarray(array[idx])
        # An evicted t still maps to a *resident* storage slot (whatever
        # timepoint currently occupies it after wraparound) -- reading it is
        # safe but the data is stale/wrong, so zero it out. Reusing the real
        # read's shape here (rather than hand-computing it) keeps this
        # correct-by-construction for every visible-axes combination ndv can
        # request.
        if evicted:
            return np.zeros_like(result)
        if isinstance(t_val, int):
            return self._blank_unwritten(result, by_label, t_val)
        return result

    def _blank_unwritten(
        self, result: np.ndarray, by_label: Mapping[str, int | slice], t: int
    ) -> np.ndarray:
        """Zero any not-yet-written (c, z) planes of the filling volume in *result*.

        *result* holds one axis per slice-indexed label of *by_label* (in dims
        order, integer-indexed labels dropped), so a plane is addressed by
        the positions of "c"/"z" among those axes, when they're sliced.
        """
        p = by_label.get("p", 0)
        if not isinstance(p, int):
            return result
        kept = [label for label in self._dims_ if isinstance(by_label[label], slice)]
        shape = dict(zip(self._dims_, self._handler.array.shape, strict=False))  # type: ignore[union-attr]

        def _planes(label: str) -> list[tuple[int, int | None]]:
            # [(logical value, axis position in result or None if pinned)]
            idx = by_label.get(label, 0)
            if isinstance(idx, int):
                return [(idx, None)]
            values = range(*idx.indices(shape[label]))
            return [(v, i) for i, v in enumerate(values)]

        out = result
        for c, c_pos in _planes("c"):
            for z, z_pos in _planes("z"):
                if not self._handler.is_unwritten(t, p, c, z):
                    continue
                if out is result:
                    out = np.array(result, copy=True)
                sel: list[int | slice] = [slice(None)] * out.ndim
                if c_pos is not None:
                    sel[kept.index("c")] = c_pos
                if z_pos is not None:
                    sel[kept.index("z")] = z_pos
                out[tuple(sel)] = 0
        return out

    def notify_frame_written(self) -> None:
        """Re-check whether the collected-data bounds changed; called after writes."""
        new_t_bound = self._handler.max_t_seen + 1
        if new_t_bound != self._last_t_bound:
            self._last_t_bound = new_t_bound
            self.dims_changed.emit()


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

        # {viewer: {"z": locked z, "c": locked c}} for viewers in "locked
        # slice" playback mode (see set_viewer_slice_locked /
        # _update_mda_viewer). Absent from this dict means "live" mode -- the
        # default, always-jump-to-latest behavior.
        self._locked_axes: dict[ndv.ArrayViewer, dict[str, int]] = {}
        # {viewer: current_index.item_changed listener} for locked viewers, so
        # set_viewer_slice_locked(locked=False) / _cleanup can disconnect them.
        self._lock_listeners: dict[ndv.ArrayViewer, Callable[..., None]] = {}

        # Per-camera preview dock widgets, keyed by physical camera label.
        # e.g. {"Camera-1": <CDockWidget>, "Camera-2": <CDockWidget>}
        self._camera_previews: dict[str, CDockWidget] = {}

        # Newest Snap/Live frame per physical camera, kept so it can be saved
        # (see latest_frames / the Save Image action).
        self._latest_frames: dict[str, LatestFrame] = {}

        # Primary "streaming driver" preview — the first camera's PygfxPreview owns
        # the Qt timer that polls the circular buffer.  Other cameras' previews
        # receive frames via a callback set on this widget.
        self._streaming_driver: PygfxPreview | None = None

        # Per-camera MDA display handlers/viewers, keyed by physical camera label.
        # Populated only for multi-camera acquisitions; the single-camera path
        # continues to use ``_own_handler`` / ``_active_mda_viewer`` below.
        self._mda_camera_handlers: dict[str, NumpyDisplayStore] = {}
        self._mda_camera_viewers: dict[str, ndv.ArrayViewer] = {}

        # Camera worker service (persistent dual-PVCAM worker processes) --
        # see _sync_worker_service_connection for why this is re-evaluated
        # on every config load rather than connected once here.
        self._connected_worker_service: CameraWorkerService | None = None

        ev = self._mmc.events
        ev.imageSnapped.connect(self._on_image_snapped)
        ev.sequenceAcquisitionStarted.connect(self._on_streaming_started)
        ev.continuousSequenceAcquisitionStarted.connect(self._on_streaming_started)
        ev.propertyChanged.connect(self._on_property_changed)
        ev.systemConfigurationLoaded.connect(self._sync_worker_service_connection)
        self._sync_worker_service_connection()

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

        Delegates to the shared ``_multi_camera_handler.physical_camera_labels``,
        which falls back to ``CameraWorkerService``'s static label list when a
        persistent worker service is active (Camera-1/Camera-2 are then never
        loaded on ``self._mmc`` at all) -- one fix point instead of this
        method independently re-deriving the same thing from
        ``getNumberOfCameraChannels``/``getPhysicalCameraDevice``, which would
        return empty/stale results once the cameras are worker-owned.
        """
        return physical_camera_labels(self._mmc)

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
        # Regions are stored in full-sensor coordinates; a cropped camera's
        # frames (and so its preview) start at its ROI origin.
        ox, oy = roi_origin(self._mmc, camera_label)
        rois = [
            (c.name, (c.rect[0] - ox, c.rect[1] - oy, c.rect[2], c.rect[3]))
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

    def latest_frames(self) -> list[LatestFrame]:
        """Return the newest Snap/Live frame of each camera, in camera order.

        Frames from the single-camera streaming path (which appends inside the
        preview's own timer, with no hook here) are read back from the
        preview's texture instead.
        """
        frames = dict(self._latest_frames)
        for label, dw in self._camera_previews.items():
            if label in frames:
                continue
            data = cast("PygfxPreview", dw.widget()).data
            # A 1x1 texture is the placeholder shown before any frame arrives.
            if data is not None and data.size > 1:
                frames[label] = LatestFrame(label, data, "live", datetime.now())
        order = {label: i for i, label in enumerate(self._get_physical_camera_labels())}
        return sorted(frames.values(), key=lambda f: order.get(f.camera_label, 99))

    def _remember_frame(
        self,
        camera_label: str,
        frame: np.ndarray,
        source: Literal["snap", "live"],
        camera_metadata: dict[str, Any] | None = None,
    ) -> None:
        self._latest_frames[camera_label] = LatestFrame(
            camera_label, frame, source, datetime.now(), dict(camera_metadata or {})
        )

    def _dispatch_snap_to_previews(self, images: dict[int, np.ndarray]) -> None:
        """Send each snapped camera image to its dedicated preview widget."""
        labels = self._get_physical_camera_labels()
        for ch_idx, img in images.items():
            label = labels[ch_idx] if ch_idx < len(labels) else f"Camera-ch{ch_idx}"
            preview, _ = self._get_or_create_camera_preview(label)
            preview.append(img)
            self._remember_frame(label, img, "snap")

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
                self._remember_frame(label, frame, "live")

        return _on_frames

    # ------------------------------------------------------------------
    # Camera worker service (persistent dual-PVCAM worker processes)
    # ------------------------------------------------------------------

    def _sync_worker_service_connection(self) -> None:
        """(Re)connect to the active ``CameraWorkerService``, if any.

        Re-evaluated on every ``systemConfigurationLoaded`` rather than
        connected once, since the active service instance can change (or
        disappear/reappear) across a config reload -- see
        ``CameraWorkerService.prepare_for_reload``.
        """
        svc = CameraWorkerService.get_active()
        if svc is self._connected_worker_service:
            return
        if self._connected_worker_service is not None:
            with suppress(RuntimeError, TypeError):
                self._connected_worker_service.frameReady.disconnect(
                    self._on_worker_frame
                )
            with suppress(RuntimeError, TypeError):
                self._connected_worker_service.roiChanged.disconnect(
                    self._on_camera_roi_changed
                )
        if svc is not None:
            svc.frameReady.connect(self._on_worker_frame)
            svc.roiChanged.connect(self._on_camera_roi_changed)
        self._connected_worker_service = svc

    def _on_camera_roi_changed(self, camera_label: str) -> None:
        """Re-place overlays (and the default zoom) after a camera's ROI changed."""
        if (dw := self._camera_previews.get(camera_label)) is None:
            return
        preview = cast("PygfxPreview", dw.widget())
        rois = self._apply_roi_overlays(preview, camera_label)
        # The next frame has a new shape, which re-applies this default view.
        preview.set_default_zoom_rect(self._roi_union_rect(rois), apply=False)
        self._latest_frames.pop(camera_label, None)

    def _on_worker_frame(
        self,
        camera_label: str,
        slice_idx: int,
        frame: np.ndarray,
        camera_metadata: dict[str, Any],
        images_remaining: int,
    ) -> None:
        """Display one Live or Snap frame delivered by a persistent camera worker.

        Frames already arrive tagged with their physical camera at the
        source (``FrameMsg.camera_label``), so -- unlike
        ``_on_streaming_started``'s ``_multicam_frame_callback`` demuxing
        below, which exists only to split one shared circular buffer --
        this can dispatch straight to the right dock via a plain lookup.
        """
        if self._is_mda_running:
            return
        preview, _ = self._get_or_create_camera_preview(camera_label)
        preview.append(frame)
        svc = self._connected_worker_service
        live = svc is not None and svc.state is CameraWorkerServiceState.LIVE
        self._remember_frame(
            camera_label, frame, "live" if live else "snap", camera_metadata
        )

    # ------------------------------------------------------------------
    # Streaming / Snap handlers
    # ------------------------------------------------------------------
    #
    # NOTE: the two handlers below react to mmc's own sequence-acquisition/
    # imageSnapped events, which never fire once Live/Snap route through a
    # persistent CameraWorkerService (see core_actions.py) -- they stay
    # fully functional, unmodified, for demo/single-camera/non-ASI rigs,
    # and are simply dormant (never invoked) for the worker-owned rig,
    # where _on_worker_frame above handles frame delivery instead.

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
                img = self._mmc.getImage()
                preview.append(img)
                self._remember_frame(label, img, "snap")
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
        for viewer, listener in self._lock_listeners.items():
            with suppress(Exception):  # viewer may already be gone/destroyed
                viewer.display_model.current_index.item_changed.disconnect(listener)
        self._lock_listeners.clear()
        self._locked_axes.clear()

    def _make_lock_listener(self, viewer: ndv.ArrayViewer) -> Callable[..., None]:
        """Return a listener that keeps ``_locked_axes[viewer]`` in sync.

        Connected to ``current_index.item_changed`` while *viewer* is locked,
        so that manually dragging to a new slice or channel re-locks to it
        (rather than the one originally captured at lock time). The
        manager's own programmatic writes in ``_update_mda_viewer`` never
        touch a locked axis, so they can't move the lock.
        """

        def _on_item_changed(key: str, new_value: object, old_value: object) -> None:
            locked = self._locked_axes.get(viewer)
            if locked is not None and key in locked:
                locked[key] = cast("int", new_value)

        return _on_item_changed

    def set_viewer_slice_locked(self, viewer: ndv.ArrayViewer, locked: bool) -> None:
        """Toggle "locked slice" playback mode for a live-MDA *viewer*.

        In the default "live" mode, the viewer jumps to show every newly
        acquired frame. In "locked" mode, it captures whichever z slice and
        channel the viewer is showing right now and stops jumping around --
        it only updates (to the newer timepoint/position) when a new frame
        arrives at that same z and channel (see ``_update_mda_viewer``), so
        watching one plane over time isn't interrupted by frames from other
        planes or channels. In a multichannel (composite) display mode every
        channel is drawn anyway, so only z gates the update there. Manually
        dragging to a different slice or channel while locked re-locks to
        it. A no-op if the sequence has neither a z nor a channel axis.
        """
        if locked:
            current = dict(viewer.display_model.current_index)
            axes = {
                key: val
                for key in ("z", "c")
                if isinstance(val := current.get(key), int)
            }
            if axes:
                self._locked_axes[viewer] = axes
                listener = self._make_lock_listener(viewer)
                self._lock_listeners[viewer] = listener
                viewer.display_model.current_index.item_changed.connect(listener)
        else:
            self._locked_axes.pop(viewer, None)
            if viewer in self._lock_listeners:
                listener = self._lock_listeners.pop(viewer)
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

        if CALIBRATION_METADATA_KEY in sequence.metadata:
            # An image-focus calibration sweep: its frames are throwaway, so no
            # viewer. Dropping the handlers above (and the active viewer) also
            # stops _on_frame_ready writing them into the previous run's store.
            self._active_mda_viewer = None
            return

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

    def _locked_out(self, viewer: ndv.ArrayViewer, event: useq.MDAEvent) -> bool:
        """Return True if *viewer* is locked to a different slice/channel than *event*.

        Composite/multichannel modes draw every channel at once, so there only
        the locked z decides whether a frame is shown.
        """
        if (locked := self._locked_axes.get(viewer)) is None:
            return False
        multichannel = viewer.display_model.channel_mode.is_multichannel()
        return any(
            event.index.get(key) != val
            for key, val in locked.items()
            if not (key == "c" and multichannel)
        )

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
        #
        # A locked viewer's frames are filtered *here*, before coalescing: a
        # whole volume typically streams in before the deferred update fires,
        # so letting other planes into the pending slot would supersede the
        # locked plane's frame and it would never be shown.
        if self._locked_out(viewer, event):
            # the "t" slider must still grow as new timepoints are written.
            if isinstance(wrapper := viewer.data_wrapper, _LabeledArrayWrapper):
                wrapper.notify_frame_written()
            return
        already_pending = viewer in self._pending_viewer_updates
        self._pending_viewer_updates[viewer] = event
        if already_pending:
            return  # already scheduled -- it will pick up this latest event

        def _update(v: ndv.ArrayViewer = viewer) -> None:
            latest = self._pending_viewer_updates.pop(v, None)
            if latest is None:
                return  # pragma: no cover
            if isinstance(wrapper := v.data_wrapper, _LabeledArrayWrapper):
                wrapper.notify_frame_written()
            # re-check: the lock may have been toggled/moved since queuing.
            if self._locked_out(v, latest):
                return
            index = dict(latest.index)
            if (locked := self._locked_axes.get(v)) is not None:
                # never move a locked axis: only advance t/p etc.
                index = {k: val for k, val in index.items() if k not in locked}
            try:
                v.display_model.current_index.update(index.items())
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
