"""Camera Alignment widget: N-layer overlay + spot tracking + displacement chart.

Manual optical-alignment aid for image-splitter / dual-camera (e.g. OPM/SPIM)
rigs. The overlay shows N tinted, additively-blended *layers*. A layer is either
a spectral-region crop of one camera's live frame (so the top and bottom regions
of the same camera, and regions across cameras, can all be co-aligned in
region-local pixel coordinates) or a whole camera frame. One layer is the
reference (by default the 488 nm / green one); every other tracked layer's
displacement is reported relative to it.

The widget reads ``.data`` off each physical camera's already-live
``PygfxPreview`` dock (via ``NDVViewersManager.get_or_create_camera_preview``)
on its own ~10 Hz poll timer -- it does not fetch frames itself, makes no
camera driver calls, and does not modify ``_ndv_viewers.py``. The poll timer
runs continuously as soon as at least 2 layers resolve, independent of whether
any spot has been picked -- picking is only needed for displacement tracking,
not for the live overlay itself.

Scope (deliberate): Live/Snap only (paused during an MDA), one spot per layer,
XY translation only (no rotation/scale), a manual aid -- nothing is corrected
automatically.
"""

from __future__ import annotations

from contextlib import suppress
from functools import partial
from html import escape
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._qt.QtCore import QObject, Qt, QTimer
from pymmcore_gui._qt.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._settings import SettingsV1
from pymmcore_gui._spectral_channel_handler import roi_origin
from pymmcore_gui._spot_tracking import (
    DEFAULT_BOX_RADIUS,
    TrackedSpot,
    TrackFunc,
    centroid_track,
    displacement,
)

from ._layer_controls import LayerControlsPanel
from ._layers import (
    MODE_CAMERAS,
    MODE_REGIONS,
    LayerDisplay,
    LayerSpec,
    camera_layer_specs,
    crop_layer_frame,
    default_reference_key,
    region_layer_specs,
    spectral_signature,
)
from ._overlay_canvas import OverlayCanvas
from ._strip_chart import DisplacementStripChart

if TYPE_CHECKING:
    import numpy as np

    from pymmcore_gui._main_window import MicroManagerGUI
    from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

POLL_INTERVAL_MS = 100  # ~10 Hz -- a manual-alignment aid, not hard-real-time
_OUTSIDE_HINT = "outside frame -- check region / camera ROI"
_MIN_LAYERS = 2


def _plural(n: int, noun: str) -> str:
    return f"{n} {noun}" if n == 1 else f"{n} {noun}s"


class CameraAlignmentWidget(QWidget):
    """Overlay N alignment layers and track a picked spot on each.

    Layers are spectral-region crops ("Spectral regions" mode) or whole camera
    frames ("Whole cameras" mode). Each layer has its own visibility, colour,
    contrast range and tracked spot; displacements are reported relative to a
    chosen reference layer.

    Parameters
    ----------
    parent : QWidget | None
        Optional parent widget. By default, None.
    mmcore : CMMCorePlus | None
        Optional [`pymmcore_plus.CMMCorePlus`][] micromanager core. By
        default, None, in which case the active (or a new)
        [`CMMCorePlus.instance`][pymmcore_plus.core._mmcore_plus.CMMCorePlus.instance]
        is used.
    """

    def __init__(
        self, parent: QWidget | None = None, *, mmcore: CMMCorePlus | None = None
    ) -> None:
        super().__init__(parent)
        self._mmc = mmcore or CMMCorePlus.instance()

        self._mode = MODE_CAMERAS
        self._mode_user_chosen = False
        self._active = False
        self._specs: list[LayerSpec] = []
        self._sig: tuple = ()
        # Colour/visibility/contrast per layer key. Never cleared, so these
        # survive refreshes and mode switches within the session.
        self._displays: dict[str, LayerDisplay] = {}
        self._previews: dict[str, PygfxPreview | None] = {}
        self._origins: dict[str, tuple[int, int]] = {}
        self._shapes: dict[str, tuple[int, ...]] = {}
        # Identity (not content) of the last frame seen from each camera, so
        # `_on_poll` can tell a genuinely new frame apart from the same stale
        # array returned tick after tick while acquisition is paused/stopped.
        self._last_frames: dict[str, np.ndarray | None] = {}
        self._crops: dict[str, np.ndarray] = {}
        self._spots: dict[str, TrackedSpot] = {}
        self._outside: set[str] = set()
        self._ref_key: str | None = None
        self._chart_key: str | None = None
        self._status_text = ""
        # The one swap point for a future FFT-cross-correlation tracker.
        self._track_fn: TrackFunc = centroid_track

        self._overlay = OverlayCanvas(self)
        self._panel = LayerControlsPanel(self)
        self._chart = DisplacementStripChart(self)
        self._readout = QLabel(self)
        self._readout.setTextFormat(Qt.TextFormat.RichText)
        self._status = QLabel(self)

        self._mode_combo = QComboBox(self)
        self._mode_combo.addItems([MODE_REGIONS, MODE_CAMERAS])
        self._mode_combo.setCurrentText(self._mode)
        self._refresh_btn = QPushButton("Refresh", self)
        self._refresh_btn.setToolTip("Re-read cameras and spectral regions")
        self._ref_combo = QComboBox(self)
        self._ref_combo.setToolTip("Reference layer: displacements are relative to it")
        self._chart_combo = QComboBox(self)
        self._chart_combo.setToolTip("Layer plotted in the strip chart")
        self._pick_all_btn = QPushButton("Pick all", self)
        self._pick_all_btn.setToolTip("One click seeds a spot on every visible layer")
        self._clear_btn = QPushButton("Clear", self)

        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self._refresh_btn.clicked.connect(self._on_refresh_clicked)
        self._ref_combo.currentIndexChanged.connect(self._on_reference_changed)
        self._chart_combo.currentIndexChanged.connect(self._on_chart_layer_changed)
        self._pick_all_btn.clicked.connect(self._on_pick_all_clicked)
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        self._panel.visibilityChanged.connect(self._on_visibility_changed)
        self._panel.colorChanged.connect(self._on_color_changed)
        self._panel.climChanged.connect(self._on_clim_changed)
        self._panel.pickRequested.connect(self._on_pick_clicked)

        top = QHBoxLayout()
        top.addWidget(QLabel("Mode:", self))
        top.addWidget(self._mode_combo)
        top.addWidget(self._refresh_btn)
        top.addWidget(QLabel("Reference:", self))
        top.addWidget(self._ref_combo)
        top.addWidget(QLabel("Chart:", self))
        top.addWidget(self._chart_combo)
        top.addStretch(1)

        buttons = QHBoxLayout()
        buttons.addWidget(self._pick_all_btn)
        buttons.addWidget(self._clear_btn)
        buttons.addStretch(1)

        layout = QVBoxLayout(self)
        layout.addWidget(self._status)
        layout.addLayout(top)
        layout.addWidget(self._overlay, 3)
        layout.addWidget(self._panel)
        layout.addLayout(buttons)
        layout.addWidget(self._readout)
        layout.addWidget(self._chart, 1)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._on_poll)

        self._mmc.events.systemConfigurationLoaded.connect(self._on_config_loaded)
        self.destroyed.connect(self._on_destroyed)
        self._on_config_loaded()

    # ------------------ layer resolution / config reactivity ----------------

    def _on_config_loaded(self) -> None:
        """Re-resolve cameras and layers; called on init and config reload.

        Widget-action widgets are cached singletons created once, so this
        must react to a later config load that changes which/how-many
        cameras exist rather than freezing at creation time (mirrors
        ``CrispWidget._rebuild``'s rationale).
        """
        self._rebuild_layers()

    def _on_refresh_clicked(self, _checked: bool = False) -> None:
        self._rebuild_layers()

    def _on_mode_changed(self, index: int) -> None:
        if index < 0:
            return
        self._mode = self._mode_combo.itemText(index)
        self._mode_user_chosen = True
        self._rebuild_layers()

    def _rebuild_layers(self) -> None:
        """Resolve cameras, layer specs, previews and ROI origins from scratch.

        Display state (colour, visibility, contrast) lives in ``self._displays``
        and is kept; tracking state (spots, markers, chart) is reset.
        """
        self._poll_timer.stop()
        self._reset_tracking()
        self._crops.clear()
        self._outside.clear()
        self._shapes.clear()

        channels = SettingsV1.instance().spectral.channels
        self._sig = spectral_signature(channels)
        labels = physical_camera_labels(self._mmc)
        region_specs = region_layer_specs(channels, labels)
        camera_specs = camera_layer_specs(labels)
        if not self._mode_user_chosen:
            self._mode = (
                MODE_REGIONS if len(region_specs) >= _MIN_LAYERS else MODE_CAMERAS
            )
        by_mode = {MODE_REGIONS: region_specs, MODE_CAMERAS: camera_specs}
        specs = by_mode[self._mode]
        self._active = len(specs) >= _MIN_LAYERS

        self._mode_combo.blockSignals(True)
        self._mode_combo.setCurrentText(self._mode)
        self._mode_combo.blockSignals(False)

        if not self._active:
            specs = []
            other = MODE_CAMERAS if self._mode == MODE_REGIONS else MODE_REGIONS
            hint = (
                f" Switch Mode to '{other}'."
                if len(by_mode[other]) >= _MIN_LAYERS
                else ""
            )
            self._status_text = (
                "Camera Alignment needs 2 cameras or 2 spectral regions "
                f"(found {_plural(len(labels), 'camera')}, "
                f"{_plural(len(region_specs), 'region')})." + hint
            )
        self._specs = specs

        # Resolve (creating if needed) each camera's live-preview widget once
        # here, rather than on every poll tick -- PygfxPreview lookup re-shows/
        # raises its dock, which we don't want firing at 10 Hz. The ROI origin
        # is a device call when no worker service is active, so it is cached
        # too (and re-read only when a frame's shape changes).
        cameras = list(dict.fromkeys(s.camera for s in specs))
        self._previews = {cam: self._preview(cam) for cam in cameras}
        self._origins = {cam: roi_origin(self._mmc, cam) for cam in cameras}

        for spec in specs:
            self._displays.setdefault(spec.key, LayerDisplay(color=spec.default_color))
        self._overlay.set_layers([(s.key, self._displays[s.key].color) for s in specs])
        for spec in specs:
            disp = self._displays[spec.key]
            self._overlay.set_visible(spec.key, disp.visible)
            self._overlay.set_clim(spec.key, None if disp.auto_contrast else disp.clim)
        self._panel.set_layers(specs, self._displays)
        for row in self._panel.rows():
            row.set_pick_enabled(self._active)
        self._set_pick_enabled(self._active)
        self._populate_combos()

        if self._active:
            cams = ", ".join(cameras)
            self._status_text = (
                f"{self._mode}: {_plural(len(specs), 'layer')} on {cams}"
            )
            # Show the live overlay immediately -- picking spots is only needed
            # for displacement tracking, not for the overlay view itself.
            self._poll_timer.start()
        self._status.setText(self._status_text)

    def _hidden_keys(self) -> list[str]:
        return [s.key for s in self._specs if not self._displays[s.key].visible]

    def _populate_combos(self) -> None:
        """Fill the Reference and Chart combos from the current layers."""
        prev_ref = self._ref_key
        keys = [s.key for s in self._specs]
        ref: str | None
        if prev_ref in keys:
            ref = prev_ref
        else:
            ref = default_reference_key(self._specs, hidden=self._hidden_keys())
        self._ref_key = ref
        self._fill_combo(self._ref_combo, ref)
        self._populate_chart_combo()

    def _populate_chart_combo(self) -> None:
        others = [s.key for s in self._specs if s.key != self._ref_key]
        chart: str | None
        if self._chart_key in others:
            chart = self._chart_key
        else:
            chart = next((k for k in others if k in self._spots), None)
            if chart is None and others:
                chart = others[0]
        self._chart_key = chart
        self._fill_combo(self._chart_combo, chart, exclude=self._ref_key)

    def _fill_combo(
        self, combo: QComboBox, current: str | None, exclude: str | None = None
    ) -> None:
        combo.blockSignals(True)
        combo.clear()
        for spec in self._specs:
            if spec.key != exclude:
                combo.addItem(spec.name, spec.key)
        index = combo.findData(current) if current is not None else -1
        combo.setCurrentIndex(index)
        combo.blockSignals(False)

    def _set_pick_enabled(self, enabled: bool) -> None:
        self._pick_all_btn.setEnabled(enabled)
        self._clear_btn.setEnabled(enabled)

    def _main_window(self) -> MicroManagerGUI | None:
        from pymmcore_gui.actions.widget_actions import _get_mm_main_window

        # _get_mm_main_window is typed against PyQt6.QtCore.QObject, while this
        # module's QWidget resolves to PySide6 under TYPE_CHECKING (see
        # pymmcore_gui._qt); both bindings are duck-type compatible at runtime.
        return _get_mm_main_window(self)  # type: ignore[arg-type]

    def _preview(self, label: str | None) -> PygfxPreview | None:
        if label is None or (win := self._main_window()) is None:
            return None
        return win.viewers_manager.get_or_create_camera_preview(label)

    def _spec(self, key: str) -> LayerSpec | None:
        return next((s for s in self._specs if s.key == key), None)

    # ------------------------------ layer controls --------------------------

    def _on_visibility_changed(self, key: str, visible: bool) -> None:
        if (disp := self._displays.get(key)) is None:
            return
        disp.visible = visible
        self._overlay.set_visible(key, visible)
        if visible and (spec := self._spec(key)) is not None:
            # Force the camera's next frame to count as new, so the layer that
            # was skipped while hidden is re-pushed on the next poll.
            self._last_frames.pop(spec.camera, None)
        self._update_readout()

    def _on_color_changed(self, key: str, color: str) -> None:
        if (disp := self._displays.get(key)) is None:
            return
        disp.color = color
        self._overlay.set_color(key, color)
        self._update_readout()

    def _on_clim_changed(self, key: str, clim: tuple[float, float] | None) -> None:
        if (disp := self._displays.get(key)) is None:
            return
        if clim is None:
            disp.auto_contrast = True
            self._overlay.set_clim(key, None)
        else:
            lo, hi = clim
            disp.auto_contrast = False
            disp.clim = (float(lo), float(hi))
            self._overlay.set_clim(key, disp.clim)

    def _on_reference_changed(self, index: int) -> None:
        if index < 0:
            return
        self._ref_key = self._ref_combo.itemData(index)
        self._populate_chart_combo()
        self._chart.clear()
        self._update_readout()

    def _on_chart_layer_changed(self, index: int) -> None:
        if index < 0:
            return
        self._chart_key = self._chart_combo.itemData(index)
        self._chart.clear()

    # ---------------------------------- picking ------------------------------

    def _on_pick_clicked(self, key: str) -> None:
        self._overlay.begin_point_pick(partial(self._on_pick_done, key))

    def _on_pick_all_clicked(self, _checked: bool = False) -> None:
        self._overlay.begin_point_pick(self._on_pick_all_done)

    def _on_pick_done(self, key: str, world_xy: tuple[float, float]) -> None:
        seed = self._seed_spot_from_click(world_xy, self._crops.get(key))
        self._spots[key] = seed
        self._overlay.set_marker(key, (seed.x, seed.y))
        if key in (self._ref_key, self._chart_key):
            self._chart.clear()
        self._update_readout()

    def _on_pick_all_done(self, world_xy: tuple[float, float]) -> None:
        for spec in self._specs:
            if not self._displays[spec.key].visible:
                continue
            seed = self._seed_spot_from_click(world_xy, self._crops.get(spec.key))
            self._spots[spec.key] = seed
            self._overlay.set_marker(spec.key, (seed.x, seed.y))
        self._chart.clear()
        self._update_readout()

    def _seed_spot_from_click(
        self, world_xy: tuple[float, float], frame: np.ndarray | None
    ) -> TrackedSpot:
        """Build the initial `TrackedSpot` for a click, snapped to the true peak.

        A raw click is rarely dead-center on the feature; immediately
        refining against the layer's current pixels means the crosshair lands
        on the true local peak right away instead of only correcting on the
        next poll tick. Falls back to the raw click position if there's no
        frame yet or the click landed too close to the frame edge to track.
        """
        px, py = self._overlay.world_to_pixel(world_xy)
        raw = TrackedSpot(x=px, y=py, box_radius=DEFAULT_BOX_RADIUS)
        if frame is None:
            return raw
        return centroid_track(frame, raw) or raw

    def _on_clear_clicked(self, _checked: bool = False) -> None:
        self._reset_tracking()

    # ---------------------------- poll / track / report -----------------------

    def _on_poll(self) -> None:
        if self._mmc.mda.is_running():
            self._set_status("Paused during MDA acquisition.")
            return
        if spectral_signature(SettingsV1.instance().spectral.channels) != self._sig:
            self._rebuild_layers()
        if not self._active:
            return
        self._set_status(self._status_text)

        tracked_updated: set[str] = set()
        for cam, specs in self._specs_by_camera().items():
            visible = [s for s in specs if self._displays[s.key].visible]
            if not visible:
                continue
            preview = self._previews.get(cam)
            frame = preview.data if preview is not None else None
            # PygfxPreview.data returns the exact object last given to it via
            # `.append()`, unchanged in between -- so identity (not content)
            # comparison cheaply detects whether acquisition actually produced
            # a new frame since the last tick, vs. is just paused/stopped.
            is_new = frame is not None and frame is not self._last_frames.get(cam)
            self._last_frames[cam] = frame
            if not is_new or frame is None:
                continue
            origin = self._origin_for(cam, frame)
            for spec in visible:
                if self._push_layer(spec, frame, origin) and spec.key in self._spots:
                    tracked_updated.add(spec.key)

        if tracked_updated:
            self._update_readout()
            self._add_chart_sample(tracked_updated)

    def _specs_by_camera(self) -> dict[str, list[LayerSpec]]:
        out: dict[str, list[LayerSpec]] = {}
        for spec in self._specs:
            out.setdefault(spec.camera, []).append(spec)
        return out

    def _origin_for(self, cam: str, frame: np.ndarray) -> tuple[int, int]:
        """Cached ROI origin; re-read only when the frame shape changes (ROI)."""
        prev = self._shapes.get(cam)
        self._shapes[cam] = frame.shape
        if prev is not None and prev != frame.shape:
            self._origins[cam] = roi_origin(self._mmc, cam)
        return self._origins.get(cam, (0, 0))

    def _push_layer(
        self, spec: LayerSpec, frame: np.ndarray, origin: tuple[int, int]
    ) -> bool:
        """Crop, display and track one layer; True if it got new pixels."""
        key = spec.key
        row = self._panel.row(key)
        crop = crop_layer_frame(frame, spec, origin)
        if crop is None:
            self._crops.pop(key, None)
            if row is not None and key not in self._outside:
                row.set_status(_OUTSIDE_HINT)
            self._outside.add(key)
            return False
        if key in self._outside:
            self._outside.discard(key)
            if row is not None:
                row.set_status("")
        self._overlay.set_frame(key, crop)
        self._crops[key] = crop
        if row is not None and self._overlay.is_auto_clim(key):
            row.set_auto_range(*self._overlay.clim(key))
        if (spot := self._spots.get(key)) is not None:
            if (new := self._track_fn(crop, spot)) is not None:
                self._spots[key] = new
                self._overlay.set_marker(key, (new.x, new.y))
        return True

    def _add_chart_sample(self, tracked_updated: set[str]) -> None:
        ref, chart = self._ref_key, self._chart_key
        if ref is None or chart is None or ref == chart:
            return
        if not (ref in tracked_updated or chart in tracked_updated):
            return
        if not (self._displays[ref].visible and self._displays[chart].visible):
            return
        ref_spot, chart_spot = self._spots.get(ref), self._spots.get(chart)
        if ref_spot is not None and chart_spot is not None:
            self._chart.add_sample(*displacement(ref_spot, chart_spot))

    def _update_readout(self) -> None:
        """Rebuild the rich-text readout: each layer's offset vs the reference."""
        ref = self._ref_key
        ref_spec = self._spec(ref) if ref is not None else None
        ref_spot = self._spots.get(ref) if ref is not None else None
        if (
            ref is None
            or ref_spec is None
            or ref_spot is None
            or not self._displays[ref].visible
        ):
            self._readout.clear()
            return
        lines = []
        for spec in self._specs:
            disp = self._displays[spec.key]
            spot = self._spots.get(spec.key)
            if spec.key == ref or spot is None or not disp.visible:
                continue
            dx, dy, mag = displacement(ref_spot, spot)
            lines.append(
                f'<span style="color:{disp.color}">{escape(spec.name)}: '
                f"dx = {dx:+.2f} px &nbsp; dy = {dy:+.2f} px &nbsp; "
                f"|d| = {mag:.2f} px</span>"
            )
        if not lines:
            self._readout.clear()
            return
        header = f"Reference: <b>{escape(ref_spec.name)}</b>"
        self._readout.setText("<br>".join([header, *lines]))

    def _set_status(self, text: str) -> None:
        if self._status.text() != text:
            self._status.setText(text)

    def _reset_tracking(self) -> None:
        self._spots.clear()
        self._last_frames.clear()
        for key in self._overlay.layer_keys():
            self._overlay.set_marker(key, None)
        self._chart.clear()
        self._readout.clear()

    def _on_destroyed(self, _obj: QObject | None = None) -> None:
        self._poll_timer.stop()
        with suppress(Exception):
            self._mmc.events.systemConfigurationLoaded.disconnect(
                self._on_config_loaded
            )
