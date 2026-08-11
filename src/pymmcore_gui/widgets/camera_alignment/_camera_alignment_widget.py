"""Camera Alignment widget: dual-camera overlay + spot tracking + displacement chart.

Manual optical-alignment aid for dual-camera (e.g. OPM/SPIM) rigs. Reads
``.data`` off the two physical cameras' already-live ``PygfxPreview`` docks
(via ``NDVViewersManager.get_or_create_camera_preview``) on its own ~10 Hz
poll timer -- it does not fetch frames itself and does not modify
``_ndv_viewers.py``. The poll timer runs continuously as soon as 2 physical
cameras are resolved, independent of whether any spot has been picked --
picking is only needed for displacement tracking, not for the live overlay
itself.

Scope (deliberate): Live/Snap only, one spot per camera, XY translation only.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus

from pymmcore_gui._multi_camera_handler import physical_camera_labels
from pymmcore_gui._qt.QtCore import QObject, QTimer
from pymmcore_gui._qt.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from pymmcore_gui._spot_tracking import (
    DEFAULT_BOX_RADIUS,
    TrackedSpot,
    TrackFunc,
    centroid_track,
    displacement,
)

from ._overlay_canvas import OverlayCanvas
from ._strip_chart import DisplacementStripChart

if TYPE_CHECKING:
    from pymmcore_gui._main_window import MicroManagerGUI
    from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

POLL_INTERVAL_MS = 100  # ~10 Hz -- a manual-alignment aid, not hard-real-time
_CAM1_COLOR = "magenta"
_CAM2_COLOR = "green"


class CameraAlignmentWidget(QWidget):
    """Overlay two cameras' live views and track a picked spot on each.

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
        self._cam1: str | None = None
        self._cam2: str | None = None
        self._preview1: PygfxPreview | None = None
        self._preview2: PygfxPreview | None = None
        self._spot1: TrackedSpot | None = None
        self._spot2: TrackedSpot | None = None
        # The one swap point for a future FFT-cross-correlation tracker.
        self._track_fn: TrackFunc = centroid_track

        self._overlay = OverlayCanvas(self, color1=_CAM1_COLOR, color2=_CAM2_COLOR)
        self._chart = DisplacementStripChart(self)
        self._readout = QLabel(self)
        self._status = QLabel(self)
        self._pick1_btn = QPushButton("Pick Spot — Camera 1", self)
        self._pick2_btn = QPushButton("Pick Spot — Camera 2", self)
        self._clear_btn = QPushButton("Clear", self)
        self._pick1_btn.clicked.connect(self._on_pick1_clicked)
        self._pick2_btn.clicked.connect(self._on_pick2_clicked)
        self._clear_btn.clicked.connect(self._on_clear_clicked)

        buttons = QHBoxLayout()
        buttons.addWidget(self._pick1_btn)
        buttons.addWidget(self._pick2_btn)
        buttons.addWidget(self._clear_btn)

        layout = QVBoxLayout(self)
        layout.addWidget(self._status)
        layout.addWidget(self._overlay, 3)
        layout.addLayout(buttons)
        layout.addWidget(self._readout)
        layout.addWidget(self._chart, 1)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._on_poll)

        self._mmc.events.systemConfigurationLoaded.connect(self._on_config_loaded)
        self.destroyed.connect(self._on_destroyed)
        self._on_config_loaded()

    # ---------------------- camera resolution / config reactivity -----------

    def _on_config_loaded(self) -> None:
        """Re-resolve physical camera labels; called on init and config reload.

        Widget-action widgets are cached singletons created once, so this
        must react to a later config load that changes which/how-many
        cameras exist rather than freezing at creation time (mirrors
        ``CrispWidget._rebuild``'s rationale).
        """
        self._reset_tracking()
        self._poll_timer.stop()
        labels = physical_camera_labels(self._mmc)
        if len(labels) < 2:
            self._cam1 = self._cam2 = None
            self._preview1 = self._preview2 = None
            self._set_pick_enabled(False)
            self._status.setText(
                f"Camera Alignment needs 2 cameras (found {len(labels)})."
            )
            return
        if len(labels) > 2:
            self._status.setText(
                f"Using first 2 of {len(labels)} cameras: {labels[0]}, {labels[1]}"
            )
        else:
            self._status.setText(f"{labels[0]}  /  {labels[1]}")
        self._cam1, self._cam2 = labels[0], labels[1]
        # Resolve (creating if needed) each camera's live-preview widget once here,
        # rather than on every poll tick -- PygfxPreview lookup re-shows/raises its
        # dock, which we don't want firing at 10 Hz.
        self._preview1 = self._preview(self._cam1)
        self._preview2 = self._preview(self._cam2)
        self._set_pick_enabled(True)
        # Show the live overlay immediately -- picking spots is only needed for
        # displacement tracking, not for the overlay view itself.
        self._poll_timer.start()

    def _set_pick_enabled(self, enabled: bool) -> None:
        self._pick1_btn.setEnabled(enabled)
        self._pick2_btn.setEnabled(enabled)
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

    # ---------------------------------- picking ------------------------------

    def _on_pick1_clicked(self) -> None:
        self._overlay.begin_point_pick(self._on_pick1_done)

    def _on_pick1_done(self, world_xy: tuple[float, float]) -> None:
        seed = self._seed_spot_from_click(world_xy, self._preview1)
        self._spot1 = seed
        self._overlay.set_marker("cam1", (seed.x, seed.y), _CAM1_COLOR)
        self._start_fresh_trace_if_ready()

    def _on_pick2_clicked(self) -> None:
        self._overlay.begin_point_pick(self._on_pick2_done)

    def _on_pick2_done(self, world_xy: tuple[float, float]) -> None:
        seed = self._seed_spot_from_click(world_xy, self._preview2)
        self._spot2 = seed
        self._overlay.set_marker("cam2", (seed.x, seed.y), _CAM2_COLOR)
        self._start_fresh_trace_if_ready()

    def _seed_spot_from_click(
        self, world_xy: tuple[float, float], preview: PygfxPreview | None
    ) -> TrackedSpot:
        """Build the initial `TrackedSpot` for a click, snapped to the true peak.

        A raw click is rarely dead-center on the feature; immediately
        refining against the current frame means the crosshair lands on the
        true local peak right away instead of only correcting on the next
        poll tick. Falls back to the raw click position if there's no frame
        yet or the click landed too close to the frame edge to track.
        """
        px, py = self._overlay.world_to_pixel(world_xy)
        raw = TrackedSpot(x=px, y=py, box_radius=DEFAULT_BOX_RADIUS)
        frame = preview.data if preview is not None else None
        if frame is None:
            return raw
        return centroid_track(frame, raw) or raw

    def _start_fresh_trace_if_ready(self) -> None:
        """Clear the chart for a new baseline once both spots are picked."""
        if self._spot1 is not None and self._spot2 is not None:
            self._chart.clear()

    def _on_clear_clicked(self, _checked: bool = False) -> None:
        self._reset_tracking()

    # ---------------------------- poll / track / report -----------------------

    def _on_poll(self) -> None:
        if self._mmc.mda.is_running():
            self._status.setText("Paused during MDA acquisition.")
            return
        self._status.setText(f"{self._cam1}  /  {self._cam2}")

        f1 = self._preview1.data if self._preview1 is not None else None
        f2 = self._preview2.data if self._preview2 is not None else None
        if f1 is not None:
            self._overlay.set_frame1(f1)
        if f2 is not None:
            self._overlay.set_frame2(f2)

        if self._spot1 is not None and f1 is not None:
            if (new1 := self._track_fn(f1, self._spot1)) is not None:
                self._spot1 = new1
                self._overlay.set_marker("cam1", (new1.x, new1.y), _CAM1_COLOR)
        if self._spot2 is not None and f2 is not None:
            if (new2 := self._track_fn(f2, self._spot2)) is not None:
                self._spot2 = new2
                self._overlay.set_marker("cam2", (new2.x, new2.y), _CAM2_COLOR)

        if self._spot1 is not None and self._spot2 is not None:
            dx, dy, mag = displacement(self._spot1, self._spot2)
            self._readout.setText(
                f"dx = {dx:+.2f} px   dy = {dy:+.2f} px   |d| = {mag:.2f} px"
            )
            self._chart.add_sample(dx, dy, mag)

    def _reset_tracking(self) -> None:
        self._spot1 = self._spot2 = None
        self._overlay.set_marker("cam1", None, _CAM1_COLOR)
        self._overlay.set_marker("cam2", None, _CAM2_COLOR)
        self._chart.clear()
        self._readout.clear()

    def _on_destroyed(self, _obj: QObject | None = None) -> None:
        self._poll_timer.stop()
        with suppress(Exception):
            self._mmc.events.systemConfigurationLoaded.disconnect(
                self._on_config_loaded
            )
