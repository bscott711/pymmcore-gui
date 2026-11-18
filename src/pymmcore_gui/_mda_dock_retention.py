"""Bound how many past MDA runs' viewer docks the GUI keeps alive.

Every MDA run adds one ndv dock per camera. Each dock holds its viewer, and the
viewer holds that run's in-RAM ``NumpyDisplayStore`` (up to the display window
budget, per camera). A dock that is merely hidden keeps all of it, so over a
long session the main process grew without bound.

``MdaDockRetention`` fixes this in two ways:

* every registered dock gets ``DockWidgetDeleteOnClose``, so closing a tab
  destroys the dock, its viewer and the display store;
* only the newest ``keep_runs`` runs' docks are kept; when a new run's dock is
  created, the docks of the oldest runs beyond that are deleted.

When a dock is destroyed, whatever else references its viewer (the manager's
slice-lock state and its current-run pointers, also while a run is acquiring) is
dropped too.
"""

from __future__ import annotations

import logging
from contextlib import suppress
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from pymmcore_gui._qt.QtAds import CDockWidget

if TYPE_CHECKING:
    from collections.abc import Callable

    import ndv
    from PyQt6Ads import CDockManager

    from pymmcore_gui._ndv_viewers import NDVViewersManager

logger = logging.getLogger("pymmcore_gui")


@dataclass
class _Entry:
    dock: CDockWidget
    viewer: ndv.ArrayViewer | None
    name: str
    label: str


@dataclass
class _Run:
    key: int
    entries: list[_Entry] = field(default_factory=list)


class MdaDockRetention:
    """Registry of MDA viewer docks, grouped by run, with a retention cap.

    Parameters
    ----------
    dock_manager : CDockManager
        The manager that owns the MDA docks.
    viewers_manager : NDVViewersManager
        Told to forget a viewer when its dock is destroyed.
    keep_runs : Callable[[], int]
        Number of newest runs whose docks are kept (read on every prune, so a
        settings change applies to the next run). Values below 1 count as 1.
    """

    def __init__(
        self,
        dock_manager: CDockManager,
        viewers_manager: NDVViewersManager,
        keep_runs: Callable[[], int],
    ) -> None:
        self._dock_manager = dock_manager
        self._viewers_manager = viewers_manager
        self._keep_runs = keep_runs
        self._runs: list[_Run] = []  # oldest first; the last one is the current run

    # ------------------------------------------------------------------

    def run_count(self) -> int:
        """Number of runs that still have at least one live dock."""
        return len(self._runs)

    def register(
        self,
        dock: CDockWidget,
        viewer: ndv.ArrayViewer,
        run_key: int,
        camera_label: str = "",
    ) -> None:
        """Track *dock* (holding *viewer*) as part of run *run_key*, then prune.

        *run_key* is the manager's run serial (one per ``sequenceStarted``): docks
        sharing it (one per camera) form one run, and a rerun of the same
        ``MDASequence`` gets a new serial, hence a new run. Call this once per
        dock, after it has been added to the dock manager.
        """
        dock.setFeature(CDockWidget.DockWidgetFeature.DockWidgetDeleteOnClose, True)

        entry = _Entry(dock, viewer, dock.objectName(), camera_label)
        if self._runs and self._runs[-1].key == run_key:
            run = self._runs[-1]
        else:
            run = _Run(run_key)
            self._runs.append(run)
        run.entries.append(entry)

        dock.destroyed.connect(lambda *_, e=entry: self._on_destroyed(e))
        self._prune()

    # ------------------------------------------------------------------

    def _on_destroyed(self, entry: _Entry) -> None:
        viewer, entry.viewer = entry.viewer, None
        # the python wrapper may outlive the C++ dock; don't let it pin the viewer
        entry.dock.__dict__.pop("_viewer", None)
        for run in self._runs:
            if entry in run.entries:
                run.entries.remove(entry)
        self._runs = [r for r in self._runs if r.entries]
        if viewer is not None:
            self._viewers_manager.forget_viewer(viewer)

    def _prune(self) -> None:
        keep = max(1, int(self._keep_runs()))
        # The newest run is the one being acquired; it is never deleted.
        while len(self._runs) > keep:
            run = self._runs.pop(0)
            names = [e.name for e in run.entries]
            logger.info(
                "Deleting MDA viewer docks %s of an older run (keeping the newest "
                "%d run(s); settings: window.mda_viewer_keep_runs) to free their "
                "display data.",
                names,
                keep,
            )
            for entry in run.entries:
                # drop the viewer reference first; the dock is deleted on the
                # next event-loop pass and ``_on_destroyed`` then finds nothing left
                viewer, entry.viewer = entry.viewer, None
                with suppress(RuntimeError):  # already deleted (e.g. closed)
                    entry.dock.deleteDockWidget()
                if viewer is not None:
                    self._viewers_manager.forget_viewer(viewer)
