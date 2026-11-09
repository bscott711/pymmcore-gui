"""MDA viewer docks must not pile up: each holds a run's in-RAM display store.

Drives the real NDVViewersManager + NumpyDisplayStore and the real
``MicroManagerGUI._on_mda_viewer_created`` (bound to a stub main window that owns a
real CDockManager), so no Micro-Manager and no ``mmcore`` fixture is needed.
"""

from __future__ import annotations

import gc
import time
import types
import weakref
from datetime import timedelta
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import pytest
import useq
from pydantic import ValidationError
from pymmcore_plus import CMMCorePlus

from pymmcore_gui import _ndv_viewers
from pymmcore_gui._main_window import MicroManagerGUI
from pymmcore_gui._mda_dock_retention import MdaDockRetention
from pymmcore_gui._numpy_display_store import NumpyDisplayStore
from pymmcore_gui._qt.QtAds import CDockManager, CDockWidget
from pymmcore_gui._qt.QtCore import QCoreApplication, QEvent
from pymmcore_gui._qt.QtWidgets import QApplication, QMainWindow, QWidget
from pymmcore_gui._settings import SettingsV1, WindowSettingsV1

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

LABELS = ["Camera-1", "Camera-2"]
SHAPE = (64, 64)


class _Host(QMainWindow):
    """Stub main window: just what ``_on_mda_viewer_created`` touches."""


def _widget() -> Any:
    return QWidget()


class _Rig:
    def __init__(self, qapp: QApplication, keep_runs: int) -> None:
        self.app = qapp
        self.keep_runs = keep_runs
        self.host: Any = _Host()
        h: Any = self.host
        self.mgr = _ndv_viewers.NDVViewersManager(h, CMMCorePlus())
        self.mgr._get_physical_camera_labels = lambda: list(LABELS)  # type: ignore[method-assign]
        h._viewers_manager = self.mgr
        h.dock_manager = CDockManager(h)
        h._central = CDockWidget("Viewers", h)
        h._central.setWidget(_widget())
        h._central_dock_area = h.dock_manager.setCentralWidget(h._central)
        h._camera_preview_areas = {}
        self.retention = MdaDockRetention(
            h.dock_manager,
            self.mgr,
            lambda: self.keep_runs,
        )
        h._mda_dock_retention = self.retention
        h._on_mda_viewer_created = types.MethodType(
            MicroManagerGUI._on_mda_viewer_created, h
        )
        self.mgr.mdaViewerCreated.connect(h._on_mda_viewer_created)
        h.resize(900, 600)
        h.show()
        self.settle()

    @property
    def dock_manager(self) -> CDockManager:
        return cast("CDockManager", self.host.dock_manager)

    def settle(self) -> None:
        # let the manager's coalesced 10 ms singleShot viewer updates fire
        end = time.monotonic() + 0.05
        while time.monotonic() < end:
            self.app.processEvents()
            # deleteLater of docks, and of PyQt's slot proxies once their sender died
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            time.sleep(0.005)

    def start_run(
        self, seq: useq.MDASequence | None = None, loops: int = 2
    ) -> useq.MDASequence:
        if seq is None:
            seq = useq.MDASequence(
                time_plan=useq.TIntervalLoops(interval=timedelta(0), loops=loops),
                z_plan=useq.ZRangeAround(range=2, step=1),
            )
        self.mgr._on_sequence_started(seq, cast("Any", {}))
        self.settle()
        return seq

    def feed(self, seq: useq.MDASequence, events: slice | None = None) -> None:
        frame = np.full(SHAPE, 7, dtype="uint16")
        for ev in list(seq)[events or slice(None)]:
            for label in LABELS:
                self.mgr._on_frame_ready(
                    frame, ev, cast("Any", {"camera_device": label})
                )
        self.settle()

    def run(self) -> useq.MDASequence:
        seq = self.start_run()
        self.feed(seq)
        self.mgr._on_sequence_finished(seq)
        self.settle()
        return seq

    def mda_docks(self) -> list[str]:
        return sorted(
            n for n in self.dock_manager.dockWidgetsMap() if n.startswith("ndv-")
        )

    def close(self) -> None:
        self.host.close()
        self.host.deleteLater()
        self.settle()


if TYPE_CHECKING:
    MakeRig = Callable[..., _Rig]


def live_stores() -> int:
    gc.collect()
    return sum(isinstance(o, NumpyDisplayStore) for o in gc.get_objects())


@pytest.fixture
def make_rig(qapp: QApplication) -> Iterator[MakeRig]:
    rigs: list[_Rig] = []

    def make(keep_runs: int = 2) -> _Rig:
        rig = _Rig(qapp, keep_runs)
        rigs.append(rig)
        return rig

    yield make
    for rig in rigs:
        rig.close()


def test_docks_and_stores_are_bounded_over_many_runs(make_rig: MakeRig) -> None:
    """Negative control: unbounded on 17d686d (4 runs x 2 cameras all stay)."""
    n_keep, n_runs = 2, 4
    rig = make_rig(n_keep)
    base_stores = live_stores()
    for _ in range(n_runs):
        rig.run()
    assert len(rig.mda_docks()) <= n_keep * len(LABELS)
    assert live_stores() - base_stores <= n_keep * len(LABELS)
    # exactly the newest runs survive
    assert rig.retention.run_count() == n_keep


def test_closing_a_dock_frees_its_store_and_lock_state(make_rig: MakeRig) -> None:
    rig = make_rig(keep_runs=5)
    base = live_stores()
    rig.run()
    assert live_stores() - base == len(LABELS)

    dock_name = (
        "ndv-"
        + str(next(iter(rig.mgr._seq_viewers)))[:8]
        + f"-{rig.mgr.run_serial}-Camera-1"
    )
    viewer = rig.mgr._mda_camera_viewers["Camera-1"]
    # lock state as set_viewer_slice_locked leaves it (the offscreen viewer has no
    # integer z/c index to capture, so build it directly)
    listener = rig.mgr._make_lock_listener(viewer)
    rig.mgr._locked_axes[viewer] = {"z": 0}
    rig.mgr._lock_listeners[viewer] = listener
    viewer.display_model.current_index.item_changed.connect(listener)
    del listener
    assert viewer in rig.mgr._locked_axes
    assert viewer in rig.mgr._lock_listeners
    del viewer

    dock = rig.dock_manager.dockWidgetsMap()[dock_name]
    dock.closeDockWidget()
    del dock
    rig.settle()

    assert dock_name not in rig.dock_manager.dockWidgetsMap()
    assert not rig.mgr._locked_axes
    assert not rig.mgr._lock_listeners
    assert "Camera-1" not in rig.mgr._mda_camera_viewers
    assert live_stores() - base == len(LABELS) - 1
    assert len(rig.mda_docks()) == len(LABELS) - 1


@pytest.mark.parametrize("close_after", [0, 3], ids=["before-1st-frame", "mid-run"])
def test_closing_current_runs_tab_mid_acquisition(
    make_rig: MakeRig, monkeypatch: pytest.MonkeyPatch, close_after: int
) -> None:
    """Closing the acquiring run's tab must not raise on later frames nor leak."""
    errors: list[str] = []
    monkeypatch.setattr(
        "sys.excepthook", lambda et, ev, tb: errors.append(f"{et.__name__}: {ev}")
    )
    rig = make_rig(keep_runs=5)
    base = live_stores()
    seq = rig.start_run(loops=4)  # 12 events per camera
    rig.feed(seq, slice(0, close_after))
    # let ndv's asynchronous data requests for those frames finish: a response that
    # lands on an already-deleted widget leaves an ndv-internal exception cycle that
    # pins the viewer (an ndv quirk, independent of this manager)
    for _ in range(6):
        rig.settle()

    dock_name = next(n for n in rig.mda_docks() if n.endswith("-Camera-1"))
    closed_store = weakref.ref(rig.mgr._mda_camera_handlers["Camera-1"])
    rig.dock_manager.dockWidgetsMap()[dock_name].closeDockWidget()
    rig.settle()
    assert dock_name not in rig.dock_manager.dockWidgetsMap()
    assert "Camera-1" not in rig.mgr._mda_camera_viewers
    assert "Camera-1" not in rig.mgr._mda_camera_handlers

    rig.feed(seq, slice(close_after, None))  # more timepoints, both cameras
    assert not errors
    # the other camera keeps updating
    handler2 = rig.mgr._mda_camera_handlers["Camera-2"]
    assert handler2.max_t_seen == 3
    viewer2 = rig.mgr._mda_camera_viewers["Camera-2"]
    assert viewer2.data_wrapper is not None
    rig.mgr._on_sequence_finished(seq)
    rig.settle()
    assert not errors
    del handler2, viewer2

    # Camera-1's store is no longer fed or routed to. (ndv may still pin it
    # briefly through its main-thread invoker's last callable, so it is only
    # checked as freed after further runs.)
    rig.keep_runs = 1
    rig.run()
    rig.run()
    assert not errors
    gc.collect()
    assert closed_store() is None
    assert live_stores() - base <= len(LABELS)


def test_closing_tab_with_ndv_request_in_flight_does_not_pin_the_store(
    make_rig: MakeRig, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Close right after frames arrive, without waiting for ndv's data responses.

    A response landing on the deleted widget raises inside ndv; the traceback
    then pins the camera's whole display store for good (and one more per such
    close). ``forget_viewer`` invalidates the viewer's pending requests.
    """
    errors: list[str] = []
    monkeypatch.setattr(
        "sys.excepthook", lambda et, ev, tb: errors.append(f"{et.__name__}: {ev}")
    )
    rig = make_rig(keep_runs=1)
    closed: list[weakref.ReferenceType[Any]] = []
    for _ in range(6):
        seq = rig.start_run(loops=3)
        rig.feed(seq, slice(0, 4))
        rig.app.processEvents()  # no settle loop: requests may still be in flight
        dock_name = next(n for n in rig.mda_docks() if n.endswith("-Camera-1"))
        closed.append(weakref.ref(rig.mgr._mda_camera_handlers["Camera-1"]))
        rig.dock_manager.dockWidgetsMap()[dock_name].closeDockWidget()
        rig.settle()
        rig.feed(seq, slice(4, None))
        rig.mgr._on_sequence_finished(seq)
        rig.settle()
    for _ in range(3):
        rig.run()
    gc.collect()
    gc.collect()
    assert not errors
    assert [r() is not None for r in closed] == [False] * len(closed)


def test_current_run_is_never_deleted_even_with_one_kept(make_rig: MakeRig) -> None:
    rig = make_rig(keep_runs=1)
    rig.run()
    first = set(rig.mda_docks())
    assert len(first) == len(LABELS)

    seq = rig.start_run()  # second run is acquiring
    current = {
        n
        for n in rig.dock_manager.dockWidgetsMap()
        if n.startswith(f"ndv-{str(seq.uid)[:8]}")
    }
    assert len(current) == len(LABELS)
    assert set(rig.mda_docks()) == current  # first run pruned, current untouched

    rig.feed(seq)  # frames still land in the live viewers
    assert set(rig.mda_docks()) == current
    rig.mgr._on_sequence_finished(seq)
    rig.settle()
    assert set(rig.mda_docks()) == current


def _same_seq() -> useq.MDASequence:
    return useq.MDASequence(
        time_plan=useq.TIntervalLoops(interval=timedelta(0), loops=1)
    )


def test_same_sequence_run_twice_counts_as_two_runs(make_rig: MakeRig) -> None:
    rig = make_rig(keep_runs=1)
    seq = _same_seq()
    names: list[set[str]] = []
    for _ in range(2):
        rig.mgr._on_sequence_started(seq, cast("Any", {}))
        rig.mgr._on_sequence_finished(seq)
        rig.settle()
        names.append(set(rig.mda_docks()))
    assert rig.retention.run_count() == 1
    # the second run's docks are all present (and registered), the first run's are
    # gone; names differ between runs, so deleting an old dock cannot drop the new
    # dock's map entry
    assert len(names[0]) == len(names[1]) == len(LABELS)
    assert names[0].isdisjoint(names[1])
    assert set(rig.mda_docks()) == names[1]
    viewers = set(rig.mgr._mda_camera_viewers.values())
    docks: list[Any] = [rig.dock_manager.dockWidgetsMap()[n] for n in names[1]]
    assert {d._viewer for d in docks} == viewers


def test_rerun_of_same_sequence_after_closing_a_tab(make_rig: MakeRig) -> None:
    """The rerun's docks all stay, even though one of its tabs was closed before."""
    rig = make_rig(keep_runs=1)
    seq = _same_seq()
    rig.mgr._on_sequence_started(seq, cast("Any", {}))
    rig.mgr._on_sequence_finished(seq)
    rig.settle()
    first = rig.mda_docks()
    assert len(first) == len(LABELS)
    rig.dock_manager.dockWidgetsMap()[first[0]].closeDockWidget()
    rig.settle()
    assert len(rig.mda_docks()) == len(LABELS) - 1

    rig.mgr._on_sequence_started(seq, cast("Any", {}))  # run 2, acquiring
    rig.settle()
    docks = rig.mda_docks()
    assert len(docks) == len(LABELS)
    assert not set(docks) & set(first)
    assert rig.retention.run_count() == 1
    viewers = rig.mgr._mda_camera_viewers
    dock_map: Any = rig.dock_manager.dockWidgetsMap()
    assert {dock_map[n]._viewer for n in docks} == set(viewers.values())
    rig.mgr._on_sequence_finished(seq)
    rig.settle()
    assert rig.mda_docks() == docks


def test_non_mda_docks_are_untouched(make_rig: MakeRig) -> None:
    rig = make_rig(keep_runs=1)
    preview = CDockWidget("preview-Camera-1", rig.host)
    preview.setWidget(_widget())
    rig.dock_manager.addDockWidgetTabToArea(
        preview,
        rig.host._central_dock_area,
    )
    other = CDockWidget("some-widget", rig.host)
    other.setWidget(_widget())
    rig.dock_manager.addDockWidgetTabToArea(
        other,
        rig.host._central_dock_area,
    )
    for _ in range(3):
        rig.run()
    names = set(rig.dock_manager.dockWidgetsMap())
    assert {"preview-Camera-1", "some-widget", "Viewers"} <= names
    # closing them only hides them (no DeleteOnClose): they stay registered
    preview.closeDockWidget()
    other.closeDockWidget()
    rig.settle()
    assert {"preview-Camera-1", "some-widget"} <= set(rig.dock_manager.dockWidgetsMap())
    assert len(rig.mda_docks()) == len(LABELS)


def test_keep_runs_setting_default_and_validation() -> None:
    assert WindowSettingsV1().mda_viewer_keep_runs == 2
    assert SettingsV1().window.mda_viewer_keep_runs == 2
    assert WindowSettingsV1(mda_viewer_keep_runs=1).mda_viewer_keep_runs == 1
    with pytest.raises(ValidationError):
        WindowSettingsV1(mda_viewer_keep_runs=0)
    # a settings file without the field still loads
    assert SettingsV1.model_validate({"window": {}}).window.mda_viewer_keep_runs == 2
