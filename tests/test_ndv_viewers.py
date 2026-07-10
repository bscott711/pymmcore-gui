from __future__ import annotations

import datetime
import gc
from typing import TYPE_CHECKING

import ndv
import numpy as np
import pytest
import useq
from useq import MDASequence

from pymmcore_gui._ndv_viewers import NDVViewersManager
from pymmcore_gui._qt.QtWidgets import QApplication, QWidget
from pymmcore_gui._vendored.mda_handlers import TensorStoreHandler

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot


def test_viewers_manager(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    """Ensure that the viewers manager creates and cleans up viewers during MDA."""
    dummy = QWidget()
    manager = NDVViewersManager(dummy, mmcore)

    assert len(manager) == 0
    mmcore.mda.run(
        MDASequence(
            time_plan=useq.TIntervalLoops(
                interval=datetime.timedelta(seconds=0.1), loops=2
            ),
            channels=["DAPI", "FITC"],  # pyright: ignore
            z_plan=useq.ZRangeAround(range=4, step=1),
        ),
    )
    assert len(manager) == 1

    with qtbot.waitSignal(dummy.destroyed, timeout=1000):
        dummy.deleteLater()
    QApplication.processEvents()
    gc.collect()
    if len(manager):
        for viewer in manager.viewers():
            if "vispy" in type(viewer._canvas).__name__.lower():
                # don't even bother... vispy is a mess of hard references
                del viewer._canvas
                continue
            referrers = gc.get_referrers(viewer)[1:]
            pytest.fail(f"Viewer {viewer} not deleted. Still referenced by {referrers}")


def test_z_lock_skips_non_matching_frames(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    """Locking a viewer to a z-index should ignore frames at other z indices."""
    dummy = QWidget()
    manager = NDVViewersManager(dummy, mmcore)

    seq = MDASequence(
        z_plan=useq.ZRangeAround(range=2, step=1),
        time_plan=useq.TIntervalLoops(interval=0, loops=2),  # pyright: ignore
    )
    events = list(seq)
    handler = TensorStoreHandler(driver="zarr", kvstore="memory://")
    handler.reset(seq)
    frame = np.zeros((4, 4), dtype="uint8")

    viewer = ndv.ArrayViewer()

    # first call just binds the viewer to the handler's store (no index
    # update happens yet); the second establishes a real current_index via
    # the normal coalesced-update path.
    first = events[0]
    handler.frameReady(frame, first, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, first)
    qtbot.wait(50)
    manager._update_mda_viewer(viewer, handler, first)
    qtbot.wait(50)
    locked_z = dict(viewer.display_model.current_index).get("z")
    assert locked_z is not None
    assert locked_z == first.index.get("z")

    manager.set_viewer_z_locked(viewer, True)
    assert manager._locked_z_axis[viewer] == locked_z

    other_z_event = next(e for e in events if e.index.get("z") != locked_z)
    handler.frameReady(frame, other_z_event, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, other_z_event)
    qtbot.wait(50)
    # a frame at a different z was skipped -- index unchanged
    assert dict(viewer.display_model.current_index).get("z") == locked_z
    assert dict(viewer.display_model.current_index).get("t") == first.index.get("t")

    same_z_new_t = next(
        e
        for e in events
        if e.index.get("z") == locked_z and e.index.get("t") != first.index.get("t")
    )
    handler.frameReady(frame, same_z_new_t, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, same_z_new_t)
    qtbot.wait(50)
    # a frame at the locked z (new timepoint) *does* update the view
    assert dict(viewer.display_model.current_index).get("z") == locked_z
    assert dict(viewer.display_model.current_index).get("t") == same_z_new_t.index.get(
        "t"
    )

    # unlocking resumes normal "jump to latest" behavior
    manager.set_viewer_z_locked(viewer, False)
    assert viewer not in manager._locked_z_axis
    manager._update_mda_viewer(viewer, handler, other_z_event)
    qtbot.wait(50)
    assert dict(viewer.display_model.current_index).get("z") == other_z_event.index.get(
        "z"
    )


def test_z_lock_tracks_manual_slice_drag(mmcore: CMMCorePlus, qtbot: QtBot) -> None:
    """Manually dragging to a new slice while locked re-locks to that slice."""
    dummy = QWidget()
    manager = NDVViewersManager(dummy, mmcore)

    seq = MDASequence(
        z_plan=useq.ZRangeAround(range=2, step=1),
        time_plan=useq.TIntervalLoops(interval=0, loops=3),  # pyright: ignore
    )
    events = list(seq)
    handler = TensorStoreHandler(driver="zarr", kvstore="memory://")
    handler.reset(seq)
    frame = np.zeros((4, 4), dtype="uint8")

    viewer = ndv.ArrayViewer()

    # first call just binds the viewer to the handler's store (no index
    # update happens yet); the second establishes a real current_index via
    # the normal coalesced-update path (see test_z_lock_skips_non_matching_frames).
    first = events[0]
    handler.frameReady(frame, first, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, first)
    manager._update_mda_viewer(viewer, handler, first)
    qtbot.waitUntil(
        lambda: dict(viewer.display_model.current_index).get("z") is not None,
        timeout=2000,
    )
    original_z = dict(viewer.display_model.current_index).get("z")
    assert original_z is not None
    assert dict(viewer.display_model.current_index).get("t") == first.index.get("t")

    manager.set_viewer_z_locked(viewer, True)
    assert manager._locked_z_axis[viewer] == original_z

    # simulate the user manually dragging the z slider to a different slice
    new_z = next(z for z in (e.index.get("z") for e in events) if z != original_z)
    viewer.display_model.current_index["z"] = new_z
    assert manager._locked_z_axis[viewer] == new_z

    # a subsequent frame at the *original* z is now skipped -- it no longer
    # matches the (updated) lock
    stale_z_event = next(
        e
        for e in events
        if e.index.get("z") == original_z and e.index.get("t") != first.index.get("t")
    )
    handler.frameReady(frame, stale_z_event, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, stale_z_event)
    qtbot.wait(200)  # give the (dropped) coalesced update a chance to fire
    assert dict(viewer.display_model.current_index).get("z") == new_z

    # a frame at the newly-locked z *does* update the view
    new_z_event = next(
        e
        for e in events
        if e.index.get("z") == new_z and e.index.get("t") != first.index.get("t")
    )
    handler.frameReady(frame, new_z_event, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, new_z_event)
    qtbot.waitUntil(
        lambda: dict(viewer.display_model.current_index).get("t")
        == new_z_event.index.get("t"),
        timeout=2000,
    )

    # unlocking disconnects the listener -- further manual drags no longer
    # touch _locked_z_axis
    manager.set_viewer_z_locked(viewer, False)
    viewer.display_model.current_index["z"] = original_z
    assert viewer not in manager._locked_z_axis
