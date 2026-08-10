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
from pymmcore_gui._numpy_display_store import NumpyDisplayStore
from pymmcore_gui._qt.QtWidgets import QApplication, QWidget

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
    handler = NumpyDisplayStore()
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
    handler = NumpyDisplayStore()
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


def test_live_frames_actually_resolve_not_just_the_raw_model(
    mmcore: CMMCorePlus, qtbot: QtBot
) -> None:
    """Regression: incoming frames must actually re-resolve, not just set the model.

    ndv's built-in fallback wrapper for a plain array exposes bare integer
    dim positions (0, 1, 2, ...) instead of string axis labels. Since
    ``_update_mda_viewer`` calls ``current_index.update(event.index.items())``
    with *string*-keyed indices ("t"/"z"/etc), every key silently failed to
    normalize against those integer dims (``ndv.models._resolve
    ._norm_current_index`` swallows the ``IndexError``/``KeyError`` and just
    drops the key) -- so ``display_model.current_index`` still *looked*
    correct (our own code writes those raw string keys into it
    unconditionally), but the *resolved*, wrapper-position-keyed state ndv
    actually uses to decide whether to re-fetch/re-render never changed. The
    live preview appeared to only update when a user manually dragged a
    slider (which drives the view's own, correctly-integer-keyed index).

    This checks the actual resolved state (not just the raw model dict the
    other z-lock tests above check) to catch that class of regression.
    """
    from ndv.models._resolve import resolve

    dummy = QWidget()
    manager = NDVViewersManager(dummy, mmcore)

    seq = MDASequence(
        z_plan=useq.ZRangeAround(range=2, step=1),
        time_plan=useq.TIntervalLoops(interval=0, loops=2),  # pyright: ignore
    )
    events = list(seq)
    handler = NumpyDisplayStore()
    handler.reset(seq)
    frame = np.zeros((4, 4), dtype="uint8")

    viewer = ndv.ArrayViewer()

    first = events[0]
    handler.frameReady(frame, first, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, first)
    qtbot.wait(50)

    second = next(e for e in events if e.index.get("z") != first.index.get("z"))
    handler.frameReady(frame, second, {})  # pyright: ignore
    manager._update_mda_viewer(viewer, handler, second)
    qtbot.wait(50)

    assert viewer.data_wrapper is not None
    z_pos = viewer.data_wrapper.normalize_axis_key("z")
    resolved = resolve(viewer.display_model, viewer.data_wrapper)
    assert resolved.current_index.get(z_pos) == second.index.get("z")


def test_display_store_does_not_eagerly_allocate_full_domain() -> None:
    """Regression: declaring a huge domain must not eagerly allocate it.

    A real ASI SPIM run (100 timepoints x 201 z-slices x 2400x2400 uint16)
    declares a ~216 GiB logical domain. An earlier ``NumpyDisplayStore``
    implementation backed this with a dense ``np.zeros(full_shape)``, which
    raised ``MemoryError`` on the very first frame -- silently, since
    psygnal swallows exceptions raised inside ``frameReady`` signal
    handlers, so the live preview just stopped updating with no visible
    error. The store is now backed by a chunked, lazily-allocated
    ``zarr.Array`` (one frame per chunk, matching the old tensorstore
    store's memory-growth profile), so only frames actually written consume
    memory -- this asserts that writing a handful of frames out of a
    20,100-frame declared sequence stays well under 1% of the fully-dense
    size, without raising.

    Uses a huge window budget to disable the (separately tested, see
    ``test_rolling_window_evicts_old_timepoints``) rolling-window t-capping,
    so this test keeps exercising the full declared domain as originally
    intended -- with the *default* budget the array's own declared "t" size
    would already be capped well below 100, making the ~216 GiB comparison
    below moot.
    """
    store = NumpyDisplayStore(window_budget_bytes=10**15)
    seq = MDASequence(
        time_plan=useq.TIntervalLoops(interval=1, loops=100),  # pyright: ignore
        z_plan=useq.ZRangeAround(range=20, step=0.1),
        channels=["488nm"],  # pyright: ignore
    )
    frame = np.zeros((2400, 2400), dtype="uint16")
    events = list(seq)
    assert len(events) == 20100

    for event in events[:5]:
        store.frameReady(frame, event, {})  # pyright: ignore

    arr = store.array
    assert arr is not None
    dense_nbytes = arr.nbytes  # the full logical (t, p, z, y, x) domain
    assert dense_nbytes > 200 * 1024**3  # ~216 GiB fully dense
    # only the 5 written frames' chunks actually consumed memory.
    assert arr.nbytes_stored < 0.01 * dense_nbytes


def test_rolling_window_evicts_old_timepoints() -> None:
    """Regression: even lazy chunked allocation must not retain every timepoint.

    Confirmed in production: a real 100-timepoint dual-camera run hit a
    genuine ``MemoryError`` (this time inside ``numcodecs.blosc.compress``,
    writing a *new* chunk) at timepoint 11 of 100 -- tens of GB of real,
    poorly-compressible camera data had already accumulated per camera, with
    no ceiling. The store now caps its "t" axis to a rolling window sized
    from a memory-byte budget: writing more timepoints than the window holds
    must evict (overwrite, not accumulate) the oldest ones, keeping both the
    array's own declared shape and its actual stored bytes bounded no matter
    how long the acquisition runs.
    """
    frame = np.zeros((4, 4), dtype="uint16")
    # budget for exactly 3 timepoints of this (1, 4, 4) uint16 frame.
    store = NumpyDisplayStore(window_budget_bytes=3 * 1 * 4 * 4 * 2)
    seq = MDASequence(
        time_plan=useq.TIntervalLoops(interval=0, loops=10),  # pyright: ignore
    )
    store.reset(seq)
    events = list(seq)
    assert len(events) == 10

    for event in events:
        store.frameReady(frame, event, {})  # pyright: ignore

    assert store.window_size == 3
    assert store.max_t_seen == 9
    arr = store.array
    assert arr is not None
    assert arr.shape[0] == 3  # declared "t" size capped, not the full 10
    assert store.is_evicted(0)
    assert store.is_evicted(6)
    assert not store.is_evicted(9)  # the current/newest timepoint
    # Only ever `window_size` chunks resident, however many timepoints ran --
    # generous per-chunk overhead allowance since Blosc's fixed framing cost
    # dominates for these tiny (32-byte) synthetic test frames.
    assert arr.nbytes_stored <= store.window_size * (frame.nbytes + 300)


def test_to_storage_index_modulo_wraparound() -> None:
    """``to_storage_index`` must correctly wrap "t" at the window boundary."""
    frame = np.zeros((4, 4), dtype="uint16")
    store = NumpyDisplayStore(window_budget_bytes=3 * 1 * 4 * 4 * 2)
    # loops must exceed the budget-computed window (3) for it to actually
    # bind -- window is also capped at the *declared* t count.
    seq = MDASequence(time_plan=useq.TIntervalLoops(interval=0, loops=10))  # pyright: ignore
    store.reset(seq)
    # allocate the array (and window_size) via a real frame first.
    store.frameReady(frame, next(iter(seq)), {})  # pyright: ignore
    assert store.window_size == 3

    assert store.to_storage_index({"t": 0}) == {"t": 0}
    assert store.to_storage_index({"t": 3}) == {"t": 0}
    assert store.to_storage_index({"t": 5}) == {"t": 2}
    # slice width is preserved across the remap.
    assert store.to_storage_index({"t": slice(5, 6)}) == {"t": slice(2, 3)}
    # non-"t" axes pass through unchanged.
    assert store.to_storage_index({"y": slice(None)}) == {"y": slice(None)}


def test_z_progress_for_newest_volume_only() -> None:
    """``z_progress_for`` tracks only the currently-filling volume.

    Older, already-complete volumes must report ``None`` (meaning "use the
    full declared z range"), never a stale partial count.
    """
    frame = np.zeros((4, 4), dtype="uint16")
    store = NumpyDisplayStore()
    seq = MDASequence(
        time_plan=useq.TIntervalLoops(interval=0, loops=2),  # pyright: ignore
        z_plan=useq.ZRangeAround(range=2, step=1),  # 3 z slices
    )
    store.reset(seq)
    events = list(seq)
    t0_events = [e for e in events if e.index.get("t") == 0]
    assert len(t0_events) == 3

    store.frameReady(frame, t0_events[0], {})  # pyright: ignore
    assert store.z_progress_for(0, 0) == 1
    store.frameReady(frame, t0_events[1], {})  # pyright: ignore
    assert store.z_progress_for(0, 0) == 2

    # a frame for a new t implies the previous (t, p) is now complete.
    t1_events = [e for e in events if e.index.get("t") == 1]
    store.frameReady(frame, t1_events[0], {})  # pyright: ignore
    assert store.z_progress_for(0, 0) is None  # old volume -- use full range
    assert store.z_progress_for(1, 0) == 1  # new volume -- in progress


def test_labeled_wrapper_bounds_and_evicted_reads() -> None:
    """``_LabeledArrayWrapper.coords``/``isel`` reflect collected data, not the domain.

    Covers: "t" always reports ``range(0, max_t_seen+1)`` (never the larger
    declared total); "z" is bounded only while viewing the still-filling
    newest volume, and reports the full range for any older (complete)
    volume; reading an evicted "t" returns zeros of the correct shape/dtype
    without raising, rather than stale or wrong data.
    """
    from pymmcore_gui._ndv_viewers import _LabeledArrayWrapper

    frame = np.ones((4, 4), dtype="uint16")
    store = NumpyDisplayStore(window_budget_bytes=3 * 3 * 4 * 4 * 2)  # window=3
    seq = MDASequence(
        time_plan=useq.TIntervalLoops(interval=0, loops=6),  # pyright: ignore
        z_plan=useq.ZRangeAround(range=2, step=1),  # 3 z slices
    )
    store.reset(seq)
    events = list(seq)

    # write only the first z-slice of t=0, then build the wrapper.
    t0_events = [e for e in events if e.index.get("t") == 0]
    store.frameReady(frame, t0_events[0], {})  # pyright: ignore
    wrapper = _LabeledArrayWrapper(store)
    wrapper.set_selection(t=0, p=0)

    coords = dict(wrapper.coords)
    assert coords["t"] == range(1)  # only t=0 collected so far
    assert coords["z"] == range(1)  # only 1 of 3 z slices written for t=0

    # finish t=0's volume -- z bound should now show the full declared range.
    for e in t0_events[1:]:
        store.frameReady(frame, e, {})  # pyright: ignore
    assert dict(wrapper.coords)["z"] == range(3)

    # advance well past the window (window=3, so t=0 gets evicted).
    for e in events:
        store.frameReady(frame, e, {})  # pyright: ignore
    assert store.is_evicted(0)
    assert dict(wrapper.coords)["t"] == range(6)  # always 0-based, full collected range

    # isel on the evicted t=0 returns zeros, not stale/wrong data, no raise.
    # (mirrors real usage: every non-visible axis is pinned, only y/x are left
    # unpinned -- see ndv's _resolve.py:build_slice_requests -- so both "t"
    # and "z" must be pinned here for a proper 2D (y, x) result.)
    t_pos = wrapper.dims.index(next(d for d in wrapper.dims if str(d) == "t"))
    z_pos = wrapper.dims.index(next(d for d in wrapper.dims if str(d) == "z"))
    result = wrapper.isel({t_pos: 0, z_pos: 0})
    assert result.shape == (4, 4)
    assert result.dtype == frame.dtype
    assert np.all(result == 0)

    # isel on a still-resident, non-evicted t returns the real (non-zero) data.
    result = wrapper.isel({t_pos: 5, z_pos: 0})
    assert np.all(result == 1)


def test_live_preview_creates_no_tensorstore(
    mmcore: CMMCorePlus, qtbot: QtBot, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Regression: the live-preview path must never touch tensorstore.

    A native tensorstore crash (STATUS_STACK_BUFFER_OVERRUN) was previously
    triggered by concurrent async writes + threaded blocking reads against a
    private per-camera ``TensorStoreHandler`` used only to back the ndv live
    preview during dual-camera MDAs. The live preview now uses a plain
    ``NumpyDisplayStore``, so this asserts that ``tensorstore.open`` (the call
    ``TensorStoreHandler.new_store`` used to make) is never invoked while
    driving the manager's multi-camera path with synthetic frames, and that
    ``_ndv_viewers`` no longer even references ``TensorStoreHandler``.

    Frames are fed directly via ``_on_sequence_started``/``_on_frame_ready``
    (as the z-lock tests above do) rather than through a real
    ``mmcore.mda.run()``: the demo "Multi Camera" device in this environment
    cannot actually snap images (a pre-existing device-adapter limitation,
    unrelated to this fix), but ``getNumberOfCameraChannels()`` /
    ``getPhysicalCameraDevice()`` -- all ``NDVViewersManager`` needs to drive
    its multi-camera branch -- work fine without a real snap.
    """
    ts = pytest.importorskip("tensorstore")
    import pymmcore_gui._ndv_viewers as ndv_viewers_mod

    assert not hasattr(ndv_viewers_mod, "TensorStoreHandler")

    def _boom(*args: object, **kwargs: object) -> None:
        raise AssertionError("live-preview path must not create a tensorstore store")

    monkeypatch.setattr(ts, "open", _boom)

    dummy = QWidget()
    manager = NDVViewersManager(dummy, mmcore)
    mmcore.setCameraDevice("Multi Camera")

    seq = MDASequence(
        channels=["DAPI", "FITC"],  # pyright: ignore
        z_plan=useq.ZRangeAround(range=2, step=1),
    )
    manager._on_sequence_started(seq, {})  # pyright: ignore
    labels = manager._get_physical_camera_labels()
    assert len(labels) == 2

    frame = np.zeros((4, 4), dtype="uint16")
    for event in seq:
        for label in labels:
            manager._on_frame_ready(frame, event, {"camera_device": label})  # pyright: ignore
    manager._on_sequence_finished(seq)

    assert len(manager._mda_camera_handlers) == 2
    for handler in manager._mda_camera_handlers.values():
        assert handler.array is not None
