"""Unit tests for the PLogic-triggered ASI SPIM MDA engines.

These exercise the event/frame bookkeeping shared by
:class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` (galvo, scanning) and
:class:`~pymmcore_gui.asi_z_stack.engine.ASIStationaryTriggerEngine` (galvo
as trigger master, both galvo and piezo held stationary) against a mocked
core and a mocked :class:`~pymmcore_gui.asi_z_stack.worker_pool.CameraWorkerPool`,
so no ASI hardware, PVCAM camera, or subprocess is required. ``exec_event``
sources frames entirely from the worker pool (see
:mod:`~pymmcore_gui.asi_z_stack.worker_pool`), so these tests mock that pool
rather than the old direct ``core.startSequenceAcquisition``/
``popNextImageAndMD`` circular-buffer draining.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, cast
from unittest.mock import MagicMock

import numpy as np
import pytest
import useq

from pymmcore_gui.asi_z_stack.camera_handoff import CameraHandoffSnapshot
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.engine import (
    ASISPIMEngine,
    ASIStationaryTriggerEngine,
    _ASITriggerEngineBase,
)
from pymmcore_gui.asi_z_stack.worker_pool import WorkerDiedError

if TYPE_CHECKING:
    from collections.abc import Generator

    from pymmcore_plus.metadata import FrameMetaV1


def _make_engine(
    n_cameras: int,
    n_slices: int,
    engine_cls: type[_ASITriggerEngineBase] = ASISPIMEngine,
) -> tuple[MagicMock, _ASITriggerEngineBase, MagicMock]:
    """Build an engine with a mocked core and a mocked, pre-armed worker pool.

    Stands in for what :meth:`_ASITriggerEngineBase._handoff_to_workers`
    would otherwise set up against real hardware -- these tests exercise
    ``exec_event``'s frame/index bookkeeping and cancellation/error handling
    directly, not the handoff itself (covered separately by real-hardware
    bench verification).

    Returns
    -------
    tuple[MagicMock, _ASITriggerEngineBase, MagicMock]
        ``(core, engine, worker_pool)`` -- ``worker_pool`` is returned
        separately (not just ``engine._worker_pool``) so callers get a
        concretely-``MagicMock``-typed handle instead of the narrower
        ``CameraWorkerPool | None`` the engine attribute is typed as.
    """
    core = MagicMock()
    core.getAutoShutter.return_value = True

    engine = engine_cls(core, HardwareConstants())
    engine._num_slices = n_slices
    engine._exposure_ms = 10.0
    engine._pixel_size_um = 0.5

    camera_labels = tuple(f"cam{i}" for i in range(n_cameras))
    engine._snapshot = CameraHandoffSnapshot(
        n_cameras=n_cameras,
        camera_labels=camera_labels,
        core_camera_role="MultiCam" if n_cameras > 1 else camera_labels[0],
        image_width=2,
        image_height=2,
        bytes_per_pixel=2,
        n_components=1,
    )
    pool = MagicMock()
    engine._worker_pool = pool
    return core, engine, pool


def _queue_frames(pool: MagicMock, entries: list[tuple[str, int]]) -> None:
    """Arm the mock worker pool's ``iter_frames`` to yield one frame per entry.

    Parameters
    ----------
    pool : MagicMock
        The engine's mocked ``_worker_pool``.
    entries : list[tuple[str, int]]
        ``(camera_label, slice_idx)`` pairs, in arrival order.
    """
    frames: list[tuple[str, int, np.ndarray, dict[str, object], int]] = [
        (label, slice_idx, np.zeros((2, 2), dtype=np.uint16), {}, 0)
        for label, slice_idx in entries
    ]
    pool.iter_frames.return_value = iter(frames)


def test_exec_event_tags_dual_camera_slices() -> None:
    """Each (camera, slice) frame gets a unique cam + z index and camera_device."""
    core, engine, pool = _make_engine(n_cameras=2, n_slices=3)
    # Interleaved arrival: slice0 cam0, slice0 cam1, slice1 cam0, ...
    _queue_frames(
        pool,
        [("cam0", 0), ("cam1", 0), ("cam0", 1), ("cam1", 1), ("cam0", 2), ("cam1", 2)],
    )

    event = useq.MDAEvent(index={"t": 0})
    payloads = list(engine.exec_event(event))

    assert len(payloads) == 6
    seen = {
        (meta["camera_device"], sub.index["cam"], sub.index["z"])
        for _img, sub, meta in payloads
    }
    assert seen == {
        ("cam0", 0, 0),
        ("cam0", 0, 1),
        ("cam0", 0, 2),
        ("cam1", 1, 0),
        ("cam1", 1, 1),
        ("cam1", 1, 2),
    }
    # the base index is preserved on every frame
    assert all(sub.index["t"] == 0 for _img, sub, _meta in payloads)

    # every camera worker is armed once, for the true per-camera slice count
    pool.arm_all.assert_called_once_with(
        3, armed_timeout=engine.hw.worker_arm_timeout_s
    )
    core.setProperty.assert_any_call("Scanner:AB:33", "SPIMState", "Running")


def test_exec_event_single_camera_has_no_cam_index() -> None:
    """Single-camera frames carry z but no cam axis, and the right camera_device."""
    _core, engine, pool = _make_engine(n_cameras=1, n_slices=4)
    _queue_frames(pool, [("cam0", 0), ("cam0", 1), ("cam0", 2), ("cam0", 3)])

    payloads = list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    assert [sub.index["z"] for _img, sub, _meta in payloads] == [0, 1, 2, 3]
    assert all("cam" not in sub.index for _img, sub, _meta in payloads)
    assert all(meta["camera_device"] == "cam0" for _img, _sub, meta in payloads)


def test_exec_event_cancel_stops_workers() -> None:
    """A generator ``send("cancel")`` stops both workers and ends the stack early.

    Regression test: the old implementation streamed frames straight off a
    shared circular buffer and never read the value sent into it, so MDA
    cancellation was silently ignored mid-stack.
    """
    _core, engine, pool = _make_engine(n_cameras=1, n_slices=5)
    _queue_frames(pool, [("cam0", i) for i in range(5)])

    # exec_event's declared return type matches the base class's Iterable
    # protocol, but it's actually implemented as a generator -- cast so the
    # test can use send()/next() on it directly.
    gen = cast(
        "Generator[tuple[np.ndarray, useq.MDAEvent, FrameMetaV1], str | None, None]",
        engine.exec_event(useq.MDAEvent(index={"t": 0})),
    )
    first = next(gen)
    assert first[1].index["z"] == 0

    with pytest.raises(StopIteration):
        gen.send("cancel")

    pool.stop_all.assert_called_once()


def test_exec_event_worker_died_stops_survivor_and_reraises() -> None:
    """A ``WorkerDiedError`` from the pool stops the survivor and propagates.

    This is the containment behavior the worker-process redesign exists for:
    a native crash in one camera's worker must not silently hang or corrupt
    the run -- it surfaces as a distinct, catchable error.
    """
    _core, engine, pool = _make_engine(n_cameras=2, n_slices=3)
    pool.iter_frames.side_effect = WorkerDiedError("cam0", -1073740791)

    with pytest.raises(WorkerDiedError):
        list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    pool.stop_all.assert_called_once()


def test_event_iterator_collapses_z_stack() -> None:
    """A pure z-stack collapses to a single event (the first slice)."""
    _core, engine, _pool = _make_engine(n_cameras=1, n_slices=3)
    events = [useq.MDAEvent(index={"z": i}) for i in range(3)]

    out = list(engine.event_iterator(iter(events)))

    assert len(out) == 1
    assert out[0].index["z"] == 0


def test_event_iterator_one_event_per_channel() -> None:
    """z+channel sequences emit one stack event per channel, all at z==0."""
    _core, engine, _pool = _make_engine(n_cameras=2, n_slices=3)
    events = [useq.MDAEvent(index={"c": c, "z": z}) for c in range(2) for z in range(3)]

    out = list(engine.event_iterator(iter(events)))

    assert len(out) == 2
    assert all(e.index["z"] == 0 for e in out)


def test_event_iterator_passthrough_without_z() -> None:
    """Sequences without a z axis pass through unchanged."""
    _core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    events = [useq.MDAEvent(index={"t": t}) for t in range(2)]

    out = list(engine.event_iterator(iter(events)))

    assert len(out) == 2


def test_stationary_engine_triggers_galvo_not_piezo() -> None:
    """ASIStationaryTriggerEngine triggers the galvo, same as ASISPIMEngine.

    Per ASI's own reference plugin (ASIdiSPIM's ControllerUtils.java), the
    piezo's SPIMState is only ever set to "Armed" (during setup_sequence,
    not exercised by this mocked-core exec_event test) -- the device that
    actually receives SPIMState="Running" to trigger a stack is always the
    galvo/scanner, confirmed independently on the bench (the piezo's
    SPIMState property has no "Running" value at all;
    getAllowedPropertyValues returned only ('Armed', 'Idle')). So
    ASIStationaryTriggerEngine's exec_event behavior is deliberately
    identical to ASISPIMEngine's here -- this is a regression check that the
    class is wired correctly, not a test of unique behavior.
    """
    core, engine, pool = _make_engine(
        n_cameras=1, n_slices=2, engine_cls=ASIStationaryTriggerEngine
    )
    _queue_frames(pool, [("cam0", 0), ("cam0", 1)])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    core.setProperty.assert_any_call("Scanner:AB:33", "SPIMState", "Running")
    assert ("PiezoStage:P:34", "SPIMState", "Running") not in [
        call.args for call in core.setProperty.call_args_list
    ]
