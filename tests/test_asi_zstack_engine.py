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

import logging
import time
from typing import TYPE_CHECKING, cast
from unittest.mock import MagicMock

import numpy as np
import pytest
import useq
from useq._mda_event import Channel as EventChannel

from pymmcore_gui.asi_z_stack import engine as engine_module
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


@pytest.mark.parametrize("engine_cls", [ASISPIMEngine, ASIStationaryTriggerEngine])
def test_setup_event_never_moves_focus_device(
    engine_cls: type[_ASITriggerEngineBase],
) -> None:
    """``setup_event`` must never call ``core.setZPosition`` for these engines.

    Regression test for the actual mis-centered-stack bug: ``event_iterator``
    collapses a whole z-stack down to its z-index-0 sub-event, and the stock
    ``MDAEngine.setup_single_event`` calls ``_set_event_z`` (->
    ``mmcore.setZPosition``) whenever that sub-event's ``z_pos`` is not
    ``None``. Since z-index 0 resolves to the *bottom* of the range for the
    default ``go_up=True`` direction, this physically pre-moved the piezo --
    this rig's Core-Focus device -- by ``-range/2`` before the galvo's own
    independent ``+/-range/2`` sweep (always centered on wherever the focus
    device is at trigger time) ran on top of it -- landing the user's actual
    pre-acquisition focus at the very last slice instead of the middle, and
    leaving the piezo parked away from where the user left it.
    ``_ASITriggerEngineBase._set_event_z`` is now a no-op specifically to
    prevent this; this test exercises the real ``setup_event`` dispatch path
    (not ``_set_event_z`` directly) so it fails if that dispatch ever changes.
    """
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=3, engine_cls=engine_cls)

    event = useq.MDAEvent(index={"t": 0, "z": 0}, z_pos=-50.0)
    engine.setup_event(event)

    core.setZPosition.assert_not_called()


@pytest.mark.parametrize("engine_cls", [ASISPIMEngine, ASIStationaryTriggerEngine])
def test_warn_if_piezo_moved(
    caplog: pytest.LogCaptureFixture,
    engine_cls: type[_ASITriggerEngineBase],
) -> None:
    """``_warn_if_piezo_moved`` only logs when the piezo's position changed.

    Defensive check for this class of bug: with ``_set_event_z`` now a
    no-op, nothing in these engines should ever move the piezo. If it moves
    anyway (e.g. an ASI-firmware auto-home side effect of the galvo's
    ``SPIMState`` going to ``"Running"``), this should surface as a warning
    instead of silently trusting the hardware.
    """
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=3, engine_cls=engine_cls)
    core.getLoadedDevices.return_value = ["PiezoStage:P:34", "Scanner:AB:33"]

    core.getPosition.return_value = 10.0
    engine._snapshot_piezo_position()
    with caplog.at_level(logging.WARNING):
        engine._warn_if_piezo_moved()
    assert not caplog.records

    core.getPosition.return_value = 10.0
    engine._snapshot_piezo_position()
    core.getPosition.return_value = 15.0
    with caplog.at_level(logging.WARNING):
        engine._warn_if_piezo_moved()
    assert len(caplog.records) == 1
    assert "10.000" in caplog.records[0].message
    assert "15.000" in caplog.records[0].message


@pytest.mark.parametrize("engine_cls", [ASISPIMEngine, ASIStationaryTriggerEngine])
def test_setup_event_never_calls_set_exposure(
    engine_cls: type[_ASITriggerEngineBase],
) -> None:
    """``setup_event`` must never call ``core.setExposure``, but still switches channel.

    Regression test for the spurious "Failed to set exposure" warning seen
    in production logs: the inherited ``MDAEngine.setup_single_event`` calls
    ``mmcore.setExposure(event.exposure)`` unconditionally whenever
    ``event.exposure`` is set, which fails on every event once
    ``_handoff_to_workers`` has released every physical camera -- this
    engine bakes exposure into the PLogic pulse width instead, so the call
    is never useful here. ``_ASITriggerEngineBase.setup_single_event`` drops
    that call but must still perform the real channel-switch
    (``_set_event_channel`` -> ``core.setConfig``).
    """
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=3, engine_cls=engine_cls)
    event = useq.MDAEvent(
        index={"t": 0, "z": 0},
        channel=EventChannel(config="488nm", group="Lasers"),
        exposure=25.0,
    )

    engine.setup_event(event)

    core.setExposure.assert_not_called()
    core.setConfig.assert_called_once_with("Lasers", "488nm")


def test_exec_event_timeout_stops_workers_and_reraises() -> None:
    """A ``TimeoutError`` from ``iter_frames`` (stall guard) still stops workers.

    Regression test for a gap in the old explicit stop_all()-on-error
    handling: only ``WorkerDiedError`` triggered a stop, so a plain
    ``TimeoutError`` (worker_pool.py's stall guard) or a worker's own
    ``ErrorMsg``-derived ``RuntimeError`` left the survivor running with no
    stop signal. ``exec_event`` now calls ``stop_all()`` in a ``finally``
    block that covers every exit path.
    """
    _core, engine, pool = _make_engine(n_cameras=2, n_slices=3)
    pool.iter_frames.side_effect = TimeoutError(
        "no message from any camera worker for 5.0s"
    )

    with pytest.raises(TimeoutError):
        list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    pool.stop_all.assert_called_once()


def test_exec_event_generator_close_stops_workers() -> None:
    """Abandoning ``exec_event`` mid-iteration (``GeneratorExit``) still stops workers.

    This is the actual mechanism behind the production deadlock: if
    ``exec_event``'s generator is closed/garbage-collected before being
    fully drained (e.g. the runner's own iteration is abandoned by an
    exception elsewhere), the interpreter throws ``GeneratorExit`` at the
    generator's current ``yield``. The old code only called ``stop_all()``
    from the ``"cancel"`` branch and the ``WorkerDiedError`` handler, so an
    abandoned generator left workers blocked forever in
    ``camera_worker._wait_for_free_slot``, waiting for a ``SlotFreeCmd``/
    ``StopCmd`` that would never come. The ``finally`` block now covers
    this path too.
    """
    _core, engine, pool = _make_engine(n_cameras=1, n_slices=3)
    _queue_frames(pool, [("cam0", 0), ("cam0", 1), ("cam0", 2)])

    gen = cast(
        "Generator[tuple[np.ndarray, useq.MDAEvent, FrameMetaV1], str | None, None]",
        engine.exec_event(useq.MDAEvent(index={"t": 0})),
    )
    next(gen)
    gen.close()

    pool.stop_all.assert_called_once()


def test_exec_event_normal_completion_also_stops_workers() -> None:
    """Ordinary, uncancelled completion also calls ``stop_all()`` exactly once.

    New (safe, intentional) side effect of moving ``stop_all()`` into a
    ``finally`` block: it now fires on every exit path, including normal
    completion, where it previously never fired at all. Harmless -- workers
    are already idle by the time ``iter_frames`` naturally exhausts, so the
    extra ``StopCmd`` lands on an idle worker as a guarded no-op -- but
    locked in here as a named test so it isn't mistaken for a regression by
    a future reader.
    """
    _core, engine, pool = _make_engine(n_cameras=1, n_slices=2)
    _queue_frames(pool, [("cam0", 0), ("cam0", 1)])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    pool.stop_all.assert_called_once()


def test_reset_channel_config_cache_clears_last_config() -> None:
    """``_reset_channel_config_cache`` unconditionally clears the cache.

    Direct unit test of the helper both ``setup_sequence`` overrides call --
    see :func:`test_setup_sequence_resets_channel_config_cache` for the
    end-to-end regression test that it's actually wired in.
    """
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    core._last_config = ("Lasers", "488nm")

    engine._reset_channel_config_cache()

    assert core._last_config == ("", "")


@pytest.mark.parametrize("engine_cls", [ASISPIMEngine, ASIStationaryTriggerEngine])
def test_setup_sequence_resets_channel_config_cache(
    monkeypatch: pytest.MonkeyPatch,
    engine_cls: type[_ASITriggerEngineBase],
) -> None:
    """``setup_sequence`` must invalidate a stale ``core._last_config``.

    Regression test for the wrong-laser bug: both ``ASISPIMEngine`` and
    ``ASIStationaryTriggerEngine.setup_sequence`` completely override the
    stock ``MDAEngine.setup_sequence`` (rather than calling ``super()``), so
    they must replicate its ``core._last_config = ("", "")`` reset
    themselves. Without it, ``_set_event_channel`` can wrongly treat the
    sequence's first channel as "already configured" (because it happens to
    match whatever config was last actually applied, e.g. from Live/Snap or
    the previous MDA run) and skip calling ``mmc.setConfig(...)`` for it --
    silently leaving that channel's whole z-stack running under whatever raw
    PLogic wiring was left over from setup instead of its own selected laser.

    Heavy hardware/network side effects (``configure_plogic_for_dual_nrt_pulses``,
    ``set_plogic_evaluation_clock``, ``log_plogic_trigger_chain_state``,
    ``summary_metadata``) are stubbed out -- they talk to PLogic's own
    process-global ``CMMCorePlus`` singleton (a separate concern, unrelated to
    the cache-reset behavior under test here) rather than the mocked ``core``.
    """
    monkeypatch.setattr(
        engine_module, "configure_plogic_for_dual_nrt_pulses", MagicMock()
    )
    monkeypatch.setattr(engine_module, "set_plogic_evaluation_clock", MagicMock())
    monkeypatch.setattr(engine_module, "log_plogic_trigger_chain_state", MagicMock())
    monkeypatch.setattr(engine_module, "summary_metadata", MagicMock(return_value=None))

    core, engine, _pool = _make_engine(n_cameras=1, n_slices=3, engine_cls=engine_cls)
    # Simulate a stale cache left over from a prior Live/Snap selection or MDA
    # run that happens to match this sequence's first (and only) channel.
    core._last_config = ("Lasers", "488nm")
    core.getExposure.return_value = 10.0
    # Real ints, not MagicMocks -- _warn_if_circular_buffer_too_small does
    # arithmetic/comparisons on these.
    core.getImageWidth.return_value = 2
    core.getImageHeight.return_value = 2
    core.getBytesPerPixel.return_value = 2

    sequence = useq.MDASequence(
        channels=(useq.Channel(config="488nm", group="Lasers", exposure=10.0),)
    )
    engine.setup_sequence(sequence)

    assert core._last_config == ("", "")


# --- Shutter-gated laser (561): whole-stack-open instead of per-slice blanking ---
#
# The 561 line is a CW laser behind a physical mechanical shutter (Oxxius
# L4C), which can't reliably follow PLogic's per-frame TTL blanking (laser
# NRT cell 10, fired every slice like the camera). These tests cover holding
# it open for a whole per-volume burst instead: see
# ``_ASITriggerEngineBase._shutter_gated_bncs`` and its use in ``exec_event``.


def test_shutter_gated_bncs_single_wavelength_preset() -> None:
    """A single shutter-gated wavelength preset returns its own BNC."""
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "561nm"

    assert engine._shutter_gated_bncs() == [40]


def test_shutter_gated_bncs_diode_preset_returns_empty() -> None:
    """A diode-only preset (not shutter-gated) returns no BNCs."""
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "488nm"

    assert engine._shutter_gated_bncs() == []


def test_shutter_gated_bncs_all_lasers_preset_returns_only_gated() -> None:
    """The "AllLasers" preset pulls out only the shutter-gated wavelength's BNC.

    Regression coverage for the "all lasers at once" case: cell 10 fires
    per-slice for every wavelength under this preset, so the diode BNCs
    (37/38/39) must stay off this list -- only 561's BNC (40) should be
    re-pointed to the always-on cell, pulling it out of the simultaneous
    per-slice triggering while the diodes keep blanking normally.
    """
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "AllLasers"

    assert engine._shutter_gated_bncs() == [40]


def test_shutter_gated_bncs_flag_off_returns_empty() -> None:
    """``laser_open_full_stack=False`` disables the feature entirely."""
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    engine.hw.laser_open_full_stack = False
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "561nm"

    assert engine._shutter_gated_bncs() == []


def test_shutter_gated_bncs_missing_config_group_returns_empty() -> None:
    """No "Lasers" config group loaded (e.g. a demo config) returns no BNCs."""
    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    core.getAvailableConfigGroups.return_value = []

    assert engine._shutter_gated_bncs() == []


def test_exec_event_opens_then_closes_shutter_gated_laser(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A shutter-gated channel opens its laser before the burst, closes after.

    The mechanical 561 shutter can't follow per-frame TTL blanking reliably,
    so ``exec_event`` must hold it open for the whole per-volume burst
    instead -- via the same ``set_laser_outputs`` primitive snap/live
    already use to gate lasers -- and close it again only after the last
    frame has been delivered.
    """
    calls: list[tuple[str, tuple[object, ...]]] = []
    mock_set_laser_outputs = MagicMock(
        side_effect=lambda *a, **kw: calls.append(("set_laser_outputs", a))
    )
    monkeypatch.setattr(engine_module, "set_laser_outputs", mock_set_laser_outputs)
    monkeypatch.setattr(time, "sleep", MagicMock())

    core, engine, pool = _make_engine(n_cameras=1, n_slices=2)
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "561nm"
    core.setProperty.side_effect = lambda *a, **kw: calls.append(("setProperty", a))
    _queue_frames(pool, [("cam0", 0), ("cam0", 1)])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    laser_calls = [c for c in calls if c[0] == "set_laser_outputs"]
    assert len(laser_calls) == 2
    assert laser_calls[0][1][2] == [40]  # bnc_addrs
    assert laser_calls[0][1][3] is True  # on=True -- opened
    assert laser_calls[1][1][3] is False  # on=False -- closed

    open_idx = calls.index(laser_calls[0])
    close_idx = calls.index(laser_calls[1])
    trigger_idx = calls.index(
        ("setProperty", (engine._master_axis_label, "SPIMState", "Running"))
    )
    assert open_idx < trigger_idx < close_idx


def test_exec_event_settle_delay_honored(monkeypatch: pytest.MonkeyPatch) -> None:
    """The configured shutter settle delay is slept before the burst fires."""
    monkeypatch.setattr(engine_module, "set_laser_outputs", MagicMock())
    mock_sleep = MagicMock()
    monkeypatch.setattr(time, "sleep", mock_sleep)

    core, engine, pool = _make_engine(n_cameras=1, n_slices=1)
    engine.hw.shutter_open_settle_ms = 25.0
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "561nm"
    _queue_frames(pool, [("cam0", 0)])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    mock_sleep.assert_called_once_with(0.025)


def test_exec_event_does_not_gate_diode_channel(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A diode-only channel (e.g. 488) never touches ``set_laser_outputs``."""
    mock_set_laser_outputs = MagicMock()
    monkeypatch.setattr(engine_module, "set_laser_outputs", mock_set_laser_outputs)

    core, engine, pool = _make_engine(n_cameras=1, n_slices=2)
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "488nm"
    _queue_frames(pool, [("cam0", 0), ("cam0", 1)])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    mock_set_laser_outputs.assert_not_called()


def test_exec_event_respects_flag_off(monkeypatch: pytest.MonkeyPatch) -> None:
    """``laser_open_full_stack=False`` disables the shutter bracket, even for 561."""
    mock_set_laser_outputs = MagicMock()
    monkeypatch.setattr(engine_module, "set_laser_outputs", mock_set_laser_outputs)

    core, engine, pool = _make_engine(n_cameras=1, n_slices=2)
    engine.hw.laser_open_full_stack = False
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "561nm"
    _queue_frames(pool, [("cam0", 0), ("cam0", 1)])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    mock_set_laser_outputs.assert_not_called()


def test_exec_event_closes_shutter_gated_laser_on_worker_death(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A ``WorkerDiedError`` still closes the shutter-gated laser via ``finally``.

    Mirrors :func:`test_exec_event_worker_died_stops_survivor_and_reraises`:
    the shutter must never be left open on an aborted run.
    """
    mock_set_laser_outputs = MagicMock()
    monkeypatch.setattr(engine_module, "set_laser_outputs", mock_set_laser_outputs)
    monkeypatch.setattr(time, "sleep", MagicMock())

    core, engine, pool = _make_engine(n_cameras=1, n_slices=2)
    core.getAvailableConfigGroups.return_value = ["Lasers"]
    core.getCurrentConfig.return_value = "561nm"
    pool.iter_frames.side_effect = WorkerDiedError("cam0", -1073740791)

    with pytest.raises(WorkerDiedError):
        list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    on_values = [call.args[3] for call in mock_set_laser_outputs.call_args_list]
    assert on_values == [True, False]


def test_teardown_sequence_closes_all_lasers_when_flag_on(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``teardown_sequence``'s safety net closes any shutter-gated laser left open.

    Strictly redundant with ``exec_event``'s own per-burst ``finally`` close,
    but guards against a run aborting between volumes rather than mid-burst.
    """
    monkeypatch.setattr(engine_module, "set_plogic_evaluation_clock", MagicMock())
    monkeypatch.setattr(engine_module, "reload_cameras_after_handoff", MagicMock())
    monkeypatch.setattr(time, "sleep", MagicMock())
    mock_close_all_lasers = MagicMock()
    monkeypatch.setattr(engine_module, "close_all_lasers", mock_close_all_lasers)

    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)

    engine.teardown_sequence(useq.MDASequence())

    mock_close_all_lasers.assert_called_once()


def test_teardown_sequence_skips_close_all_lasers_when_flag_off(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The teardown safety net is itself gated by ``laser_open_full_stack``."""
    monkeypatch.setattr(engine_module, "set_plogic_evaluation_clock", MagicMock())
    monkeypatch.setattr(engine_module, "reload_cameras_after_handoff", MagicMock())
    monkeypatch.setattr(time, "sleep", MagicMock())
    mock_close_all_lasers = MagicMock()
    monkeypatch.setattr(engine_module, "close_all_lasers", mock_close_all_lasers)

    core, engine, _pool = _make_engine(n_cameras=1, n_slices=1)
    engine.hw.laser_open_full_stack = False

    engine.teardown_sequence(useq.MDASequence())

    mock_close_all_lasers.assert_not_called()
