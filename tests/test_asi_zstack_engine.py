"""Unit tests for the PLogic-triggered ASI SPIM MDA engines.

These exercise the event/frame bookkeeping shared by
:class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` (galvo, scanning) and
:class:`~pymmcore_gui.asi_z_stack.engine.ASIStationaryTriggerEngine` (galvo
as trigger master, both galvo and piezo held stationary) against a mocked
core, so no ASI hardware (or even a live ``CMMCorePlus``) is required.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import numpy as np
import useq

from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.engine import (
    ASISPIMEngine,
    ASIStationaryTriggerEngine,
    _ASITriggerEngineBase,
)


class _FakeMeta:
    """Minimal stand-in for a circular-buffer ``Metadata`` object."""

    def __init__(self, channel: int) -> None:
        self._channel = channel

    def items(self) -> list[tuple[str, Any]]:
        # The Multi Camera adapter tags each frame with a `*CameraChannelIndex`.
        return [("Andor-CameraChannelIndex", self._channel), ("Foo", "bar")]

    def GetSingleTag(self, tag: str) -> Any:
        # Force the fallback to getPhysicalCameraDevice(channel) in the engine.
        raise KeyError(tag)


def _fake_frame_metadata(event: useq.MDAEvent, **kwargs: Any) -> dict[str, Any]:
    # Stand in for MDAEngine.get_frame_metadata (which needs a live core).
    return {"camera_device": kwargs.get("camera_device")}


def _make_engine(
    n_cameras: int,
    n_slices: int,
    engine_cls: type[_ASITriggerEngineBase] = ASISPIMEngine,
) -> tuple[MagicMock, _ASITriggerEngineBase]:
    core = MagicMock()
    core.getCameraDevice.return_value = "MultiCam"
    core.getNumberOfCameraChannels.return_value = n_cameras
    core.getPhysicalCameraDevice.side_effect = lambda i: f"cam{i}"
    core.getAutoShutter.return_value = True
    core.isSequenceRunning.return_value = True

    engine = engine_cls(core, HardwareConstants())
    engine._num_slices = n_slices
    engine._exposure_ms = 10.0
    engine.get_frame_metadata = _fake_frame_metadata  # type: ignore[method-assign,assignment]
    return core, engine


def _queue_frames(core: MagicMock, channels: list[int]) -> None:
    """Arm the mock core to hand out one frame per entry in *channels*."""
    frames = [(np.zeros((2, 2), dtype=np.uint16), _FakeMeta(ch)) for ch in channels]
    core.getRemainingImageCount.side_effect = lambda: len(frames)
    core.popNextImageAndMD.side_effect = lambda: frames.pop(0)


def test_exec_event_tags_dual_camera_slices() -> None:
    """Each (camera, slice) frame gets a unique cam + z index and camera_device."""
    core, engine = _make_engine(n_cameras=2, n_slices=3)
    # Interleaved arrival: slice0 camA, slice0 camB, slice1 camA, ...
    _queue_frames(core, [0, 1, 0, 1, 0, 1])

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

    # the whole stack is armed once and the galvo is triggered
    core.startSequenceAcquisition.assert_called_once()
    assert core.startSequenceAcquisition.call_args.args[1] == 6  # total_images
    core.setProperty.assert_any_call("Scanner:AB:33", "SPIMState", "Running")


def test_exec_event_single_camera_has_no_cam_index() -> None:
    """Single-camera frames carry z but no cam axis, and the right camera_device."""
    core, engine = _make_engine(n_cameras=1, n_slices=4)
    _queue_frames(core, [0, 0, 0, 0])

    payloads = list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    assert [sub.index["z"] for _img, sub, _meta in payloads] == [0, 1, 2, 3]
    assert all("cam" not in sub.index for _img, sub, _meta in payloads)
    assert all(meta["camera_device"] == "cam0" for _img, _sub, meta in payloads)


def test_event_iterator_collapses_z_stack() -> None:
    """A pure z-stack collapses to a single event (the first slice)."""
    _core, engine = _make_engine(n_cameras=1, n_slices=3)
    events = [useq.MDAEvent(index={"z": i}) for i in range(3)]

    out = list(engine.event_iterator(iter(events)))

    assert len(out) == 1
    assert out[0].index["z"] == 0


def test_event_iterator_one_event_per_channel() -> None:
    """z+channel sequences emit one stack event per channel, all at z==0."""
    _core, engine = _make_engine(n_cameras=2, n_slices=3)
    events = [useq.MDAEvent(index={"c": c, "z": z}) for c in range(2) for z in range(3)]

    out = list(engine.event_iterator(iter(events)))

    assert len(out) == 2
    assert all(e.index["z"] == 0 for e in out)


def test_event_iterator_passthrough_without_z() -> None:
    """Sequences without a z axis pass through unchanged."""
    _core, engine = _make_engine(n_cameras=1, n_slices=1)
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
    core, engine = _make_engine(
        n_cameras=1, n_slices=2, engine_cls=ASIStationaryTriggerEngine
    )
    _queue_frames(core, [0, 0])

    list(engine.exec_event(useq.MDAEvent(index={"t": 0})))

    core.setProperty.assert_any_call("Scanner:AB:33", "SPIMState", "Running")
    assert ("PiezoStage:P:34", "SPIMState", "Running") not in [
        call.args for call in core.setProperty.call_args_list
    ]
