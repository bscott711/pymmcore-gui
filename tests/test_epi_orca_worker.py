"""The EPI Orca worker path: trigger-neutral, and a 2048x2048 uint16 frame end to end.

The Orca (Hamamatsu C11440-22C, MM adapter ``HamamatsuHam``) must stay on its own
default trigger: the worker the EPI service spawns never writes ``TriggerMode``,
expose-out or any timing property, even if the camera exposes properties with those
names. ``manage_trigger=True`` (the default, PVCAM/OPM) is pinned unchanged. All
hardware-free: ``run_camera_worker`` runs in-process against a fake core and a real
shared-memory segment.
"""

from __future__ import annotations

import sys
from multiprocessing.shared_memory import SharedMemory
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest
from PyQt6.QtCore import QObject

from pymmcore_gui._settings import EpiCameraSettingsV1
from pymmcore_gui.asi_z_stack import camera_worker as cw_module
from pymmcore_gui.asi_z_stack.camera_worker import (
    CameraWorkerConfig,
    run_camera_worker,
)
from pymmcore_gui.asi_z_stack.worker_messages import (
    ArmCmd,
    ArmedMsg,
    ArmLiveCmd,
    ErrorMsg,
    FrameMsg,
    PropertiesSetMsg,
    SetPropertiesCmd,
    StopCmd,
)
from pymmcore_gui.asi_z_stack.worker_pool import CameraWorkerHandle
from pymmcore_gui.epi._camera_service import EpiCameraWorkerService

if TYPE_CHECKING:
    from collections.abc import Iterator

LABEL = "Orca"
HEIGHT = WIDTH = 2048
DTYPE = "uint16"

#: Names an Orca might (or might not) expose; the neutral worker must write none.
_TRIGGER_LIKE = (
    "TriggerMode",
    "Trigger Polarity",
    "Expose Out Mode",
    "ExposeOutMode",
    "Port",
    "ClearMode",
    "Timing-ReadoutTime",
)


class _OrcaCore:
    """Fake core that records every property write and serves 2048x2048 frames."""

    def __init__(self, props: dict[str, str]) -> None:
        self.props = dict(props)
        self.writes: list[tuple[str, str, str]] = []
        self.exposure_writes: list[float] = []
        self.exposure = 10.0
        self.running = False
        self.frames: list[np.ndarray] = []
        self.infinite = False
        self._n = 0

    # -- load / init (all no-ops)
    def setCircularBufferMemoryFootprint(self, *_: object) -> None: ...
    def loadDevice(self, *_: object) -> None: ...
    def initializeDevice(self, *_: object) -> None: ...
    def setCameraDevice(self, *_: object) -> None: ...
    def unloadDevice(self, *_: object) -> None: ...
    def clearCircularBuffer(self) -> None:
        self.frames.clear()

    # -- properties
    def getDevicePropertyNames(self, _label: str) -> list[str]:
        return list(self.props)

    def hasProperty(self, _label: str, prop: str) -> bool:
        return prop in self.props

    def getProperty(self, _label: str, prop: str) -> str:
        return self.props[prop]

    def setProperty(self, label: str, prop: str, value: str) -> None:
        self.writes.append((label, prop, value))
        self.props[prop] = value

    def getAllowedPropertyValues(self, _label: str, prop: str) -> list[str]:
        if prop == "TriggerMode":
            return ["Internal Trigger", "Edge Trigger", "Level Trigger"]
        if "xpose" in prop:
            return ["Rolling Shutter", "All Rows"]
        return []

    def getExposure(self, _label: str) -> float:
        return self.exposure

    def setExposure(self, _label: str, value: float) -> None:
        self.exposure_writes.append(value)
        self.exposure = value

    def getROI(self, _label: str) -> list[int]:
        return [0, 0, WIDTH, HEIGHT]

    # -- acquisition
    def _frame(self) -> np.ndarray:
        self._n += 1
        return np.full((HEIGHT, WIDTH), self._n, dtype=DTYPE)

    def startSequenceAcquisition(self, *_: object) -> None:
        self.running = True
        self.frames = [self._frame()]

    def startContinuousSequenceAcquisition(self, *_: object) -> None:
        self.running = True
        self.infinite = True

    def isSequenceRunning(self, *_: object) -> bool:
        return self.running

    def stopSequenceAcquisition(self, *_: object) -> None:
        self.running = False
        self.infinite = False

    def getRemainingImageCount(self) -> int:
        return 1 if self.infinite else len(self.frames)

    def popNextImageAndMD(self) -> tuple[np.ndarray, dict[str, str]]:
        return self.frames.pop(0), {}

    def getLastImageAndMD(self) -> tuple[np.ndarray, dict[str, str]]:
        return self._frame(), {}


class _Conn:
    """In-process pipe half: scripted commands in, recorded messages out.

    A ``StopCmd`` is injected right after the first live ``FrameMsg`` so Live ends.
    """

    def __init__(self, cmds: list[object]) -> None:
        self.cmds = list(cmds)
        self.inject: list[object] = []
        self.sent: list[object] = []
        self.live = False

    def poll(self, _timeout: float = 0.0) -> bool:
        return bool(self.inject)

    def recv(self) -> object:
        if self.inject:
            return self.inject.pop(0)
        if not self.cmds:
            raise EOFError
        cmd = self.cmds.pop(0)
        self.live = isinstance(cmd, ArmLiveCmd)
        return cmd

    def send(self, msg: object) -> None:
        self.sent.append(msg)
        if isinstance(msg, FrameMsg) and self.live:
            self.inject.append(StopCmd())


def _props(**extra: str) -> dict[str, str]:
    props = {
        "Exposure": "10",
        "TriggerMode": "Internal Trigger",
        "Trigger Polarity": "Positive",
        "Expose Out Mode": "Rolling Shutter",
        "Port": "Sensitivity",
        "ClearMode": "Pre-Sequence",
        "Timing-ReadoutTime": "5",
    }
    props.update(extra)
    return props


@pytest.fixture
def handle() -> Iterator[CameraWorkerHandle]:
    """A handle with a real 2048x2048 uint16 shm ring (not spawned)."""
    cfg = CameraWorkerConfig(
        camera_label=LABEL,
        adapter_module="HamamatsuHam",
        adapter_device_name="HamamatsuHam_DCAM",
        geometry=(HEIGHT, WIDTH, DTYPE),
    )
    h = CameraWorkerHandle.from_config(cfg, n_slots=2)
    h.shm = SharedMemory(create=True, size=h.slot_nbytes * h.n_slots)
    try:
        yield h
    finally:
        h.shm.close()
        h.shm.unlink()


def _run(
    monkeypatch: pytest.MonkeyPatch,
    handle: CameraWorkerHandle,
    core: _OrcaCore,
    cmds: list[object],
    **overrides: Any,
) -> _Conn:
    monkeypatch.setattr(sys, "stderr", sys.stderr)
    monkeypatch.setattr(sys, "stdout", sys.stdout)
    monkeypatch.setattr("faulthandler.enable", lambda **_: None)
    monkeypatch.setattr(cw_module, "CMMCorePlus", lambda: core)
    monkeypatch.setattr(cw_module, "available_commit_bytes", lambda: None)
    assert handle.shm is not None
    kwargs: dict[str, Any] = {
        "camera_label": LABEL,
        "adapter_module": "HamamatsuHam",
        "adapter_device_name": "HamamatsuHam_DCAM",
        "shm_name": handle.shm.name,
        "slot_nbytes": handle.slot_nbytes,
        "n_slots": handle.n_slots,
        "live_max_fps": 0.0,
        **overrides,
    }
    conn = _Conn(cmds)
    run_camera_worker(CameraWorkerConfig(**kwargs), conn)  # type: ignore[arg-type]
    assert not [m for m in conn.sent if isinstance(m, ErrorMsg)], conn.sent
    return conn


def _frames(conn: _Conn) -> list[FrameMsg]:
    return [m for m in conn.sent if isinstance(m, FrameMsg)]


# ------------------------------------------------- what the EPI service spawns


def test_the_epi_service_spawns_a_trigger_neutral_orca_worker(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(EpiCameraWorkerService, "_active", None)
    monkeypatch.setattr(
        EpiCameraWorkerService, "spawn_async", lambda self, on_ready=None: None
    )
    owner = QObject()
    svc = EpiCameraWorkerService(owner)
    svc.begin(EpiCameraSettingsV1())
    cfg = svc._build_worker_handle(LABEL).config
    assert cfg.manage_trigger is False
    assert cfg.pre_init_props == ()
    assert cfg.property_snapshot == {}
    assert (cfg.adapter_module, cfg.adapter_device_name) == (
        "HamamatsuHam",
        "HamamatsuHam_DCAM",
    )
    assert cfg.geometry == (2048, 2048, "uint16")


def test_the_default_worker_config_still_manages_the_trigger() -> None:
    assert CameraWorkerConfig(camera_label="camA").manage_trigger is True


# ----------------------------------------------- trigger-neutral worker behaviour


def test_neutral_worker_writes_no_trigger_property_for_snap_live_and_arm(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle
) -> None:
    core = _OrcaCore(_props())
    conn = _run(
        monkeypatch,
        handle,
        core,
        [
            ArmCmd(1, external_trigger=False, exposure_ms=25.0),  # the EPI MDA arm
            ArmCmd(1, external_trigger=False, exposure_ms=25.0),  # Snap
            ArmLiveCmd(exposure_ms=30.0),  # Live
        ],
        manage_trigger=False,
        # Even a snapshot that names trigger/timing properties is not restored.
        property_snapshot={
            "TriggerMode": "Edge Trigger",
            "Port": "Speed",
            "Exposure": "12",
        },
    )
    assert len(_frames(conn)) == 3
    assert len([m for m in conn.sent if isinstance(m, ArmedMsg)]) == 3
    # No write to any trigger/expose-out/timing-like property; the only property
    # write is the plain Exposure from the snapshot, everything else is setExposure.
    assert [prop for _, prop, _ in core.writes] == ["Exposure"]
    assert core.exposure_writes == [25.0, 30.0]
    for name in _TRIGGER_LIKE:
        assert all(prop != name for _, prop, _ in core.writes)
    assert core.props["TriggerMode"] == "Internal Trigger"
    assert core.props["Expose Out Mode"] == "Rolling Shutter"


def test_neutral_worker_ignores_a_requested_trigger_or_expose_out_mode(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle
) -> None:
    core = _OrcaCore(_props())
    _run(
        monkeypatch,
        handle,
        core,
        [
            ArmCmd(
                1,
                external_trigger=True,
                trigger_mode="Edge Trigger",
                expose_out_mode="All Rows",
            )
        ],
        manage_trigger=False,
    )
    assert core.writes == []


def test_neutral_worker_never_reads_a_missing_expose_out_property_as_an_error(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle
) -> None:
    """An Orca without TriggerMode/expose-out works as before: just Exposure."""
    core = _OrcaCore({"Exposure": "10"})
    _run(
        monkeypatch,
        handle,
        core,
        [ArmCmd(1, external_trigger=False, exposure_ms=5.0), ArmLiveCmd()],
        manage_trigger=False,
    )
    assert core.writes == []
    assert core.exposure_writes == [5.0]


def test_neutral_worker_refuses_a_trigger_property_set_but_takes_exposure(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle
) -> None:
    core = _OrcaCore(_props())
    conn = _run(
        monkeypatch,
        handle,
        core,
        [
            SetPropertiesCmd(LABEL, (("TriggerMode", "Edge Trigger"),)),
            SetPropertiesCmd(LABEL, (("Exposure", "40"),)),
        ],
        manage_trigger=False,
    )
    first, second = [m for m in conn.sent if isinstance(m, PropertiesSetMsg)]
    assert first.error is not None and "trigger-neutral" in first.error
    assert second.error is None
    assert core.writes == [(LABEL, "Exposure", "40")]


# ------------------------------------------------ the default (PVCAM) is pinned


def test_default_worker_still_sets_trigger_and_expose_out_per_arm(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle
) -> None:
    """Pin: with ``manage_trigger`` left True nothing about PVCAM/OPM changed."""
    core = _OrcaCore(_props())
    _run(
        monkeypatch,
        handle,
        core,
        [
            ArmCmd(1, external_trigger=True, expose_out_mode="All Rows"),
            ArmCmd(1, external_trigger=False),
        ],
    )
    assert core.writes == [
        (LABEL, "TriggerMode", "Level Trigger"),  # startup: external picked
        (LABEL, "Expose Out Mode", "All Rows"),
        (LABEL, "TriggerMode", "Internal Trigger"),
        (LABEL, "Expose Out Mode", "Rolling Shutter"),
    ]


def test_default_worker_restores_the_snapshot_then_picks_external_trigger(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle
) -> None:
    core = _OrcaCore(_props())
    _run(
        monkeypatch,
        handle,
        core,
        [],
        property_snapshot={"Port": "Speed", "TriggerMode": "Internal Trigger"},
    )
    assert core.writes == [
        (LABEL, "Port", "Speed"),
        (LABEL, "TriggerMode", "Internal Trigger"),
        (LABEL, "TriggerMode", "Level Trigger"),
    ]


# ------------------------------------------------ 2048x2048 uint16 round trip


@pytest.mark.parametrize("live", [False, True])
def test_a_2048_square_uint16_frame_round_trips(
    monkeypatch: pytest.MonkeyPatch, handle: CameraWorkerHandle, live: bool
) -> None:
    assert handle.slot_nbytes == 2048 * 2048 * 2
    core = _OrcaCore(_props())
    cmd = ArmLiveCmd() if live else ArmCmd(1, external_trigger=False)
    conn = _run(monkeypatch, handle, core, [cmd], manage_trigger=False)
    (msg,) = _frames(conn)
    assert (msg.height, msg.width) == (HEIGHT, WIDTH)
    assert msg.nbytes == HEIGHT * WIDTH * 2
    img = handle.read_frame(msg.slot_index, msg.nbytes, msg.height, msg.width)
    assert img.shape == (HEIGHT, WIDTH)
    assert img.dtype == np.uint16
    assert np.all(img == 1)
