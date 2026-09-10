from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

import numpy as np
import useq

from pymmcore_gui._async_writer import AsyncWriter
from pymmcore_gui._multi_camera_handler import MultiCameraHandler

if TYPE_CHECKING:
    from pathlib import Path

SEQ = useq.MDASequence(
    channels=["DAPI"],  # pyright: ignore[reportArgumentType]
    z_plan=useq.ZRangeAround(range=2, step=1),  # 3 z slices
)
N_FRAMES = len(list(SEQ))
META: Any = {}  # FrameMetaV1 stand-in; the writers under test don't read it


class _RecordingWriter:
    """Minimal writer that records the frames it is handed."""

    def __init__(self, *, raise_on: int | None = None, delay: float = 0.0) -> None:
        self.frames: list[int] = []
        self.started = False
        self.finished = False
        self._raise_on = raise_on
        self._delay = delay
        self._n = 0

    def sequenceStarted(self, seq: Any, meta: Any = None) -> None:
        self.started = True

    def frameReady(self, frame: np.ndarray, event: useq.MDAEvent, meta: Any) -> None:
        self._n += 1
        if self._delay:
            time.sleep(self._delay)
        if self._raise_on is not None and self._n == self._raise_on:
            raise RuntimeError("boom")
        self.frames.append(int(frame.flat[0]))

    def sequenceFinished(self, seq: Any) -> None:
        self.finished = True


def _drive(handler: Any) -> None:
    handler.sequenceStarted(SEQ, {})
    for i, event in enumerate(SEQ):
        handler.frameReady(np.full((4, 4), i, dtype=np.uint16), event, {})
    handler.sequenceFinished(SEQ)


def test_async_writer_happy_path() -> None:
    inner = _RecordingWriter()
    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(inner, name="cam", reporter=lambda e, c: reports.append((e, c)))

    _drive(w)

    assert inner.started and inner.finished
    assert inner.frames == list(range(N_FRAMES))  # every frame, in order
    assert not w.incomplete
    assert reports == []


def test_async_writer_survives_and_surfaces_writer_exception() -> None:
    inner = _RecordingWriter(raise_on=2)
    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(inner, name="cam", reporter=lambda e, c: reports.append((e, c)))

    # the acquisition side must never see the writer's exception
    _drive(w)

    assert w.incomplete
    assert "write error" in w.status_detail
    assert any("boom" in repr(e) for e, _ in reports)
    # every frame except the failing one still got written
    assert len(inner.frames) == N_FRAMES - 1
    assert inner.finished


def test_async_writer_bounds_backlog_and_alarms() -> None:
    # a slow writer + a budget smaller than two frames -> drops after the first
    inner = _RecordingWriter(delay=0.05)
    reports: list[tuple[BaseException, str]] = []
    frame_bytes = 32 * 32 * 2
    w = AsyncWriter(
        inner,
        name="cam",
        backlog_budget_bytes=frame_bytes + 1,
        reporter=lambda e, c: reports.append((e, c)),
    )
    event = next(iter(SEQ))
    w.sequenceStarted(SEQ, {})
    for _ in range(20):
        w.frameReady(np.zeros((32, 32), dtype=np.uint16), event, META)
    w.sequenceFinished(SEQ)

    assert w.incomplete
    assert "dropped" in w.status_detail
    assert any("keep up" in str(e) for e, _ in reports)
    assert len(inner.frames) < 20  # peak backlog stayed bounded


def test_multi_camera_handler_surfaces_writer_failure(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """A failing per-camera writer is reported, not silently swallowed."""
    from pymmcore_gui import _multi_camera_handler as mod

    reports: list[tuple[BaseException, str]] = []
    monkeypatch.setattr(
        "pymmcore_gui._async_writer.report_background_exception",
        lambda e, c="": reports.append((e, c)),
    )
    monkeypatch.setattr(
        mod, "handler_for_path", lambda *a, **k: _RecordingWriter(raise_on=2)
    )

    class _StubCore:
        def getNumberOfCameraChannels(self) -> int:
            return 2

        def getCameraDevice(self) -> str:
            return "MultiCam"

        def getPhysicalCameraDevice(self, i: int) -> str:
            return ["Camera", "Camera2"][i]

    handler = MultiCameraHandler(
        tmp_path / "acq.ome.zarr",
        mmcore=_StubCore(),  # type: ignore[arg-type] # pyright: ignore[reportArgumentType]
    )
    frame = np.zeros((4, 4), np.uint16)
    handler.sequenceStarted(SEQ, {})
    for event in SEQ:
        for cam in ("Camera", "Camera2"):
            meta: Any = {"camera_device": cam}
            handler.frameReady(frame, event, meta)
    handler.sequenceFinished(SEQ)  # must not raise

    assert any("boom" in repr(e) for e, _ in reports)


def test_ome_zarr_writer_uncompressed_by_default(tmp_path: Path) -> None:
    import zarr

    from pymmcore_gui._vendored.mda_handlers import handler_for_path

    w: Any = handler_for_path(tmp_path / "exp.ome.zarr")
    w.sequenceStarted(SEQ, {})
    for event in SEQ:
        w.frameReady(np.zeros((4, 4), np.uint16), event, {})
    w.sequenceFinished(SEQ)

    arr: Any = zarr.open(str(tmp_path / "exp.ome.zarr"), mode="r")["p0"]
    assert arr.compressor is None
