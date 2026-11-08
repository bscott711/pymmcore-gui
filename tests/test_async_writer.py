from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest
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


# ---------------------------------------------------------------------------
# zero-frame stores: persistent warning + .run_status.json sidecar
# ---------------------------------------------------------------------------


def _relpaths(root: Path) -> set[str]:
    return {str(p.relative_to(root)) for p in root.rglob("*")}


def _zarr_writer(path: Path) -> Any:
    from pymmcore_gui._vendored.mda_handlers import handler_for_path

    return handler_for_path(path)


def test_zero_frame_store_gets_warning_and_sidecar(tmp_path: Path) -> None:
    import json

    from pymmcore_gui._async_writer import ZeroFrameSaveError

    store = tmp_path / "exp_561.ome.zarr"
    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(
        _zarr_writer(store),
        name="561",
        reporter=lambda e, c: reports.append((e, c)),
        finish_reason=lambda: "errored",
    )
    w.sequenceStarted(SEQ, {})
    w.sequenceFinished(SEQ)

    assert w.frames_written == 0
    assert len(reports) == 1
    err, ctx = reports[0]
    assert isinstance(err, ZeroFrameSaveError)
    assert err.persistent_notice is True
    assert ctx == "MDA save"
    assert "exp_561.ome.zarr" in str(err)
    assert "errored" in str(err)

    sidecar = store / ".run_status.json"
    data = json.loads(sidecar.read_text())
    assert data["format"] == 1
    assert data["run_status"] == "zero_frames"
    assert data["finish_reason"] == "errored"
    assert data["frames_written"] == 0
    assert data["frames_received"] == 0
    assert data["frames_dropped"] == 0
    assert data["store"] == "exp_561.ome.zarr"
    assert data["writer_error"] is None
    assert data["finished_at"]
    assert not list(store.glob("*.part"))
    # the shell itself is unchanged: no root .zattrs, no p0
    assert not (store / ".zattrs").exists()
    assert not (store / "p0").exists()


def test_store_with_frames_has_no_sidecar_and_same_files(tmp_path: Path) -> None:
    ref = tmp_path / "ref.ome.zarr"
    w_ref: Any = _zarr_writer(ref)
    w_ref.sequenceStarted(SEQ, {})
    for event in SEQ:
        w_ref.frameReady(np.zeros((4, 4), np.uint16), event, {})
    w_ref.sequenceFinished(SEQ)

    store = tmp_path / "exp.ome.zarr"
    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(
        _zarr_writer(store),
        name="cam",
        reporter=lambda e, c: reports.append((e, c)),
        finish_reason=lambda: "completed",
    )
    _drive(w)

    assert w.frames_written == N_FRAMES
    assert reports == []
    assert not (store / ".run_status.json").exists()
    assert _relpaths(store) == _relpaths(ref)
    assert {".zgroup", ".zattrs", "p0"} <= _relpaths(store)


@pytest.mark.parametrize("provider", ["raises", "none"])
def test_zero_frame_finish_reason_unknown(tmp_path: Path, provider: str) -> None:
    import json

    def _raises() -> object:
        raise RuntimeError("no status")

    store = tmp_path / "exp.ome.zarr"
    w = AsyncWriter(
        _zarr_writer(store),
        name="cam",
        reporter=lambda e, c: None,
        finish_reason=_raises if provider == "raises" else (lambda: None),
    )
    w.sequenceStarted(SEQ, {})
    w.sequenceFinished(SEQ)

    data = json.loads((store / ".run_status.json").read_text())
    assert data["finish_reason"] == "unknown"


def test_finish_reason_enum_value(tmp_path: Path) -> None:
    import enum
    import json

    class _Reason(str, enum.Enum):
        CANCELED = "canceled"

    store = tmp_path / "exp.ome.zarr"
    w = AsyncWriter(
        _zarr_writer(store),
        name="cam",
        reporter=lambda e, c: None,
        finish_reason=lambda: _Reason.CANCELED,
    )
    w.sequenceStarted(SEQ, {})
    w.sequenceFinished(SEQ)

    assert json.loads((store / ".run_status.json").read_text())["finish_reason"] == (
        "canceled"
    )


def test_sidecar_write_failure_is_logged_not_raised(
    tmp_path: Path, monkeypatch: Any, caplog: Any
) -> None:
    import logging

    def _boom(*a: Any, **k: Any) -> None:
        raise OSError("disk gone")

    import os

    class _OS:
        replace = staticmethod(_boom)

        def __getattr__(self, name: str) -> Any:
            return getattr(os, name)

    # only the sidecar's os.replace fails (zarr itself uses os.replace too)
    monkeypatch.setattr("pymmcore_gui._async_writer.os", _OS())

    store = tmp_path / "exp.ome.zarr"
    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(
        _zarr_writer(store),
        name="cam",
        reporter=lambda e, c: reports.append((e, c)),
        finish_reason=lambda: "completed",
    )
    w.sequenceStarted(SEQ, {})
    with caplog.at_level(logging.WARNING, logger="pymmcore_gui._async_writer"):
        w.sequenceFinished(SEQ)  # must not raise

    assert not (store / ".run_status.json").exists()
    assert any("run_status" in r.getMessage() for r in caplog.records)
    assert len(reports) == 1  # the zero-frame warning still fires


def test_zero_frame_non_zarr_writer_warns_without_sidecar(tmp_path: Path) -> None:
    from pymmcore_gui._async_writer import ZeroFrameSaveError

    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(
        _RecordingWriter(),
        name="cam",
        reporter=lambda e, c: reports.append((e, c)),
        finish_reason=lambda: "completed",
    )
    w.sequenceStarted(SEQ, {})
    w.sequenceFinished(SEQ)

    assert len(reports) == 1
    assert isinstance(reports[0][0], ZeroFrameSaveError)
    assert not list(tmp_path.rglob(".run_status.json*"))


def test_zero_frame_canceled_run_is_reported_but_not_sticky(tmp_path: Path) -> None:
    from pymmcore_gui._async_writer import ZeroFrameSaveError

    store = tmp_path / "exp.ome.zarr"
    reports: list[tuple[BaseException, str]] = []
    w = AsyncWriter(
        _zarr_writer(store),
        name="cam",
        reporter=lambda e, c: reports.append((e, c)),
        finish_reason=lambda: "canceled",
    )
    w.sequenceStarted(SEQ, {})
    w.sequenceFinished(SEQ)

    assert len(reports) == 1
    err = reports[0][0]
    assert isinstance(err, ZeroFrameSaveError)
    assert err.persistent_notice is False
    assert ZeroFrameSaveError.persistent_notice is True  # class default unchanged
    assert (store / ".run_status.json").exists()
