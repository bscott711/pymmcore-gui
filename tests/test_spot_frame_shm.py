"""The spot camera's shared-memory frame buffer (seqlock, no Qt, no hardware).

Run with ``--noconftest``.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.spot_frame_shm import (
    HEADER_SIZE,
    FrameBuffer,
    unlink_by_name,
)

SENSOR = (2048, 1536)
META = {
    "roi": (0, 0, 64, 48),
    "sensor": SENSOR,
    "bits": 8,
    "spot": None,
    "spot_ok": False,
    "saturated_frac": 0.0,
    "max_value": 0,
}


@pytest.fixture
def writer():
    fb = FrameBuffer.create(SENSOR[0] * SENSOR[1] * 2)
    yield fb
    fb.close()
    fb.unlink()


def _frame(rows: int, cols: int, dtype=np.uint8, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, np.iinfo(dtype).max, (rows, cols), dtype=dtype)


def test_round_trip_keeps_pixels_and_metadata(writer):
    reader = FrameBuffer.attach(writer.name)
    try:
        img = _frame(48, 64, np.uint8, 1)
        seq = writer.write(
            img,
            t=12.5,
            roi=(100, 200, 64, 48),
            sensor=SENSOR,
            bits=8,
            spot=(131.25, 224.5),
            spot_ok=True,
            saturated_frac=0.125,
            max_value=250,
        )
        got = reader.read()
        assert got is not None
        assert got.seq == seq == 1
        np.testing.assert_array_equal(got.image, img)
        assert got.image.dtype == np.uint8
        assert got.t == 12.5
        assert got.roi == (100, 200, 64, 48)
        assert got.sensor == SENSOR
        assert got.bits == 8
        assert got.spot == (131.25, 224.5)
        assert got.spot_ok is True
        assert got.saturated_frac == 0.125
        assert got.max_value == 250
        # The reader holds a copy: a later write does not change it.
        writer.write(
            _frame(48, 64, np.uint8, 2), t=13.0, **{**META, "roi": (0, 0, 64, 48)}
        )
        np.testing.assert_array_equal(got.image, img)
    finally:
        reader.close()


def test_read_returns_none_until_there_is_something_newer(writer):
    reader = FrameBuffer.attach(writer.name)
    try:
        assert reader.read() is None  # nothing written yet
        writer.write(_frame(8, 8), t=1.0, **META)
        first = reader.read()
        assert first is not None
        assert reader.read(first.seq) is None
        writer.write(_frame(8, 8, seed=3), t=2.0, **META)
        second = reader.read(first.seq)
        assert second is not None and second.seq == first.seq + 1
        assert reader.read(second.seq) is None
    finally:
        reader.close()


def test_uint16_and_any_roi_up_to_the_full_sensor(writer):
    reader = FrameBuffer.attach(writer.name)
    try:
        for rows, cols, dtype in [
            (48, 64, np.uint8),
            (1536, 2048, np.uint8),  # full sensor, 8-bit
            (1536, 2048, np.uint16),  # full sensor at the largest sample size
            (8, 8, np.uint16),
            (480, 640, np.uint8),
        ]:
            img = _frame(rows, cols, dtype, rows)
            writer.write(img, t=0.0, **{**META, "roi": (0, 0, cols, rows)})
            got = reader.read()
            assert got is not None
            assert got.image.shape == (rows, cols)
            assert got.image.dtype == dtype
            np.testing.assert_array_equal(got.image, img)
    finally:
        reader.close()


def test_a_frame_that_does_not_fit_is_refused(writer):
    small = FrameBuffer.create(100)
    try:
        with pytest.raises(ValueError, match="does not fit"):
            small.write(np.zeros((20, 20), np.uint8), t=0.0, **META)
        with pytest.raises(ValueError):
            small.write(np.zeros((4, 4), np.float32), t=0.0, **META)
    finally:
        small.close()
        small.unlink()


def test_spot_may_be_absent(writer):
    reader = FrameBuffer.attach(writer.name)
    try:
        writer.write(_frame(8, 8), t=0.0, **META)
        got = reader.read()
        assert got is not None and got.spot is None and not got.spot_ok
    finally:
        reader.close()


def test_attach_rejects_a_foreign_segment():
    from multiprocessing import shared_memory

    shm = shared_memory.SharedMemory(create=True, size=4096)
    try:
        with pytest.raises(ValueError, match="not a spot frame buffer"):
            FrameBuffer.attach(shm.name)
    finally:
        shm.close()
        shm.unlink()


@pytest.mark.skipif(os.name == "nt", reason="POSIX names vanish on unlink")
def test_unlink_removes_the_name_and_attach_does_not_destroy_it():
    fb = FrameBuffer.create(1024)
    name = fb.name
    reader = FrameBuffer.attach(name)
    reader.close()  # an attached reader closing must not remove the segment
    again = FrameBuffer.attach(name)
    again.close()
    fb.close()
    fb.unlink()
    with pytest.raises(FileNotFoundError):
        FrameBuffer.attach(name)
    unlink_by_name(name)  # already gone: quietly nothing


def test_close_is_idempotent_and_blocks_use(writer):
    reader = FrameBuffer.attach(writer.name)
    reader.close()
    reader.close()
    assert reader.closed
    assert reader.read() is None


def test_no_torn_read_under_a_concurrent_writer_thread():
    """Every frame is one constant value that is also written into the metadata.

    A torn copy mixes two frames, so its pixels are not uniform or disagree
    with ``t``. The writer paces itself with tiny sleeps so the reader really
    sees both quiet and in-flight slots.
    """
    rows, cols = 600, 800
    fb = FrameBuffer.create(rows * cols * 2)
    reader = FrameBuffer.attach(fb.name)
    stop = threading.Event()
    n_written = [0]

    def write() -> None:
        v = 0
        buf = np.empty((rows, cols), np.uint16)
        while not stop.is_set():
            v = (v + 1) % 60000
            buf.fill(v)
            fb.write(
                buf, t=float(v), roi=(0, 0, cols, rows), sensor=SENSOR, bits=16,
                max_value=v,
            )  # fmt: skip
            n_written[0] += 1
            time.sleep(0)

    th = threading.Thread(target=write, daemon=True)
    th.start()
    try:
        torn = 0
        reads = 0
        last = None
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            got = reader.read(last)
            if got is None:
                continue
            reads += 1
            last = got.seq
            v = got.image.flat[0]
            if not (got.image == v).all() or got.t != float(v) or got.max_value != v:
                torn += 1
        assert reads > 20, reads
        assert torn == 0, f"{torn} torn reads out of {reads}"
    finally:
        stop.set()
        th.join(5.0)
        reader.close()
        fb.close()
        fb.unlink()
    assert n_written[0] > reads  # the writer was ahead: frames were skipped, not torn


def test_negative_control_a_copy_without_the_seqlock_does_tear():
    """The tear check above is only worth something if tears do happen."""
    from pymmcore_gui.asi_z_stack.spot_frame_shm import HEADER_SIZE as H

    rows, cols = 600, 800
    fb = FrameBuffer.create(rows * cols * 2)
    stop = threading.Event()

    def write() -> None:
        v = 0
        buf = np.empty((rows, cols), np.uint16)
        while not stop.is_set():
            v = (v + 1) % 60000
            buf.fill(v)
            fb.write(buf, t=float(v), roi=(0, 0, cols, rows), sensor=SENSOR, bits=16)

    th = threading.Thread(target=write, daemon=True)
    th.start()
    time.sleep(0.2)
    torn = 0
    t_end = time.monotonic() + 1.5
    try:
        while time.monotonic() < t_end:
            a = np.frombuffer(
                fb._buf, dtype=np.uint16, count=rows * cols, offset=H
            ).copy()
            if not (a == a[0]).all():
                torn += 1
    finally:
        stop.set()
        th.join(5.0)
        fb.close()
        fb.unlink()
    assert torn > 0


def _hammer(name_q, stop_ev, rows: int, cols: int) -> None:
    """Writer in another process: constant-valued frames, value in the metadata."""
    fb = FrameBuffer.create(rows * cols * 2)
    name_q.put(fb.name)
    v = 0
    buf = np.empty((rows, cols), np.uint16)
    while not stop_ev.is_set():
        v = (v + 1) % 60000
        buf.fill(v)
        fb.write(
            buf, t=float(v), roi=(0, 0, cols, rows), sensor=SENSOR, bits=16, max_value=v
        )
        time.sleep(0.002)  # a camera delivers at most ~50 fps; never flat out
    fb.close()
    fb.unlink()


def test_no_torn_read_from_a_writer_process():
    """The same check across a real process boundary (no shared GIL)."""
    import pymmcore_gui

    ctx = mp.get_context("spawn")
    q = ctx.Queue()
    stop = ctx.Event()
    rows, cols = 600, 800
    src = str(Path(pymmcore_gui.__file__).resolve().parent.parent)
    old = os.environ.get("PYTHONPATH", "")
    os.environ["PYTHONPATH"] = src + (os.pathsep + old if old else "")
    proc = ctx.Process(target=_hammer, args=(q, stop, rows, cols), daemon=True)
    try:
        proc.start()
        name = q.get(timeout=30.0)
        reader = FrameBuffer.attach(name)
        torn = reads = 0
        last = None
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            got = reader.read(last)
            if got is None:
                continue
            reads += 1
            last = got.seq
            v = got.image.flat[0]
            if not (got.image == v).all() or got.t != float(v) or got.max_value != v:
                torn += 1
        assert reads > 20, reads
        assert torn == 0, f"{torn} torn reads out of {reads}"
        reader.close()
    finally:
        stop.set()
        proc.join(10.0)
        if proc.is_alive():
            proc.kill()
        if old:
            os.environ["PYTHONPATH"] = old
        else:
            os.environ.pop("PYTHONPATH", None)
    if os.name != "nt":
        with pytest.raises(FileNotFoundError):  # the writer unlinked on exit
            FrameBuffer.attach(name)


def test_the_header_leaves_the_pixels_aligned():
    assert HEADER_SIZE % 64 == 0
