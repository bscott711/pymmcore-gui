"""Unit tests for worker_pool.py's pure helpers.

``CameraWorkerPool``/``CameraWorkerHandle`` need a real subprocess and
shared memory, so they're left to bench/integration verification
(consistent with this package's existing convention -- see
``test_asi_zstack_camera_worker.py``). ``_worker_stderr_file`` takes only a
camera label and reads the current logfile path, so it's directly
unit-testable with a mocked logfile.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import pymmcore_gui.asi_z_stack.worker_pool as worker_pool_mod
from pymmcore_gui.asi_z_stack.worker_pool import _worker_stderr_file

if TYPE_CHECKING:
    import pytest


def test_worker_stderr_file_is_sibling_of_main_logfile(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Same directory/stem as the main process's own pymmcore-plus logfile."""
    main_log = Path("/logs/pymmcore-plus-pid1234.log")
    monkeypatch.setattr(worker_pool_mod, "current_logfile", lambda _logger: main_log)

    result = _worker_stderr_file("Camera-1")

    assert result == str(Path("/logs/pymmcore-plus-pid1234-worker-Camera-1-stderr.log"))


def test_worker_stderr_file_sanitizes_unsafe_characters_in_label(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Regression guard: a camera label isn't necessarily filesystem-safe."""
    main_log = Path("/logs/pymmcore-plus-pid1234.log")
    monkeypatch.setattr(worker_pool_mod, "current_logfile", lambda _logger: main_log)

    result = _worker_stderr_file("Camera:1/weird")

    assert "/" not in Path(result).name
    assert ":" not in Path(result).name


def test_worker_stderr_file_empty_when_no_main_logfile_configured(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """No sensible sibling location exists -- caller must leave stderr inherited."""
    monkeypatch.setattr(worker_pool_mod, "current_logfile", lambda _logger: None)

    assert _worker_stderr_file("Camera-1") == ""


def test_iter_frames_times_out_when_worker_only_reports_stalls() -> None:
    """Rig hang 2026-09-23: a stalled worker's repeated StalledMsg reset the
    stall guard forever, so the MDA hung with no error and no way to cancel."""
    import threading
    import time
    from multiprocessing import Pipe
    from types import SimpleNamespace

    import pytest

    from pymmcore_gui.asi_z_stack.worker_messages import StalledMsg
    from pymmcore_gui.asi_z_stack.worker_pool import (
        CameraWorkerHandle,
        CameraWorkerPool,
    )

    parent, child = Pipe(duplex=True)
    never_ready, _keep_open = Pipe(duplex=False)
    handle = CameraWorkerHandle(
        "Camera-1",
        None,  # type: ignore[arg-type]
        height=1,
        width=1,
        dtype="uint16",
        n_slots=1,
    )
    handle.conn = parent
    handle.process = SimpleNamespace(sentinel=never_ready, exitcode=None)  # type: ignore[assignment]

    stop = threading.Event()

    def _spam_stalls() -> None:
        while not stop.is_set():
            child.send(StalledMsg("Camera-1", 50, 6.0))
            time.sleep(0.05)

    spammer = threading.Thread(target=_spam_stalls, daemon=True)
    spammer.start()
    try:
        start = time.monotonic()
        with pytest.raises(TimeoutError, match="no frame"):
            list(CameraWorkerPool([handle]).iter_frames(stall_timeout_s=0.5))
        assert time.monotonic() - start < 3.0
    finally:
        stop.set()
        spammer.join(timeout=1)
