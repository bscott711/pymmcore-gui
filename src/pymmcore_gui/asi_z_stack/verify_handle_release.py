"""Bench verification of whether a released PVCAM camera handle survives a handoff.

Standalone diagnostic, not part of the acquisition path. Run this once, on the real
rig, before building any of the camera-worker-process machinery it exists to justify
(see the "Dual-camera MDA: isolate PVCAM cameras into worker processes" plan). It
answers the single riskiest assumption that plan depends on: that when the main
process unloads a PVCAM camera device (``CMMCorePlus.unloadDevice``, which calls the
device adapter's ``Shutdown()``), the underlying PVCAM camera handle is released
cleanly enough for a *different* process to immediately open the same physical
camera exclusively. If this fails outright, the whole two-process design needs
rethinking before further investment; if it only succeeds after a delay, that delay
becomes ``HardwareConstants.worker_ready_timeout_s``.

Run with::

    python -m pymmcore_gui.asi_z_stack.verify_handle_release [--camera Camera-1]

Uses plain ``print()`` rather than the shared :func:`~pymmcore_gui.asi_z_stack.
_logging.configure_asi_logging` machinery for its result output: importing
``pymmcore_gui`` (unavoidable, since this lives in its ``asi_z_stack`` subpackage)
can attach a root-level logging handler before this module's own logging is
configured, which silently swallows ``INFO``-level messages under
``configure_asi_logging``'s "only add a handler if none exists yet" idempotency
check. ``print()`` sidesteps that entirely, which matters here since the whole
point of this script is a human reading its PASS/FAIL result.
"""

from __future__ import annotations

import argparse
import multiprocessing
import sys
import time
import traceback
from multiprocessing import get_context
from typing import TYPE_CHECKING

from .common import HardwareConstants

if TYPE_CHECKING:
    from multiprocessing.context import SpawnContext
    from multiprocessing.queues import SimpleQueue

_ADAPTER_MODULE = "PVCAM"
_RETRY_DELAYS_S = (0.5, 1.0, 2.0, 5.0, 10.0)


def _log(message: str) -> None:
    """Print *message* to stderr, flushed immediately.

    Parameters
    ----------
    message : str
        The line to print.
    """
    print(message, file=sys.stderr, flush=True)


def _open_and_snap(camera_label: str, device_name: str) -> None:
    """Load, initialize, and snap *camera_label* on a fresh ``CMMCorePlus``.

    Parameters
    ----------
    camera_label : str
        MMCore device label to load the camera under (e.g. ``"Camera-1"``).
    device_name : str
        Device name within the ``PVCAM`` adapter (matches *camera_label* in this
        rig's hardware profile, e.g. ``Device,Camera-1,PVCAM,Camera-1``).
    """
    from pymmcore_plus import CMMCorePlus

    mmc = CMMCorePlus()
    mmc.loadDevice(camera_label, _ADAPTER_MODULE, device_name)
    mmc.initializeDevice(camera_label)
    mmc.setCameraDevice(camera_label)
    mmc.snapImage()
    img = mmc.getImage()
    if img is None or img.size == 0:
        raise RuntimeError("snapImage() produced an empty image")
    mmc.unloadDevice(camera_label)


def _worker_entry(
    camera_label: str, device_name: str, result_queue: SimpleQueue[tuple[bool, str]]
) -> None:
    """``Process`` target: attempt :func:`_open_and_snap`, report the outcome.

    Parameters
    ----------
    camera_label : str
        MMCore device label to load the camera under.
    device_name : str
        Device name within the ``PVCAM`` adapter.
    result_queue : SimpleQueue[tuple[bool, str]]
        Queue to report ``(success, error_text)`` back to the parent process.
    """
    try:
        _open_and_snap(camera_label, device_name)
    except Exception:
        result_queue.put((False, traceback.format_exc()))
    else:
        result_queue.put((True, ""))


def _try_reopen_in_subprocess(
    ctx: SpawnContext, camera_label: str, device_name: str
) -> tuple[bool, str]:
    """Spawn one worker process that attempts to reopen *camera_label*.

    Parameters
    ----------
    ctx : SpawnContext
        Spawn context to launch the worker process with.
    camera_label : str
        MMCore device label to load the camera under.
    device_name : str
        Device name within the ``PVCAM`` adapter.

    Returns
    -------
    tuple[bool, str]
        ``(success, error_text)`` -- ``error_text`` is empty on success.
    """
    result_queue: SimpleQueue[tuple[bool, str]] = ctx.SimpleQueue()
    proc = ctx.Process(
        target=_worker_entry,
        args=(camera_label, device_name, result_queue),
        daemon=False,
    )
    proc.start()
    ok, err = result_queue.get()
    proc.join(timeout=10)
    return ok, err


def run(camera_label: str) -> bool:
    """Run the full handle-release check for *camera_label*.

    Parameters
    ----------
    camera_label : str
        MMCore device label to test (e.g. ``"Camera-1"``).

    Returns
    -------
    bool
        Whether the handle-release check passed.
    """
    device_name = camera_label
    from pymmcore_plus import CMMCorePlus

    _log(f"[main] Loading + snapping {camera_label!r} in the main process...")
    mmc = CMMCorePlus()
    mmc.loadDevice(camera_label, _ADAPTER_MODULE, device_name)
    mmc.initializeDevice(camera_label)
    mmc.setCameraDevice(camera_label)
    mmc.snapImage()
    img = mmc.getImage()
    _log(f"[main] Snap OK -- shape={img.shape}, dtype={img.dtype}")

    _log(f"[main] Unloading {camera_label!r}...")
    mmc.unloadDevice(camera_label)
    _log("[main] Unloaded. Spawning a fresh process immediately (no delay)...")

    ctx = get_context("spawn")
    t0 = time.monotonic()
    ok, err = _try_reopen_in_subprocess(ctx, camera_label, device_name)
    elapsed = time.monotonic() - t0

    if ok:
        _log(
            f"[worker] PASS -- reopened {camera_label!r} in a fresh process after "
            f"{elapsed:.2f}s with no artificial delay. Handle release is clean; "
            "HardwareConstants.worker_ready_timeout_s can stay small."
        )
        return True

    _log(f"[worker] FAIL immediately:\n{err}")
    _log("[main] Retrying with backoff to check for a timing race...")
    for delay in _RETRY_DELAYS_S:
        time.sleep(delay)
        ok, err = _try_reopen_in_subprocess(ctx, camera_label, device_name)
        if ok:
            _log(
                f"[worker] PASS after an extra {delay:.1f}s delay -- handle release "
                "is a timing race, not a hard failure. Size "
                "HardwareConstants.worker_ready_timeout_s with this much slack."
            )
            return True
        _log(f"[worker] still FAIL after {delay:.1f}s extra delay:\n{err}")

    _log(
        "[main] FAIL -- handle was not released even with backoff up to "
        f"{sum(_RETRY_DELAYS_S):.1f}s. The two-process design needs rethinking "
        "before further investment (see the plan's linchpin-assumption note)."
    )
    return False


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="verify_handle_release",
        description=(
            "Bench check: does a PVCAM camera handle released by one process "
            "become immediately openable by another?"
        ),
    )
    parser.add_argument(
        "--camera",
        default=None,
        help="Camera device label to test (default: HardwareConstants.camera_a_label).",
    )
    return parser.parse_args()


def main() -> int:
    """CLI entry point.

    Returns
    -------
    int
        Process exit code (``0`` on PASS, ``1`` on FAIL).
    """
    args = _parse_args()
    camera_label = args.camera or HardwareConstants().camera_a_label
    return 0 if run(camera_label) else 1


if __name__ == "__main__":
    multiprocessing.freeze_support()
    sys.exit(main())
