"""IPC message types exchanged between the main process and a camera worker.

Every message is a frozen, plain-data dataclass -- no numpy arrays, no
``CMMCorePlus``/device references -- so they pickle cleanly across the
``multiprocessing.Pipe`` connecting a :class:`~pymmcore_gui.asi_z_stack.
worker_pool.CameraWorkerHandle` to its :func:`~pymmcore_gui.asi_z_stack.
camera_worker.run_camera_worker` subprocess. Frame pixel data itself travels
separately, via shared memory (see :mod:`~pymmcore_gui.asi_z_stack.worker_pool`);
:class:`FrameMsg` only carries the shared-memory slot index and metadata needed
to interpret it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

# ---------------------------------------------------------------------------
# Main process -> worker
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ArmCmd:
    """Arm the camera for a hardware-triggered sequence of *n_images* frames."""

    n_images: int


@dataclass(frozen=True)
class SlotFreeCmd:
    """Tell the worker that shared-memory slot *slot_index* is free to reuse."""

    slot_index: int


@dataclass(frozen=True)
class StopCmd:
    """Stop the current sequence acquisition, if any, and return to idle."""


@dataclass(frozen=True)
class ShutdownCmd:
    """Stop if running, unload the camera device, and exit the worker process."""


# ---------------------------------------------------------------------------
# Worker -> main process
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ReadyMsg:
    """The worker has loaded and initialized its camera and is idle."""

    camera_label: str


@dataclass(frozen=True)
class ArmedMsg:
    """The worker's camera has accepted ``startSequenceAcquisition``."""

    camera_label: str


@dataclass(frozen=True)
class FrameMsg:
    """One frame is ready to read from the worker's shared-memory ring buffer."""

    camera_label: str
    slot_index: int
    slice_idx: int
    nbytes: int
    camera_metadata: dict[str, Any] = field(default_factory=dict)
    images_remaining: int = 0
    worker_perf_counter: float = 0.0


@dataclass(frozen=True)
class StoppedMsg:
    """The worker reached *images_collected* frames or acknowledged a StopCmd."""

    camera_label: str
    images_collected: int


@dataclass(frozen=True)
class StalledMsg:
    """No new frame has arrived for *seconds_since_last_image* seconds."""

    camera_label: str
    images_collected: int
    seconds_since_last_image: float


@dataclass(frozen=True)
class ErrorMsg:
    """The worker hit an exception while armed; it has returned to idle."""

    camera_label: str
    exc_type: str
    message: str
    traceback_text: str


WorkerToMainMsg = ReadyMsg | ArmedMsg | FrameMsg | StoppedMsg | StalledMsg | ErrorMsg
MainToWorkerMsg = ArmCmd | SlotFreeCmd | StopCmd | ShutdownCmd
