"""Signals the image-based focus lock emits while a sequence runs.

The lock itself lives in :mod:`~pymmcore_gui.asi_z_stack.image_focus_run`
(driven by the acquisition engine) and knows nothing about Qt. It reports what
it is doing through the module-level :data:`focus_lock_signals`, which a UI
panel can connect to.

**Threading:** every signal but ``box_changed`` is emitted from the **MDA
thread** (the engine's ``exec_event`` / ``setup_sequence`` /
``teardown_sequence`` caller), never the GUI thread. A slot that touches Qt
widgets must hop threads itself: ``connect(slot, thread="main")`` delivers
nothing unless psygnal's emission queue is being drained, so the UI forwards
each signal to a ``pyqtSignal`` on a GUI-thread ``QObject`` (see
``ImageFocusWidget``). A slot that raises is swallowed and logged by the
emitting side, so a broken panel cannot stall an acquisition.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from psygnal import Signal, SignalGroup

from pymmcore_gui.asi_z_stack.image_focus import FocusCalibration, LockDecision

#: Key under ``MDASequence.metadata`` that marks a sequence as an image-focus
#: **calibration** run (a piezo sweep, see
#: :func:`~pymmcore_gui.asi_z_stack.image_focus_run.build_calibration_sequence`).
#: Its value is ``{"steps_um": [...], "reps": int, "box": {camera, preset,
#: slice_index, rect}, "window": str}``.
CALIBRATION_METADATA_KEY = "image_focus_calibration"


@dataclass(frozen=True)
class FocusLockReport:
    """What the focus lock did after one volume."""

    index: dict[str, int]
    """The volume's MDA index, e.g. ``{"t": 3, "c": 0}``."""
    mode: str
    """``"image"`` (holding focus) or ``"calibrate"`` (piezo sweep)."""
    piezo_um: float | None
    """Piezo position after this volume's correction (absolute, um); ``None``
    if it could not be read."""
    decision: LockDecision | None
    """The controller's decision (image mode, once a reference exists); ``None``
    for the reference volume, a volume with no frame, and calibration volumes."""
    message: str = field(default="")
    """Human-readable note, e.g. ``"reference captured"`` or ``"no frame for
    box (1/3)"``."""


class FocusLockSignals(SignalGroup):
    """The signals of :data:`focus_lock_signals` (see the module docstring)."""

    report = Signal(FocusLockReport)
    """One per volume the lock handled."""
    calibrated = Signal(str, FocusCalibration)
    """``(calibration_key, fit)`` after a calibration run was fitted and saved."""
    stopped = Signal(str)
    """The lock ended or gave up; the argument is the reason."""
    box_changed = Signal()
    """``image_focus.box`` was drawn, moved or cleared in the UI (emitted on the
    GUI thread, unlike the others, which come from the MDA thread)."""


#: Process-wide instance the engine emits on and the UI connects to.
focus_lock_signals = FocusLockSignals()
