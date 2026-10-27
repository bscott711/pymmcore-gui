"""Image-based focus lock, run side: glue between the engine and ``image_focus``.

:mod:`~pymmcore_gui.asi_z_stack.image_focus` is pure numpy (box, profile, shift,
calibration, controller). This module is what lets the SPIM engine use it during
a sequence without the engine learning any of the details: the engine owns one
:class:`ImageFocusRun` per sequence and calls four hooks.

====================  ========================================================
``for_sequence``      ``setup_sequence``: decide the mode, check the box and
                      calibration exist. No hardware is touched.
``start``             end of ``setup_sequence``: hand the piezo over (unlock
                      CRISP, read the start position; calibration also moves
                      to its first step).
``on_frame``          ``exec_event``, once per frame, BEFORE it is yielded:
                      keep the row profile of the box's frame.
``after_volume``      end of ``exec_event`` after the cameras are stopped and
                      drained -- the idle gap before the next timepoint --
                      measure, then move the piezo.
``teardown``          ``teardown_sequence``: restore the piezo, write the CSV.
====================  ========================================================

Modes (see :attr:`ImageFocusRun.mode`):

- ``"image"`` -- ``SettingsV1.image_focus.mode == "image"``: the lock proper.
  The first volume with a profile becomes the reference; every later volume's
  profile is compared with it and the piezo is moved between volumes.
- ``"calibrate"`` -- the sequence carries
  :data:`~pymmcore_gui.asi_z_stack.image_focus_events.CALIBRATION_METADATA_KEY`
  (see :func:`build_calibration_sequence`): step the piezo through a list of
  offsets, one volume per step, then fit rows-per-um and store it in the
  settings.
- ``"off"`` -- anything else. Every hook returns immediately.

Safety rules this module keeps (it moves a real objective):

- Nothing in here raises into the engine. Every hook catches, logs, and
  deactivates the lock; the acquisition itself goes on without it.
- The piezo is moved only between volumes, never while a camera is armed.
- CRISP-P and this lock share the piezo and CRISP refuses moves while locked
  (Tiger error COD 47). The lock unlocks CRISP once at ``start`` and re-checks
  right before every move; if CRISP has locked again it gives up rather than
  fight it. This module never locks CRISP.
- A calibration that fits badly (``r2 < min_r2``) is never saved over a good one.

Signals are emitted on the MDA thread (see
:mod:`~pymmcore_gui.asi_z_stack.image_focus_events`).
"""

from __future__ import annotations

import csv
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import useq

from pymmcore_gui._settings import CalibrationRecord, SettingsV1

from .image_focus import (
    FocusBox,
    FocusLockController,
    calibration_key,
    fit_calibration,
    measure_shift,
    row_profile,
)
from .image_focus_events import (
    CALIBRATION_METADATA_KEY,
    FocusLockReport,
    focus_lock_signals,
)

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._settings import ImageFocusSettingsV1, SpectralChannelSettingsV1

    from .common import HardwareConstants
    from .image_focus import FocusCalibration, LockDecision

logger = logging.getLogger(__name__)

#: Piezo offsets (um, relative to where the sweep starts) of a calibration run.
DEFAULT_CALIBRATION_STEPS_UM = (-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5)

#: A calibration step further than this from the start is refused as a typo;
#: the lock itself never leaves ``max_excursion_um`` (3 um by default).
_MAX_CALIBRATION_STEP_UM = 10.0

#: Output suffixes stripped from ``save_name`` to name the CSV.
_SAVE_SUFFIXES = (".ome.tiff", ".ome.tif", ".ome.zarr", ".zarr", ".tiff", ".tif")

#: ``MDASequence.metadata`` key the MDA widget stores ``save_dir``/``save_name``
#: under (``pymmcore_widgets.useq_widgets.PYMMCW_METADATA_KEY``).
_WIDGETS_METADATA_KEY = "pymmcore_widgets"

#: After asking CRISP to unlock, how often / how long to re-read its state.
_CRISP_POLLS = 5
_CRISP_POLL_S = 0.2

CSV_COLUMNS = (
    "t",
    "c",
    "piezo_before",
    "error_rows",
    "error_um",
    "move_um",
    "confidence",
    "reason",
)


@dataclass
class FocusVolumeRow:
    """What the lock measured and did for one volume (one CSV line)."""

    t: int | None
    c: int | None
    piezo_before: float | None
    error_rows: float | None = None
    error_um: float | None = None
    move_um: float | None = None
    confidence: float | None = None
    reason: str = ""


def window_for_box(box: FocusBox, spectral: SpectralChannelSettingsV1 | None) -> str:
    """Name of the spectral window *box* lies in, ``""`` if there is none.

    Calibrations are keyed by ``(camera, preset, window)`` (see
    :func:`~pymmcore_gui.asi_z_stack.image_focus.calibration_key`): the window is
    the enabled spectral channel on the box's camera whose region contains the
    whole box. ``""`` when the spectral feature is off or no region contains it.
    """
    if spectral is None or not spectral.enabled:
        return ""
    bx, by, bw, bh = box.rect
    for channel in spectral.channels:
        if channel.camera != box.camera or channel.rect is None:
            continue
        x, y, w, h = channel.rect
        if x <= bx and y <= by and bx + bw <= x + w and by + bh <= y + h:
            return channel.name
    return ""


def _camera_roi(mmc: CMMCorePlus, camera: str) -> tuple[int, int, int, int] | None:
    """The camera's hardware ROI (no Snap); a seam so tests need no worker service."""
    from pymmcore_gui._spectral_channel_handler import camera_roi

    return camera_roi(mmc, camera)


def _strip_suffix(name: str) -> str:
    lowered = name.lower()
    for suffix in _SAVE_SUFFIXES:
        if lowered.endswith(suffix):
            return name[: -len(suffix)]
    return name.rstrip("/").rstrip("\\")


class ImageFocusRun:
    """One sequence's image-based focus lock (see the module docstring).

    Build it with :meth:`for_sequence`, never directly; it is cheap and does no
    hardware access until :meth:`start`.

    Parameters
    ----------
    mmc : CMMCorePlus
        The core; used only for the piezo (``hw.piezo_a_label``) and CRISP.
    hw : HardwareConstants
        Device labels.
    crisp_locked : Callable[[], bool]
        Whether CRISP-P is currently holding focus (the engine's own check).
    settings : ImageFocusSettingsV1
        A **snapshot** (deep copy) of ``SettingsV1.image_focus``, so edits made
        while the sequence runs apply to the next run only.
    """

    def __init__(
        self,
        mmc: CMMCorePlus,
        hw: HardwareConstants,
        *,
        crisp_locked: Callable[[], bool],
        settings: ImageFocusSettingsV1,
    ) -> None:
        self._mmc = mmc
        self._hw = hw
        self._crisp_locked = crisp_locked
        self.settings = settings

        self.mode: str = "off"
        """``"image"``, ``"calibrate"`` or ``"off"``."""
        self.active = False
        """Whether the hooks do anything; cleared on any failure / fall back."""
        self.intentional_um = 0.0
        """Net piezo motion this run commanded itself (the engine's piezo-moved
        warning allows for it)."""
        self.last_row: FocusVolumeRow | None = None
        """The row of the volume :meth:`after_volume` last handled."""
        self.rows: list[FocusVolumeRow] = []

        self._box: FocusBox | None = None
        self._key = ""
        self._origin = (0, 0)
        self._roi: tuple[int, int, int, int] | None = None
        self._geometry_checked = False
        self._profile: np.ndarray | None = None
        self._controller: FocusLockController | None = None
        self._start_um: float | None = None
        self._missing = 0
        self._crisp_refused = False
        self._stop_emitted = False
        self._save_dir: str | None = None
        self._save_name: str | None = None
        # calibration
        self._cal_steps: tuple[float, ...] = ()
        self._cal_reps = 1
        self._cal_window = ""
        self._cal_data: list[tuple[float, np.ndarray | None]] = []
        self._volume_no = 0

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def for_sequence(
        cls,
        mmc: CMMCorePlus,
        hw: HardwareConstants,
        sequence: useq.MDASequence,
        *,
        crisp_locked: Callable[[], bool],
        image_focus: ImageFocusSettingsV1 | None = None,
        spectral: SpectralChannelSettingsV1 | None = None,
    ) -> ImageFocusRun:
        """Decide this sequence's mode; never raises, touches no hardware.

        *image_focus* / *spectral* default to deep copies of the live settings.
        A mode that cannot run (no box, no calibration, bad calibration
        metadata) logs a WARNING, emits ``stopped`` and returns an inactive run:
        the acquisition goes on with no focus lock.
        """
        if image_focus is None:
            image_focus = SettingsV1.instance().image_focus.model_copy(deep=True)
        if spectral is None:
            spectral = SettingsV1.instance().spectral.model_copy(deep=True)
        run = cls(mmc, hw, crisp_locked=crisp_locked, settings=image_focus)
        try:
            run._configure(sequence, spectral)
        except Exception as err:
            run._stop(f"could not set up: {err}", logging.ERROR, exc_info=True)
        return run

    def _configure(
        self, sequence: useq.MDASequence, spectral: SpectralChannelSettingsV1 | None
    ) -> None:
        cfg = self.settings
        meta = sequence.metadata or {}
        widgets_meta = meta.get(_WIDGETS_METADATA_KEY) or {}
        self._save_dir = widgets_meta.get("save_dir") or None
        self._save_name = widgets_meta.get("save_name") or None

        if CALIBRATION_METADATA_KEY in meta:
            self._configure_calibration(meta[CALIBRATION_METADATA_KEY])
            return
        if cfg.mode != "image":
            return  # "crisp" / "off": no-op, zero cost

        self.mode = "image"
        if cfg.box is None:
            self._stop("no focus box drawn; running without the image focus lock")
            return
        box = cfg.box.to_box()
        key = calibration_key(box.camera, box.preset, window_for_box(box, spectral))
        if key not in cfg.calibrations:
            self._stop(
                f"image focus lock is not calibrated for {key!r}; running "
                "without it (run a calibration first)"
            )
            return
        self._box = box
        self._key = key
        self.active = True

    def _configure_calibration(self, payload: dict[str, Any]) -> None:
        self.mode = "calibrate"
        box = payload["box"]
        self._box = FocusBox(
            str(box["camera"]),
            str(box["preset"]),
            int(box["slice_index"]),
            tuple(int(v) for v in box["rect"]),  # type: ignore[arg-type]
        )
        self._cal_steps = tuple(float(s) for s in payload["steps_um"])
        self._cal_reps = int(payload["reps"])
        self._cal_window = str(payload.get("window", ""))
        if len(self._cal_steps) < 3 or self._cal_reps < 1:
            raise ValueError("a calibration needs at least 3 steps and 1 repeat")
        if max(abs(s) for s in self._cal_steps) > _MAX_CALIBRATION_STEP_UM:
            raise ValueError(
                f"calibration steps {self._cal_steps} exceed "
                f"+/-{_MAX_CALIBRATION_STEP_UM:g} um"
            )
        self._key = calibration_key(
            self._box.camera, self._box.preset, self._cal_window
        )
        self.active = True

    # ------------------------------------------------------------------
    # State shared with the engine
    # ------------------------------------------------------------------

    @property
    def owns_piezo(self) -> bool:
        """Whether this run has been commanding the piezo (``start`` succeeded)."""
        return self._start_um is not None and self.mode in ("image", "calibrate")

    # ------------------------------------------------------------------
    # start
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Take over the piezo. Call once, at the end of ``setup_sequence``.

        Unlocks CRISP if it is holding focus (INFO), reads the start position and
        the piezo's travel limits (best effort), builds the controller (image
        mode) or moves to the first calibration step (calibrate mode).
        """
        if not self.active:
            return
        try:
            self._start()
        except Exception as err:
            self._stop(f"could not start: {err}", logging.ERROR, exc_info=True)

    def _start(self) -> None:
        mmc, hw = self._mmc, self._hw
        assert self._box is not None
        if hw.piezo_a_label not in mmc.getLoadedDevices():
            self._stop(f"piezo {hw.piezo_a_label!r} is not loaded")
            return
        if not self._release_crisp():
            return

        # Where frames start on the sensor: boxes are in full-sensor pixels.
        self._roi = _camera_roi(mmc, self._box.camera)
        self._origin = (self._roi[0], self._roi[1]) if self._roi else (0, 0)

        self._start_um = float(mmc.getPosition(hw.piezo_a_label))
        travel = self._read_travel_limits()
        if self.mode == "image":
            assert self._key in self.settings.calibrations
            calibration = self.settings.calibrations[self._key].to_calibration()
            self._controller = FocusLockController(
                calibration,
                self.settings.to_limits(),
                start_position_um=self._start_um,
                travel_limits_um=travel,
            )
            logger.info(
                f"Image focus lock ON for {self._key!r}: box {self._box.rect} on "
                f"slice {self._box.slice_index}, {calibration.rows_per_um:+.3f} "
                f"rows/um (r2 {calibration.r2:.3f}), start piezo "
                f"{self._start_um:.3f} um, end of run: {self.settings.end_of_run}."
            )
        else:
            lo = self._start_um + min(self._cal_steps)
            hi = self._start_um + max(self._cal_steps)
            if travel is not None and not (travel[0] <= lo and hi <= travel[1]):
                raise ValueError(
                    f"calibration range {lo:.2f}..{hi:.2f} um is outside the "
                    f"piezo's travel {travel[0]:.2f}..{travel[1]:.2f} um"
                )
            logger.info(
                f"Image focus CALIBRATION for {self._key!r}: {len(self._cal_steps)} "
                f"steps x {self._cal_reps} from piezo {self._start_um:.3f} um."
            )
            self._move_piezo(self._start_um + self._cal_steps[0])

    def _release_crisp(self) -> bool:
        """Unlock CRISP if it is holding the piezo. ``False`` if it would not."""
        if not self._crisp_locked():
            return True
        mmc, label = self._mmc, self._hw.crisp_label
        unlock = None
        try:
            from CRISPy.controller import ASICrispController

            unlock = getattr(ASICrispController(label, mmc), "set_unlock", None)
        except Exception:
            logger.debug("CRISPy controller unavailable.", exc_info=True)
        if unlock is not None:
            unlock()
        else:
            mmc.setProperty(label, "CRISP State", "Ready")
        for _ in range(_CRISP_POLLS):
            if not self._crisp_locked():
                logger.info(
                    "CRISP was locked; unlocked it (Ready) because the image "
                    "focus lock now owns the piezo."
                )
                return True
            time.sleep(_CRISP_POLL_S)
        self._stop(
            "CRISP is locked and did not unlock; it owns the piezo, so the "
            "image focus lock is not running"
        )
        return False

    def _read_travel_limits(self) -> tuple[float, float] | None:
        """The piezo's travel limits if the device exposes them, else ``None``."""
        mmc, label = self._mmc, self._hw.piezo_a_label
        try:
            for prop in ("Position", "Position(um)"):
                if mmc.hasProperty(label, prop) and mmc.hasPropertyLimits(label, prop):
                    lo = float(mmc.getPropertyLowerLimit(label, prop))
                    hi = float(mmc.getPropertyUpperLimit(label, prop))
                    if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
                        return lo, hi
        except Exception:
            logger.debug("Could not read piezo travel limits.", exc_info=True)
        return None

    # ------------------------------------------------------------------
    # on_frame
    # ------------------------------------------------------------------

    def on_frame(
        self, camera: str, preset: str | None, slice_idx: int, img: np.ndarray
    ) -> None:
        """Keep the box's row profile if this frame is the one it watches.

        Called from the ``exec_event`` loop before the frame is yielded, so it
        must be cheap (a few comparisons for every frame but one per volume) and
        must never raise. The profile is computed here, from a copy: the frame
        is owned by the pipeline once it is yielded.
        """
        if not self.active:
            return
        box = self._box
        if box is None or camera != box.camera or preset != box.preset:
            return
        # A calibration volume holds exactly one slice: the box's own.
        if slice_idx != (0 if self.mode == "calibrate" else box.slice_index):
            return
        try:
            if not self._geometry_checked:
                self._geometry_checked = True
                roi = self._roi
                if roi is not None and tuple(img.shape[-2:]) != (roi[3], roi[2]):
                    raise ValueError(
                        f"frame is {img.shape[-1]}x{img.shape[-2]} but camera "
                        f"{camera} reports ROI {roi}; the box position would be "
                        "wrong"
                    )
            self._profile = row_profile(box.crop(img, self._origin))
        except Exception as err:
            self._stop(f"could not read the focus box: {err}", logging.ERROR, True)

    # ------------------------------------------------------------------
    # after_volume
    # ------------------------------------------------------------------

    def after_volume(self, index: dict[str, int], preset: str | None = None) -> None:
        """Measure the volume that just finished and move the piezo for the next.

        Call once per volume, only after it ran to completion (not after a
        cancel or an error) and after the cameras were stopped and drained.
        *preset* is the volume's laser preset: a volume of another channel is
        none of this lock's business and is skipped without counting as a miss.
        Never raises.
        """
        self.last_row = None  # so the engine never logs a previous volume's row
        if not self.active:
            return
        try:
            if self.mode == "calibrate":
                self._calibration_step(index)
            else:
                self._image_step(index, preset)
        except Exception as err:
            self._stop(f"error after volume {dict(index)}: {err}", logging.ERROR, True)
        finally:
            self._profile = None

    # -- image mode ----------------------------------------------------

    def _image_step(self, index: dict[str, int], preset: str | None) -> None:
        box, controller = self._box, self._controller
        assert box is not None and controller is not None
        if preset is not None and preset != box.preset:
            return
        piezo = self._piezo()
        row = FocusVolumeRow(index.get("t"), index.get("c"), piezo)
        profile = self._profile

        if profile is None:
            # Hold; the controller counts rejected measurements, but a missing
            # frame never reaches it, so count those here against the same cap.
            self._missing += 1
            limit = self.settings.max_strikes
            row.reason = f"no frame for box ({self._missing}/{limit})"
            self._record(index, row, piezo, None, row.reason)
            if self._missing >= limit:
                self._stop(f"{self._missing} volumes in a row had no frame for the box")
            return
        self._missing = 0

        if not controller.has_reference:
            controller.set_reference(profile)
            row.reason = "reference captured"
            self._record(index, row, piezo, None, row.reason)
            return

        assert piezo is not None
        decision = controller.update(profile, piezo)
        row.error_rows = decision.error_rows
        row.error_um = decision.error_um
        row.confidence = decision.confidence
        row.move_um = 0.0
        row.reason = decision.reason or ""

        if decision.fell_back:
            self._record(index, row, piezo, decision, row.reason)
            self._stop(f"fell back: {decision.reason} (piezo left at {piezo:.3f} um)")
            return
        if decision.move_um != 0.0:
            if self._move_piezo(decision.target_um):
                self.intentional_um += decision.move_um
                row.move_um = decision.move_um
                piezo = decision.target_um
            else:
                row.reason = "refused: CRISP locked"
                self._record(index, row, piezo, decision, row.reason)
                return
        self._record(index, row, piezo, decision, decision.reason or "")

    def _record(
        self,
        index: dict[str, int],
        row: FocusVolumeRow,
        piezo_after: float | None,
        decision: LockDecision | None,
        message: str,
    ) -> None:
        self.rows.append(row)
        self.last_row = row
        self._emit(
            focus_lock_signals.report,
            FocusLockReport(dict(index), self.mode, piezo_after, decision, message),
        )

    # -- calibrate mode ------------------------------------------------

    def _calibration_step(self, index: dict[str, int]) -> None:
        steps, reps = self._cal_steps, self._cal_reps
        total = len(steps) * reps
        k = self._volume_no
        self._volume_no += 1
        if k >= total or self._start_um is None:
            return
        offset = steps[k // reps]
        self._cal_data.append((offset, self._profile))
        message = f"calibration volume {k + 1}/{total} at {offset:+.2f} um"
        if self._profile is None:
            message += " (no frame for box)"
        row = FocusVolumeRow(
            index.get("t"), index.get("c"), self._start_um + offset, reason=message
        )
        if k + 1 < total:
            nxt = steps[(k + 1) // reps]
            if nxt != offset and not self._move_piezo(self._start_um + nxt):
                return
            self._record(index, row, self._start_um + nxt, None, message)
            return
        self._record(index, row, self._start_um + offset, None, message)
        self._finish_calibration()

    def _finish_calibration(self) -> None:
        """Fit the sweep and save the calibration (last calibration volume)."""
        assert self._box is not None
        steps = self._cal_steps
        ref_offset = min(steps, key=abs)
        refs = [p for off, p in self._cal_data if off == ref_offset and p is not None]
        if not refs:
            raise ValueError(f"no frame at the {ref_offset:+.2f} um reference step")
        reference = np.mean(refs, axis=0)
        offsets: list[float] = []
        shifts: list[float] = []
        weak = 0
        for off, profile in self._cal_data:
            if profile is None:
                weak += 1
                continue
            m = measure_shift(reference, profile)
            if m.confidence < self.settings.min_confidence:
                weak += 1  # the content left the box / washed out at this step
                continue
            # Relative to the reference step, as FocusLockController expects.
            offsets.append(off - ref_offset)
            shifts.append(m.shift_rows)
        fit = fit_calibration(offsets, shifts)
        self.active = False  # nothing more to do; teardown still restores
        logger.info(
            f"Image focus calibration {self._key!r}: {fit.rows_per_um:+.3f} rows/um, "
            f"r2 {fit.r2:.3f}, range {fit.range_um[0]:+.2f}..{fit.range_um[1]:+.2f} "
            f"um, {fit.n_points}/{len(self._cal_data)} points ({weak} dropped as "
            "weak or missing)."
        )
        if fit.r2 < self.settings.min_r2:
            self._emit_stopped(
                f"calibration fit r2 {fit.r2:.3f} is below {self.settings.min_r2:.3f}"
                "; not saved"
            )
            logger.warning("Image focus calibration not saved: poor fit.")
            return
        self._save_calibration(fit)
        self._emit(focus_lock_signals.calibrated, self._key, fit)

    def _save_calibration(self, fit: FocusCalibration) -> None:
        live = SettingsV1.instance()
        live.image_focus.calibrations[self._key] = CalibrationRecord.from_calibration(
            fit
        )
        live.flush()

    # -- piezo ---------------------------------------------------------

    def _piezo(self) -> float | None:
        try:
            return float(self._mmc.getPosition(self._hw.piezo_a_label))
        except Exception:
            logger.debug("Could not read the piezo.", exc_info=True)
            return None

    def _move_piezo(self, target_um: float) -> bool:
        """Move the piezo to *target_um* and wait; ``False`` if CRISP forbids it.

        CRISP is re-checked here, right before the command, not trusted from
        ``start``: someone may have locked it since. A refusal ends the lock.
        """
        if self._crisp_locked():
            self._crisp_refused = True
            self._stop(
                "CRISP locked the piezo again; not moving it (the image focus "
                "lock is off for the rest of the run)"
            )
            return False
        label = self._hw.piezo_a_label
        self._mmc.setPosition(label, float(target_um))
        self._mmc.waitForDevice(label)
        return True

    # ------------------------------------------------------------------
    # teardown
    # ------------------------------------------------------------------

    def teardown(self) -> None:
        """Restore the piezo, write the CSV, emit ``stopped``. Never raises.

        The piezo goes back to its start position for a calibration always, and
        for an image run when ``end_of_run == "restore"`` (``"stay"`` leaves it
        where the lock put it). Nothing is moved if CRISP has taken the piezo.
        """
        try:
            self._restore_piezo()
        except Exception:
            logger.error("Could not restore the piezo after the run.", exc_info=True)
        try:
            self._write_csv()
        except Exception:
            logger.warning("Could not write the focus log.", exc_info=True)
        if self.mode != "off":
            self._emit_stopped("run finished")
        self.active = False

    def _restore_piezo(self) -> None:
        start = self._start_um
        if start is None or self._crisp_refused:
            return
        wanted = self.mode == "calibrate" or (
            self.mode == "image" and self.settings.end_of_run == "restore"
        )
        if not wanted:
            return
        current = self._piezo()
        if current is None or abs(current - start) < 1e-3:
            self.intentional_um = 0.0
            return
        if self._move_piezo(start):
            logger.info(
                f"Image focus lock: piezo restored {current:.3f} -> {start:.3f} um."
            )
            self.intentional_um = 0.0

    def _write_csv(self) -> None:
        """``<save_name>_focus_log.csv`` next to the run's data, if it has a home."""
        if self.mode != "image" or not self.rows:
            return
        if not self._save_dir or not self._save_name:
            return
        directory = Path(str(self._save_dir))
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{_strip_suffix(str(self._save_name))}_focus_log.csv"
        with path.open("w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(CSV_COLUMNS)
            for r in self.rows:
                writer.writerow(
                    [
                        "" if getattr(r, col) is None else getattr(r, col)
                        for col in CSV_COLUMNS
                    ]
                )
        logger.info(f"Image focus log written to {path}")

    # ------------------------------------------------------------------
    # stopping / signals
    # ------------------------------------------------------------------

    def _stop(self, reason: str, level: int = logging.WARNING, exc_info: bool = False):
        """Deactivate the lock (the acquisition continues) and say why."""
        self.active = False
        logger.log(
            level,
            f"Image focus lock ({self.mode}) stopped: {reason}",
            exc_info=exc_info,
        )
        self._emit_stopped(reason)

    def _emit_stopped(self, reason: str) -> None:
        if self._stop_emitted:
            return
        self._stop_emitted = True
        self._emit(focus_lock_signals.stopped, reason)

    @staticmethod
    def _emit(signal: Any, *args: Any) -> None:
        """Emit, never letting a listener's exception reach the engine."""
        try:
            signal.emit(*args)
        except Exception:
            logger.warning("A focus lock listener raised.", exc_info=True)


# ----------------------------------------------------------------------
# Calibration sequence
# ----------------------------------------------------------------------


def _single_slice_plan(z_plan: Any | None, slice_index: int) -> Any | None:
    """A z-plan with only slice *slice_index* of *z_plan*, at the same plane.

    The SPIM engine places slice ``i`` at ``list(z_plan)[i]`` (relative to the
    Live plane for relative plans, absolute focus-device coordinates otherwise,
    see ``z_scan.compute_galvo_scan``), so a one-position plan of the same kind
    makes it sweep exactly that plane: one slice, zero galvo amplitude, centre
    on the slice.
    """
    if z_plan is None:
        if slice_index != 0:
            raise ValueError(
                f"The focus box is on slice {slice_index} but the base sequence "
                "has no z plan (a single slice, index 0)."
            )
        return None
    try:
        positions = [float(p) for p in z_plan]
    except Exception as err:
        raise ValueError(
            f"The base z plan cannot be resolved to slices: {err}"
        ) from err
    if not positions:
        raise ValueError("The base z plan has no slices.")
    if not 0 <= slice_index < len(positions):
        raise ValueError(
            f"The focus box is on slice {slice_index} but the base z plan has "
            f"{len(positions)} slices."
        )
    z = positions[slice_index]
    if z_plan.is_relative:
        return useq.ZRelativePositions(relative=[z])
    return useq.ZAbsolutePositions(absolute=[z])


def build_calibration_sequence(
    base: useq.MDASequence,
    box: FocusBox,
    window: str = "",
    steps_um: tuple[float, ...] | list[float] = DEFAULT_CALIBRATION_STEPS_UM,
    reps: int = 2,
) -> useq.MDASequence:
    """Build the sequence that calibrates the image focus lock for *box*.

    The result is a minimal run of the SPIM engine that images only the box's
    slice, once per ``(step, repeat)``, with the piezo stepped through
    *steps_um* between volumes (volume ``k`` is acquired at step ``k // reps``;
    see :class:`ImageFocusRun`):

    - channel: *base*'s channel with the box's laser preset (so group and
      exposure match what will run);
    - z plan: one position, the base plan's slice ``box.slice_index`` (see
      :func:`_single_slice_plan`);
    - time plan: ``len(steps_um) * reps`` loops at interval 0;
    - stage position: *base*'s first one, if any;
    - metadata: :data:`CALIBRATION_METADATA_KEY` payload, and **no save name /
      save directory**, which is what makes the Argus streamer skip the run
      (``_build_session_header`` returns ``None`` without a ``save_name``) and
      keeps it out of the user's data folders.

    Raises
    ------
    ValueError
        If *base* has no channel for the box's preset, or its z plan cannot be
        resolved to the box's slice.
    """
    steps = tuple(float(s) for s in steps_um)
    if len(steps) < 3:
        raise ValueError("A calibration needs at least 3 piezo steps.")
    if reps < 1:
        raise ValueError("reps must be at least 1.")
    channel = next((c for c in base.channels if c.config == box.preset), None)
    if channel is None:
        raise ValueError(
            f"The base sequence has no channel with preset {box.preset!r} "
            f"(it has {[c.config for c in base.channels]})."
        )
    z_plan = _single_slice_plan(base.z_plan, box.slice_index)
    return useq.MDASequence(
        channels=[channel.model_copy()],
        z_plan=z_plan,
        time_plan=useq.TIntervalLoops(interval=0, loops=len(steps) * reps),
        stage_positions=list(base.stage_positions[:1]),
        metadata={
            CALIBRATION_METADATA_KEY: {
                "steps_um": list(steps),
                "reps": int(reps),
                "box": {
                    "camera": box.camera,
                    "preset": box.preset,
                    "slice_index": box.slice_index,
                    "rect": list(box.rect),
                },
                "window": window,
            }
        },
    )
