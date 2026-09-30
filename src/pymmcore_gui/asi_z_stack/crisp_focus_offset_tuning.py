# src/pymmcore_gui/asi_z_stack/crisp_focus_offset_tuning.py
"""Bench probe for driving CRISP focus with the lock-offset setpoint.

Companion to :mod:`~pymmcore_gui.asi_z_stack.crisp_piezo_tuning`: an
interactive, run-it-by-hand tool (pymmcore-gui's embedded console, or
``python -i``) against an already-configured ``CMMCorePlus`` with CRISP
loaded and **locked** on the piezo. Not imported by the acquisition path.

Purpose: before building the per-channel focus-offset feature (correcting
axial chromatic aberration by holding a slightly different focal plane per
excitation wavelength), establish empirically whether the writable CRISP
property ``"Set Lock Offset (Advanced Users Only)"`` is a usable lever:

* Does writing it, while CRISP stays locked, actually shift focus (the servo
  drives the piezo to a new plane)?
* What is the real counts-per-micron scale and sign, versus the value derived
  from the error-signal calibration
  (:meth:`~CRISPy.controller.ASICrispController.get_sensitivity_counts_per_um`,
  ``|Calibration Gain| / Calibration Range(um)``)?
* How long does the piezo take to stop moving after a small (1-3 um) step --
  i.e. how much dead time a per-volume channel switch would add?
* Does the ASITiger adapter round-trip the written value, or does it change
  the hardware while its own cached property stays stale (the failure mode
  that defeated the raw-serial ``J`` joystick-binding attempts -- see the
  ``asi-crisp-piezo-joystick-drift-fix`` project note)?

This module writes exactly one CRISP property (``Set Lock Offset...``) and
always restores it. It never re-runs calibration, never touches
``"CRISP State"``, and steps one commanded value at a time -- the judgement
call (is the scale linear? is the settle fast enough? GO or NO-GO?) is left
to a human reading :func:`summarize`'s output, per this project's established
"verify CRISP against real hardware one confirmed step at a time" rule.
"""

from __future__ import annotations

import csv
import dataclasses
import logging
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

from CRISPy.controller import ASICrispController

from ._logging import configure_asi_logging
from .asi_controller import mmc

if TYPE_CHECKING:
    from collections.abc import Sequence

    from pymmcore_plus import CMMCorePlus

logger = logging.getLogger(__name__)

# See the module docstring -- make this module's own log output visible by
# default when run interactively, matching crisp_piezo_tuning.py.
configure_asi_logging("INFO")

# Writable integer setpoint: the servo's target offset from the calibrated
# in-focus error value. Present and non-readonly on CRISPAFocus:P:34 in the
# live crisp_dump.txt capture.
SET_LOCK_OFFSET_PROP = "Set Lock Offset (Advanced Users Only)"
# Readonly: the offset the servo is currently tracking (near 0 when in focus).
LOCK_OFFSET_PROP = "Lock Offset"
ERROR_PROP = "Dither Error"  # same signal crisp_piezo_tuning.py samples

# CRISP states in which the servo is actively holding focus, so a setpoint
# write is meaningful. Anything else (Ready/Idle/calibration) -> refuse.
LOCKED_STATES = ("Lock", "In Focus")

# Default probe: a symmetric sweep spanning a bit more than any plausible
# chromatic offset (405<->638 is typically well under 1 um on an apo
# objective; 3 um is deliberate headroom to see linearity/hysteresis),
# bracketed by 0 so drift between steps is visible.
DEFAULT_STEPS_UM = (0.0, 0.25, 0.5, 1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.0)

# CRISP output CSVs already live here (see .gitignore "/CRISPTest").
DEFAULT_OUT_DIR = "CRISPTest"


@dataclass
class LockOffsetSample:
    """One commanded lock-offset step and the piezo/servo response to it.

    ``empirical_*`` compare the *incremental* commanded counts against the
    *incremental* piezo motion since the previous step, so they are only
    meaningful when ``delta_piezo_um`` is well above readback noise.
    """

    step_um: float
    """Requested focus shift relative to the pre-probe home offset."""
    commanded_counts: int
    """Absolute value written to ``SET_LOCK_OFFSET_PROP`` for this step."""
    prev_commanded_counts: int
    """The previous step's ``commanded_counts`` (for the incremental scale)."""
    setprop_readback: float | None
    """``SET_LOCK_OFFSET_PROP`` read back immediately after the write."""
    roundtrip_ok: bool
    """Whether ``setprop_readback`` equals ``commanded_counts``."""
    pre_piezo_um: float
    post_piezo_um: float
    delta_piezo_um: float
    """``post_piezo_um - pre_piezo_um`` (this step's focus motion)."""
    pre_lock_offset_counts: float | None
    post_lock_offset_counts: float | None
    """Readonly ``LOCK_OFFSET_PROP`` before/after (servo tracking error)."""
    settle_s: float
    """Time from the write until the piezo position stopped changing."""
    settle_capped: bool
    """True if the piezo was still moving when ``settle_timeout_s`` elapsed."""
    pre_state: str
    post_state: str
    pre_error_counts: float | None
    post_error_counts: float | None
    empirical_counts_per_um: float | None
    empirical_um_per_count: float | None


def _refresh(core: CMMCorePlus, label: str) -> None:
    """Force the ASITiger adapter to re-read *label*'s sensor properties."""
    if core.hasProperty(label, "RefreshPropertyValues"):
        try:
            core.setProperty(label, "RefreshPropertyValues", "Yes")
        except Exception as e:  # pragma: no cover - defensive, logged not raised
            logger.debug(f"RefreshPropertyValues failed for {label}: {e}")


def _read_float(core: CMMCorePlus, label: str, prop: str) -> float | None:
    """Read *label*/*prop* as a float, returning ``None`` instead of raising."""
    try:
        return float(core.getProperty(label, prop))
    except Exception:
        return None


def _poll_until_stable(
    core: CMMCorePlus,
    label: str,
    tol_um: float,
    poll_s: float,
    timeout_s: float,
) -> tuple[float, float, bool]:
    """Poll ``getPosition(label)`` until it stops changing, or *timeout_s*.

    Returns ``(final_position_um, elapsed_s, capped)`` where ``capped`` is
    True if the position was still moving by more than *tol_um* between polls
    when the timeout was hit.
    """
    t0 = time.monotonic()
    prev = core.getPosition(label)
    while True:
        time.sleep(poll_s)
        now = core.getPosition(label)
        elapsed = time.monotonic() - t0
        if abs(now - prev) <= tol_um:
            return now, elapsed, False
        if elapsed >= timeout_s:
            return now, elapsed, True
        prev = now


def probe_lock_offset_response(
    crisp_label: str = "CRISPAFocus:P:34",
    piezo_label: str = "PiezoStage:P:34",
    steps_um: Sequence[float] = DEFAULT_STEPS_UM,
    counts_per_um: float | None = None,
    stable_tol_um: float = 0.03,
    poll_s: float = 0.02,
    settle_timeout_s: float = 1.0,
    dwell_s: float = 0.5,
    mmcore: CMMCorePlus | None = None,
) -> list[LockOffsetSample]:
    """Step the CRISP lock offset and measure how the piezo/servo responds.

    CRISP must already be **locked** on *piezo_label*. For each entry in
    *steps_um* this writes ``home + step_um * scale`` to
    ``SET_LOCK_OFFSET_PROP`` (``home`` = the property's value on entry,
    ``scale`` = *counts_per_um* or the device sensitivity), waits for the
    piezo to stop moving, dwells *dwell_s*, and records a
    :class:`LockOffsetSample`. The property is always restored to ``home``
    before returning.

    Parameters
    ----------
    crisp_label : str
        The CRISP AutoFocus device servoing the piezo.
    piezo_label : str
        The piezo stage CRISP drives (its ``getPosition`` is the focus proxy).
    steps_um : Sequence[float]
        Focus shifts to command, in microns, relative to the home offset.
    counts_per_um : float | None
        Override the counts-per-micron scale; ``None`` uses
        ``get_sensitivity_counts_per_um()``.
    stable_tol_um : float
        Successive-poll position change at or below which the piezo counts as
        stopped.
    poll_s : float
        Delay between position polls while waiting for the piezo to settle.
    settle_timeout_s : float
        Hard cap on the settle wait per step.
    dwell_s : float
        Extra hold after settling before the post-step snapshot.
    mmcore : CMMCorePlus | None
        Core to use; defaults to the shared ``asi_z_stack`` instance.

    Returns
    -------
    list[LockOffsetSample]
        One entry per commanded step, in order.
    """
    core = mmcore or mmc
    controller = ASICrispController(crisp_label, core)

    state = controller.get_state()
    if state not in LOCKED_STATES:
        raise RuntimeError(
            f"{crisp_label} is in state {state!r}, not one of {LOCKED_STATES}. "
            "Lock CRISP on the piezo (CRISPy panel) before probing the lock offset."
        )

    scale = counts_per_um or controller.get_sensitivity_counts_per_um()
    if not scale:
        raise RuntimeError(
            f"No counts-per-um scale for {crisp_label} (Calibration Gain / "
            "Range unavailable) and none was passed explicitly."
        )

    home = _read_float(core, crisp_label, SET_LOCK_OFFSET_PROP) or 0.0
    home_counts = round(home)
    start_pos = core.getPosition(piezo_label)
    logger.info(
        f"Probing {crisp_label} lock offset: state={state}, scale={scale:.1f} "
        f"counts/um, home={home_counts} counts, piezo start={start_pos:.4f} um."
    )

    samples: list[LockOffsetSample] = []
    prev_commanded = home_counts
    try:
        for step_um in steps_um:
            commanded = round(home_counts + step_um * scale)

            _refresh(core, crisp_label)
            pre_pos = core.getPosition(piezo_label)
            pre_lock = _read_float(core, crisp_label, LOCK_OFFSET_PROP)
            pre_state = controller.get_state()
            pre_err = _read_float(core, crisp_label, ERROR_PROP)

            core.setProperty(crisp_label, SET_LOCK_OFFSET_PROP, str(commanded))
            readback = _read_float(core, crisp_label, SET_LOCK_OFFSET_PROP)
            roundtrip_ok = readback is not None and round(readback) == commanded

            _settled_pos, settle_s, capped = _poll_until_stable(
                core, piezo_label, stable_tol_um, poll_s, settle_timeout_s
            )
            time.sleep(dwell_s)

            _refresh(core, crisp_label)
            post_pos = core.getPosition(piezo_label)
            post_lock = _read_float(core, crisp_label, LOCK_OFFSET_PROP)
            post_state = controller.get_state()
            post_err = _read_float(core, crisp_label, ERROR_PROP)

            d_piezo = post_pos - pre_pos
            d_counts = commanded - prev_commanded
            if abs(d_piezo) > max(stable_tol_um, 1e-6) and d_counts != 0:
                emp_cpu: float | None = d_counts / d_piezo
                emp_upc: float | None = d_piezo / d_counts
            else:
                emp_cpu = emp_upc = None

            sample = LockOffsetSample(
                step_um=step_um,
                commanded_counts=commanded,
                prev_commanded_counts=prev_commanded,
                setprop_readback=readback,
                roundtrip_ok=roundtrip_ok,
                pre_piezo_um=pre_pos,
                post_piezo_um=post_pos,
                delta_piezo_um=d_piezo,
                pre_lock_offset_counts=pre_lock,
                post_lock_offset_counts=post_lock,
                settle_s=settle_s,
                settle_capped=capped,
                pre_state=pre_state,
                post_state=post_state,
                pre_error_counts=pre_err,
                post_error_counts=post_err,
                empirical_counts_per_um=emp_cpu,
                empirical_um_per_count=emp_upc,
            )
            samples.append(sample)
            logger.info(
                f"  step {step_um:+.2f} um -> {commanded} counts "
                f"(readback {readback}, roundtrip {'ok' if roundtrip_ok else 'FAIL'}): "
                f"piezo {pre_pos:.4f} -> {post_pos:.4f} um (d={d_piezo:+.4f}), "
                f"settle {settle_s * 1000:.0f} ms{' (CAPPED)' if capped else ''}, "
                f"state {pre_state}->{post_state}"
            )
            prev_commanded = commanded
    finally:
        try:
            core.setProperty(crisp_label, SET_LOCK_OFFSET_PROP, str(home_counts))
            rest_pos, rest_s, rest_capped = _poll_until_stable(
                core, piezo_label, stable_tol_um, poll_s, settle_timeout_s
            )
            logger.info(
                f"Restored {crisp_label} {SET_LOCK_OFFSET_PROP} to {home_counts}: "
                f"piezo now {rest_pos:.4f} um (started {start_pos:.4f}, "
                f"delta {rest_pos - start_pos:+.4f} um, settle {rest_s * 1000:.0f} ms"
                f"{' CAPPED' if rest_capped else ''})."
            )
        except Exception:  # pragma: no cover - best-effort restore
            logger.error(
                f"Failed to restore {crisp_label} {SET_LOCK_OFFSET_PROP} to "
                f"{home_counts} -- do it by hand in the CRISPy panel.",
                exc_info=True,
            )

    return samples


_CSV_FIELDS = [f.name for f in dataclasses.fields(LockOffsetSample)]


def write_csv(samples: list[LockOffsetSample], path: str | Path) -> Path:
    """Write *samples* to *path* as CSV (one row per commanded step)."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDS)
        writer.writeheader()
        for s in samples:
            writer.writerow(dataclasses.asdict(s))
    logger.info(f"Wrote {len(samples)} rows to {out}.")
    return out


def summarize(
    samples: list[LockOffsetSample], counts_per_um: float | None = None
) -> None:
    """Log a GO/NO-GO-oriented summary of a :func:`probe_lock_offset_response` run.

    Prints the per-step table and flags the things that decide whether the
    lock-offset setpoint is a usable focus lever: adapter round-trip,
    monotonic response, scale consistency vs *counts_per_um* (or the device
    sensitivity), sign, and settle time. The final call is still the
    operator's.
    """
    if not samples:
        logger.warning("No samples to summarize.")
        return

    ref_scale = counts_per_um
    if ref_scale is None:
        try:
            ref_scale = ASICrispController(
                "CRISPAFocus:P:34", mmc
            ).get_sensitivity_counts_per_um()
        except Exception:
            ref_scale = None

    all_roundtrip = all(s.roundtrip_ok for s in samples)
    any_capped = any(s.settle_capped for s in samples)
    moving = [s for s in samples if s.step_um != 0.0 and s.empirical_counts_per_um]
    emp = [s.empirical_counts_per_um for s in moving if s.empirical_counts_per_um]
    max_settle_ms = max((s.settle_s for s in samples), default=0.0) * 1000

    roundtrip_msg = (
        "OK for all steps"
        if all_roundtrip
        else "FAILED -- adapter cache not tracking the write (cf. the J-command saga)"
    )

    logger.info("--- lock-offset probe summary ---")
    for s in samples:
        cpu = (
            f"{s.empirical_counts_per_um:.1f}"
            if s.empirical_counts_per_um is not None
            else "  n/a"
        )
        logger.info(
            f"  {s.step_um:+.2f} um | cmd {s.commanded_counts:>6} | "
            f"dpiezo {s.delta_piezo_um:+.4f} um | emp {cpu} counts/um | "
            f"settle {s.settle_s * 1000:4.0f} ms{'!' if s.settle_capped else ' '} | "
            f"{s.pre_state}->{s.post_state} | rt {'ok' if s.roundtrip_ok else 'FAIL'}"
        )

    logger.info(f"round-trip: {roundtrip_msg}")
    if emp:
        lo, hi = min(emp), max(emp)
        mean = sum(emp) / len(emp)
        sign_msg = "consistent" if len({e > 0 for e in emp}) == 1 else "INCONSISTENT"
        spread_pct = 100 * (hi - lo) / (abs(mean) or 1)
        logger.info(
            f"empirical scale: {lo:.1f} .. {hi:.1f} counts/um across moving steps "
            f"(spread {spread_pct:.0f}% of mean); sign {sign_msg}"
        )
        if ref_scale:
            mag_pct = 100 * (abs(mean) - ref_scale) / ref_scale
            logger.info(
                f"vs device sensitivity {ref_scale:.1f} counts/um: empirical mean "
                f"{mean:.1f} ({mag_pct:+.0f}% magnitude, sign "
                f"{'+' if mean > 0 else '-'})"
            )
    else:
        logger.warning(
            "no step produced measurable piezo motion -- the setpoint write is "
            "NOT moving focus (NO-GO for this mechanism)."
        )
    logger.info(
        f"settle: worst {max_settle_ms:.0f} ms"
        f"{' -- SOME STEPS HIT THE TIMEOUT CAP' if any_capped else ''}"
    )
    logger.info(
        "GO if: round-trip OK, motion monotonic with command, scale spread small "
        "and within ~25% of the device value, sign consistent, worst settle for a "
        "1-3 um step is short. Otherwise investigate a fallback (see module docstring)."
    )


def run(
    crisp_label: str = "CRISPAFocus:P:34",
    piezo_label: str = "PiezoStage:P:34",
    steps_um: Sequence[float] = DEFAULT_STEPS_UM,
    counts_per_um: float | None = None,
    out_dir: str | Path = DEFAULT_OUT_DIR,
    mmcore: CMMCorePlus | None = None,
) -> tuple[list[LockOffsetSample], Path]:
    """Probe, write a timestamped CSV, and print the summary in one call."""
    samples = probe_lock_offset_response(
        crisp_label=crisp_label,
        piezo_label=piezo_label,
        steps_um=steps_um,
        counts_per_um=counts_per_um,
        mmcore=mmcore,
    )
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = write_csv(samples, Path(out_dir) / f"crisp_lock_offset_{stamp}.csv")
    summarize(samples, counts_per_um=counts_per_um)
    return samples, path


if __name__ == "__main__":  # pragma: no cover - manual bench entry point
    run()
