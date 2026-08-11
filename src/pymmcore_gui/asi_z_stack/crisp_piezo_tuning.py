# src/pymmcore_gui/asi_z_stack/crisp_piezo_tuning.py
"""Manual bench tools for stabilizing ASI CRISP focus lock on a piezo axis.

Companion to :mod:`~pymmcore_gui.asi_z_stack.diagnostics`: one-off tools meant
to be run interactively (e.g. pymmcore-gui's embedded console, or ``python
-i``) against an already-configured ``CMMCorePlus`` instance with CRISP
loaded, not imported by the acquisition path.

Background: CRISP on a piezo axis needs different loop dynamics than on a
motorized Z axis (ASI's own docs: "Increase Update_Rate to improve stability
when using piezo Z-axis drive systems... You will want to increase the loop
gain with LR T=# as you increase the update rate" -- see the ``UL`` command
reference at asiimaging.com/docs/commands/unlock). These helpers script the
mechanical, measurable parts of that tuning pass -- reading/writing
properties and sampling the live error signal -- while leaving the actual
judgment calls (is it stable yet? should the alignment screw move?) to a
human watching the output, one step at a time. Do not turn this into a
closed-loop auto-tuner: per this project's established lesson (see the
``asi-hardware-debug-approach`` memory), CRISP behavior should be verified
against real hardware one confirmed step at a time, not driven by an
unattended loop.

Also includes :func:`log_crisp_drift`, a *passive* (never writes a CRISP
property) long-duration logger for a distinct failure mode: CRISP reporting
a clean, healthy lock (good SNR/Sum, near-zero Dither Error, "In Focus")
while the true image focus silently drifts away underneath it -- ASI's own
docs confirm CRISP's health metrics are entirely self-referential (they
describe how well the servo tracks its own setpoint, not whether that
setpoint still matches true focus), so this can't be caught by watching the
CRISPy panel alone. Because this never mutates hardware state, it's safe to
run unattended for many minutes, unlike the tuning helpers above -- but it
still blocks whatever thread calls it, so for anything longer than a few
seconds use :func:`log_crisp_drift_async` instead of calling it directly
from pymmcore-gui's embedded console (see that function's docstring for why).
"""

from __future__ import annotations

import csv
import logging
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import tifffile
from CRISPy.controller import ASICrispController
from superqt.utils import create_worker

from ._logging import configure_asi_logging
from .asi_controller import _HW, mmc

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_plus import CMMCorePlus
    from superqt.utils import FunctionWorker

logger = logging.getLogger(__name__)

# See the module docstring above -- this makes the module's own log output
# visible by default when run interactively, matching diagnostics.py.
configure_asi_logging("INFO")

# Raw error-signal property CRISPy's own panel tracks for its "Fluctuation"
# readout (CRISPy/src/CRISPy/ui.py, _ERROR_PROP) -- kept identical here so a
# script-driven sample and the live panel reading are directly comparable.
_ERROR_PROP = "Dither Error"

STATE_PROP = "CRISP State"
UPDATE_RATE_PROP = "Number of Skips"  # ASI serial UL Y (Update Rate, ms)
GAIN_PROP = "GainMultiplier"  # ASI serial LR T (loop gain)
LOCK_RANGE_PROP = "Max Lock Range(mm)"  # ASI serial LR Z


def _try_get_property(mmcore: CMMCorePlus, label: str, prop: str) -> str:
    """Read a device property, returning ``""`` instead of raising if unavailable."""
    try:
        return str(mmcore.getProperty(label, prop))
    except Exception:
        return ""


@dataclass
class FluctuationStats:
    """Rolling stability of a CRISP axis's focus-error signal.

    Headless equivalent of ``CrispControlPanel._update_stability`` (CRISPy's
    live "Fluctuation" readout) -- same math (population std-dev and
    peak-to-peak of :data:`_ERROR_PROP`, converted to microns via the axis's
    calibration sensitivity), usable from a script without the GUI open.
    """

    n_samples: int
    std_counts: float
    ptp_counts: float
    sensitivity_counts_per_um: float | None
    std_um: float | None
    ptp_um: float | None


def dump_crisp_state(
    labels: list[str], mmcore: CMMCorePlus | None = None
) -> dict[str, dict[str, str]]:
    """Snapshot every device property of each CRISP axis in *labels*.

    Reusable, timestamp-free version of the one-off ``crisp_dump.txt``
    capture used to diagnose this rig's Z-vs-piezo CRISP disparity -- take
    one dump before changing anything and another after, then pass both to
    :func:`diff_crisp_state` to see exactly what changed.
    """
    core = mmcore or mmc
    dump: dict[str, dict[str, str]] = {}
    for label in labels:
        if label not in core.getLoadedDevices():
            logger.warning(f"'{label}' not loaded -- skipping.")
            continue
        props: dict[str, str] = {}
        for prop in core.getDevicePropertyNames(label):
            try:
                props[prop] = str(core.getProperty(label, prop))
            except Exception as e:  # pragma: no cover - defensive, logged not raised
                logger.debug(f"Could not read {label}.{prop}: {e}")
        dump[label] = props
    return dump


def diff_crisp_state(
    before: dict[str, dict[str, str]], after: dict[str, dict[str, str]]
) -> dict[str, dict[str, tuple[str, str]]]:
    """Return the properties that changed between two :func:`dump_crisp_state` calls.

    Parameters
    ----------
    before : dict[str, dict[str, str]]
        A dump taken before a tuning change.
    after : dict[str, dict[str, str]]
        A dump taken after the change.

    Returns
    -------
    dict[str, dict[str, tuple[str, str]]]
        ``{label: {prop: (before_value, after_value)}}``, restricted to
        properties present in both dumps whose values differ.
    """
    changed: dict[str, dict[str, tuple[str, str]]] = {}
    for label, before_props in before.items():
        after_props = after.get(label, {})
        label_changes = {
            prop: (value, after_props[prop])
            for prop, value in before_props.items()
            if prop in after_props and after_props[prop] != value
        }
        if label_changes:
            changed[label] = label_changes
    return changed


def sample_fluctuation(
    label: str,
    n_samples: int = 20,
    interval_s: float = 0.75,
    mmcore: CMMCorePlus | None = None,
) -> FluctuationStats:
    """Poll a CRISP axis's error signal *n_samples* times and summarize its spread.

    Forces a fresh hardware read each poll via ``RefreshPropertyValues``
    (the ASITiger adapter does not emit ``propertyChanged`` for sensor
    values -- see CRISPy/CLAUDE.md's "Event-driven UI" rule), matching
    CRISPy's own polling practice. Run this before and after a candidate
    ``Number of Skips``/``GainMultiplier`` change (see
    :func:`apply_and_sample`) to compare stability numerically instead of by
    eye alone.
    """
    core = mmcore or mmc
    controller = ASICrispController(label, core)
    sensitivity = controller.get_sensitivity_counts_per_um()

    samples: list[float] = []
    can_refresh = core.hasProperty(label, "RefreshPropertyValues")
    for _ in range(n_samples):
        if can_refresh:
            try:
                core.setProperty(label, "RefreshPropertyValues", "Yes")
            except Exception as e:  # pragma: no cover - defensive, logged not raised
                logger.debug(f"RefreshPropertyValues failed for {label}: {e}")
        try:
            samples.append(float(core.getProperty(label, _ERROR_PROP)))
        except Exception as e:  # pragma: no cover - defensive, logged not raised
            logger.debug(f"Could not read {label}.{_ERROR_PROP}: {e}")
        time.sleep(interval_s)

    if len(samples) < 2:
        return FluctuationStats(
            n_samples=len(samples),
            std_counts=0.0,
            ptp_counts=0.0,
            sensitivity_counts_per_um=sensitivity,
            std_um=None,
            ptp_um=None,
        )

    std = statistics.pstdev(samples)
    ptp = max(samples) - min(samples)
    std_um = std / sensitivity if sensitivity else None
    ptp_um = ptp / sensitivity if sensitivity else None
    return FluctuationStats(
        n_samples=len(samples),
        std_counts=std,
        ptp_counts=ptp,
        sensitivity_counts_per_um=sensitivity,
        std_um=std_um,
        ptp_um=ptp_um,
    )


def apply_and_sample(
    label: str,
    update_rate_ms: int,
    gain_multiplier: int,
    n_samples: int = 20,
    interval_s: float = 0.75,
    settle_s: float = 1.0,
    prompt: bool = True,
    mmcore: CMMCorePlus | None = None,
) -> FluctuationStats:
    """Apply one ``(Number of Skips, GainMultiplier)`` candidate, then sample stability.

    Sets both properties (plain ``setProperty`` calls -- both are already
    normal MM device properties, no serial passthrough needed), waits
    *settle_s* for the servo to settle onto the new settings, samples
    :func:`sample_fluctuation`, prints the result, and -- if *prompt* is
    True -- blocks on ``input()`` before returning, mirroring
    ``diagnostics.sweep_backplane_ttl_addrs``'s step-and-confirm pattern so a
    human decides whether to keep going, not a loop.
    """
    core = mmcore or mmc
    core.setProperty(label, UPDATE_RATE_PROP, str(update_rate_ms))
    core.setProperty(label, GAIN_PROP, str(gain_multiplier))
    time.sleep(settle_s)

    stats = sample_fluctuation(
        label, n_samples=n_samples, interval_s=interval_s, mmcore=core
    )
    if stats.std_um is None:
        logger.info(
            f"[{label}] Update Rate={update_rate_ms}ms Gain={gain_multiplier}: "
            f"std=±{stats.std_counts:.0f} cts, pk-pk={stats.ptp_counts:.0f} cts "
            "(uncalibrated -- no µm conversion)"
        )
    else:
        logger.info(
            f"[{label}] Update Rate={update_rate_ms}ms Gain={gain_multiplier}: "
            f"std=±{stats.std_um:.3f} µm, pk-pk={stats.ptp_um:.3f} µm"
        )
    if prompt:
        input("Press Enter to try the next setting (Ctrl+C to stop here)...")
    return stats


def capture_focus_curve(
    label: str,
    duration_s: float = 10.0,
    poll_interval_s: float = 0.05,
    mmcore: CMMCorePlus | None = None,
) -> list[str]:
    """Best-effort capture of ASI's Generate Focus Curve output (serial ``LK F=97``).

    Sets ``CRISP State`` to ``"Curve"`` (a normal allowed value on this
    adapter -- confirmed present in the live ``crisp_dump.txt`` capture),
    which the ASITiger adapter translates to the equivalent serial command
    itself, the same way ``ASICrispController.set_dither()``/``set_log_cal()``
    already do for their own states (``controller.py``). Afterwards this
    only *reads* the Tiger hub's ``SerialResponse`` property -- it
    deliberately never sends another ``SerialCommand`` while the curve is
    running, since ASI's serial protocol is stateful and competing traffic
    on the same line while the controller is mid-command is exactly the
    class of bug already hit once in this codebase (see the
    ``asi-spim-triggering-status`` memory). Collects each new (i.e.
    changed-from-the-previous-poll) response line for *duration_s*.

    **Unverified**: it is not confirmed whether the ASITiger adapter's
    ``SerialResponse`` property actually streams the full multi-line curve
    table (time, position, error) as unsolicited data arrives, or only ever
    reflects the last explicit command/response -- this needs bench
    verification before being relied on. If the returned list looks sparse
    or empty, read the curve from the physical LCD / ASI Console instead
    (Tools > Serial Terminal), which is guaranteed to show it per ASI's own
    manual.
    """
    core = mmcore or mmc
    axis = _try_get_property(core, label, "AxisLetter")
    hex_addr = _try_get_property(core, label, "TigerHexAddress")
    hub = _HW.tiger_comm_hub_label

    lines: list[str] = []
    try:
        core.setProperty(label, STATE_PROP, "Curve")
    except Exception as e:
        logger.warning(f"Could not set {label}.{STATE_PROP}=Curve: {e}")
        return lines

    logger.info(
        f"Generating focus curve on {label} (axis {axis}, addr {hex_addr}) -- "
        f"passively polling SerialResponse for {duration_s:.0f}s..."
    )
    last_seen: str | None = None
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if core.hasProperty(hub, "SerialResponse"):
            try:
                response = str(core.getProperty(hub, "SerialResponse"))
            except Exception as e:  # pragma: no cover - defensive, logged not raised
                logger.debug(f"Could not read {hub}.SerialResponse: {e}")
                response = None
            if response and response != last_seen:
                lines.append(response)
                last_seen = response
        time.sleep(poll_interval_s)

    if not lines:
        logger.warning(
            "No SerialResponse lines captured -- the curve table may not be "
            "exposed through this property on this adapter build. Check the "
            "physical LCD or ASI Console's serial terminal instead."
        )
    return lines


_DRIFT_LOG_FIELDS = [
    "t_s",
    "timestamp",
    "crisp_label",
    "actuator_label",
    "crisp_state",
    "snr_db",
    "sum",
    "dither_error_counts",
    "lock_offset_counts",
    "log_amp_agc",
    "actuator_position_um",
    "snapshot_path",
    "focus_score",
]


def _focus_score(image: np.ndarray) -> float:
    """A simple, standard focus proxy: variance of the image's discrete Laplacian.

    Not a rigorous focus metric -- just a cheap, dependency-free (numpy-only,
    no scipy/opencv) way to see whether a saved snapshot got softer over
    time, for eyeballing against the telemetry columns in the same CSV. Higher
    is sharper.
    """
    img = image.astype(np.float64)
    lap = (
        -4 * img[1:-1, 1:-1]
        + img[:-2, 1:-1]
        + img[2:, 1:-1]
        + img[1:-1, :-2]
        + img[1:-1, 2:]
    )
    return float(lap.var())


def log_crisp_drift(
    crisp_to_actuator: dict[str, str],
    duration_s: float,
    interval_s: float = 5.0,
    camera_label: str | None = None,
    snapshot_interval_s: float = 60.0,
    out_dir: str | Path = ".",
    mmcore: CMMCorePlus | None = None,
) -> Path:
    """Log CRISP telemetry and actuator position over time to catch silent drift.

    Unlike :func:`apply_and_sample`/:func:`capture_focus_curve`, this never
    writes a CRISP property -- it only reads -- so it's safe to run
    unattended for the full *duration_s* without a human confirming each
    step. Designed to distinguish two very different failure modes that both
    look identical on the CRISPy panel (a clean, healthy "In Focus" state):

    - If the logged *actuator position* drifts while Dither Error stays near
      zero, the servo is actively moving the actuator to chase a shifting
      apparent target -- consistent with LED-intensity drift or ADEPT
      strain-gauge thermal drift (ASI's own error-calculation tech note
      shows a pure intensity change alone produces a residual error the
      servo will "correct"; ASI's piezo manual separately states room
      temperature changes can require strain-gauge recalibration).
    - If the actuator position stays flat but a saved snapshot's
      :func:`_focus_score` still degrades, CRISP is correctly holding its
      own reference rock-solid while that reference has decoupled from the
      true image plane -- a different, likely mechanical/thermal-alignment
      problem, not something more CRISP tuning can fix.

    Parameters
    ----------
    crisp_to_actuator : dict[str, str]
        Maps each CRISP autofocus device label to the actuator device label
        it servos (e.g. ``{"CRISPAFocus:P:34": "PiezoStage:P:34"}``). Pass
        both axes to compare them side by side in the same log.
    duration_s : float
        Total time to log for, in seconds (e.g. 900 for 15 minutes).
    interval_s : float
        Seconds between telemetry polls (default 5s -- gentle on the serial
        port for a long unattended run; the CRISPy panel itself defaults to
        750ms for short live viewing, which is unnecessarily chatty here).
    camera_label : str | None
        If given (with *snapshot_interval_s*), periodically snaps a frame
        from this camera and saves it as a TIFF alongside a focus score.
        Temporarily sets this as the Core's active camera device for each
        snapshot (matching the snap convention in
        :mod:`~pymmcore_gui.asi_z_stack.verify_handle_release`).
    snapshot_interval_s : float
        Seconds between camera snapshots, decoupled from *interval_s* since
        snapping is slower and produces files on disk (default 60s).
    out_dir : str | Path
        Directory to write the timestamped CSV (and a ``snapshots/``
        subdirectory of TIFFs, if *camera_label* is given) into.
    mmcore : CMMCorePlus | None
        Core instance to use; defaults to the shared session instance.

    Returns
    -------
    Path
        Path to the written CSV. Written incrementally (flushed after every
        row), so an interrupted run still leaves usable partial data.
    """
    core = mmcore or mmc
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"crisp_drift_{datetime.now():%Y%m%d_%H%M%S}.csv"
    snapshot_dir = out_dir / "snapshots"
    if camera_label:
        snapshot_dir.mkdir(parents=True, exist_ok=True)

    logger.info(
        f"Logging CRISP drift for {duration_s:.0f}s to {csv_path} "
        f"(axes: {list(crisp_to_actuator)})"
        + (
            f", camera snapshots every {snapshot_interval_s:.0f}s"
            if camera_label
            else ""
        )
    )

    start = time.monotonic()
    next_snapshot_at = 0.0
    snapshot_idx = 0
    with csv_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_DRIFT_LOG_FIELDS)
        writer.writeheader()

        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration_s:
                break
            now = datetime.now().isoformat(timespec="seconds")

            for crisp_label, actuator_label in crisp_to_actuator.items():
                row = dict.fromkeys(_DRIFT_LOG_FIELDS, "")
                row["t_s"] = f"{elapsed:.1f}"
                row["timestamp"] = now
                row["crisp_label"] = crisp_label
                row["actuator_label"] = actuator_label
                if core.hasProperty(crisp_label, "RefreshPropertyValues"):
                    try:
                        core.setProperty(crisp_label, "RefreshPropertyValues", "Yes")
                    except Exception as e:  # pragma: no cover - defensive
                        logger.debug(f"RefreshPropertyValues failed: {e}")
                for prop, key in (
                    ("CRISP State", "crisp_state"),
                    ("Signal Noise Ratio", "snr_db"),
                    ("Sum", "sum"),
                    (_ERROR_PROP, "dither_error_counts"),
                    ("Lock Offset", "lock_offset_counts"),
                    ("LogAmpAGC", "log_amp_agc"),
                ):
                    try:
                        row[key] = core.getProperty(crisp_label, prop)
                    except Exception as e:  # pragma: no cover - defensive
                        logger.debug(f"Could not read {crisp_label}.{prop}: {e}")
                try:
                    row["actuator_position_um"] = str(core.getPosition(actuator_label))
                except Exception as e:  # pragma: no cover - defensive
                    logger.debug(f"Could not read {actuator_label} position: {e}")
                writer.writerow(row)
            fh.flush()

            if camera_label and elapsed >= next_snapshot_at:
                snap_row = dict.fromkeys(_DRIFT_LOG_FIELDS, "")
                snap_row["t_s"] = f"{elapsed:.1f}"
                snap_row["timestamp"] = now
                snap_row["crisp_label"] = "__camera__"
                snap_row["actuator_label"] = camera_label
                try:
                    core.setCameraDevice(camera_label)
                    core.snapImage()
                    image = core.getImage()
                    snap_path = snapshot_dir / f"{snapshot_idx:04d}_t{elapsed:.0f}.tiff"
                    tifffile.imwrite(snap_path, image)
                    snap_row["snapshot_path"] = str(snap_path)
                    snap_row["focus_score"] = f"{_focus_score(image):.3f}"
                    snapshot_idx += 1
                except Exception as e:  # pragma: no cover - defensive
                    logger.warning(f"Snapshot failed at t={elapsed:.0f}s: {e}")
                writer.writerow(snap_row)
                fh.flush()
                next_snapshot_at += snapshot_interval_s

            time.sleep(interval_s)

    logger.info(f"Drift log complete: {csv_path}")
    return csv_path


def log_crisp_drift_async(
    crisp_to_actuator: dict[str, str],
    duration_s: float,
    interval_s: float = 5.0,
    camera_label: str | None = None,
    snapshot_interval_s: float = 60.0,
    out_dir: str | Path = ".",
    mmcore: CMMCorePlus | None = None,
    on_done: Callable[[Path], None] | None = None,
) -> FunctionWorker:
    """Run :func:`log_crisp_drift` on a background thread instead of blocking.

    pymmcore-gui's embedded console (``Ctrl+Shift+C``) runs an *in-process*
    IPython kernel that shares the Qt GUI thread -- calling
    :func:`log_crisp_drift` directly from it blocks the entire app, including
    the live camera view you'd actually want to watch during a multi-minute
    drift test, for the full *duration_s*. This wraps it in
    :func:`superqt.utils.create_worker` instead, the same pattern already
    used elsewhere in this codebase for long-running hardware calls (see
    :func:`~pymmcore_gui.asi_z_stack.asi_controller.
    ensure_circular_buffer_capacity_async`). Reading CRISP/Tiger properties
    from a background thread while the GUI thread runs normally is already
    proven safe here -- it's exactly what CRISPy's own telemetry poller
    (``pymmcore_widgets.control._async_poller.AsyncPoller``) does today.

    One caveat *log_crisp_drift* itself doesn't have when run synchronously:
    if you pass *camera_label*, this call mutates the Core's shared "current
    camera device" for each snapshot. Avoid running this concurrently with
    Live view on a different camera, since they'd fight over that same
    global state.

    Returns the running worker immediately (does not block). The CSV is
    flushed after every row, so you can open and inspect it at any point
    during the run rather than waiting for it to finish -- there's no need
    to cancel early just to check progress. *on_done*, if given, is called
    with the final CSV path once logging completes; failures are logged via
    the worker's ``errored`` signal either way.

    Parameters
    ----------
    crisp_to_actuator : dict[str, str]
        See :func:`log_crisp_drift`.
    duration_s : float
        See :func:`log_crisp_drift`.
    interval_s : float
        See :func:`log_crisp_drift`.
    camera_label : str | None
        See :func:`log_crisp_drift`.
    snapshot_interval_s : float
        See :func:`log_crisp_drift`.
    out_dir : str | Path
        See :func:`log_crisp_drift`.
    mmcore : CMMCorePlus | None
        See :func:`log_crisp_drift`.
    on_done : Callable[[Path], None] | None
        Called on completion with the CSV path (from the worker thread, not
        the GUI thread -- keep it lightweight, e.g. a log line, not a Qt
        widget update).
    """

    def _on_returned(path: Path) -> None:
        logger.info(f"Drift log finished: {path}")
        if on_done is not None:
            on_done(path)

    def _on_errored(exc: Exception) -> None:
        logger.error(f"Drift log failed: {exc}")

    worker = create_worker(
        log_crisp_drift,
        crisp_to_actuator,
        duration_s,
        interval_s=interval_s,
        camera_label=camera_label,
        snapshot_interval_s=snapshot_interval_s,
        out_dir=out_dir,
        mmcore=mmcore,
        _start_thread=True,
    )
    worker.returned.connect(_on_returned)
    worker.errored.connect(_on_errored)
    return worker
