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
"""

from __future__ import annotations

import logging
import statistics
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from CRISPy.controller import ASICrispController

from ._logging import configure_asi_logging
from .asi_controller import _HW, mmc

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

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
