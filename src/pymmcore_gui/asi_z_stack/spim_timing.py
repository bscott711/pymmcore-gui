"""Per-slice timing for galvo-scanned SPIM with rolling-shutter Kinetix cameras.

Each slice: the galvo steps, waits ``settle`` ms, then the card's TTL (PLogic
backplane address 41) fires the camera; the galvo holds that plane for
``hold`` ms after the trigger before stepping again, so

    slice period = settle + hold

Two illumination schemes:

``"all_rows"``
    Pseudo-global exposure ("All Rows") made by delaying the laser. The
    camera stays in Level Trigger with a pulse of ``C = S + E`` (plus
    margins); its rows start over one readout time S, so every row is
    integrating during ``[S, S+E]``. PLogic fires the laser only in that
    window (a delay cell of ``D = S + margin`` feeding the E-long laser
    one-shot), so every row gets exactly E of light, the same as Live/Snap
    with a continuous laser. The galvo may step as soon as the camera pulse
    (and the laser inside it) ends:

    - Level Trigger: the next frame can't start until the readout finishes,
      at ``C + S``, so ``hold = max(C, C + S - settle)``.
    - Level Trigger Overlap: the next frame may start while the previous one
      is still reading out, so ``hold = C`` and the slice period is about
      ``S + E + settle``.

    A laser that can't be pulsed (one behind a mechanical shutter, held on
    for the whole volume) can't be delayed: the camera pulse is E, every row
    integrates E under constant light, and the galvo waits for the last row
    (``hold = S + E + margin``) in plain Level Trigger.

``"legacy"``
    The original scheme: camera (Level Trigger) and laser are the same
    E-long PLogic pulse. Rows that start late in the rolling readout get
    less light -- MDA frames come out dimmer than Live, with a top-to-bottom
    falloff. ``hold = max(E, S + E - settle)``, which reproduces the
    original timing (35.75 ms at E = 10 ms, settle = 25.75 ms). Run from the
    settings (:func:`timing_from_settings`), legacy always uses the original
    settle -- it is the known-good fallback, not a tunable mode.

All times are in ms and rounded up to the galvo card's 0.25 ms clock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from pymmcore_gui._settings import SpimTimingSettingsV1

TimingMode = Literal["all_rows", "legacy"]

#: The galvo card's and PLogic's clock period.
TICK_MS = 0.25

#: Galvo settle the rig ran with before advanced timing existed (the card's
#: stored ``SPIMDelayBeforeLaser``). Legacy mode always uses it, whatever the
#: All Rows settle is set to.
LEGACY_SETTLE_MS = 25.75


def ceil_tick(ms: float) -> float:
    """Round *ms* up to the next 0.25 ms clock tick (to absorb float noise)."""
    return math.ceil(round(ms / TICK_MS, 6)) * TICK_MS


@dataclass(frozen=True)
class SliceTiming:
    """The timing one slice of a hardware-triggered volume will use.

    Attributes
    ----------
    mode : {"all_rows", "legacy"}
        Illumination scheme (see the module docstring).
    exposure_ms : float
        E: light each row should receive (the channel's exposure).
    readout_ms : float
        S: the camera's rolling readout time at its current ROI.
    settle_ms : float
        Galvo settle between its step and the camera trigger.
    hold_ms : float
        How long the galvo holds the plane after the trigger.
    camera_pulse_ms : float
        PLogic camera pulse width; with Level Trigger this is each row's
        integration time.
    volume_start_settle_ms : float
        Extra settle before the first slice of each volume (the galvo's jump
        back to the start of the sweep).
    laser_held_on : bool
        The laser can't be pulsed (stays on for the whole volume).
    laser_delay_ms : float
        Delay from the trigger to the laser pulse (All Rows; 0 in legacy).
    laser_pulse_ms : float
        Laser pulse width (E, rounded to the clock).
    overlap : bool
        The camera runs in Level Trigger Overlap.
    """

    mode: TimingMode
    exposure_ms: float
    readout_ms: float
    settle_ms: float
    hold_ms: float
    camera_pulse_ms: float
    volume_start_settle_ms: float
    laser_held_on: bool
    laser_delay_ms: float = 0.0
    laser_pulse_ms: float = 0.0
    overlap: bool = False

    @property
    def slice_period_ms(self) -> float:
        """Time from one galvo step to the next."""
        return self.settle_ms + self.hold_ms

    def volume_ms(self, n_slices: int) -> float:
        """Duration of one *n_slices* volume, excluding software overhead."""
        return self.volume_start_settle_ms + n_slices * self.slice_period_ms

    @property
    def first_row_light_ms(self) -> float:
        """Light received by the first-read row."""
        return self.exposure_ms

    @property
    def last_row_light_ms(self) -> float:
        """Light received by the last-read row.

        All Rows and held-on lasers give every row E; a legacy pulsed laser
        has already switched off by the time late rows start integrating.
        """
        if self.mode == "all_rows" or self.laser_held_on:
            return self.exposure_ms
        return max(0.0, self.exposure_ms - self.readout_ms)

    @property
    def mean_row_light_fraction(self) -> float:
        """Average light per row as a fraction of E (1.0 = matches Live)."""
        e, s = self.exposure_ms, self.readout_ms
        if self.mode == "all_rows" or self.laser_held_on or s <= 0 or e <= 0:
            return 1.0
        if s <= e:
            return (e - s / 2) / e
        return e / (2 * s)

    @property
    def galvo_moves_during_exposure(self) -> bool:
        """Whether some row is still lit while the galvo steps (blur)."""
        if self.laser_held_on:
            light_end = self.readout_ms + self.camera_pulse_ms  # last row stops
        else:
            light_end = self.laser_delay_ms + self.laser_pulse_ms
        return self.hold_ms + 1e-9 < light_end

    def describe(self, n_slices: int | None = None) -> str:
        """One-line human summary for logs and the timing panel."""
        text = (
            f"{self.mode}{' (overlap)' if self.overlap else ''}: settle "
            f"{self.settle_ms:g} + hold {self.hold_ms:g} = "
            f"{self.slice_period_ms:g} ms/slice (E {self.exposure_ms:g}, readout "
            f"{self.readout_ms:g}, camera pulse {self.camera_pulse_ms:g} ms"
        )
        if self.laser_delay_ms:
            text += f", laser {self.laser_pulse_ms:g} ms after {self.laser_delay_ms:g}"
        text += ")"
        if n_slices:
            text += f"; {n_slices} slices = {self.volume_ms(n_slices) / 1000:.2f} s"
        if self.mean_row_light_fraction < 0.999:
            text += (
                f"; rows get {self.first_row_light_ms:g} -> "
                f"{self.last_row_light_ms:g} ms of light "
                f"(mean {self.mean_row_light_fraction:.0%} of Live)"
            )
        if self.galvo_moves_during_exposure:
            text += "; WARNING: galvo steps while rows are still lit"
        return text


def compute_slice_timing(
    mode: TimingMode,
    exposure_ms: float,
    readout_ms: float,
    settle_ms: float,
    *,
    margin_ms: float = 0.5,
    overlap: bool = False,
    volume_start_settle_ms: float = 0.0,
    laser_held_on: bool = False,
) -> SliceTiming:
    """Compute one slice's timing (see the module docstring for the model).

    Parameters
    ----------
    mode : {"all_rows", "legacy"}
        Illumination scheme.
    exposure_ms : float
        E, the channel's exposure.
    readout_ms : float
        S, the camera's rolling readout time (max over cameras).
    settle_ms : float
        Galvo settle between its step and the camera trigger.
    margin_ms : float
        All Rows safety margin: added to the laser delay (so it starts after
        the last row has started), to the camera pulse (so it ends after the
        laser), and to a held-on laser's galvo hold. Legacy ignores it.
    overlap : bool
        All Rows with the camera in Level Trigger Overlap (pulsed lasers only).
    volume_start_settle_ms : float
        Extra settle before each volume's first slice.
    laser_held_on : bool
        The channel's laser can't be pulsed.
    """
    e, s = exposure_ms, max(0.0, readout_ms)
    settle = ceil_tick(settle_ms)
    laser = ceil_tick(e)
    delay = 0.0
    if mode == "all_rows" and laser_held_on:
        overlap = False  # constant light: rows must all integrate the full E
        pulse = laser
        hold = ceil_tick(s + e + margin_ms)
    elif mode == "all_rows":
        delay = ceil_tick(s + margin_ms)
        pulse = ceil_tick(delay + laser + margin_ms)
        hold = pulse if overlap else ceil_tick(max(pulse, pulse + s - settle))
    elif mode == "legacy":
        overlap = False
        pulse = laser
        hold = ceil_tick(max(e, s + e - settle_ms))
    else:  # pragma: no cover -- guarded by the Literal type
        raise ValueError(f"Unknown timing mode {mode!r}")
    return SliceTiming(
        mode=mode,
        exposure_ms=e,
        readout_ms=s,
        settle_ms=settle,
        hold_ms=hold,
        camera_pulse_ms=pulse,
        volume_start_settle_ms=ceil_tick(volume_start_settle_ms),
        laser_held_on=laser_held_on,
        laser_delay_ms=delay,
        laser_pulse_ms=laser,
        overlap=overlap,
    )


def timing_from_settings(
    cfg: SpimTimingSettingsV1,
    exposure_ms: float,
    readout_ms: float,
    *,
    laser_held_on: bool = False,
) -> SliceTiming:
    """The slice timing an MDA runs with under *cfg*.

    All Rows uses the configured settle, volume-start settle, margin and
    overlap switch. Legacy ignores them and reproduces the original timing
    (:data:`LEGACY_SETTLE_MS`, no volume-start settle), so switching back to
    legacy always restores the known-good acquisition.

    Parameters
    ----------
    cfg : SpimTimingSettingsV1
        The SPIM timing settings.
    exposure_ms : float
        E, the channel's exposure.
    readout_ms : float
        S, the camera's rolling readout time.
    laser_held_on : bool
        The channel's laser can't be pulsed.
    """
    if cfg.mode == "legacy":
        return compute_slice_timing(
            "legacy",
            exposure_ms,
            readout_ms,
            LEGACY_SETTLE_MS,
            laser_held_on=laser_held_on,
        )
    return compute_slice_timing(
        cfg.mode,
        exposure_ms,
        readout_ms,
        cfg.galvo_settle_ms,
        margin_ms=cfg.margin_ms,
        overlap=cfg.level_trigger_overlap,
        volume_start_settle_ms=cfg.volume_start_settle_ms,
        laser_held_on=laser_held_on,
    )


def readout_ms_from_camera_timing(timing: dict[str, str]) -> float | None:
    """The rolling readout time (ms) a camera reported, if it reported one.

    Reads PVCAM's ``Timing-ReadoutTimeNs`` (any property whose name has
    "readout" and "time" in it; ``Ns``/``Us`` suffixes are converted), as
    sent in :class:`~pymmcore_gui.asi_z_stack.worker_messages.ArmedMsg`.
    """
    for name, value in timing.items():
        lowered = name.lower()
        if "readout" not in lowered or "time" not in lowered:
            continue
        try:
            number = float(value)
        except ValueError:
            continue
        if lowered.endswith("ns"):
            return number / 1e6
        if lowered.endswith("us"):
            return number / 1e3
        return number
    return None
