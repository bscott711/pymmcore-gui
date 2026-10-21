"""Per-slice timing for galvo-scanned SPIM with rolling-shutter Kinetix cameras.

Each slice: the galvo steps, waits ``settle`` ms, then the card's TTL (PLogic
backplane address 41) fires the camera; the galvo holds that plane for
``hold`` ms after the trigger before stepping again, so

    slice period = settle + hold

Two illumination schemes:

``"all_rows"``
    The Kinetix's pseudo-global "All Rows" mode. The camera starts exposing
    on the trigger edge (Edge Trigger, exposure = E), rolls its rows in over
    one readout time S, and raises Expose Out only while *every* row is
    integrating -- the window ``[S, S+E]`` after the trigger. PLogic gates
    the laser with Expose Out, so every row gets exactly E of light, the same
    as Live/Snap with a continuous laser. The frame finishes reading out at
    ``2S + E``. The galvo must not step before the light ends and the next
    trigger must wait for the readout:

        hold = max(S + E, 2S + E - settle) + margin

    A laser that can't be pulsed (one behind a mechanical shutter, held on
    for the whole volume) uses the camera's normal rolling exposure instead,
    and the galvo waits for the last row: ``hold = S + E + margin``.

``"legacy"``
    The original scheme: camera (Level Trigger) and laser are the same
    E-long PLogic pulse. Rows that start late in the rolling readout get
    less light -- MDA frames come out dimmer than Live, with a top-to-bottom
    falloff. ``hold = max(E, S + E - settle)``, which reproduces the
    original timing (35.75 ms at E = 10 ms, settle = 25.75 ms).

All times are in ms and rounded up to the galvo card's 0.25 ms clock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

TimingMode = Literal["all_rows", "legacy"]

#: The galvo card's and PLogic's clock period.
TICK_MS = 0.25


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
        PLogic camera trigger pulse width (Edge Trigger: a short edge;
        legacy Level Trigger: the exposure itself).
    volume_start_settle_ms : float
        Extra settle before the first slice of each volume (the galvo's jump
        back to the start of the sweep).
    laser_held_on : bool
        The laser can't be pulsed (stays on for the whole volume).
    """

    mode: TimingMode
    exposure_ms: float
    readout_ms: float
    settle_ms: float
    hold_ms: float
    camera_pulse_ms: float
    volume_start_settle_ms: float
    laser_held_on: bool

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
        light_end = self.readout_ms + self.exposure_ms
        if self.mode == "legacy" and not self.laser_held_on:
            light_end = self.exposure_ms  # the pulse ends; late rows are just dark
        return self.hold_ms + 1e-9 < light_end

    def describe(self, n_slices: int | None = None) -> str:
        """One-line human summary for logs and the timing panel."""
        text = (
            f"{self.mode}: settle {self.settle_ms:g} + hold {self.hold_ms:g} = "
            f"{self.slice_period_ms:g} ms/slice (E {self.exposure_ms:g}, "
            f"readout {self.readout_ms:g}, trigger {self.camera_pulse_ms:g} ms)"
        )
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
    trigger_pulse_ms: float = 1.0,
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
        Safety margin added to the galvo hold (All Rows only; legacy keeps
        the original timing exactly).
    trigger_pulse_ms : float
        Width of the camera's Edge Trigger pulse (All Rows only).
    volume_start_settle_ms : float
        Extra settle before each volume's first slice.
    laser_held_on : bool
        The channel's laser can't be pulsed.
    """
    e, s = exposure_ms, max(0.0, readout_ms)
    if mode == "all_rows":
        if laser_held_on:
            hold = s + e + margin_ms
        else:
            hold = max(s + e, 2 * s + e - settle_ms) + margin_ms
        pulse = trigger_pulse_ms
    elif mode == "legacy":
        hold = max(e, s + e - settle_ms)
        pulse = e
    else:  # pragma: no cover -- guarded by the Literal type
        raise ValueError(f"Unknown timing mode {mode!r}")
    return SliceTiming(
        mode=mode,
        exposure_ms=e,
        readout_ms=s,
        settle_ms=ceil_tick(settle_ms),
        hold_ms=ceil_tick(hold),
        camera_pulse_ms=ceil_tick(pulse),
        volume_start_settle_ms=ceil_tick(volume_start_settle_ms),
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
