# src/pymmcore_gui/asi_z_stack/z_scan.py
"""Translate a ``useq`` z-plan into ASI galvo slice-scan parameters.

The galvo sweeps ``SingleAxisYAmplitude(deg)`` centered on
``SingleAxisYOffset(deg)``, while the focus device (piezo) never moves during
a SPIM MDA. So each Z Stack mode has to be expressed as *where the sweep is
centered relative to the current focus* plus *how far and in which direction
it sweeps*:

- ``ZRangeAround``: centered on current focus (offset 0).
- ``ZAboveBelow``: relative, asymmetric -- center shifted by
  ``(above - below) / 2``; e.g. ``below=0`` sweeps only upward from focus.
- ``ZTopBottom`` (and other absolute plans): absolute focus-device
  coordinates, so the center is measured against the focus position at
  acquisition time.

"Current focus" is the plane Live shows: the piezo where it is *and* the
galvo at the offset Live sits at. The sweep is therefore placed relative to
that pre-MDA galvo offset, not to galvo zero, so slice *i* images the Live
plane + ``z[i] - reference`` in every mode.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from useq._z import ZPlan

# Allowed deviation between consecutive z steps before a plan is rejected as
# non-uniform (the galvo can only step uniformly).
_STEP_TOLERANCE_UM = 1e-3


@dataclass(frozen=True)
class GalvoScan:
    """Galvo slice-scan parameters derived from a z-plan.

    Attributes
    ----------
    z_positions : tuple[float, ...]
        The plan's positions, in acquisition order (relative or absolute,
        as the plan defines them).
    step_um : float
        Signed step between consecutive slices (negative sweeps downward).
    center_offset_um : float
        Sweep center relative to the current focus position.
    amplitude_deg : float
        Signed ``SingleAxisYAmplitude(deg)``; its sign sets sweep direction.
    offset_deg : float
        ``SingleAxisYOffset(deg)`` placing the sweep center (absolute galvo
        position, including the Live offset).
    reference_um : float
        Plan coordinate of the Live plane: ``0`` for relative plans, the
        focus position for absolute ones.
    """

    z_positions: tuple[float, ...]
    step_um: float
    center_offset_um: float
    amplitude_deg: float
    offset_deg: float
    reference_um: float = 0.0

    @property
    def num_slices(self) -> int:
        """Number of slices (galvo trigger pulses) in the sweep."""
        return len(self.z_positions)

    @property
    def span_um(self) -> float:
        """Signed distance from the first to the last slice."""
        return self.z_positions[-1] - self.z_positions[0]

    def live_plane_slice(self) -> str:
        """Where the Live plane falls in the stack, for the run log."""
        rel = [z - self.reference_um for z in self.z_positions]
        for i, r in enumerate(rel):
            if abs(r) <= _STEP_TOLERANCE_UM:
                return f"slice {i}"
        for i, (a, b) in enumerate(itertools.pairwise(rel)):
            if min(a, b) < 0 < max(a, b):
                return f"between slices {i} and {i + 1}"
        return "outside the stack"

    def describe(self) -> str:
        """Planes relative to the Live plane, for the run log."""
        # "+ 0.0" turns useq's -0.0 into 0.0 so it doesn't print as "-0.000"
        first = self.z_positions[0] - self.reference_um + 0.0
        last = self.z_positions[-1] - self.reference_um + 0.0
        return (
            f"{self.num_slices} slices, step {self.step_um:+.3f} um, planes "
            f"{first:+.3f} .. {last:+.3f} um from the Live plane; Live plane = "
            f"{self.live_plane_slice()}"
        )


def compute_galvo_scan(
    z_plan: ZPlan,
    current_focus_um: float,
    slope_um_per_deg: float,
    live_offset_deg: float = 0.0,
) -> GalvoScan:
    """Compute galvo amplitude/offset that reproduce ``z_plan`` exactly.

    Parameters
    ----------
    z_plan : ZPlan
        The sequence's z-plan. Relative plans (``is_relative``) are offsets
        from the current focus; absolute plans are focus-device coordinates.
    current_focus_um : float
        Focus-device position at acquisition time. Only used for absolute
        plans such as ``ZTopBottom``.
    slope_um_per_deg : float
        Galvo slice calibration (microns of z per degree of galvo).
    live_offset_deg : float
        The galvo ``SingleAxisYOffset(deg)`` Live sits at before the MDA; the
        sweep is placed relative to it.

    Raises
    ------
    ValueError
        If the plan is empty or its steps aren't uniform (the galvo can only
        sweep at a constant step size).
    """
    z = tuple(float(p) for p in z_plan)
    if not z:
        raise ValueError("Z plan contains no positions.")

    step = z[1] - z[0] if len(z) > 1 else 0.0
    for i, (a, b) in enumerate(itertools.pairwise(z)):
        if abs((b - a) - step) > _STEP_TOLERANCE_UM:
            raise ValueError(
                "The galvo can only scan uniformly spaced z-slices, but this "
                f"z plan's step changes at slice {i + 1} "
                f"({b - a:+.4f} um vs {step:+.4f} um)."
            )

    midpoint = (z[0] + z[-1]) / 2
    reference = 0.0 if z_plan.is_relative else current_focus_um
    center_offset_um = midpoint - reference
    return GalvoScan(
        z_positions=z,
        step_um=step,
        center_offset_um=center_offset_um,
        amplitude_deg=(z[-1] - z[0]) / slope_um_per_deg,
        offset_deg=live_offset_deg + center_offset_um / slope_um_per_deg,
        reference_um=reference,
    )
