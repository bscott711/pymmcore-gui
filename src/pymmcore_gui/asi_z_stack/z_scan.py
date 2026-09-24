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
        ``SingleAxisYOffset(deg)`` placing the sweep center.
    """

    z_positions: tuple[float, ...]
    step_um: float
    center_offset_um: float
    amplitude_deg: float
    offset_deg: float

    @property
    def num_slices(self) -> int:
        """Number of slices (galvo trigger pulses) in the sweep."""
        return len(self.z_positions)

    @property
    def span_um(self) -> float:
        """Signed distance from the first to the last slice."""
        return self.z_positions[-1] - self.z_positions[0]


def compute_galvo_scan(
    z_plan: ZPlan, current_focus_um: float, slope_um_per_deg: float
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
    center_offset_um = midpoint if z_plan.is_relative else midpoint - current_focus_um
    return GalvoScan(
        z_positions=z,
        step_um=step,
        center_offset_um=center_offset_um,
        amplitude_deg=(z[-1] - z[0]) / slope_um_per_deg,
        offset_deg=center_offset_um / slope_um_per_deg,
    )
