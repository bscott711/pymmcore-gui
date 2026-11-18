"""Static galvo slice-axis moves for the ASI Tiger scanner (card axis B).

Writing the ``SingleAxisYOffset(deg)`` property only sends ``SAO B=...``: the
*centre of the single-axis / SPIM sweep*, which does not move an idle mirror
(ASITiger ``ASIScanner.cpp`` ``OnSAOffsetY``). The adapter's only static move is
``CMMCore.setGalvoPosition`` -> serial ``M A=<x> B=<y>`` (``ASIScanner.cpp``
``SetPosition``), in the same degree units as the ``(deg)`` properties. The
adapter silently skips an axis whose ``SingleAxis?Mode`` is not disabled, and
everything when ``BeamEnabled`` is ``No``, so moves are read back and logged.
This module never writes a property; it only moves and diagnoses.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

logger = logging.getLogger(__name__)

SLICE_OFFSET_PROP = "SingleAxisYOffset(deg)"
POSITION_TOLERANCE_DEG = 0.01

# Properties that make the adapter silently ignore a static move.
_DIAGNOSTIC_PROPS = ("BeamEnabled", "SingleAxisYMode", "SPIMState")


def read_galvo_position(mmc: CMMCorePlus, device: str) -> tuple[float, float] | None:
    """Read the galvo's ``(x_deg, y_deg)`` position from the card.

    Parameters
    ----------
    mmc : CMMCorePlus
        The core to query.
    device : str
        Galvo (scanner) device label.

    Returns
    -------
    tuple[float, float] | None
        ``(x_deg, y_deg)``, or ``None`` if the call fails or the result cannot
        be converted to two floats.
    """
    try:
        raw = mmc.getGalvoPosition(device)
        x, y = raw
        return float(x), float(y)
    except Exception as e:
        logger.warning("Could not read galvo position of %r: %r", device, e)
        return None


def _diagnostic_state(mmc: CMMCorePlus, device: str) -> str:
    """Return ``name=value`` for each diagnostic property that can be read."""
    parts: list[str] = []
    for prop in _DIAGNOSTIC_PROPS:
        try:
            if mmc.hasProperty(device, prop):
                parts.append(f"{prop}={mmc.getProperty(device, prop)}")
        except Exception as e:
            logger.debug("Could not read %s.%s: %r", device, prop, e)
    return ", ".join(parts) or "unavailable"


def move_galvo_slice(
    mmc: CMMCorePlus, device: str, y_deg: float
) -> tuple[float, float] | None:
    """Statically move the slice axis (Y / card axis B) to ``y_deg``.

    The sheet axis (X / card axis A) is kept at its current position. Never
    raises; problems are logged.

    Parameters
    ----------
    mmc : CMMCorePlus
        The core to drive.
    device : str
        Galvo (scanner) device label.
    y_deg : float
        Target slice-axis position, in degrees.

    Returns
    -------
    tuple[float, float] | None
        The ``(x_deg, y_deg)`` read back from the card after the move, or
        ``None`` if the move or the readback failed.
    """
    try:
        current = read_galvo_position(mmc, device)
        if current is None:
            logger.info("Galvo %r X unreadable; moving with x=0.0", device)
            x_deg = 0.0
        else:
            x_deg = current[0]

        logger.info("setGalvoPosition(%r, x=%.4f, y=%.4f)", device, x_deg, y_deg)
        try:
            mmc.setGalvoPosition(device, x_deg, y_deg)
        except Exception as e:
            logger.warning("setGalvoPosition(%r) failed: %r", device, e)
            return None

        readback = read_galvo_position(mmc, device)
        if readback is None:
            return None
        logger.info(
            "Galvo %r readback: x=%.4f, y=%.4f", device, readback[0], readback[1]
        )
        if abs(readback[1] - y_deg) > POSITION_TOLERANCE_DEG:
            logger.warning(
                "Galvo %r did not reach y=%.4f (read %.4f); the adapter ignores "
                "moves when the beam is off or single-axis mode is on (%s)",
                device,
                y_deg,
                readback[1],
                _diagnostic_state(mmc, device),
            )
        return readback
    except Exception as e:
        logger.warning("move_galvo_slice(%r, %r) failed: %r", device, y_deg, e)
        return None
