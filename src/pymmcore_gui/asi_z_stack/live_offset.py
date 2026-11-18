"""Remember the SPIM galvo's Live offset and re-apply it to the card.

The card comes up with ``SingleAxisYOffset(deg)`` at 0 after a restart or a
power cycle. The offset the user last set (Stage Control galvo group) is kept in
the app settings and re-applied -- property write plus a static mirror move --
whenever a hardware config loads. Only ``SingleAxisYOffset(deg)`` is written
here; the beam, the SPIM state and PLogic are never touched.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from pymmcore_gui._settings import GalvoSettingsV1, Settings
from pymmcore_gui.asi_z_stack.common import HardwareConstants
from pymmcore_gui.asi_z_stack.galvo_position import move_galvo_slice

if TYPE_CHECKING:
    from pymmcore_plus import CMMCorePlus

logger = logging.getLogger(__name__)

OFFSET_PROP = "SingleAxisYOffset(deg)"


def _default_deg() -> float:
    return GalvoSettingsV1().live_offset_deg


def saved_live_offset_deg() -> float:
    """Return the saved Live offset in degrees.

    Returns
    -------
    float
        The stored ``live_offset_deg``; the field default if the stored value is
        not finite.
    """
    deg = Settings.instance().galvo.live_offset_deg
    if not math.isfinite(deg):
        default = _default_deg()
        logger.warning(
            "Saved galvo Live offset %r is not finite; using %s", deg, default
        )
        return default
    return deg


def save_live_offset_deg(deg: float) -> None:
    """Store *deg* as the Live offset and write the settings file.

    Never raises; non-finite values are ignored.

    Parameters
    ----------
    deg : float
        The offset in degrees (the card's ``SingleAxisYOffset(deg)``).
    """
    if not math.isfinite(deg):
        logger.warning("Not saving non-finite galvo Live offset %r", deg)
        return
    try:
        settings = Settings.instance()
        settings.galvo.live_offset_deg = float(deg)
        settings.flush()
        logger.debug("Saved galvo Live offset %.6f deg", deg)
    except Exception as e:
        logger.warning("Could not save the galvo Live offset: %r", e)


def _clamp_to_limits(mmc: CMMCorePlus, device: str, deg: float) -> float:
    if mmc.hasPropertyLimits(device, OFFSET_PROP):
        lo, hi = sorted(
            (
                float(mmc.getPropertyLowerLimit(device, OFFSET_PROP)),
                float(mmc.getPropertyUpperLimit(device, OFFSET_PROP)),
            )
        )
        return min(max(deg, lo), hi)
    return deg


def restore_live_offset(mmc: CMMCorePlus, device: str | None = None) -> float | None:
    """Apply the saved Live offset to the card (property write + mirror move).

    Does nothing while an MDA is running, or when the galvo or its
    ``SingleAxisYOffset(deg)`` property is absent (non-ASI configs). Never
    raises.

    Parameters
    ----------
    mmc : CMMCorePlus
        The core to drive.
    device : str | None
        Galvo device label; defaults to ``HardwareConstants().galvo_a_label``.

    Returns
    -------
    float | None
        The degrees applied, or ``None`` if nothing was done.
    """
    try:
        device = device or HardwareConstants().galvo_a_label
        if mmc.mda.is_running():
            logger.info("Not restoring the galvo Live offset: an MDA is running")
            return None
        if device not in mmc.getLoadedDevices():
            logger.debug("Not restoring the galvo Live offset: %r not loaded", device)
            return None
        if not mmc.hasProperty(device, OFFSET_PROP):
            logger.info(
                "Not restoring the galvo Live offset: %r lacks %s", device, OFFSET_PROP
            )
            return None

        deg = _clamp_to_limits(mmc, device, saved_live_offset_deg())
        mmc.setProperty(device, OFFSET_PROP, f"{deg:.6f}")
        position = move_galvo_slice(mmc, device, deg)

        slope = HardwareConstants().slice_calibration_slope_um_per_deg or 100.0
        mirror = "unreadable" if position is None else f"{position[1]:.4f} deg"
        logger.info(
            "Restored galvo Live offset on %r: %.4f deg (%.2f um); mirror read-back %s",
            device,
            deg,
            deg * slope,
            mirror,
        )
        return deg
    except Exception as e:
        logger.warning("Could not restore the galvo Live offset: %r", e)
        return None
