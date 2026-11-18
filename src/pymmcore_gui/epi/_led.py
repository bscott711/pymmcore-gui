"""Hardware-free helpers for the ASI TGLED epifluorescence LEDs.

The TGLED card (Tiger address ``37``) appears in Micro-Manager as one
``ASITiger`` shutter device per channel, labelled ``LED:<axis>:<addr>:<n>``.
Everything in EPI code finds LEDs through :func:`resolve_led_channels` only, so
an unrelated LED such as ``LED:X:31`` (an X-axis LED on the XY card) can never
be shown, opened or closed.
"""

from __future__ import annotations

import logging
from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_plus import DeviceType

from pymmcore_gui.epi._mode import ScopeMode, current_mode

if TYPE_CHECKING:
    from collections.abc import Sequence

    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui._settings import EpiLedChannel

logger = logging.getLogger("pymmcore_gui")

#: Device library that provides the TGLED shutter devices.
LED_LIBRARY = "ASITiger"
#: Per-channel intensity property: MM Integer, limits 1..100, default 50
#: (ASITiger ``ASILED.cpp:106-109``). 0 is not legal; closing the shutter turns the
#: LED off. # O2: confirm name/limits on the rig
LED_INTENSITY_PROP = "LED Intensity(%)"
#: Default Tiger card address of the TGLED.
DEFAULT_CARD_ADDRESS = "37"
#: Channel numbers a TGLED card can have.
LED_CHANNELS = (1, 2, 3, 4)
#: Config-preset prefix: preset ``LED488`` maps to the settings channel named "488".
LED_PRESET_PREFIX = "LED"


def classify_led_label(
    label: str, card_address: str = DEFAULT_CARD_ADDRESS
) -> int | None:
    """Return the TGLED channel number for *label*, or ``None``.

    A label qualifies only if it is exactly four ``:``-separated tokens
    ``LED``, ``<axis>``, ``<addr>``, ``<n>`` with ``<addr>`` equal to
    *card_address* (exact string match) and ``<n>`` a digit 1-4. The axis
    letter is not checked. So ``LED:X:31`` (three tokens) and ``LED:L:38:1``
    (another card) are rejected.
    """
    tokens = label.split(":")
    if len(tokens) != 4:
        return None
    prefix, _axis, addr, num = tokens
    if prefix != "LED" or addr != card_address:
        return None
    if len(num) != 1 or num not in "1234":
        return None
    return int(num)


def resolve_led_channels(
    mmc: CMMCorePlus, card_address: str = DEFAULT_CARD_ADDRESS
) -> dict[int, str]:
    """Map TGLED channel number to device label for the loaded devices."""
    found: dict[int, str] = {}
    for label in mmc.getLoadedDevicesOfType(DeviceType.ShutterDevice):
        channel = classify_led_label(label, card_address)
        if channel is None:
            continue
        with suppress(RuntimeError):
            if mmc.getDeviceLibrary(label) == LED_LIBRARY:
                found[channel] = label
    return dict(sorted(found.items()))


def pct_to_prop(p: float) -> int:
    """Round *p* to the card's integer percent, clamped to [1, 100]."""
    return max(1, min(100, round(p)))


def channel_for_preset(
    preset: str, leds: Sequence[EpiLedChannel]
) -> EpiLedChannel | None:
    """Return the settings channel for config preset ``LED<name>``, else ``None``."""
    if not preset.startswith(LED_PRESET_PREFIX):
        return None
    name = preset[len(LED_PRESET_PREFIX) :]
    for led in leds:
        if led.name == name:
            return led
    return None


def apply_led(mmc: CMMCorePlus, label: str, pct: float) -> None:
    """Write the intensity, then open the LED *label*. EPI mode only.

    The percent is written first so the LED never lights at a stale value.
    """
    if current_mode() is not ScopeMode.EPI:
        raise RuntimeError(
            f"Refusing to switch on LED {label!r}: the app is not in EPI mode."
        )
    mmc.setProperty(label, LED_INTENSITY_PROP, pct_to_prop(pct))
    mmc.setShutterOpen(label, True)


def all_leds_off(mmc: CMMCorePlus, card_address: str = DEFAULT_CARD_ADDRESS) -> None:
    """Close every TGLED channel (including hidden ones). Allowed in any mode.

    Only closes shutters: the intensity property is never written. A device that
    is missing or unloaded is logged and skipped.
    """
    try:
        channels = resolve_led_channels(mmc, card_address)
    except Exception:
        logger.exception("Could not enumerate TGLED channels")
        return
    for label in channels.values():
        try:
            mmc.setShutterOpen(label, False)
        except Exception:
            logger.exception("Could not close LED %s", label)
