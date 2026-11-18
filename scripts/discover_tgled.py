"""List the devices a Tiger controller reports, flagging TGLED LED devices.

Only address-37 four-token labels (``LED:<axis>:37:<n>``) are marked as TGLED
channels. Any other ``LED*`` device (e.g. the X-axis ``LED:X:31``) is marked
"other LED, not the TGLED".

Usage: ``uv run python scripts/discover_tgled.py COM4``

Read-only: loads the ASITiger hub on the given port and asks it which devices
are installed. Close pyMM first -- it holds the serial port.
"""

from __future__ import annotations

import sys

from pymmcore_plus import CMMCorePlus

from pymmcore_gui.epi._led import classify_led_label


def device_line(name: str, card_address: str = "37") -> str:
    """Return the ``Device,...`` config line for *name*, with an LED comment."""
    mark = ""
    if name.startswith("LED"):
        channel = classify_led_label(name, card_address)
        if channel is not None:
            mark = f"  # TGLED channel {channel}"
        else:
            mark = "  # other LED, not the TGLED"
    return f"Device,{name},ASITiger,{name}{mark}"


def main(port: str) -> None:
    """Print the `Device,...` config lines for everything on the Tiger at `port`.

    Parameters
    ----------
    port : str
        Serial port of the Tiger controller, e.g. ``"COM4"``.
    """
    mmc = CMMCorePlus()
    mmc.loadDevice(port, "SerialManager", port)
    # same port settings as the OPM profiles
    for prop, value in {
        "AnswerTimeout": "500.0000",
        "BaudRate": "115200",
        "DTR": "Disable",
        "DataBits": "8",
        "DelayBetweenCharsMs": "0.0000",
        "Fast USB to Serial": "Disable",
        "Handshaking": "Off",
        "Parity": "None",
        "StopBits": "1",
    }.items():
        mmc.setProperty(port, prop, value)
    mmc.initializeDevice(port)
    mmc.loadDevice("TigerCommHub", "ASITiger", "TigerCommHub")
    mmc.setProperty("TigerCommHub", "Port", port)
    mmc.initializeDevice("TigerCommHub")

    installed = mmc.getInstalledDevices("TigerCommHub")
    print(f"{len(installed)} devices reported by TigerCommHub on {port}:")
    for name in installed:
        print(device_line(name))
    if not any(classify_led_label(n) is not None for n in installed):
        print("No TGLED (address 37) channels found; check the card is powered.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
