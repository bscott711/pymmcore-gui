"""List the devices a Tiger controller reports, flagging TGLED LED devices.

Usage: ``uv run python scripts/discover_tgled.py COM4``

Read-only: loads the ASITiger hub on the given port and asks it which devices
are installed. Close pyMM first -- it holds the serial port.
"""

from __future__ import annotations

import sys

from pymmcore_plus import CMMCorePlus


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
        mark = "  # LED" if name.startswith("LED") else ""
        print(f"Device,{name},ASITiger,{name}{mark}")
    if not any(n.startswith("LED") for n in installed):
        print("No LED* devices found; check the card is powered and addressed.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
