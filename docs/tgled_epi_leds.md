# TGLED epifluorescence LEDs

The **Plugins > Epi LEDs (TGLED)** panel switches the ASI TGLED card's LEDs on
and off and sets their intensity. It is independent of the OPM/SPIM acquisition
path and locks itself while an MDA is running.

The panel lists every loaded `ASITiger` shutter device whose label starts with
`LED`, so it needs no code changes once the devices are in the hardware config.

## One-time setup: find the card's device names

The LED devices are created by the Tiger hub when it scans the controller, so
their labels (which include the card address) are only known from the hardware.
With the pyMM app **closed** (it holds COM4):

```powershell
uv run python scripts/discover_tgled.py COM4
```

It prints the `Device,...` lines to add to the hardware profile, e.g.

```text
Device,LED:X:37,ASITiger,LED:X:37
```

Add them to the profile you launch (next to the other `ASITiger` devices, and
add `Parent,<label>,TigerCommHub` alongside the other hub references), then
reload the config. Existing profiles in `hardware_profiles/` are not modified
by this feature.

## Notes

- Exact property names are read from the device at runtime; the intensity box
  appears for the first property whose name contains "intensity".
- If the panel shows "No ASITiger LED devices", the loaded config has none.
