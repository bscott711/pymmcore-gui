# TGLED epifluorescence LEDs

The **Plugins > Epi LEDs (TGLED)** panel switches the ASI TGLED card's LEDs on
and off and sets each one's intensity. It is an EPI-mode panel: its controls
are active only when the app is in EPI mode, and disabled in OPM mode. It is
independent of the OPM/SPIM acquisition path and locks itself while an MDA is
running.

## Which devices are the TGLED

The TGLED card is at Tiger address **37**. Its channels appear in
Micro-Manager as `ASITiger` shutter devices labelled `LED:<axis>:37:<n>`, with
`<n>` from 1 to 4. The rig log from 2026-09-15 shows the card reporting axis
letter `L`, so the expected labels are `LED:L:37:1` to `LED:L:37:4`. The axis
letter is not checked, but the address (`37`) must match exactly.

`LED:X:31` is **not** the TGLED. It is an unrelated X-axis LED on the XY stage
card. EPI code never shows, opens or closes it, and neither does any other
four-token LED label at a different address (for example `LED:L:38:1`). All
discovery goes through `resolve_led_channels`, which applies these rules.

## One-time setup: find the card's device names

The LED devices are created by the Tiger hub when it scans the controller, so
their labels are only known from the hardware. With the pyMM app **closed** (it
holds COM4):

```powershell
uv run python scripts/discover_tgled.py COM4
```

It prints the `Device,...` lines for everything on the hub. Address-37 LED
channels are marked `# TGLED channel <n>`; any other `LED*` device is marked
`# other LED, not the TGLED`.

The profile `hardware_profiles/20261118-OPM_EPI.cfg` is a generated superset of
`20260630-OPM.cfg` that adds the four `LED:L:37:<n>` devices and an `EPI LEDs`
config group (`LED488`, `LED561`, `LED642`). The labels are placeholders taken
from the rig log: confirm them with the script. Edit `20260630-OPM.cfg` and
regenerate the EPI profile, never edit only one of the two.

## Intensity

- Each channel has an integer percent from 1 to 100, shown as a slider and a
  spinbox. It is the card's native `LED Intensity(%)` unit, **not calibrated
  power**: 50 % is not half the light of 100 %.
- On/off is a separate toggle. Off closes the shutter; 0 is never written.
- The percent is remembered in the app settings (`epi.leds`) and survives
  restarts, but is never applied automatically. It is written to the card only
  just before a channel is switched on, so opening the panel or changing a
  percent while a channel is off never emits light.
- Channel names (`488`, `561`, `642`, `405`) and which card channel carries
  which band are placeholders until confirmed on the rig; set them in the
  settings. Channel 4 (`405`) is hidden by default. It is still closed by "All
  off" and at shutdown.

## Notes

- Closing the app closes every TGLED channel, in any mode.
- If the panel says no TGLED channels are shown, the loaded config has no
  address-37 LED devices: load `20261118-OPM_EPI.cfg`.
