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
config group (`LED488`, `LED561`, `LED642`). The label letter `L` is taken from
the rig log and is still to be confirmed on the rig with the script (O2, along
with the channel count and the `LED Intensity(%)` limits). The app looks the
card up by address 37 plus channel number with any letter. Edit `20260630-OPM.cfg` and
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
- Channel names (`488`, `561`, `642`, `405`) match the confirmed channel map
  below and can be overridden in the settings. Channel 4 (`405`) is hidden by
  default (not connected). It is still closed by "All off" and at shutdown.

### Channel map

The ASI LED command letters are X for channel 1, Y for channel 2, Z for
channel 3 and F for channel 4 (ASI docs: <https://docs.asiimaging.com/tgled>
and <https://docs.asiimaging.com/commands/led>). These are the serial command
letters, **not** the axis letter in the Micro-Manager device label.

| Channel | ASI letter | Band | Preset   | Visible            |
| ------- | ---------- | ---- | -------- | ------------------ |
| 1       | X          | 488  | `LED488` | yes                |
| 2       | Y          | 561  | `LED561` | yes                |
| 3       | Z          | 642  | `LED642` | yes                |
| 4       | F          | 405  | `LED405` | no (not connected) |

Micro-Manager exposes the card as four shutter devices under ASITiger. The ASI
docs treat 0 % as off; the app keeps 1 to 100 percent plus a separate on/off
and never writes 0.

## Notes

- Closing the app closes every TGLED channel, in any mode.
- If the panel says no TGLED channels are shown, the loaded config has no
  address-37 LED devices: load `20261118-OPM_EPI.cfg`.
