# EPI mode

EPI mode runs the epifluorescence path of the scope: an Orca camera lit by the
TGLED card's LEDs. It sits next to the normal OPM (SPIM) path and never mixes
with it.

- **Mutually exclusive with OPM.** The app is in exactly one mode. In EPI the
  OPM cameras, lasers and galvo are idle.
- **Always starts in OPM.** The mode is never saved. After a restart you are in
  OPM and every LED is dark.
- **Any config load returns to OPM.** File > Load, the Config Wizard and the
  Console all do this. Switch to EPI again afterwards.

The LED panel itself is described in [TGLED epifluorescence LEDs](tgled_epi_leds.md).
This page covers the mode as a whole.

## One-time setup

### Load the EPI profile

1. In the app, use File > Load and pick `hardware_profiles/20261118-OPM_EPI.cfg`.
2. The app remembers the last loaded config, so this profile now loads at
   startup.

`hardware_profiles/20260630-OPM.cfg` is untouched and stays the fallback: load it
to run without any EPI devices. The EPI profile is a superset of the OPM one
(it adds the `LED:L:37:<n>` devices, the `EPI LED Selector` device and the
`EPI LEDs` config group). Keep both
in sync: after editing the OPM profile, regenerate the EPI one. A test fails
when the two drift apart.

Without the EPI profile loaded, switching to EPI is refused with a message
telling you to load it.

### Orca camera settings

The Orca is in no cfg file. The app starts its own camera process for it on the
first switch to EPI, then keeps it for the rest of the session. Its identity
comes from the app settings, section `epi.camera` of `pmm_settings.json` in the
app's user data folder. The section is pre-filled with the rig's Orca (see
[Confirmed hardware](#confirmed-hardware)), so there is nothing to enter. Edit
the JSON, with the app closed, only to override a value. There is no settings
dialog for it yet. EPI entry is refused ("Set the Orca camera in settings") if
any of the first six fields is emptied:

| Field            | Meaning                                                                         | Default             |
| ---------------- | ------------------------------------------------------------------------------- | ------------------- |
| `label`          | Device label the camera is loaded under                                         | `Orca`              |
| `adapter_module` | Micro-Manager device adapter (library) for the Orca                             | `HamamatsuHam`      |
| `device_name`    | Device name inside that adapter                                                 | `HamamatsuHam_DCAM` |
| `width`          | Frame width in pixels                                                           | `2048`              |
| `height`         | Frame height in pixels                                                          | `2048`              |
| `dtype`          | Numpy dtype of a frame                                                          | `uint16`            |
| `pre_init_props` | Optional list of `[property, value]` pairs set before the camera is initialised | none                |

### LED channel names

Settings section `epi.leds` has one entry per card channel (1 to 4) with a
`name` (band, for example `488`), `visible` and the remembered `percent`. The
defaults match the confirmed channel map (`488`, `561`, `642` and a hidden
`405`), so there is nothing to set. Change them only to override.

## Using EPI mode

### Switching

Use the **Mode** toolbar: an OPM / EPI toggle plus a "Mode:" label. A switch is
refused, with a message, when an MDA is running, Live is on, no `LED:L:37:<n>`
device is loaded, or the Orca settings are empty.

What entering EPI does:

- stops OPM Live and closes all lasers;
- sets the channel group to `EPI LEDs`, so the Optical Configs toolbar offers
  the LED presets;
- closes every LED (nothing is written to the intensity; saved percentages are
  not applied);
- starts the Orca camera process if it is not running, and installs the EPI
  acquisition engine.

What leaving EPI does: closes every LED, restores the previous channel group
and the OPM acquisition engine, and checks that the galvo is idle.

What the switch does **not** do, in either direction:

- it never locks, unlocks or changes CRISP, and never moves stage Z or the piezo;
- it never touches the joystick wheels;
- it never changes Core Focus or Core Shutter.

### LED panel

Open **Plugins > Epi LEDs (TGLED)**. Each channel has an on/off toggle and a
1 to 100 percent slider and spinbox.

- Percent is the card's native unit, **not calibrated power**.
- Off means the shutter is closed. 0 is never written.
- Percentages are remembered across restarts but are never applied by
  themselves. A percent reaches the card only just before a channel is opened.
- In OPM mode the panel is visible but its controls are disabled.

### Snap and Live

Snap and Live use the Orca and light only the LED of the currently selected
`EPI LEDs` preset (for example `LED488`) at its stored percent. The LED is
closed again when Snap or Live ends. Pick the preset in the Optical Configs
toolbar; the preset is read from the `EPI LED Selector`, so it does not depend
on which LEDs are on. If the selector has no matching preset, Snap and Live ask
you to select one first. Known limit: changing the preset during Live does not
switch the lit LED; it takes effect when Live is restarted. The exposure widget and the
camera ROI widget follow the Orca while in EPI. Right after switching, the Orca
may still be starting: a Snap in that moment is refused, so retry.

### Time series (MDA)

Version 1 runs a **single-plane time series** with `EPI LEDs` channels:

- no z-stack, autofocus, grid or more than one stage position (refused with a
  message before anything moves);
- at least one channel (a run with nothing to light is refused);
- every channel must be an `EPI LEDs` preset that maps to a loaded, visible LED;
- no lasers, PLogic or galvo are touched.

CRISP Z is **recommended but not enforced**. Lock CRISP Z (`CRISPAFocus:Z:32`) in
the CRISPy panel before you start. The engine never moves Z. If the lock is
off, or the device is not loaded, the run still goes ahead and one warning is
written to the log. The lock state is read at every time point; a change during
the run is logged once as a warning, and the run never stops because of it.

For each frame the engine writes the run's percent for that LED, opens the LED,
takes one image with the Orca, and closes the LED again, even if the snap
fails. The percentages are frozen at the start of the run, so changing settings
mid-run has no effect on it.

Saving uses the normal single-camera path: one file for the Orca, written off
the acquisition thread. The metadata records the LED, channel, name, percent
and CRISP Z state per frame under `extra.epi`, plus a run summary (the LEDs, the
CRISP Z label and state, the stage Z read once, read-only, and the percent
units). The run's LED map is also copied into the sequence metadata under
`pymmcore_gui.epi`. The percent is stored as "card native, uncalibrated". Argus
streaming is skipped in EPI.

Save as OME-Zarr (recommended) or image sequence to keep per-frame `extra.epi`;
OME-TIFF drops it. The TIFF writer stores no per-frame metadata, and the
sequence metadata and run summary do not reach the TIFF either, so the LED
percentages of a TIFF run are lost.

### OPM panels in EPI mode

The OPM panels stay visible, but anything that would drive the OPM cameras or
galvo refuses with a message and changes nothing:

- Camera ROI (the OPM crop) and Spectral Channels save;
- the galvo offset controls;
- OPM camera exposure and the `Cameras` group presets;
- an MDA that is not a valid EPI run (see above).

The Property Browser, the Console and the stage "set as default" action have no
guard. They are for experts: if they change Core Focus, Core Shutter or the
channel group, the app restores the old values when you leave EPI and logs it.

## Confirmed hardware

The TGLED card is at Tiger address 37. The ASI LED command letters are X for
channel 1, Y for channel 2, Z for channel 3 and F for channel 4 (see the ASI
docs for the [TGLED](https://docs.asiimaging.com/tgled) and its
[LED command](https://docs.asiimaging.com/commands/led)). These are the serial
command letters, not the axis letter in the Micro-Manager device label.

| Channel | ASI letter | Band | Preset   | Visible            |
| ------- | ---------- | ---- | -------- | ------------------ |
| 1       | X          | 488  | `LED488` | yes                |
| 2       | Y          | 561  | `LED561` | yes                |
| 3       | Z          | 642  | `LED642` | yes                |
| 4       | F          | 405  | `LED405` | no (not connected) |

Micro-Manager exposes the card as four shutter devices under ASITiger.
Intensity is set through `LED Intensity(%)`, on/off through the shutter
`State`. The ASI docs treat 0 % as off, but the app keeps 1 to 100 percent plus
a separate on/off and never writes 0.

The `State` property is a string, not a number (driver source `ASILED.cpp`).
Its only legal values are `Open` and `Closed`. `0` and `1` are rejected by the
driver with `Invalid property value: State`. The app never writes `State`
directly: it uses the shutter API (`setShutterOpen`), which sends the card
`LED <letter>=0` on close and `LED <letter>=<cached percent>` on open.
`LED Intensity(%)` is an integer limited to 1 to 100; writing it while the LED
is closed only updates the cached value and shows no light.

The `EPI LEDs` presets (`LED488`, `LED561`, `LED642`) all set the same single
property: `Physical Shutter 1` of the `EPI LED Selector`, a Micro-Manager
`Utilities` `Multi Shutter` device, to the label `LED:L:37:1`, `:2` or `:3`.
The selector is used only to record which LED a channel uses. Setting that
property only stores the label (driver source `MultiShutter.cpp`); it does no
device I/O, so a preset never lights or darkens an LED, whatever else is
running. The app maps preset to LED by preset name (settings `epi.leds`), then
opens the LED itself (percent first) for Snap, Live and each MDA frame. A
preset never sets Core Shutter, an LED `State`, `LED Intensity(%)` or any other
device, and channel 4 (405) has no preset. The app never opens
`EPI LED Selector`; do not open it by hand, because it would open the selected
LED at its cached intensity.

Why not one preset per LED (for example `LED:L:37:1,State,Closed`)? The Optical
Configs widget (`PresetsWidget`) requires every preset of a group to set the
same (device, property) and deletes rows that differ, so the second and third
presets were stripped when the toolbar was built. And with every LED closed all
three presets matched, so `getCurrentConfig` always returned the first one and
Snap and Live always lit LED 1. A shared selector property gives each preset a
distinct, readable value.

The Orca is a Hamamatsu C11440-22C (ORCA-Flash4.0 V2), S/N 100201, DCAM 4.00,
Camera Link. Defaults in `settings.epi.camera`:

| Setting        | Value                                        |
| -------------- | -------------------------------------------- |
| Adapter        | `HamamatsuHam` (`mmgr_dal_HamamatsuHam.dll`) |
| Device         | `HamamatsuHam_DCAM`                          |
| Label          | `Orca`                                       |
| Frame          | 2048 x 2048, `uint16`                        |
| Pre-init props | none                                         |
| Exposure       | standard `Exposure` property                 |
| Trigger        | camera default; the app never sets a trigger |

## Needs rig confirmation

Nothing in the app guesses these.

| Id | What is missing                                                       | Where it shows up              |
| -- | --------------------------------------------------------------------- | ------------------------------ |
| O2 | Channel count (`LED:L:37:1` to `4`)                                   | `LED:L:37:1` to `4` in the cfg |

O2 is mostly settled. The label letter `L` is confirmed by the rig: an
`EPI LEDs` preset error named `LED:L:37:1`. The `LED Intensity(%)` limits
(1 to 100), the `State` strings (`Open` / `Closed`) and the ASI letters X/Y/Z/F
per channel are confirmed from the driver source (`ASILED.cpp`). Still to
confirm on the rig: that all four channels (`LED:L:37:1` to `4`) load.

The app looks the card up by address 37 plus channel number with any axis
letter, so a different letter still works. Also to confirm on the rig:
`HamamatsuHam_DCAM` exposes `Exposure` under that name, and frames come back
2048 x 2048 `uint16`. For O2, close the app and run `scripts/discover_tgled.py`
(see the LED doc). The percent response of the LEDs is also unmeasured: the rig
checklist records it.

## Rig checklist

Do these in order on the rig. Steps 2 and 3 compare against each other, so
note the BNC1 trigger count, frame counts and zarr shape each time.

1. Start with the OPM profile (`20260630-OPM.cfg`): the app is in OPM, no Orca
   process exists, logs look as before. Toggle to EPI: refused with a message
   to load `20261118-OPM_EPI.cfg`.
2. OPM baseline: Live, Snap and a spectral MDA. Record the numbers above.
3. File > Load `20261118-OPM_EPI.cfg`. The log shows `LED:L:37:1..N` loaded and
   OPM load time is unchanged. Repeat step 2: the results must match.
4. Toggle to EPI and straight back without acquiring. CRISP Z and CRISP P
   states are the same before and after. Repeat step 2.
5. In EPI:
   - with the app closed, confirm the `LED:L:37:n` labels, the channel count
     and the intensity limits with `scripts/discover_tgled.py` (O2). The script
     must list `LED:X:31` as "other LED";
   - after loading the EPI cfg, the `EPI LEDs` combo in Optical Configs lists
     all 3 presets (`LED488`, `LED561`, `LED642`); switching between them
     changes `EPI LED Selector` / `Physical Shutter 1` (Property Browser) and
     lights no LED, and Snap lights the LED of the selected preset. Changing
     the preset during Live does not switch the lit LED until Live restarts;
   - confirm channel 1, 2 and 3 light 488, 561 and 642 at a low percent;
   - confirm the Orca loads as `HamamatsuHam_DCAM`, exposes `Exposure`, and
     returns 2048 x 2048 `uint16` on Snap and Live. Its trigger setting stays
     at the camera default (the app never changes it);
   - LED panel on and off; Orca Live and Snap; the exposure spinbox changes the
     Orca, not the OPM cameras;
   - for each channel, set 10, 50 and 100 percent and Snap: the image mean (or a
     power meter) should rise steadily. Record how linear it is;
   - with the OPM panels still open, Camera ROI apply and Spectral save are
     refused with a message, the galvo offset is refused, and the OPM cameras
     log nothing;
   - run an MDA with CRISP Z **unlocked**: it runs, with a warning in the log
     (it is not refused). An MDA with a Z plan, autofocus, a grid or two
     positions is refused;
   - lock CRISP Z in CRISPy and run a single-plane, 2-channel time series at
     different percentages. Save as OME-Zarr (recommended) or image sequence
     to keep per-frame `extra.epi`; OME-TIFF drops it. The saved file has the
     right frame count, `frame_meta` shows the right `extra.epi.led_pct` per
     channel and the CRISP Z state, and the engine sent no piezo or stage Z
     command.
6. Back to OPM: repeat step 2. CRISP states are as you left them and both
   joystick wheels work.
7. Restart the app: it starts in OPM, percentages are remembered, every LED is
   dark.
8. Close the app while in EPI: LEDs go off and the Orca process exits.
9. Reload a config while in EPI, once with File > Load and once with the Config
   Wizard. The mode is OPM, step 2 still passes, and the Orca process is still
   alive and reused on the next switch to EPI.
