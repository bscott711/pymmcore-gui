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
(it adds the `LED:L:37:<n>` devices and the `EPI LEDs` config group). Keep both
in sync: after editing the OPM profile, regenerate the EPI one. A test fails
when the two drift apart.

Without the EPI profile loaded, switching to EPI is refused with a message
telling you to load it.

### Fill in the Orca camera settings

The Orca is in no cfg file. The app starts its own camera process for it on the
first switch to EPI, then keeps it for the rest of the session. Its identity
comes from the app settings, section `epi.camera` of `pmm_settings.json` in the
app's user data folder. There is no settings dialog for it yet, so edit the
JSON with the app closed. Every field is empty until you fill it in, and EPI
entry is refused ("Set the Orca camera in settings") until all of the first six
are set:

| Field            | Meaning                                                                         |
| ---------------- | ------------------------------------------------------------------------------- |
| `label`          | Device label the camera is loaded under                                         |
| `adapter_module` | Micro-Manager device adapter (library) for the Orca                             |
| `device_name`    | Device name inside that adapter                                                 |
| `width`          | Frame width in pixels                                                           |
| `height`         | Frame height in pixels                                                          |
| `dtype`          | Numpy dtype of a frame, for example `uint16`                                    |
| `pre_init_props` | Optional list of `[property, value]` pairs set before the camera is initialised |

These values are not known yet (see [Needs rig or user input](#needs-rig-or-user-input)).

### Name the LED channels

Settings section `epi.leds` has one entry per card channel (1 to 4) with a
`name` (band, for example `488`), `visible` and the remembered `percent`. The
defaults are `488`, `561`, `642` and a hidden `405`. Which physical card channel
carries which band is a placeholder until it is checked on the rig, so set the
names to match the hardware.

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
toolbar. If an LED is left on from the LED panel, the group matches no preset
and Snap and Live ask you to select one first. The exposure widget and the
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

## Needs rig or user input

These are placeholders. Nothing in the app guesses them.

| Id | What is missing                                                              | Where it shows up                                      |
| -- | ---------------------------------------------------------------------------- | ------------------------------------------------------ |
| O1 | Which card channel carries 488, 561 and 642 nm                               | `epi.leds` names, `EPI LEDs` presets in the EPI cfg    |
| O2 | Exact LED labels, axis letter, channel count, `LED Intensity(%)` limits      | `LED:L:37:1` to `4` in the cfg                         |
| U1 | The Orca's adapter module, device name, label, geometry, pre-init properties | `epi.camera` in settings (also exposure property name) |

For O2, close the app and run `scripts/discover_tgled.py` (see the LED doc).
The percent response of the LEDs is also unmeasured: the rig checklist records
it.

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
   - confirm the LED labels, N, the axis letter and the intensity limits with
     `scripts/discover_tgled.py` or the load log, and note which channel is
     488, 561 and 642 (O1, O2). The script must list `LED:X:31` as "other LED";
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
