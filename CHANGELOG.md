# Changelog

All notable changes to this fork of
[pymmcore-gui](https://github.com/pymmcore-plus/pymmcore-gui) are documented in
this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/) (versions
come from git tags via `hatch-vcs`; there is no version field to edit).

This fork is based on upstream `pymmcore-gui` (upstream commits up to
2026-04-07) and is **not published to PyPI**. It is deployed by cloning the
repository. Entries below describe what the fork adds on top of upstream.
Numbers like `#12` are pull requests on this fork
([bscott711/pymmcore-gui](https://github.com/bscott711/pymmcore-gui)), not on
upstream.

## [Unreleased]

### Added

- **`probe_galvo_park`** (bench diagnostic in `asi_z_stack/diagnostics.py`): tries
  parking the galvo at the sweep start between volumes, via `SPIMState=Armed`
  or an Idle offset write, so the mechanism can be chosen on the rig before
  any engine change.
- **Galvo control in Stage Control.** A group per galvo device (and the ASI
  `Scanner:AB:33` if it is loaded) sets the Live offset
  `SingleAxisYOffset(deg)`, the reference the Z stack sweeps around. The
  offset is shown in um-equivalent with the degrees beside it, with a step
  size, -/+ buttons and "Go to 0". Writes are clamped to the card's property
  limits, the display follows the card, and the group is locked while an MDA
  runs.

## [0.2.0] - 2026-10-30

### Breaking changes

- The **Spot Focus Lock** dock and its Plugins menu entry are gone. The spot
  edge lock now lives in CRISPy 0.2.0 and is the "Spot lock" section of the
  piezo CRISP panel; the Spot camera view opens from that panel. The
  `spot_focus` settings group is unchanged (same name, shape and defaults).
- The row-shift **image focus lock** (added in #50 to #52 after 0.1.0) is
  removed again: the focus box on the MDA viewer, the lock inside the SPIM
  engine, its calibration sweep and the `image_focus` settings group. The spot
  edge lock is the only O3 focus lock. A settings file that still has an
  `image_focus` section loads fine; the section is ignored (one "not found in
  model" warning) and dropped the next time settings are saved. A saved
  `pymmcore_gui.image_focus_widget` entry in the open-widgets list shows one
  "Unable to reload widget" notice on the next start and does nothing else.
- **CRISPy >= 0.2.0 is required** (sibling checkout `../CRISPy`, as before). It
  now carries the spot lock, so pymmcore-gui no longer ships any spot code.
- Reloading the hardware configuration rebuilds the CRISP panel, which now
  stops a running spot lock (the old separate dock did not listen to config
  loads). Start the lock again after a reload.

### Added

#### Spot edge focus lock (now in CRISPy)

- Spot focus lock for the O3 piezo, driven by the AmScope/Toupcam camera that
  sees the CRISP LED spot (#53): core, camera worker subprocess and piezo
  service, settings, and a panel with Start/Stop, calibration, live spot and a
  strip chart.
- The lock follows the spot's **edge** rather than its centre: edge-based error
  signal (Spot is the only O3 focus mode), an optional second edge box fused
  into one focus error, a selectable edge detector (Gaussian derivative by
  default), a 1 um limit on how far the two edges may disagree, and a hold on
  the edge at its target.
- Spot camera live view (#57): shared-memory frames, a Preview window, live
  exposure/gain/channel/ROI controls, LED control in the panel (Preview lights
  the CRISP LED, #58), coarse block-sum peak search for large frames, and
  Bayer moire removal in the camera window. The view can be docked in the
  panel.
- The panel and its settings models (`SpotFocusSettingsV1`) moved to CRISPy;
  pymmcore-gui only supplies a small settings adapter to the CRISPy piezo panel
  (settings, CRISP and piezo labels, data directory, save).

#### Image focus lock (added, then removed in this release)

- Per-volume piezo correction from the row shift of a drawn box, with a
  calibration sweep, a focus panel and a box that follows the viewer's slice and
  channel (#50, #51, #52). Superseded by the spot lock and removed; see
  Breaking changes.

#### CRISPy focus curve

- CRISPy's focus curve snaps through the toolbar's Snap path, and its images
  show in the Preview windows' widget.

#### ASI SPIM timing and acquisition

- Advanced SPIM slice timing panel and the Kinetix **All Rows** mode: the laser
  is delayed so the cameras stay in Level Trigger; edits in the timing panel
  apply at once, and Legacy mode sends exactly the proven PLogic and galvo setup
  and per-slice timing. The galvo's `SPIMCameraDuration` follows the exposure,
  and the sweep amplitude (not just the offset) is restored after an MDA.
- Cameras crop to the bounding box of the spectral regions; spectral channel
  names are editable with an nm wavelength suffix.
- Each MDA logs the timing and drift evidence it ran with (per-volume piezo and
  CRISP state, drift summary).
- Ctrl+S saves the newest Snap/Live frame of each camera. The stage panel no
  longer offers "Zero" on piezo stages, and the Z stage's axis polarity is set
  at startup.

#### Argus backfill sender

- `opym-backfill-send`: an SSH backfill sender shipped with the app (#26). It
  sends each run once and says what it cannot send (#27), never walks the
  recycle bin or system folders (#28), scans without reading chunk files and
  shows progress (#30), uses as many tunnels as the live stream (4, #32),
  confirms single volumes and completes partial copies held elsewhere (#35),
  and probes hashes from one plane instead of a whole volume (#37).
- The sender records its plan and results, can resume and retry, and follows
  renamed runs (#39). It checks stream/Globus copies against the PC before they
  count as sent (#41), drops a dead tunnel instead of waiting on it and keeps
  the PC usable (#43), and reads and hashes volumes side by side with 8 files in
  flight (#47, #48). Argus paths stay POSIX when a copy is held elsewhere (#49).

### Fixed

- Streaming: a new run closes a run that never got its finished signal (#24),
  and a stream never goes into the void silently (#25). The backfill metadata
  guard no longer refuses good stores (#45).
- Live view shows the newest frame instead of draining the worker buffer in
  order (#21); a locked viewer slice advances `t` when frames arrive in bursts,
  holds slice and channel, and keeps the full z slider range.
- Spot lock: no fault on a negative piezo position, CRISP is restored on
  shutdown even before the worker reports running, the worker never blocks on
  output (Windows pipe), and Toupcam opens RAW8-only cameras and releases the
  handle on failure.
- Only pass slot priority for psygnal signals.

### Notes

- The version comes from the `v0.2.0` tag (`hatch-vcs`); there is no version
  field to edit. `uv.lock` pins `crispy` 0.2.0, which adds `psygnal` and
  `pydantic` as dependencies.
- The rig ran `feat/spot-only-focus`, which this release contains; main and
  `feat/mda-live-parity` are ancestors of it, so the rig can switch to `main`
  or fast-forward the branch it tracks.
- Rig pull order: pull `../CRISPy` (>= 0.2.0) first, then pymmcore-gui. The
  launcher only pulls the pymmcore-gui checkout, and `pyproject.toml` now
  requires `crispy>=0.2.0`, so an old CRISPy fails to resolve.

## [0.1.0] - 2026-09-30

First tagged release of the fork.

### Added

#### Live laser control and full MDA integration

- Snap, Live and Multi-Dimensional Acquisition (MDA) work with lasers on the ASI
  SPIM rig. `ensure_global_shutter_open` raises the global shutter and sets up
  the always-on PLogic cell when a system configuration loads, so lasers
  actually fire during snap/live. Hardware profiles under `hardware_profiles/`
  were added or refined for the OPM rig (#1).
- ASI SPIM z-stack engine (`asi_z_stack/`), dual-camera support, and correct
  multi-camera routing for images and viewers (#1).
- ASI SPIM engines honor the MDA Z Stack mode (Top/Bottom, Above/Below,
  direction) in the galvo sweep, and stretch the galvo slice period to the
  exposure (at long exposures the fixed period fired slice triggers faster than
  the camera could take them) (#19).
- Shutter-gated lasers are held open for the whole SPIM burst instead of being
  pulsed per slice (#19).

#### Multi-ROI save and spectral channel splitter

- Multi-ROI save with a spectral-channel configuration widget (Plugins menu) and
  handler: each camera's sensor is cropped into image-splitter sub-regions and
  each region is saved to its own file (#2).
- Spectral regions are auto-selected from the MDA's lasers, and the format
  chosen in the MDA save widget is authoritative (#3).
- Local saving runs on its own bounded writer threads, off the MDA relay thread,
  so a slow disk cannot silently truncate a run; OME-Zarr chunk compression is
  off by default (`mda_writer.zarr_compression` turns it back on) (#19).

#### Argus streaming (real-time deskew/deconvolution)

Completed MDA volumes can be streamed, in addition to the normal local save, to
a receiver on the Argus GPU server for real-time processing. It is off by
default (`argus_stream.enabled`) and it never delays acquisition or the local
save.

- Streaming client (`pymmcore_gui._argus_stream`): a ZeroMQ client that reaches
  the receiver through SSH port-forwards. It streams the same cropped spectral
  regions that are saved locally, and reads camera geometry from the run's
  summary metadata (#19). `SESSION_START` matches the receiver's `raw_root`
  contract, and a contract test keeps the two in step (#4).
- The SSH tunnel starts at app launch instead of on the first MDA run (#4).
- Blosc compression (lz4 + bitshuffle, about 2.5x on camera data) through the
  SSH tunnel, once the receiver advertises it; `compress_over_tunnel` turns it
  off (#10).
- Parallel SSH links: each run streams over `stream_links` connections (default
  4) because one SSH connection is capped by sshd's channel window. Volumes are
  resent on the surviving links when a link drops or goes silent (5 s
  heartbeat), a receiver restart or idle timeout resumes the run, `SESSION_END`
  is resent until confirmed, and a hard RAM cap pauses streaming rather than
  stalling acquisition (the status bar then says to send the run by Globus). ssh
  prefers AES-GCM and runs at below-normal priority on Windows (#11, #12).
- `python -m pymmcore_gui._argus_stream.linkbench`: link throughput benchmark,
  used with a sink on the receiver side (#11).
- Optional direct 10 GbE link (`direct_endpoint`) with automatic fallback to the
  SSH tunnel after 3 s, and slab streaming (about 16 MB runs of planes sent
  while the volume is still being acquired). `SESSION_END` now lingers 2 s so it
  is not dropped when the socket closes (#9).
- Latency-trace timestamps on every `FRAME` (`acq_first_s`, `acq_last_s`,
  `queued_s`, `sent_s`, `clock_offset_s`) (#8).
- Per-run wire statistics in the log: bytes sent per link, bytes resent, link
  drops, RESUMEs and resyncs (#14).
- `output_format` setting (`both`, `ome-zarr` or `tiff`) chooses which format
  the processed result is kept in on the server (#19).
- `PREPARE` warm-up: when the MDA widget opens, and after each edit once the
  plan has been still for a second, the GUI tells the receiver the planned
  volume so a GPU server is warm before the first timepoint. The same plan is
  not re-sent within a minute (#16).

#### Argus streaming: reliability

- Resend everything unACKed at once when the receiver sends `resend` in an ACK
  (it dropped frames while short of staging space and has room again), instead
  of waiting for the stale-ACK timeout (#17).
- Retry a `SESSION_START` (or RESUME) the receiver never answers, with backoff
  from 5 s to 30 s. A `rejected` reason from the receiver is shown in the status
  label. A finished run that gets no ACK for 10 minutes now pauses instead of
  holding its buffer until the app exits (#18).
- The ACK-staleness clock only runs while frames are in flight and its threshold
  grows with the bytes outstanding, which removes a spurious RESUME on the first
  volume of every run (#19).

#### Replay

- `python -m pymmcore_gui._argus_stream.replay <channel .ome.zarr>` streams a
  saved run through the real sender to a test receiver, without a microscope,
  and sends a SHA-1 of every volume so the copy can be checked bit for bit
  (#12).
- Replay is paced from the run's recorded frame times (`--pace`), reads ahead
  from disk (`--readahead`) and reports `read_stall_s` and `planes_late_s`; the
  z step comes from the run's recorded sequence (#15).
- Replay refuses a multi-channel store instead of misreading it as timepoints
  (#13).
- Replay reports the run's wire statistics together with its log (#14).

#### Live QC verdicts

- Every Argus session asks for the receiver's live quality-control verdicts
  (cell cut off by a face of the volume, drift, focus, bleaching). The newest
  verdict is shown in the status bar and the **Plugins > Argus QC** panel shows
  the verdict, advice tagged *now* or *next run*, the cell's margin on each
  face, and a strip of recent verdicts. It is advisory only; it never changes
  the acquisition (#5).

#### Deploy

- Pull-based deploy: `launch_gui.ps1` (started hidden from the desktop shortcut
  by `launch_gui.vbs`) runs `git pull --ff-only` before every launch and logs to
  `update.log`. It never blocks launching on a failed pull. This replaced a
  short-lived reverse-SSH-forward design (#6, reverted in #7). See "Deploying to
  the acquisition PC" in `CONTRIBUTING.md` (#7).

#### Camera and preview tooling

- PVCAM cameras run in per-camera worker processes during hardware-triggered MDA
  runs, to contain a native `pvcam64.dll` crash under concurrent dual-camera use
  (#3). Those workers are now persistent: one per camera, created at config load
  and shared by Live, Snap, MDA and the Camera ROI widget. Worker stderr is
  captured to the log (#19).
- Exposure (ms) spin box in the camera toolbar, routed to the camera workers
  (#19).
- Live preview is backed by a bounded in-RAM display store with a rolling time
  window instead of a per-camera tensorstore handler (#19).
- Dual-camera overlay alignment widget (**Plugins > Camera Alignment**) with a
  live displacement chart (#19).
- Splash screen during startup, and the large circular-buffer grow runs on a
  background thread (#19).
- CRISPy Autofocus panel (**Plugins** menu) (#1). CRISP bench tooling: a passive
  drift logger and piezo/lock-offset tuning helpers (#19), and a `focus`
  settings group with per-wavelength focus offsets (groundwork only; not yet
  applied by the acquisition engine) (#4).
- `microscope-control` entry point: loads the configured hardware profile and
  starts the GUI with the ASI SPIM engine registered.

### Fixed

- ASI SPIM hardware triggering: the global-shutter call no longer overwrites
  PLogic BNC1 camera-cell routing on every MDA run (#3).
- Shutdown: closing the window now stops acquisition and unloads devices, and
  background workers are quiesced first, which fixed a Windows BSOD after exit
  with a camera still streaming (#2, #3).
- ASI SPIM engines no longer move the piezo focus device mid-stack, and fire the
  correct laser per MDA channel (#19).
- A stalled camera worker can now be cancelled: the stall guard measures time
  since the last frame (#19).
- Streamed volumes mirror the local save directory structure under the
  receiver's raw root, so two sessions that reuse a sample name cannot collide
  (#19).

### Notes

- Fork-only settings live in the user settings file (`pmm_settings.json` in the
  app's user-data directory): `argus_stream`, `spectral`, `mda_writer` and
  `focus`. Streaming (`argus_stream.enabled`) and spectral cropping
  (`spectral.enabled`) are off by default.
- The former working branch `feat/mda-ux-perf-fixes` was landed on `main` in #19
  and is frozen. `main` is the base branch for pull requests from here on.

[Unreleased]: https://github.com/bscott711/pymmcore-gui/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/bscott711/pymmcore-gui/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/bscott711/pymmcore-gui/releases/tag/v0.1.0
