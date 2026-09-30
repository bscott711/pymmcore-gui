# Changelog

All notable changes to this fork of
[pymmcore-gui](https://github.com/pymmcore-plus/pymmcore-gui) are documented in
this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/) (versions
come from git tags via `hatch-vcs`; there is no version field to edit).

This fork is based on upstream `pymmcore-gui` (upstream commits up to 2026-04-07)
and is **not published to PyPI**. It is deployed by cloning the repository. Entries
below describe what the fork adds on top of upstream. Numbers like `#12` are pull
requests on this fork
([bscott711/pymmcore-gui](https://github.com/bscott711/pymmcore-gui)), not on
upstream.

## [Unreleased]

## [0.1.0] - 2026-09-30

First tagged release of the fork.

### Added

#### Live laser control and full MDA integration
- Snap, Live and Multi-Dimensional Acquisition (MDA) work with lasers on the ASI
  SPIM rig. `ensure_global_shutter_open` raises the global shutter and sets up the
  always-on PLogic cell when a system configuration loads, so lasers actually fire
  during snap/live. Hardware profiles under `hardware_profiles/` were added or
  refined for the OPM rig (#1).
- ASI SPIM z-stack engine (`asi_z_stack/`), dual-camera support, and correct
  multi-camera routing for images and viewers (#1).
- ASI SPIM engines honor the MDA Z Stack mode (Top/Bottom, Above/Below, direction)
  in the galvo sweep, and stretch the galvo slice period to the exposure so long
  exposures no longer swallow slice triggers (#19).
- Shutter-gated lasers are held open for the whole SPIM burst instead of being
  pulsed per slice (#19).

#### Multi-ROI save and spectral channel splitter
- Multi-ROI save with a spectral-channel configuration widget (Plugins menu) and
  handler: each camera's sensor is cropped into image-splitter sub-regions and each
  region is saved to its own file (#2).
- Spectral regions are auto-selected from the MDA's lasers, and the format chosen
  in the MDA save widget is authoritative (#3).
- Local saving runs on its own bounded writer threads, off the MDA relay thread,
  so a slow disk cannot silently truncate a run; OME-Zarr chunk compression is off
  by default (`mda_writer.zarr_compression` turns it back on) (#19).

#### Argus streaming (real-time deskew/deconvolution)
Completed MDA volumes can be streamed, in addition to the normal local save, to a
receiver on the Argus GPU server for real-time processing. It is off by default
(`argus_stream.enabled`) and it never delays acquisition or the local save.
- Streaming client (`pymmcore_gui._argus_stream`): a ZeroMQ client that reaches the
  receiver through SSH port-forwards. It streams the same cropped spectral regions
  that are saved locally, and reads camera geometry from the run's summary
  metadata (#19). `SESSION_START` matches the receiver's `raw_root` contract, and a
  contract test keeps the two in step (#4).
- The SSH tunnel starts at app launch instead of on the first MDA run (#4).
- Blosc compression (lz4 + bitshuffle, about 2.5x on camera data) through the SSH
  tunnel, once the receiver advertises it; `compress_over_tunnel` turns it off (#10).
- Parallel SSH links: each run streams over `stream_links` connections (default 4)
  because one SSH connection is capped by sshd's channel window. Volumes are resent
  on the surviving links when a link drops or goes silent (5 s heartbeat), a
  receiver restart or idle timeout resumes the run, `SESSION_END` is resent until
  confirmed, and a hard RAM cap pauses streaming rather than stalling acquisition
  (the status bar then says to send the run by Globus). ssh prefers AES-GCM and
  runs at below-normal priority on Windows (#11, #12).
- `python -m pymmcore_gui._argus_stream.linkbench`: link throughput benchmark, used
  with a sink on the receiver side (#11).
- Optional direct 10 GbE link (`direct_endpoint`) with automatic fallback to the
  SSH tunnel after 3 s, and slab streaming (about 16 MB runs of planes sent while
  the volume is still being acquired). `SESSION_END` now lingers 2 s so it is not
  dropped when the socket closes (#9).
- Latency-trace timestamps on every `FRAME` (`acq_first_s`, `acq_last_s`,
  `queued_s`, `sent_s`, `clock_offset_s`) (#8).
- Per-run wire statistics in the log: bytes sent per link, bytes resent, link
  drops, RESUMEs and resyncs (#14).
- `output_format` setting (`both`, `ome-zarr` or `tiff`) chooses which format the
  processed result is kept in on the server (#19).
- `PREPARE` warm-up: when the MDA widget opens, and after each edit once the plan
  has been still for a second, the GUI tells the receiver the planned volume so a
  GPU server is warm before the first timepoint. The same plan is not re-sent
  within a minute (#16).

#### Argus streaming: reliability
- Resend everything unACKed at once when the receiver sends `resend` in an ACK
  (it dropped frames while short of staging space and has room again), instead of
  waiting for the stale-ACK timeout (#17).
- Retry a `SESSION_START` (or RESUME) the receiver never answers, with backoff from
  5 s to 30 s. A `rejected` reason from the receiver is shown in the status label.
  A finished run that gets no ACK for 10 minutes now pauses instead of holding its
  buffer until the app exits (#18).
- The ACK-staleness clock only runs while frames are in flight and its threshold
  grows with the bytes outstanding, which removes a spurious RESUME on the first
  volume of every run (#19).

#### Replay
- `python -m pymmcore_gui._argus_stream.replay <channel .ome.zarr>` streams a saved
  run through the real sender to a test receiver, without a microscope, and sends
  a SHA-1 of every volume so the copy can be checked bit for bit (#12).
- Replay is paced from the run's recorded frame times (`--pace`), reads ahead from
  disk (`--readahead`) and reports `read_stall_s` and `planes_late_s`; the z step
  comes from the run's recorded sequence (#15).
- Replay refuses a multi-channel store instead of misreading it as timepoints (#13).
- Replay reports the run's wire statistics together with its log (#14).

#### Live QC verdicts
- Every Argus session asks for the receiver's live quality-control verdicts (cell
  cut off by a face of the volume, drift, focus, bleaching). The newest verdict is
  shown in the status bar and the **Plugins > Argus QC** panel shows the verdict,
  advice tagged *now* or *next run*, the cell's margin on each face, and a strip of
  recent verdicts. It is advisory only; it never changes the acquisition (#5).

#### Deploy
- Pull-based deploy: `launch_gui.ps1` (started hidden from the desktop shortcut by
  `launch_gui.vbs`) runs `git pull --ff-only` before every launch and logs to
  `update.log`. It never blocks launching on a failed pull. This replaced a short-lived
  reverse-SSH-forward design (#6, reverted in #7). See "Deploying to the
  acquisition PC" in `CONTRIBUTING.md` (#7).

#### Camera and preview tooling
- PVCAM cameras run in per-camera worker processes during hardware-triggered MDA
  runs, to contain a native `pvcam64.dll` crash under concurrent dual-camera use
  (#3). Those workers are now persistent: one per camera, created at config load
  and shared by Live, Snap, MDA and the Camera ROI widget. Worker stderr is
  captured to the log (#19).
- Exposure (ms) spin box in the camera toolbar, routed to the camera workers (#19).
- Live preview is backed by a bounded in-RAM display store with a rolling time
  window instead of a per-camera tensorstore handler (#19).
- Dual-camera overlay alignment widget (**Plugins > Camera Alignment**) with a
  live displacement chart (#19).
- Splash screen during startup, and the large circular-buffer grow runs on a
  background thread (#19).
- CRISPy Autofocus panel (**Plugins** menu) (#1). CRISP bench tooling: a passive
  drift logger and piezo/lock-offset tuning helpers (#19), and a `focus` settings
  group with per-wavelength focus offsets (groundwork only; not yet applied by the
  acquisition engine) (#4).
- `microscope-control` entry point: loads the configured hardware profile and starts
  the GUI with the ASI SPIM engine registered.

### Fixed
- ASI SPIM hardware triggering: the global-shutter call no longer overwrites
  PLogic BNC1 camera-cell routing on every MDA run (#3).
- Shutdown: closing the window now stops acquisition and unloads devices, and
  background workers are quiesced first, which fixed a Windows BSOD after exit with
  a camera still streaming (#2, #3).
- ASI SPIM engines no longer move the piezo focus device mid-stack, and fire the
  correct laser per MDA channel (#19).
- A stalled camera worker can now be cancelled: the stall guard measures time
  since the last frame (#19).
- Streamed volumes mirror the local save directory structure under the receiver's
  raw root, so two sessions that reuse a sample name cannot collide (#19).

### Notes
- Fork-only settings live in the user settings file (`pmm_settings.json` in the
  app's user-data directory): `argus_stream`, `spectral`, `mda_writer` and `focus`.
  All new features are off, or harmless, by default.
- The former working branch `feat/mda-ux-perf-fixes` was landed on `main` in #19 and
  is frozen. `main` is the base branch for pull requests from here on.

[Unreleased]: https://github.com/bscott711/pymmcore-gui/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/bscott711/pymmcore-gui/releases/tag/v0.1.0
