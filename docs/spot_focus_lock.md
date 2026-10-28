# Spot focus lock (CRISP LED through O2/O3, AmScope camera)

A fourth O3 focus-lock mode, next to CRISP-P, Image and Off. The CRISP LED (720 nm) is held on with CRISP in **Ready**. Its beam
passes O2/O3 and a pickoff dichroic sends it to an AmScope MD310C-BS (Sony IMX123 colour sensor, USB 2.0, ToupTek
OEM). A focus drift of O3 moves the spot along one fixed direction on that camera. The loop measures the spot and moves
piezo P (`PiezoStage:P:34`) to bring it back. The method follows Nguyen et al. 2025, Biomed. Opt. Express 16(3):1216.
Their code has no licence, so nothing here is copied from it.

Rules:
- The loop runs continuously from Start until Stop, on its own and not tied to an MDA. The MDA ignores P.
- Z-stacks are run on the Tiger by PLogic, so the serial line is quiet during a volume.
- **All camera work runs in a dedicated subprocess.** That covers the SDK, frames, spot measurement, the controller and
  fault checks. The subprocess never touches MMCore or the Tiger.
- The **main process** owns the Tiger port. It receives move requests and issues `mmc.setPosition(P, target)` with
  **no wait and no readback**. It never polls the Tiger.
- Moves are rare: a dead band, at most `max_rate_hz` moves per second (default 2), and averaging over several frames.
- On a fault (spot lost, saturated, lateral drift, range limit, worker died) the loop stops sending moves and reports.
  It never searches for the spot.
- No scipy. The code uses numpy only, plus the stdlib `multiprocessing` (spawn context, for Windows).

## Modules (`src/pymmcore_gui/asi_z_stack/`)

### `spot_focus.py`: pure numpy, no Qt, no MMCore
- `SpotFix` (frozen dataclass): `x, y` in full-frame pixels, `amplitude, background, sigma_x, sigma_y, peak: float`,
  `saturated: bool`, `ok: bool`, `reason: str` (empty when ok).
- `locate_spot(frame, *, roi_half=50, channel="sum", bayer="RGGB", saturation=None, min_amplitude=0.0) -> SpotFix`
  1. Takes raw 2-D frames: uint8 or uint16, mono or Bayer. `channel="red"` keeps only the red sites of the Bayer mosaic
     and works in full-frame coordinates; `"sum"` uses all pixels.
  2. Finds the peak after a small box blur and crops an ROI around it.
  3. Takes the ROI border median as the background.
  4. Fits a 1-D Gaussian to the row projection and to the column projection: moment initialisation, then a few
     Gauss-Newton or LM iterations in numpy. It falls back to the centroid if the fit diverges.
- `SpotCalibration` (dataclass, `to_dict`/`from_dict`):
  - `x0, y0`: the setpoint spot position.
  - `ux, uy`: unit vector of spot motion per +1 um of P.
  - `slope_px_per_um`, `residual_px`, `p0_um` (P at the setpoint), `n_points`, `created` (ISO time).
- `fit_spot_calibration(p_um, xs, ys, *, p0_um=None) -> SpotCalibration`
  - Least-squares line of (x, y) against p. The direction comes from the slope vector and `slope_px_per_um` is its norm.
  - The setpoint is the fitted position at `p0_um`, which defaults to the middle of the sweep.
  - Raises `ValueError` on fewer than 3 points or a slope under 0.2 px/um.
- `focus_error(fix, cal) -> tuple[float, float]`: `(along_um, lateral_px)`. `along_um` is the dot product of the
  displacement and u, divided by the slope; its sign is such that **moving P by −along_um returns the spot to the
  setpoint**. `lateral_px` is the perpendicular component.
- `SpotLoopParams` (dataclass, `to_dict`/`from_dict`):

  | field | default | field | default |
  |---|---|---|---|
  | `kp` | 0.6 | `lateral_limit_px` | 8.0 |
  | `ki` | 0.05 | `min_amplitude` | 20.0 |
  | `deadband_um` | 0.03 | `p_min_um` | 0.0 |
  | `max_step_um` | 0.5 | `p_max_um` | 150.0 |
  | `max_rate_hz` | 2.0 | `max_excursion_um` | 5.0 (from start P) |
  | `avg_frames` | 8 | `strikes` | 5 (consecutive bad measurements before fault) |
- `SpotFocusController(params, p_start_um)`
  - `update(along_um, now_s) -> float | None` returns the new target P, or None for no move.
  - Proportional-integral control on the averaged error, with the dead band (inside it, no move and the integral is
    held), step clamp and rate cap.
  - If the target would leave [p_min, p_max] or the excursion limit, it sets `fault` instead of moving.
  - `bad(reason)` counts a strike; `good()` resets the count.
  - Properties: `p_cmd_um` (last commanded target), `fault: str | None`, `n_moves`.

### `toupcam_source.py`: frame sources (used inside the worker)
- `FrameSource` Protocol: `open()`, `close()`, `latest(timeout_s) -> tuple[np.ndarray, float] | None` (newest raw frame and
  a monotonic timestamp, dropping older ones), `set_exposure_ms(float)`, `set_gain(int)`, `set_roi(x, y, w, h)`,
  `info() -> dict`, `set_actuator_um(p)` (no-op for real cameras; used by the fake).
- `ToupcamSource(serial=None, sdk_dir=None)`
  - Loads the vendor's ctypes wrapper (`toupcam.py`, or `amcam.py` as an alias) from `sdk_dir`, or the env var
    `TOUPCAM_SDK_DIR`, at `open()` time, and raises a clear error if it is missing. The SDK is never bundled.
  - Puts the camera in raw mode, with auto-exposure off and pull mode. The SDK callback sets an event and the newest
    frame is copied into a 1-slot buffer.
- `FakeSpotSource(cal_like: dict, drift_um_per_s=0.0, noise=2.0, amplitude=800.0, sigma_px=6.0, shape=(480, 640), lateral_px_per_s=0.0, seed=0, fps=50.0)`
  - Renders a Gaussian spot at `x0 + ux*slope*(drift(t) + p - p0)` and the matching y, plus Poisson or Gaussian noise.
  - `set_actuator_um(p)` sets p. Drift accrues from `open()`.
  - Supports a `lateral_px_per_s` drift for the negative control.
  - Built from a plain dict spec so it can be constructed inside a spawned process.
- `make_source(spec: dict) -> FrameSource`: `{"kind": "toupcam", ...}` or `{"kind": "fake", ...}`.

### `spot_focus_worker.py`: subprocess entry
- `worker_main(conn, source_spec: dict)` is a module-level function so spawn can pickle it. It uses a `multiprocessing`
  Pipe; every message is a dict with a `"type"`.
- Main to worker:

  | type | payload |
  |---|---|
  | `start` | `params`, `calibration`, `p_start_um` |
  | `stop` | |
  | `update_params` | `params` |
  | `set_camera` | `exposure_ms`, `gain`, `roi` |
  | `measure` | `n_frames`, `p_um`; replies `measurement` with the mean and SD of x, y and the amplitude |
  | `moved` | `seq`, `ok`, `dt_ms` |
  | `shutdown` | |

- Worker to main:

  | type | payload |
  |---|---|
  | `ready` | `info` |
  | `move` | `seq`, `target_um`, `along_um` |
  | `telemetry` | ~10 Hz: `t, x, y, along_um, lateral_px, amplitude, fps, p_cmd_um, n_moves` |
  | `thumb` | ~5 Hz: uint8 ROI, at most 128x128 |
  | `fault` | `reason` |
  | `status` | `state` (idle, running, stopped, fault) |
  | `error` | `message`, `traceback` |

- The worker tells its source the commanded P (`set_actuator_um`) when it sends a move, so the fake closes the loop.

### `spot_focus_service.py`: main process, Qt-free (psygnal signals)
- `SpotFocusService(mmc, piezo_label="PiezoStage:P:34", crisp_label=None, source_spec=...)`
  - Starts the worker process (spawn context, daemon) and a reader thread.
  - On `move` it calls `mmc.setPosition(piezo_label, target)` **without** `waitForDevice`, times the call, and replies
    `moved`.
  - It drops a move if the previous one is still in flight, so moves are never queued.
- Signals: `telemetry(dict)`, `thumb(ndarray)`, `fault(str)`, `state(str)`, `moved(dict)`.
- `start(params, calibration)`
  - Reads P once with `getPosition`, calls `led_on(owner="lock")` (CRISP to Ready, previous state remembered), then sends
    `start`.
- `stop()` sends `stop` and lets go of the lock's claim on the LED. It never moves P.
- CRISP LED (the LED is lit only in CRISP State Ready / Lock / Dither..., off in Idle, the power-up state):
  - `led_on(intensity=None, owner="manual")` remembers the previous CRISP state once, sets Ready unless CRISP is already
    in a lit state, and sets `"LED Intensity"` (default: the constructor's `led_intensity`).
  - `led_off(owner)` drops that owner's claim. Owners are "preview", "lock", "manual" and "calibrate"; the previous state
    is restored when the last one is gone, so stopping the lock with Preview on keeps the LED lit.
  - `led_is_on`, `crisp_state()`, `crisp_label`, `set_led_intensity(v)` (live). With no CRISP label all of it is a no-op
    and `led_is_on` stays False.
- `shutdown()` and context manager support.
- If the worker process dies, the service emits `fault("worker died")`, the state becomes fault, and no further moves
  are sent.
- `calibrate(span_um=2.5, steps=11, n_frames=20) -> SpotCalibration` runs only while the loop is stopped: step P with
  `waitForDevice` (fine here), `measure` each step, call `fit_spot_calibration`, then return P to the start.
- `stats()` returns `n_moves`, `mean_dt_ms`, `max_dt_ms` and `moves_per_min`.

## Tests (`tests/`, run with `--noconftest`, no hardware, no MM install)
- `test_spot_focus.py`:
  - Synthetic spots at known sub-pixel positions, plus the Bayer red channel.
  - Calibration round trip and the sign convention.
  - Dead band, step clamp, rate cap, range and excursion faults.
  - **Negative control:** a purely lateral shift gives `along_um` of about 0 (no move) and lateral faults past the limit.
  - With the sign flipped, the controller diverges.
- `test_spot_focus_worker.py` uses a real spawned worker with `FakeSpotSource` and a fake mmc (records `setPosition`
  calls and never waits). Cases:
  - Under drift, closed-loop convergence holds the error within the dead band.
  - **Command rate:** under slow drift, the number of moves per minute stays at or under the cap.
  - Killing the worker gives a fault and no more moves.
  - Calibration recovers the fake's slope and direction.
  - `stop` restores the CRISP state.

## Live camera view and controls (2026-10-27)

The user needs to see the camera **live at full resolution** to align the beam, place the ROI and set exposure. No
downsampled view: images are drawn at native resolution, with nearest-neighbour pixels when zoomed.

Rig camera: ICMOS03100KPB (MD310C-BS), 2048x1536, **RAW8 only**, RGGB. 11.5 fps at full frame, 31 fps at a 640x480 ROI.
Exposure runs 0.105 ms to 15 s, gain 100 to 5000. After a settings change the camera delivers stale frames for up to 3-4
frames.

### Frames: shared memory, never the pipe (`asi_z_stack/spot_frame_shm.py`)
- `FrameBuffer.create(max_bytes) -> FrameBuffer` is the writer, in the worker.
- `FrameBuffer.attach(name) -> FrameBuffer` is the reader, in the main process. It uses `track=False`.
- `.name`, `.close()`, `.unlink()`.
- `write(image, *, t, roi, sensor, bits, spot, spot_ok, saturated_frac, max_value)`. A seqlock header (sequence number odd
  while writing) means a reader never sees a torn frame.
- `read(last_seq=None) -> LiveFrame | None` returns None when there's nothing newer than `last_seq`. It copies the
  image out.
- `LiveFrame` (dataclass) fields:
  - `image`: 2-D uint8 or uint16, ROI-sized
  - `seq`, `t`
  - `roi`: `(x, y, w, h)` in sensor px
  - `sensor`: `(W, H)`
  - `bits`
  - `spot`: `(x, y)` sensor px, or None
  - `spot_ok`, `saturated_frac`, `max_value`

### Worker
- States: `idle`, `preview`, `running`, `stopped`, `fault`. New messages: `preview_start` and `preview_stop`.
- Every frame goes to the shared buffer while previewing, running, or measuring for calibration.
  - The spot is fitted for the overlay at most 5 Hz in preview.
  - While running, it is fitted every frame, as before.
- `thumb` messages are dropped; the live view replaces them.
- **All spot positions are in sensor pixels**: the ROI offset is added. That covers fixes, the calibration, telemetry,
  measurements and the setpoint. A calibration therefore survives ROI changes, as long as the spot stays inside.
- `set_camera` applies live:
  - `exposure_ms`, `gain`, `channel` (`sum` or `red`)
  - `roi`: `(x, y, w, h)`; `(0, 0, 0, 0)` means full frame
- **Settle is time-based.** After a move ack or any camera change, frames are ignored until both of these hold: at least
  4 frames have arrived, and `max(2*exposure + 1/fps, 0.05)` s has passed. This replaces `SETTLE_FRAMES = 2`.
- The `ready` info carries: `shm`, `sensor`, `bits`, `model`, `serial`, `exposure_range_ms`, `gain_range`, `roi`,
  `exposure_ms`, `gain`.

### Service
- `start_preview()` and `stop_preview()`; `preview_active`.
- `latest_frame(last_seq=None) -> LiveFrame | None`.
- `camera_info`: the ready info.
- `set_camera(exposure_ms=None, gain=None, roi=None, channel=None)`.
- `stop()` returns to `preview` if preview was on.
- Calibrate works while previewing.

### GUI
- **Camera section** of the Spot panel:
  - SDK folder: a line edit plus Browse, saved to `spot_focus.source.sdk_dir`.
  - A camera info label: model, serial, sensor, bits.
  - A **Preview** toggle. It lights the CRISP LED (CRISP State Ready) and turning it off restores CRISP, unless the lock is
    running.
  - An **LED** toggle and a live **LED %** spin (saved, default 50), next to a readout of CRISP's state and device label,
    read once after each change. With no CRISP device the fault label says "no CRISP device (focus.crisp_label)".
  - Exposure (ms), gain and channel. Each applies live and is saved.
  - ROI x, y, w, h, with **Apply ROI** and **Full frame**.
  - Readouts: fps, max value, % saturated.
  - Camera errors show in the panel's fault label.
- **Live view** (`widgets/_spot_live_view.py`), a separate resizable top-level window titled "Spot camera":
  - It is a QGraphicsView whose scene is the sensor. Each frame is drawn at native resolution at its ROI offset.
  - Wheel zooms around the cursor, left-drag pans, and there are **Fit** and
    **1:1** buttons. Pixels are nearest-neighbour at 100% and above; below
    100% the view resamples smoothly (bilinear).
  - **Bayer to grey** (default on): the camera is a colour sensor read as raw
    Bayer, so for display only each pixel is replaced by the
    `[1, 2, 1] x [1, 2, 1] / 16` average of its 3x3 neighbourhood, which
    cancels the 2x2 colour mosaic (otherwise it aliases into a Moire grid at
    fractional zoom). The pixel readout and spot detection use the raw
    values. Turn it off for a mono camera or to inspect the raw mosaic.
  - **Draw ROI** mode: dragging a rectangle (snapped to even ints, clamped to the sensor) emits
    `roiDrawn(x, y, w, h)`. The panel fills its ROI spins and applies them.
  - Overlays: the sensor outline, the current ROI, the spot (a cross) and the calibrated setpoint (a circle).
  - Levels are linear. The fixed range is 0 to 2^bits−1, with an **Auto levels** option (min to max).
  - Under the cursor it shows x, y and the pixel value.
  - A 30 Hz timer polls `latest_frame` and repaints only on a new `seq`.
  - Closing the window stops preview, unless the lock is running.
