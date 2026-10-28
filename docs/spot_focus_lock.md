# Spot focus lock (CRISP LED through O2/O3, AmScope camera)

Spot is the **only** O3 focus-lock mode. The old CRISP-P / Image / Off
selector is gone from the UI (the "Image Focus Lock" action now opens the Spot
panel directly, titled "Spot Focus Lock"). The engine's image-lock code is
still present but inert: it only acts when the settings mode is `"image"`, and
the mode is now fixed to `"spot"`.

The CRISP LED (720 nm) is held on with CRISP in **Ready**. Its beam passes
O2/O3 and a pickoff dichroic sends it to an AmScope MD310C-BS (Sony IMX123
colour sensor, USB 2.0, ToupTek OEM). On that camera the LED shows up as a
bright, roughly square **patch**. A focus drift of O3 moves the patch along one
fixed direction. The loop measures the position of one straight **edge** of the
patch and moves piezo P (`PiezoStage:P:34`) to bring the edge back to where it
was when the sample was in focus. The method follows Nguyen et al. 2025,
Biomed. Opt. Express 16(3):1216. Their code has no licence, so nothing here is
copied from it.

> **Rig-unverified.** No real LED-patch frame, no P sweep and no saved spot
> calibration existed when this was written. All numbers below (patch size,
> ghost position, tail level, noise) come from one screenshot. The synthetic
> renderer (`spot_synthetic.py`) and `FakeSpotSource` were modelled on that
> screenshot, so every test passes against a model, not against the rig. Work
> through the rig test plan at the end before trusting the lock.

## Rules

- The loop runs continuously from Lock until Stop, on its own and not tied to
  an MDA. The MDA ignores P.
- Z-stacks are run on the Tiger by PLogic, so the serial line is quiet during
  a volume.
- **All camera work runs in a dedicated subprocess.** That covers the SDK,
  frames, the edge measurement, the controller and fault checks. The
  subprocess never touches MMCore or the Tiger.
- The **main process** owns the Tiger port. While the lock runs it receives
  move requests and issues `mmc.setPosition(P, target)` with **no wait and no
  readback**. It never polls the Tiger.
- Moves are rare: a dead band, at most `max_rate_hz` moves per second
  (default 2), and averaging over several frames.
- On a fault (edge lost, saturated, edge rotated or not straight, range limit,
  worker died) the loop stops sending moves and reports. It never searches for
  the edge and never moves P back.
- No scipy. The code uses numpy only, plus the stdlib `multiprocessing` (spawn
  context, for Windows).
- **Calibration exception.** `SpotFocusService.calibrate` is the one place that
  uses `waitForDevice` (which polls the Tiger). It is allowed only with the
  lock stopped and no MDA running (`mmc.mda.is_running()` is false); otherwise
  it refuses. The "never poll the Tiger" rule exists for the running lock and
  for volumes. A calibration is a short, user-started, bounded procedure on an
  otherwise idle serial line. `zero` and `measure` do not wait on the Tiger.

## Metric: a straight edge of the LED patch

The metric is one signed number: the position, in sensor pixels, of a straight
edge of the patch that moves with focus. The user draws a small **edge box**
across that edge. Inside the box the code finds, row by row, where the
brightness crosses half way between the dark outside and the bright inside,
fits a straight line through the crossings, and reports where the line crosses
the box centre. No search, sub-pixel, and the sign tells which way focus moved.

`measure_edge(frame, box, ...)` takes a raw 2-D frame (uint8 or uint16, RGGB
Bayer or mono), its sensor offset `roi_offset=(rx, ry)` and the box
`(x, y, w, h)` in sensor pixels:

1. Snap the box to even sensor coordinates (minimum 16 x 16). If it is not
   fully inside the frame's sensor rectangle the fix is not ok, reason
   `"edge box outside the camera ROI"`.
2. Crop and make a half-resolution grey image aligned to the 2x2 Bayer cell:
   `channel="sum"` is the mean of each cell, `channel="red"` is the red site
   of each cell. Values stay in raw counts. This removes the colour mosaic,
   which would otherwise bias the crossings. The Bayer pattern is anchored to
   the sensor, not to the ROI start, so an odd ROI offset gives the same
   result as the full frame. For `"red"` the binned pixel centre is the red
   site itself; for `"sum"` it is the cell centre.
3. If more than `max_saturated_frac` (default 0.002) of the raw crop is
   saturated, not ok: `"saturated"`.
4. Axis and polarity: `axis="x"` (normal along x, a vertical-ish edge) when the
   mean |d/dx| is at least the mean |d/dy|, else `"y"`. `polarity=+1` when the
   bright side is at larger coordinates along the normal, else -1. Calibration
   stores both; later measurements reuse them.
5. Arrange rows along the edge and columns along the normal. Smooth each row
   with `[1, 2, 1] / 4` along the normal.
6. Levels: `low` is the median of the outer 15 % of columns on the dark side,
   `high` the same on the bright side. If `contrast = high - low` is below
   `min_contrast` (default 10 counts), not ok: `"edge lost (contrast c < min)"`.
7. Threshold `low + 0.5 * contrast`. Per row, scan from the dark side to the
   first column at or above the threshold and interpolate linearly with the
   previous column. Rows without a crossing are invalid; fewer than
   `min_valid_fraction` (default 0.5) valid rows gives
   `"edge lost (no crossing in most rows)"`.
8. Robust line fit `c(r) = a + b (r - r_c)` through the valid crossings: three
   rounds of least squares, keeping points within `max(3 * 1.4826 * MAD, 0.5)`
   binned pixels. `a` is the edge position at the box centre,
   `angle_deg = degrees(atan(b))`, `residual_px` is the inlier RMS in sensor
   pixels.
9. If `a` is inside the outer 15 % bands the edge has walked to the box
   border: not ok, `"edge left the box"`.
10. The result is an `EdgeFix`: `position` (sensor px along the normal), the
    overlay point `x, y` (the edge point at the box centre), `axis`,
    `polarity`, `contrast`, `low`, `high`, `angle_deg`, `residual_px`,
    `valid_fraction`, `saturated_frac`, `peak`, `ok`, `reason`. A failed fix
    still carries finite numbers (position and x, y default to the box
    centre).

### Why it is robust

- **Faint tail (about 2-5 % of the max).** Far below the 50 % threshold, and
  the line fit rejects outliers.
- **Ghost (dim copy below the patch).** It sits about 390 px below the patch
  with a gap of about 130 px, outside a box drawn on the patch. Keep it out.
- **Texture inside the patch.** It moves with the patch, so it is a constant
  offset; the line fit averages 50-150 rows.
- **Defocus blur.** Symmetric blur widens the edge but does not move its 50 %
  point, so the measured motion is the directional shift.
- **Exposure, LED level, gain.** The threshold is half way between the measured
  low and high levels, so the 50 % point does not change.
- **Display Auto levels.** It never touches raw data.
- **Hot pixels, texture blotches.** Rejected by the robust line fit.

**Texture offset note.** With strong texture and a soft edge, the global
half-way level puts the measured 50 % point a few pixels off the geometric edge
(in the synthetic full-scale frame with 15 % texture and edge sigma 12 px: 2-3
px; with no texture: 0.003 px). This offset is a property of the patch, it
moves with the patch (a 2.4 px shift is measured to within 0.2 px), and so it
only affects the absolute edge value, not focus control, which works from
differences.

**Box-width guidance.** The levels come from the outer 15 % of the box on each
side along the normal, so each side needs about 4 sigma of flat plateau outside
those bands. Draw the box about **14 x the edge sigma** wide along the normal,
and at least about 2x the expected travel plus a margin on each side. Along the
edge, about 60 % of the edge length is plenty. Keep the ghost and the patch
corners out of the box.

Measured on synthetic data: bias below 0.05 sensor px on noise-free sub-pixel
shifts; single-frame SD 0.004 px at 0.4 counts of noise and 0.012 px at 2
counts; 0.47 ms median for a full 2048x1536 frame.

## Calibration and zero

The workflow is CRISPy-style: the user focuses on the cells by eye, then
presses **Calibrate**. That P is best focus and becomes the zero (setpoint).

```python
SpotFocusService.calibrate(
    box, *, span_um=2.0, steps=9, n_frames=3, settle_s=0.1, record_path=None
) -> EdgeCalibration
```

It runs off the GUI thread. The box is snapped with `snap_box` and the snapped
box is stored in the calibration, so it can differ from the drawn box by less
than 2 px.

1. Refuse (`RuntimeError`) if the lock is running or an MDA runs. Read
   `p_c = getPosition(P)`. Turn the LED on (owner "calibrate"). Pause the Live
   preview and remember it (the worker gets `preview_stop`, the service still
   counts the preview as on).
2. Measure at `p_c` with `2 * n_frames` snapped frames: `e_start`, `noise_px`
   (SD), and the detected axis, polarity, angle and contrast. If the fix is not
   ok, fail with `ValueError("edge not measurable: <reason>")` (wrong box, LED
   off...).
3. Sweep `p_c + linspace(-span/2, +span/2, steps)` in increasing order. At each
   step: `setPosition`, `waitForDevice`, sleep `settle_s`, then `measure` with
   `n_frames`, the detected axis and polarity, and one frame returned. The
   worker drops stale frames until its time-based settle passes, so every
   frame used was exposed after the move. That is the "snap". The live view
   shows the frames that were measured.
4. Go back to `p_c`, wait, measure again with `2 * n_frames`: `e_end`. Warn if
   `|e_end - e_start| > max(3 * noise_px, 0.05 * |slope| um)` (hysteresis or
   drift). If this re-measure fails, `e0 = e_start` and a warning is added.
5. Fit with `fit_edge_calibration`: a least-squares line
   `edge = e_c + s (p - p_c)`. The stored zero is
   `e0_px = (e_start + e_end) / 2` at `p0_um = p_c`; `slope_px_per_um` is
   signed. `linear_range_um` is half the span.
   - Errors (`ValueError`, plain message): fewer than 5 good points; `|s|`
     below 0.3 px/um ("edge barely moves; wrong edge or span too small"); R2
     below 0.95 ("not linear; reduce the span"); non-monotonic steps; the edge
     left the box at some step ("edge left the box: widen the box along the
     edge normal or reduce the span").
   - Warnings (kept and shown): R2 below 0.98, `|s|` below 1 px/um (noise in
     um will be large), hysteresis, sweep points dropped, a record that could
     not be written.
6. If `record_path` is given, save `np.savez_compressed` (`.npz` is appended if
   missing, the parent folder is created) with `p_um`, `edge_px`, `frames`
   (one raw frame per step, zeros for a failed step), `box`, `axis`,
   `polarity`, `p_c`, `e_start`, `e_end` and `noise_px`. It is written before
   the fit, so a failed fit still leaves a record. A write error is logged and
   added to the warnings, not raised. The panel writes
   `USER_DATA_DIR / "spot_calibration_<timestamp>.npz"`.
7. Always, even after a failure: `setPosition(p_c)`, `waitForDevice`, release
   the LED claim, restore the preview. Camera settings are untouched.

The calibration holds: box, axis, polarity, signed slope (px/um), R2, residual,
`linear_range_um` (half the span), noise, reference angle and contrast, zero
edge `e0_px`, zero P `p0_um`, the sweep points, warnings and the creation time.

**Zero here** (`zero(cal, n_frames=8)`) works only while stopped. It reads P
once, takes a short LED claim (owner "zero"), measures the edge with the
calibration's box, axis and polarity, and returns
`rezero(cal, e0_px=edge, p0_um=P)`: same slope, new setpoint. It moves nothing
and waits on nothing. The panel resets the offset to 0. Use it after
refocusing by eye without redoing the sweep.

**Offset (um)** shifts the lock setpoint along the focus axis, like CRISPy's
lock offset. It is live, persisted with the loop params, and reset to 0 by Zero
here and Calibrate. A positive offset holds P at `p0 + offset` (see the sign
convention below).

**Lock** with no valid calibration runs Calibrate first (the user has focused,
so that P is the zero; the offset is forced to 0 for that lock) and then
locks. It needs an edge box either way. If that calibration fails, nothing
locks and the panel shows the error.

### Persistence and old calibrations

The calibration is stored in `SpotFocusSettingsV1.calibration` as
`EdgeCalibration.to_dict()` with `"kind": "edge"` (lists for `box` and
`sweep`, so it round-trips through JSON). Settings added for the edge workflow:
`edge_box` (`x, y, w, h` or `None`), `sweep_span_um` (2.0), `sweep_steps` (9),
`sweep_frames` (3). `SpotFocusSettingsV1.calibration_status()` returns the
`load_calibration` result and `to_calibration()` returns its first element.

`load_calibration(d) -> (EdgeCalibration | None, message)` never raises:

- `None` or `{}` gives `(None, "no spot calibration yet; press Calibrate")`.
- An old Gaussian calibration (no `kind`), garbage, or a malformed dict gives
  `(None, "old spot calibration ignored; recalibrate")`.

The panel shows that message and never crashes its constructor. Unknown keys in
old params are ignored (`SpotLoopParams.from_dict`, settings `extra="ignore"`).

## Control law

- **Error.** `edge_error(fix, cal, offset_um) =
  (fix.position - cal.e0_px) / cal.slope_px_per_um - offset_um`, in um.
  **Moving P by `-error` returns the edge to the zero plus offset.** Check: at
  `P = p0 + offset` the edge is at `e0 + s * offset`, so the error is 0. The
  slope is signed, so no separate direction is needed.
- **Averaging.** `avg_frames` good frames (default 4; at 100 ms exposure 8
  frames took 0.8 s). The window is cleared after each decision, after each
  move and when `offset_um` changes, and frames are ignored until the
  time-based settle passes after the move ack.
- **PI.** `step = -(kp * e + ki * I)` with `kp = 0.6`, `ki = 0.05`. The I term
  alone never exceeds one step.
- **Dead band.** Inside `deadband_um` (0.03) there is no move and the integral
  is held.
- **Clamp and rate cap.** The step is clamped to `max_step_um` (0.5) and moves
  are limited to `max_rate_hz` (2).
- **Outside the linear range** (`|e| > linear_range_um`): the step is a full
  `-sign(e) * max_step_um`, with no integral. The sign is reliable while the
  edge is still inside the box, so the loop walks back without any search.
- **Anti-windup.** The integral only accumulates when the step is not clamped
  and the error is inside the linear range.
- **Range.** A target more than `max_excursion_um` (5) from the lock's start P
  sets a fault and no move is sent; this is the primary guard, since the real
  piezo P can be negative (about -45 um on the rig). `p_min_um` / `p_max_um`
  are optional absolute bounds (None = unknown, not checked). On Start the
  service reads the piezo's `Position` property limits once, if the device
  exposes them and they contain the start P, and fills the bounds; a missing
  or unreadable limit never faults. A fault names the target, the range and
  the start P.
- **Per-frame checks** (`check_fix(fix, cal, params)`, empty string when fine):
  - the fix is not ok (edge lost, saturated, left the box, outside the ROI);
  - `|angle - cal.angle_deg| > max_angle_change_deg` (5): "edge rotated; the
    feature changed";
  - `residual_px > max_edge_residual_px` (3): "edge not straight".

  A bad frame is a strike and is not averaged. `strikes` (5) in a row set a
  fault; a good frame resets the count. Transient bad frames below that count
  are simply skipped.
- **Fault.** No more moves; P is held where it is (never moved back, never
  searched). The worker keeps measuring and publishing, so the panel shows why.
  If the worker dies, the service emits `fault("worker died")`. **Recovery is
  manual:** fix the cause (LED, exposure, box, refocus) and press Lock again. P
  is re-read, a new controller is made and the integral is reset.

## Modules (`src/pymmcore_gui/asi_z_stack/`)

### `spot_focus.py`: pure numpy, no Qt, no MMCore

- `EdgeFix` (frozen dataclass): fields as in the metric section.
- `snap_box(box, *, min_size=16) -> tuple[int, int, int, int]`
- `measure_edge` raises `ValueError` for a non-2D frame or a bad channel,
  bayer or axis:

  ```python
  measure_edge(
      frame, box, *, roi_offset=(0, 0), channel="sum", bayer="RGGB",
      axis=None, polarity=None, saturation=None, max_saturated_frac=0.002,
      min_contrast=10.0, min_valid_fraction=0.5,
  ) -> EdgeFix
  ```

- `EdgeCalibration` (dataclass, `to_dict` / `from_dict`). `from_dict` raises
  `ValueError` on a wrong `kind`, missing keys, a zero or non-finite slope, or
  a malformed box, axis or polarity.
- `load_calibration(d) -> tuple[EdgeCalibration | None, str]`: never raises.
- `fit_edge_calibration` drops non-finite edge values (counted toward the
  "at least 5" error, otherwise a "sweep point(s) dropped" warning). The
  service itself raises the "edge left the box" error when a sweep step
  reports that reason:

  ```python
  fit_edge_calibration(
      p_um, edge_px, *, box, axis, polarity, p0_um, e0_px=None,
      noise_px=0.0, angle_deg=0.0, contrast=0.0, linear_range_um=None,
      min_slope_px_per_um=0.3, min_r2=0.95,
  ) -> EdgeCalibration
  ```

- `rezero(cal, *, e0_px, p0_um) -> EdgeCalibration`
- `edge_error(fix, cal, offset_um=0.0) -> float`
- `check_fix(fix, cal, params) -> str`
- `SpotLoopParams` (dataclass, `to_dict` / `from_dict`, unknown keys ignored).
  Defaults:
  - loop: `kp` 0.6, `ki` 0.05, `deadband_um` 0.03, `max_step_um` 0.5,
    `max_rate_hz` 2.0, `avg_frames` 4, `offset_um` 0.0, `strikes` 5;
  - frame checks: `min_contrast` 10.0, `max_saturated_frac` 0.002,
    `min_valid_fraction` 0.5, `max_angle_change_deg` 5.0,
    `max_edge_residual_px` 3.0;
  - range: `max_excursion_um` 5.0 (from the start P); `p_min_um` and
    `p_max_um` None (no absolute bound unless set or read at Start).
- `SpotFocusController(params, p_start_um, *, linear_range_um=None)`:
  `update(along_um, now_s) -> float | None` returns the new target P or None;
  `bad(reason)` counts a strike; `good()` resets it. Properties: `p_cmd_um`,
  `fault: str | None`, `n_moves`.

Removed with the Gaussian spot code: the spot fit and locator, the old spot
calibration and its fit, the old focus-error function, and the
`lateral_limit_px` and `min_amplitude` parameters.

### `spot_synthetic.py`: LED-patch renderer for tests and the fake camera

Numpy only.

```python
render_led_patch(
    shape, center, *, size=(96, 92), angle_deg=0.0, edge_sigma=2.0,
    level=100.0, background=1.0, texture=0.15, tail=0.04, ghost=0.3,
    ghost_offset=(-0.11, 1.35), ghost_center=None, seed=0,
) -> float64 ndarray
```

- A rounded square with a soft edge, multiplicative texture and dark blotches,
  a faint tail up and right, and a dim ghost below (at `ghost_offset` times the
  patch size, or at an explicit `ghost_center`). Modelled on **one
  screenshot**.
- `add_noise(img, *, noise, rng, hot_pixels=0, bits=8)` returns a uint8 or
  uint16 array.
- Angle convention: a vertical edge has image slope dx/dy = tan(angle), a
  horizontal edge dy/dx = -tan(angle), so the `EdgeFix.angle_deg` of a top or
  bottom edge is the negative of `angle_deg`.

### `toupcam_source.py`: frame sources (used inside the worker)

- `FrameSource` Protocol: `open()`, `close()`, `latest(timeout_s)` (newest raw
  frame and a monotonic timestamp as a tuple, dropping older ones, or None),
  `set_exposure_ms(float)`, `set_gain(int)`, `set_roi(x, y, w, h)`,
  `info() -> dict`, `set_actuator_um(p)` (no-op for real cameras; used by the
  fake).
- `ToupcamSource(serial=None, sdk_dir=None)`
  - Loads the vendor's ctypes wrapper (`toupcam.py`, or `amcam.py` as an alias)
    from `sdk_dir`, or the env var `TOUPCAM_SDK_DIR`, at `open()` time, and
    raises a clear error if it is missing. The SDK is never bundled.
  - Puts the camera in raw mode, with auto-exposure off and pull mode. The SDK
    callback sets an event and the newest frame is copied into a 1-slot
    buffer.
- `FakeSpotSource` renders the LED patch with `render_led_patch` (the Gaussian
  renderer is gone):

  ```python
  FakeSpotSource(
      cal_like, drift_um_per_s=0.0, noise=2.0, amplitude=800.0,
      sigma_px=6.0,  # accepted, ignored
      shape=(480, 640), lateral_px_per_s=0.0, seed=0, fps=50.0, bits=16,
      patch_px=96.0, blur_px_per_um=1.0, ghost=0.3, tail=0.04,
      hot_pixels=0, ghost_motion=-0.5,
  )
  ```

  - `cal_like` gives the fake's *true* image motion (`x0, y0, ux, uy,
    slope_px_per_um, p0_um`); the whole patch moves along `(ux, uy)` by
    `slope * (P + drift - p0)`, and the ghost moves with factor
    `ghost_motion`.
  - The edge sigma grows with defocus: `2.0 + blur_px_per_um * |P + drift -
    p0|` px.
  - `set_actuator_um(p)` sets P. Drift accrues from `open()`.
    `set_level(scale)` scales the brightness (exposure or LED change tests;
    level 0 means LED off). The worker never calls it.
  - 8-bit amplitude is capped at 40 % of full scale; hot pixels are static
    (70 % of full scale).
  - Built from a plain dict spec so it can be constructed inside a spawned
    process.
- `make_source(spec: dict) -> FrameSource`: `{"kind": "toupcam", ...}` or
  `{"kind": "fake", ...}`.

### `spot_focus_worker.py`: subprocess entry

`worker_main(conn, source_spec: dict)` is a module-level function so spawn can
pickle it. It uses a `multiprocessing` Pipe; every message is a dict with a
`"type"`.

Main to worker:

- `start`: `params`, `calibration` (an `EdgeCalibration` dict), `p_start_um`.
  A malformed or old calibration becomes a worker fault `bad calibration: ...`
  (the worker does not crash).
- `stop`.
- `update_params`: `params`. A changed `offset_um` also clears the averaging
  window.
- `set_camera`: `exposure_ms`, `gain`, `channel`, `roi`.
- `set_edge_box`: `box` or None. It is the preview overlay box, measured at up
  to 5 Hz in preview. A malformed box gives an `error` message and leaves the
  box unchanged. When not running, it clears the fix and overlay at once.
- `measure`: `n_frames`, `p_um`, `box`, `axis` (or None), `polarity` (or
  None), `return_frame`, and an optional `timeout_s`. A `box` of None falls
  back to the preview edge box; with neither, the reply has `n = 0` and reason
  `"no edge box"`.
- `moved`: `seq`, `ok`, `dt_ms`.
- `preview_start`, `preview_stop`.
- `shutdown`.

Worker to main:

- `ready`: `info`.
- `move`: `seq`, `target_um`, `along_um`.
- `telemetry`: about 10 Hz, in preview or while running, and only when a fix
  exists (so with no edge box there is no telemetry). Fields: `t`, `edge_px`,
  `x`, `y`, `along_um`, `contrast`, `angle_deg`, `residual_px`,
  `valid_fraction`, `fps`, `p_cmd_um`, `n_moves`, `ok`, `reason`, `max_value`,
  `saturated_frac`. `along_um` is None unless the worker holds a calibration
  whose box is the measured box (always true while running). `ok` is true when
  the fix is ok and `check_fix` has no complaint; `reason` is the `check_fix`
  reason.
- `measurement`: the reply to `measure`. Fields: `p_um`, `n`, `edge_mean`,
  `edge_sd` (ddof 1; 0.0 for n = 1; NaN for n = 0), `x_mean`, `y_mean`,
  `contrast_mean`, `angle_mean`, `residual_mean`, `axis`, `polarity`,
  `reason`, `roi`, and `frame`. `reason` is the last bad reason when `n == 0`,
  else empty (`"no frames from camera"` if nothing settled at all). `frame` is
  the last good frame (full ROI, raw dtype), only if `return_frame` was set
  and a good frame exists.
- `fault`: `reason`.
- `status`: `state` (idle, preview, running, stopped, fault).
- `error`: `message`, `traceback`.

Behaviour:

- Which box is measured: while running, the calibration's box (with its axis
  and polarity). In preview, the edge box (the calibration's axis and polarity
  are used only if the snapped edge box equals the calibration's box, else they
  are auto-detected). In `measure`, the request's box. With no box there is no
  fix, no overlay and no telemetry.
- The worker tells its source the commanded P (`set_actuator_um`) when it sends
  a move, so the fake closes the loop.
- `measure` snaps: it ignores frames until the settle passes, writes every
  settled frame to the live view buffer (good or not, so a failing box is
  still visible), averages the good ones, and detects axis and polarity once.
  With no good frame so far it gives up after `n + 6` settled bad frames
  instead of waiting for the full deadline.
- The shared-memory `spot` field carries the edge overlay point
  `(fix.x, fix.y)`. `lateral_px` and `amplitude` no longer exist in
  telemetry.

### `spot_focus_service.py`: main process, Qt-free (psygnal signals)

```python
SpotFocusService(
    mmc, piezo_label="PiezoStage:P:34", crisp_label=None, source_spec=...,
    *, led_intensity=None, ready_timeout_s=30.0,
)
```

- Starts the worker process (spawn context, daemon) and a reader thread.
- On `move` it calls `mmc.setPosition(piezo_label, target)` **without**
  `waitForDevice`, times the call, and replies `moved`. It drops a move if the
  previous one is still in flight, so moves are never queued.
- Signals: `telemetry(dict)`, `camera_changed(dict)` (the camera's info after a
  `set_camera`), `fault(str)`, `state(str)`, `moved(dict)`.
- Properties: `current_state`, `preview_active`, `camera_info`, `info`,
  `last_telemetry`, `worker_pid`; `wait_ready(timeout_s=None)`.
- `start(params, calibration, led_intensity=None)`
  - Reads P once with `getPosition`, calls `led_on(owner="lock",
    force_ready=True)` (CRISP to Ready, previous state remembered), then sends
    `start`.
- `update_params(params)` applies new loop params live (offset, gains,
  limits).
- `stop()` sends `stop` and lets go of the lock's claim on the LED. It never
  moves P. It returns to `preview` if preview was on, else `stopped`.
- `calibrate(box, *, span_um=2.0, steps=9, n_frames=3, settle_s=0.1,
  record_path=None) -> EdgeCalibration`: see the calibration section. Only
  with the lock stopped and no MDA running.
- `zero(cal, *, n_frames=8) -> EdgeCalibration`: re-zero at the current P and
  edge, only while stopped.
- `set_edge_box(box | None)`: edge box for the preview overlay.
- `measure(n_frames=20, p_um=None, *, box, axis=None, polarity=None,
  return_frame=False) -> dict`: refuses while the lock runs. Raises
  `RuntimeError` ("worker is dead" or "did not answer") instead of a bare
  `queue.Empty`.
- CRISP LED (the LED is lit only in CRISP State Ready / Lock / Dither..., off
  in Idle, the power-up state):
  - `led_on(intensity=None, *, owner="manual", force_ready=False)` remembers
    the previous CRISP state once, sets Ready unless CRISP is already in a lit
    state (or `force_ready`), and sets `"LED Intensity"` (default: the
    constructor's `led_intensity`).
  - `led_off(owner)` drops that owner's claim. Owners are "preview", "lock",
    "manual", "calibrate" and "zero"; the previous state is restored when the
    last one is gone, so stopping the lock with Preview on keeps the LED lit.
  - `led_is_on`, `crisp_state()`, `crisp_label`, `set_led_intensity(v)`
    (live). With no CRISP label all of it is a no-op and `led_is_on` stays
    False.
- `shutdown()` and context manager support. Shutdown restores CRISP.
- If the worker process dies, the service emits `fault("worker died")`, the
  state becomes fault, and no further moves are sent.
- `stats()` returns `n_moves`, `mean_dt_ms`, `max_dt_ms` and `moves_per_min`.

## Tests (`tests/`, no hardware, no MM install)

How to run them:

- Pure numpy and worker tests run with `--noconftest`:

  ```bash
  uv run pytest --noconftest tests/test_spot_focus.py \
      tests/test_spot_focus_worker.py tests/test_spot_frame_shm.py \
      tests/test_toupcam_source.py
  ```

- The Qt tests (`test_spot_focus_widget.py`, `test_spot_live_view.py`) must
  run **without** `--noconftest`, and with `QT_QPA_PLATFORM=offscreen`. With
  both PyQt6 and PySide6 installed, pytest-qt otherwise picks PySide6 and
  `qtbot.addWidget` fails ("Need to pass a QWidget"). The widget tests spawn
  several workers and take about a minute.

  ```bash
  QT_QPA_PLATFORM=offscreen uv run pytest \
      tests/test_spot_focus_widget.py tests/test_spot_live_view.py
  ```

Files:

- `test_spot_focus.py` (metric, calibration, controller, renderer):
  - `measure_edge` on synthetic patches: sub-pixel bias below 0.05 px over
    shifts and edge orientations, both polarities, mono and Bayer (sum and red
    channels), odd ROI offsets, and noise levels.
  - Disturbances: ghost outside and inside the box, tail, hot pixels, texture
    (constant offset, shift measured), blur (50 % point does not move),
    brightness change.
  - Failure reasons: box outside the ROI, saturated, edge lost, edge left the
    box.
  - Calibration: fit recovers slope and zero, error cases (too few points,
    barely moves, not linear, not monotonic), `rezero`, JSON round trip,
    `load_calibration` with old, malformed and empty dicts.
  - `edge_error` sign convention and offset; `check_fix` (rotated, not
    straight).
  - Controller: dead band, step clamp, rate cap, range and excursion faults,
    outside-linear-range sign-only step, anti-windup, strikes. With the sign
    flipped, the controller diverges.
- `test_toupcam_source.py`: SDK loading and the frame-source plumbing, against
  a fake `toupcam.py`.
- `test_spot_focus_worker.py` uses a real spawned worker with `FakeSpotSource`
  and a fake mmc (records `setPosition` calls and never waits). Cases:
  - Under drift, closed-loop convergence holds the error within the dead band.
  - **Command rate:** under slow drift, the number of moves per minute stays at
    or under the cap.
  - Killing the worker gives a fault and no more moves.
  - Calibration through the service (both slope signs) recovers the fake's
    slope, zero, axis and polarity, returns P, restores CRISP and the preview,
    and writes a record; a calibration that loses the edge mid-sweep or finds
    no edge raises the documented text and cleans up. Calibrate, zero and
    measure refuse while running; calibrate refuses during an MDA.
  - `zero` re-zeroes without moves or waits; an offset change moves the held P
    and back.
  - LED off or exposure change: fault on edge loss, and no move when only the
    contrast changes.
  - A bad calibration dict faults the worker without killing it.
  - Preview: overlay appears with an edge box and goes away without one; ROI
    changes keep sensor coordinates; `measure` reply fields; live exposure,
    gain and channel.
  - `stop` restores the CRISP state; shutdown and orphan handling; settle
    formula.
- `test_spot_focus_widget.py`: settings (defaults, round trip, old and
  malformed calibrations, old `mode: image` files) and the Spot panel (buttons
  Calibrate / Zero here / Lock / Stop, offset, edge box, sweep spins, camera,
  LED, ROI, a lock run, Calibrate and Zero here off the GUI thread, Lock
  without a calibration) and its dock action.
- `test_spot_live_view.py`: the live view (zoom, ROI and edge-box drawing,
  overlays, levels).
- `test_spot_frame_shm.py`: the shared-memory frame buffer (seqlock, no torn
  frames).
- `test_spot_tracking.py` covers the separate centroid tracker
  (`_spot_tracking.py`), not the focus lock.

## Live camera view and controls

The user needs to see the camera **live at full resolution** to align the beam,
place the ROI and the edge box, and set exposure. No downsampled view: images
are drawn at native resolution, with nearest-neighbour pixels when zoomed.

Rig camera: ICMOS03100KPB (MD310C-BS), 2048x1536, **RAW8 only**, RGGB. 11.5 fps
at full frame, 31 fps at a 640x480 ROI. Exposure runs 0.105 ms to 15 s, gain
100 to 5000. After a settings change the camera delivers stale frames for up to
3-4 frames.

### Frames: shared memory, never the pipe (`asi_z_stack/spot_frame_shm.py`)

- `FrameBuffer.create(max_bytes) -> FrameBuffer` is the writer, in the worker.
- `FrameBuffer.attach(name) -> FrameBuffer` is the reader, in the main process.
  It uses `track=False`.
- `.name`, `.close()`, `.unlink()`.
- `write(image, *, t, roi, sensor, bits, spot, spot_ok, saturated_frac,
  max_value)`. A seqlock header (sequence number odd while writing) means a
  reader never sees a torn frame.
- `read(last_seq=None) -> LiveFrame | None` returns None when there's nothing
  newer than `last_seq`. It copies the image out.
- `LiveFrame` (dataclass) fields:
  - `image`: 2-D uint8 or uint16, ROI-sized
  - `seq`, `t`
  - `roi`: `(x, y, w, h)` in sensor px
  - `sensor`: `(W, H)`
  - `bits`
  - `spot`: `(x, y)` sensor px of the edge overlay point, or None
  - `spot_ok`, `saturated_frac`, `max_value`

### Worker

- States: `idle`, `preview`, `running`, `stopped`, `fault`. Messages
  `preview_start` and `preview_stop`.
- Every frame goes to the shared buffer while previewing, running, or
  measuring.
  - The edge is measured for the overlay at most 5 Hz in preview, when an edge
    box is set.
  - While running, it is measured every frame.
- **All edge positions are in sensor pixels**: the ROI offset is accounted for
  in fixes, the calibration, telemetry, measurements and the setpoint. A
  calibration therefore survives ROI changes, as long as the edge box stays
  inside the ROI.
- `set_camera` applies live:
  - `exposure_ms`, `gain`, `channel` (`sum` or `red`)
  - `roi`: `(x, y, w, h)`; `(0, 0, 0, 0)` means full frame
- **Settle is time-based.** After a move ack or any camera change, frames are
  ignored until both of these hold: 5 frames have arrived (the first four are
  dropped), and `max(2*exposure + 1/fps, 0.05)` s has passed.
- The `ready` info carries: `shm`, `sensor`, `bits`, `model`, `serial`,
  `exposure_range_ms`, `gain_range`, `roi`, `exposure_ms`, `gain`.

### Service

- `start_preview()` and `stop_preview()`; `preview_active`.
- `latest_frame(last_seq=None) -> LiveFrame | None`.
- `camera_info`: the ready info, kept current by `set_camera`.
- `set_camera(exposure_ms=None, gain=None, roi=None, channel=None)`.
- `stop()` returns to `preview` if preview was on.
- Calibrate pauses the preview while it runs and restores it afterwards.

### Settings (`spot_focus` in the app settings)

- `source`: `kind` (`"toupcam"` or `"fake"`), `serial`, `sdk_dir`, `fake`
  (keyword arguments of `FakeSpotSource`).
- Camera: `exposure_ms` (5.0), `gain` (100, percent), `roi` (None is full
  frame), `channel` (`"red"` by default, or `"sum"`), `bayer` (`"RGGB"`),
  `led_intensity` (None leaves the CRISP value alone).
- `params`: the `SpotLoopParams` fields, including `offset_um`.
- `calibration`, `edge_box`, `sweep_span_um`, `sweep_steps`, `sweep_frames` as
  described above.

### GUI

- **Spot panel** (`widgets/_spot_focus_panel.py`, hosted in a scroll area by
  the "Spot Focus Lock" action). Top to bottom:
  - Buttons **Calibrate** (tooltip: focus on the cells first; sweeps P around
    here), **Zero here**, **Lock** and **Stop**. Lock is enabled whenever the
    panel is idle, because Lock without a calibration calibrates first.
  - **Offset (um)** spin (-5 to 5): live, persisted, reset to 0 by Zero here
    and Calibrate.
  - State, fault and calibration labels. The calibration label shows axis,
    signed slope (px/um), R2, residual, range, noise in um, zero P and the
    created time. A second line in the attention colour shows the warnings, and
    "edge box changed since the calibration; recalibrate" when the saved box
    snaps to a different box than the calibration's.
  - Group **Edge box and calibration sweep**: a **Draw edge box** toggle and
    the saved box as x, y, w, h ("none" until drawn), then spins for span (um),
    steps and frames per step.
  - Group **Camera**:
    - SDK folder: a line edit plus Browse, saved to
      `spot_focus.source.sdk_dir`.
    - A camera info label: model, serial, sensor, bits.
    - A **Preview** toggle and a **Show view** button. Preview lights the CRISP
      LED (CRISP State Ready) and turning it off restores CRISP, unless the
      lock is running.
    - An **LED** toggle and a live **LED %** spin (saved, default 50), next to
      a readout of CRISP's state and device label, read once after each
      change. With no CRISP device the fault label says "no CRISP device
      (focus.crisp_label)".
    - Exposure (ms), gain and channel (red or sum). Each applies live and is
      saved.
    - ROI x, y, w, h, with **Apply ROI** and **Full frame**.
    - Readouts: fps, max value, % saturated.
    - Camera errors show in the panel's fault label.
  - A strip chart.
  - Readouts: Moves / min, Move time (mean / max), Edge (px), Contrast, Frame
    rate, Error (along). The error shows a dash when `along_um` is None, else
    the error in um.
  - Group **Parameters**: Deadband, Kp, Ki, Max step, Max rate, Min contrast
    (counts), Min valid rows (fraction), Max angle change (degrees) and Max
    edge residual (px). `avg_frames` and `max_saturated_frac` have no spin.
- Panel behaviour:
  - **Calibrate** needs a saved edge box, else the fault label says "Draw an
    edge box across the edge that moves with focus". It runs the sweep on a
    thread. On success it saves the calibration, resets the offset to 0 and
    moves the setpoint ring to the zero edge point. A failure shows
    "Calibration failed: ...".
  - **Zero here** is enabled with a valid calibration and the lock stopped. A
    failure shows "Zero failed: ...".
  - **Draw edge box** puts the live view in edge-box draw mode, shows the
    window and starts Preview if it is not on. The panel tells the user to
    draw the box wide along the edge normal (about 14 x the edge sigma). The
    box is saved in `spot_focus.edge_box` and sent to the worker with
    `set_edge_box` (also when the service is created, and again after Stop).
  - While the lock runs, Zero here, Calibrate and Draw edge box are disabled,
    and a box drawn then is refused ("Stop the lock before moving the edge
    box.").
  - An old or malformed saved calibration shows its message and never crashes
    the constructor.
- **Live view** (`widgets/_spot_live_view.py`), a separate resizable top-level
  window titled "Spot camera":
  - It is a QGraphicsView whose scene is the sensor. Each frame is drawn at
    native resolution at its ROI offset.
  - Wheel zooms around the cursor, left-drag pans, and there are **Fit** and
    **1:1** buttons. Pixels are nearest-neighbour at 100% and above; below 100%
    the view resamples smoothly (bilinear).
  - **Bayer to grey** (default on): the camera is a colour sensor read as raw
    Bayer, so for display only each pixel is replaced by the
    `[1, 2, 1] x [1, 2, 1] / 16` average of its 3x3 neighbourhood, which
    cancels the 2x2 colour mosaic (otherwise it aliases into a Moire grid at
    fractional zoom). The pixel readout and the edge measurement use the raw
    values. Turn it off for a mono camera or to inspect the raw mosaic.
  - **Draw ROI** and **Draw edge box** buttons are mutually exclusive draw
    modes (`draw_target` is `"roi"`, `"edge"` or None).
    - Draw ROI: dragging a rectangle (snapped to even ints, clamped to the
      sensor) emits `roiDrawn(x, y, w, h)`. The panel fills its ROI spins and
      applies them.
    - Draw edge box: the same drag (minimum 16 px) emits
      `edgeBoxDrawn(x, y, w, h)`. The panel stores the box and sends it to the
      worker.
    - A valid drag turns its button off (one box per click); a click or a
      sliver keeps the mode. The panel can switch the mode with
      `set_edge_draw_mode(on)` and follows it through `edgeDrawModeChanged`.
  - Overlays: the sensor outline, the current ROI, the edge box (dashed
    violet), the edge point (a cross at the box centre on the measured edge)
    and the zero edge point (a ring). The edge box is user data: frames and
    `clear()` do not remove it. The status text reads "edge (x, y)", plus
    "(not ok)" for a failed fix.
  - Levels are linear. The fixed range is 0 to 2^bits-1, with an **Auto
    levels** option (min to max). Auto levels is display only; the measurement
    always uses raw counts.
  - Under the cursor it shows x, y and the pixel value.
  - A 30 Hz timer polls `latest_frame` and repaints only on a new `seq`.
  - Closing the window stops preview. If the lock is running, the preview is
    switched off after Stop instead.

## Rig test plan (GO / NO-GO)

Do these in order. Stop at the first NO-GO and fix it before going on.

1. **Find the edge.** Preview on, LED on, exposure so the patch is about
   100-180 counts with 0 % saturated. Watch the patch while jogging P by +-1 um
   by hand (lock stopped) and identify the edge that moves most.
   - GO: the edge visibly moves. Otherwise try another edge.
2. **Draw the edge box** across that edge: about 60 % of the edge length, about
   2x the expected travel plus a margin on each side along the normal, ghost
   out. With P still, the cross should sit on the edge and stay still (noise
   below 0.2 px).
3. **Calibrate.** Focus on the cells by eye (OPM image), then press Calibrate
   (default +-1 um, 9 steps).
   - GO: `|slope| >= 1 px/um`, R2 `>= 0.98`, residual below 0.3 px, no "left
     the box", no hysteresis warning.
   - NO-GO: shrink the span, move or enlarge the box, or pick another edge.
4. **Lock** with offset 0. Set the offset to +0.3 um and back to 0.
   - GO: P follows within 2-3 s with at most 3 moves and settles inside the
     dead band.
   - Check that the cells stay sharp over 30 min with moves/min low (below 10).
5. **Fault drills.**
   - LED off: fault "edge lost" after 5 bad frames, P held.
   - Exposure doubled: no move (the contrast changes, the edge position does
     not).
   - Start an MDA Z-stack while locked: no serial traffic is added during
     volumes.
