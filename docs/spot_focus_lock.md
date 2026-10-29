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
patch, or of two edges fused into one number, and moves piezo P
(`PiezoStage:P:34`) to bring the edge back to where it was when the sample was
in focus. The method follows Nguyen et al. 2025, Biomed. Opt. Express
16(3):1216. Their code has no licence, so nothing here is copied from it.

> **Mostly rig-unverified.** The patch size, ghost position, tail level and
> noise below come from one screenshot, and the synthetic renderer
> (`spot_synthetic.py`) and `FakeSpotSource` were modelled on it, so every test
> passes against a model, not against the rig. The first real P sweep
> (2026-10-28) is summarised in "First rig results"; the closed lock and the
> second edge have not been run on the rig yet. Work through the rig test plan
> at the end before trusting the lock.

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
- On a fault (edge lost, saturated, edge rotated or not straight, the two
  edges disagree, range limit, worker died) the loop stops sending moves and
  reports. It never searches for the edge and never moves P back.
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

### Optional second edge and fusion

The user may draw a second edge box on a different edge of the patch (for
example box 1 on the top edge, box 2 on the left edge). Each box is measured
with `measure_edge` and calibrated on its own, with its own signed slope, from
the same snapped frames. The two position errors in um are then fused into one
number.

- **`FocusCalibration`** holds one or two `EdgeCalibration` entries (`edges`),
  the shared zero P `p0_um`, `created`, `record_path`, `warnings` and
  `kind="edges"`. Properties: `boxes` and `linear_range_um` (the smallest
  range over the edges). `rezeroed(e0_px, p0_um)` takes one zero edge value
  per edge.
- **`edge_noise_um(e)`** is the per-edge noise used for weighting:
  `max(e.noise_px, e.residual_px, 0.02) / |e.slope_px_per_um|`. The line-fit
  residual is included because it contains the repeatability that the static
  noise misses.
- **`fuse_edges(fixes, cal, params) -> FusedError`** checks each edge with
  `check_fix`. A good edge gives `a_i = edge_error(fix_i, edge_i, 0)`.
  - Two good edges: if `D = |a1 - a2|` is above
    `max(max_edge_disagreement_um, 4 * sqrt(n1^2 + n2^2))` (limit default 1.0
    um, `n_i` from `edge_noise_um`), the edges disagree and the mean cannot be
    trusted. Then the **hold band** applies: let `k` be the edge whose
    estimate is closest to the target (smallest `|a_k - offset_um|`). If that
    distance is at most `hold_band_um` (default 0.15 um), the frame **holds on
    edge `k`** alone: `along_um = a_k - offset_um`, `sigma_um` is the noise of
    edge `k`, `held_edge = k` (0-based), and `note` is `"edges disagree by D
    um; holding on edge K (X um from target)"` (1-based `K`). That is not a
    strike. If neither edge is within the band, the result is a bad
    measurement with reason `"edges disagree by D um (lateral drift or a
    changed image)"`: a strike and no move. When the edges agree, the result
    is the inverse-variance mean, with `sigma_um = 1 / sqrt(w1 + w2)`.
    - Why hold: a lateral image shift moves one edge but not the other. An
      edge that still sits at its target is the one to trust, and faulting
      would stop a lock that is fine. The risk: if one edge is stuck at its
      target while the other genuinely reports a focus change, the lock holds
      on the stuck edge and ignores the real change. The panel shows
      "holding on edge K" in the Disagreement readout and the note line, so
      this is visible. Set `hold_band_um` to 0 to turn holding off.
  - One good edge of two: the loop uses it, with the note
    `"edge K lost (<reason>); using edge J"` (1-based). That is not a strike.
  - No good edge: the reason of edge 1, a strike, and `along_um` is None.
  - `offset_um` is subtracted once, at the end. `FusedError` carries
    `along_um`, `sigma_um`, `used` (which edges passed `check_fix`),
    `per_edge_um` (raw, no offset), `disagreement_um`, `reason`, `note` and
    `held_edge` (None unless the frame held on one edge).
  - A one-edge calibration gives the same result as the old single-edge path.

Why two edges:

- **Lower noise.** On the first rig sweep, fusing the top and left edges
  (inverse variance, residuals almost uncorrelated) gives 0.024 um against
  0.048 um for the top edge alone, about 2x better.
- **A health check.** A focus change moves both edges consistently with their
  calibrated slopes. A lateral image shift (beam walk) moves the left edge
  (along x) but not the top edge (along y), so the two um estimates disagree
  and the loop stops moving instead of following the walk.

## Calibration and zero

The workflow is CRISPy-style: the user focuses on the cells by eye, then
presses **Calibrate**. That P is best focus and becomes the zero (setpoint).

```python
SpotFocusService.calibrate(
    boxes, *, span_um=2.0, steps=9, n_frames=3, settle_s=0.25,
    record_path=None,
) -> FocusCalibration
```

It runs off the GUI thread. `boxes` is one box or a list of one or two boxes.
Each box is snapped with `snap_box` and the snapped box is stored in its edge
calibration, so it can differ from the drawn box by less than 2 px.

1. Refuse (`RuntimeError`) if the lock is running or an MDA runs. Read
   `p_c = getPosition(P)`. Turn the LED on (owner "calibrate"). Pause the Live
   preview and remember it (the worker gets `preview_stop`, the service still
   counts the preview as on).
2. Measure at `p_c` with `max(2 * n_frames, 10)` snapped frames. Per box this
   gives `e_start`, `noise_px` (SD) and the detected axis, polarity, angle and
   contrast. A box whose fix is not ok (wrong box, LED off...) is dropped with
   the warning `"edge K dropped: edge not measurable: <reason>"` (K is the
   1-based box number as passed).
3. Sweep `p_c + linspace(-span/2, +span/2, steps)` in increasing order. At each
   step: `setPosition`, `waitForDevice`, sleep `settle_s` (default 0.25 s),
   then `measure` with `n_frames`, the detected axis and polarity, and one
   frame returned. **One snap per step measures all surviving boxes from the
   same frames.** The worker drops stale frames until its time-based settle
   passes, so every frame used was exposed after the move. That is the "snap".
   The live view shows the frames that were measured. A box that reports "edge
   left the box" at a step is dropped. If every box has left its box the
   sweep aborts early.
4. Go back to `p_c`, wait, measure again with `max(2 * n_frames, 10)` frames:
   `e_end`. Warn if `|e_end - e_start| > max(3 * noise_px, 0.05 * |slope| um)`
   (hysteresis or drift). If this re-measure fails for an edge, its
   `e0 = e_start` and a warning is added.
5. Fit **each edge separately** with `fit_edge_calibration`: a least-squares
   line `edge = e_c + s (p - p_c)`. The stored zero is
   `e0_px = (e_start + e_end) / 2` at the shared `p0_um = p_c`;
   `slope_px_per_um` is signed and can differ in sign and size between edges.
   `linear_range_um` is half the span.
   - Errors (`ValueError`, plain message): fewer than 5 good points; `|s|`
     below 0.3 px/um ("edge barely moves; wrong edge or span too small"); R2
     below 0.95 ("not linear; reduce the span"); non-monotonic steps; the edge
     left the box at some step ("edge left the box: widen the box along the
     edge normal or reduce the span").
   - **An edge whose fit raises is dropped with a warning**
     (`"edge K dropped: <message>"`) and the calibration continues with the
     other edge. The call fails only if no edge survives; the error is then
     the message of the lowest-numbered edge.
   - Warnings (kept and shown): R2 below 0.98, `|s|` below 1 px/um (noise in
     um will be large), hysteresis, sweep points dropped, dropped edges, a
     record that could not be written. With two boxes the per-edge warnings
     are prefixed with "edge K:" in the calibration's `warnings`.
6. If `record_path` is given, save `np.savez_compressed` (`.npz` is appended if
   missing, the parent folder is created). The arrays are per edge:
   - `p_um` (steps), `edge_px` (steps x n_boxes, NaN where not measured),
     `boxes` (n x 4), `axes` (n, empty string if the box was dropped at the
     zero), `polarities` (n, 0 if dropped), `frames` (steps x h x w, one raw
     frame per step shared by both edges, zeros for a failed step), `p_c`,
     `e_start` (n), `e_end` (n) and `noise_px` (n).
   - The old scalar keys `box`, `axis` and `polarity` are gone.

   It is written before the fit, so a failed fit still leaves a record. A
   write error is logged and added to the warnings, not raised. The panel
   writes `USER_DATA_DIR / "spot_calibration_<timestamp>.npz"`.
7. Always, even after a failure: `setPosition(p_c)`, `waitForDevice`, release
   the LED claim, restore the preview. Camera settings are untouched.

Each edge calibration holds: box, axis, polarity, signed slope (px/um), R2,
residual, `linear_range_um` (half the span), noise, reference angle and
contrast, zero edge `e0_px` and its own warnings. The `FocusCalibration` adds
the shared zero P `p0_um`, the record path, the calibration warnings and the
creation time.

**Zero here** (`zero(cal, n_frames=8)`) works only while stopped. It reads P
once, takes a short LED claim (owner "zero"), measures every edge with its
calibration box, axis and polarity, and returns `cal.rezeroed(e0s, P)`: same
slopes, new setpoint for every edge at the current P. It moves nothing and
waits on nothing. If any edge cannot be measured it raises `ValueError`
(`edge K: edge not measurable: ...` when the calibration has two edges). The
panel resets the offset to 0. Use it after refocusing by eye without redoing
the sweep.

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
`FocusCalibration.to_dict()` with `"kind": "edges"` (lists for the boxes and
sweeps, so it round-trips through JSON). Settings added for the edge workflow:
`edge_box` and `edge_box_2` (`x, y, w, h` or `None`; box 2 is optional),
`sweep_span_um` (2.0), `sweep_steps` (9), `sweep_frames` (3), and the loop
parameters `max_edge_disagreement_um` (1.0) and `hold_band_um` (0.15).
`SpotFocusSettingsV1.calibration_status()` returns the `load_calibration`
result and `to_calibration()` returns its first element.

`load_calibration(d) -> (FocusCalibration | None, message)` never raises:

- `None` or `{}` gives `(None, "no spot calibration yet; press Calibrate")`.
- A new `"edges"` dict loads as one or two edges. **An old single `"edge"`
  dict (`EdgeCalibration.to_dict()`, `"kind": "edge"`) loads as a one-edge
  `FocusCalibration`**, with `p0_um`, `created`, `record_path` and `warnings`
  taken from it. The rig's existing calibration (slope 1.919 px/um) keeps
  working this way.
- An old Gaussian calibration (no `kind`), garbage, a malformed dict, or an
  edge count other than 1 or 2 gives
  `(None, "old spot calibration ignored; recalibrate")`.

The panel shows that message and never crashes its constructor. Unknown keys in
old params are ignored (`SpotLoopParams.from_dict`, settings `extra="ignore"`).

## Control law

- **Error.** `edge_error(fix, cal, offset_um) =
  (fix.position - cal.e0_px) / cal.slope_px_per_um - offset_um`, in um.
  **Moving P by `-error` returns the edge to the zero plus offset.** Check: at
  `P = p0 + offset` the edge is at `e0 + s * offset`, so the error is 0. The
  slope is signed, so no separate direction is needed.
- **Two edges.** With two edges the error fed to the loop is the fused one from
  `fuse_edges` (see "Optional second edge and fusion"), with `offset_um`
  applied once. A disagreement between the edges beyond the limit is a bad
  frame (a strike), unless one edge is within `hold_band_um` of its target:
  then the loop holds on that edge (a note, no strike). A single lost edge is
  only a note and the loop continues on the other one.
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

  With two edges `fuse_edges` runs `check_fix` on each edge and adds the
  disagreement check and the hold band. A bad frame (including a disagreement
  with no edge inside the band) is a strike and is not averaged; a held frame
  is a good frame. `strikes`
  (5) in a row set a fault; a good frame resets the count. Transient bad
  frames below that count are simply skipped.
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
- `FocusCalibration` (dataclass: `edges` (1 or 2 `EdgeCalibration`), `p0_um`,
  `created`, `record_path`, `warnings`, `kind="edges"`; properties `boxes` and
  `linear_range_um`; `to_dict`, `from_dict`, `rezeroed(e0_px, p0_um)`).
  `from_dict` accepts an `"edges"` dict and a single `"edge"` dict (wrapped as
  one edge) and raises `ValueError` otherwise or when the edge count is not 1
  or 2. `rezeroed` takes one `e0_px` per edge (`ValueError` on a count
  mismatch).
- `load_calibration(d) -> tuple[FocusCalibration | None, str]`: never raises.
- `edge_noise_um(e) -> float`, `FusedError` (frozen dataclass: `along_um`,
  `sigma_um`, `used`, `per_edge_um`, `disagreement_um`, `reason`, `note`) and
  `fuse_edges(fixes, cal, params) -> FusedError`, as described under "Optional
  second edge and fusion". `fuse_edges` raises `ValueError` if the number of
  fixes differs from the number of edges.
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
  - two edges: `max_edge_disagreement_um` 1.0 (the fusion uses the larger of
    this and `4 * sqrt(n1^2 + n2^2)`) and `hold_band_um` 0.15 (beyond the
    limit, an edge within this of the target is held on instead of faulting);
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

- `start`: `params`, `calibration` (a `FocusCalibration` dict, or a single-edge
  dict), `p_start_um`. A malformed or old calibration becomes a worker fault
  `bad calibration: ...` (the worker does not crash).
- `stop`. The worker keeps its calibration for the preview.
- `update_params`: `params`. A changed `offset_um` also clears the averaging
  window.
- `set_camera`: `exposure_ms`, `gain`, `channel`, `roi`.
- `set_calibration`: `calibration` (dict or None). It is held for the preview,
  so the worker can compute the fused error without a lock. Ignored (and
  logged) while running, since the controller was built from the start
  calibration. A bad dict gives an `error` message and keeps the old one.
- `set_edge_boxes`: `boxes`, a list of zero to two `(x, y, w, h)`. These are
  the preview overlay boxes, measured at up to 5 Hz in preview. More than two
  boxes or a malformed box gives an `error` message and leaves the boxes
  unchanged. When not running, it clears the fixes and overlay at once.
  `set_edge_box` (`box` or None) is kept as an alias for one box or none.
- `measure`: `n_frames`, `p_um`, `boxes` (a list of `{"box", "axis",
  "polarity"}` entries; `axis` and `polarity` may be None), `return_frame`,
  and an optional `timeout_s`. The old `box`, `axis` and `polarity` keys still
  work and make one entry. With neither, it falls back to the preview boxes;
  with no boxes at all, the reply has `edges == []`, `n = 0` and reason
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
  `saturated_frac`, and the two-edge fields below. The top-level `edge_px`,
  `x`, `y`, `contrast`, `angle_deg`, `residual_px` and `valid_fraction` are
  those of edge 1.
  - `along_um` is the **fused** error with the offset applied, or None when
    unusable (no matching calibration, or a bad measurement). In preview it is
    computed only when the worker holds a calibration (from `start` or
    `set_calibration`) and the preview boxes equal its boxes after `snap_box`
    (same count and order).
  - `sigma_um`: the fused noise estimate, None when not available.
  - `disagreement_um`: `|a1 - a2|`, None unless two edges are both good.
  - `held_edge`: 0-based index of the edge the frame holds on when the edges
    disagree beyond the limit but that edge is within `hold_band_um` of its
    target, else None. The panel numbers edges from 1.
  - `note`: for example `"edge 2 lost (<reason>); using edge 1"` or
    `"edges disagree by 1.23 um; holding on edge 1 (0.04 um from target)"`.
  - `edges`: a list with one entry per box, each `{edge_px, x, y, ok, reason,
    along_um}`. `along_um` is that edge's own error (no offset), None when the
    frame is not fused or that edge is bad.
  - `ok` is true when the fused error exists (`along_um is not None`); without
    a calibration it is true when the fix is ok and `check_fix` has no
    complaint. `reason` is the fused reason, else the edge-1 reason.
- `measurement`: the reply to `measure`. Fields: `p_um`, `edges`, `roi` and
  `frame`, plus the first entry's fields repeated at the top level. Each entry
  of `edges` has `n`, `edge_mean`, `edge_sd` (ddof 1; 0.0 for n = 1; NaN for
  n = 0), `x_mean`, `y_mean`, `contrast_mean`, `angle_mean`, `residual_mean`,
  `axis`, `polarity` and `reason`. `reason` is the last bad reason when
  `n == 0`, else empty (`"no frames from camera"` if nothing settled at all).
  `frame` is the last frame with any good box (full ROI, raw dtype), only if
  `return_frame` was set.
- `fault`: `reason`.
- `status`: `state` (idle, preview, running, stopped, fault).
- `error`: `message`, `traceback`.

Behaviour:

- Which boxes are measured: while running, the calibration's boxes (with their
  axes and polarities). In preview, the edge boxes (the calibration's axis and
  polarity are used only if the snapped box equals the calibration's box, else
  they are auto-detected). In `measure`, the request's boxes. With no box
  there is no fix, no overlay and no telemetry.
- Every frame measures all boxes, then `fuse_edges` gives either a bad
  measurement (`ctl.bad(reason)`, a strike) or a good one (`ctl.good()`, and
  the fused `along_um` goes into the averaging window). The controller's
  `linear_range_um` is the calibration's (the smallest edge range).
- The worker tells its source the commanded P (`set_actuator_um`) when it sends
  a move, so the fake closes the loop.
- `measure` snaps: it ignores frames until the settle passes, writes every
  settled frame to the live view buffer (good or not, so a failing box is
  still visible), averages the good ones per box, and detects axis and polarity
  once per box. It ends when every box has `n` good frames or at the deadline.
  A box with no good frame after `n + 6` settled frames is given up on
  individually (the other boxes continue), instead of waiting for the full
  deadline.
- The shared-memory `spot` field carries the edge-1 overlay point
  `(fix.x, fix.y)`. `lateral_px` and `amplitude` no longer exist in
  telemetry. The second cross in the live view is fed from telemetry `edges`.

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
  - `calibration` is a `FocusCalibration`. An `EdgeCalibration` is also
    accepted and wrapped as a one-edge calibration.
- `calibrate(boxes, *, span_um=2.0, steps=9, n_frames=3, settle_s=0.25,
  record_path=None) -> FocusCalibration`: see the calibration section. `boxes`
  is one box or a list of one or two. Only with the lock stopped and no MDA
  running.
- `zero(cal, *, n_frames=8) -> FocusCalibration`: re-zero every edge at the
  current P, only while stopped.
- `set_calibration(cal | None)`: gives the worker a calibration for the preview,
  so the fused error is computed (and charted) before Lock. See "Worker".
- `set_edge_boxes(boxes)`: the list of zero to two edge boxes for the preview
  overlay. `set_edge_box(box | None)` is kept as the one-box form.
- `measure(n_frames=20, p_um=None, *, box, axis=None, polarity=None,
  return_frame=False) -> dict`: `box` is one box or a list of two, and `axis`
  and `polarity` are scalars (one box) or per-box lists. Refuses while the lock
  runs. Raises `RuntimeError` ("worker is dead" or "did not answer") instead of
  a bare `queue.Empty`, and `ValueError` for a scalar axis with two boxes or a
  list of the wrong length.
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
  - Two edges: `FocusCalibration` round trip, a single `"edge"` dict wrapped
    as one edge (rig-shaped dict), `edge_noise_um`, `fuse_edges` (inverse
    variance mean, sigma drops by sqrt 2 for equal edges, offset subtracted
    once, disagreement is a bad measurement unless an edge is inside the hold
    band, in which case the frame holds on it, one bad edge gives a note, both
    bad gives a strike with the reason of edge 1), and synthetic cases:
    diagonal motion gives agreeing estimates, a lateral move gives a
    disagreement, the two-edge loop converges, stops on a lateral shift when
    neither edge is at target, and holds through one when an edge is.
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
  - Two edges: two-box calibration recovers both slopes (both signs) and the
    per-edge npz arrays; a box without an edge is dropped with a warning and
    no edge in any box names edge 1; a single-edge calibration still starts;
    the two-edge loop holds under drift; lateral drift makes the edges
    disagree and nothing moves; a held frame reports `held_edge` and never
    faults; a lost edge is a note and the lock continues
    on the other; `zero` re-zeroes both edges; `set_calibration` gives the
    preview an error and `stop` keeps it; `measure` with two boxes and with
    the old single-box keys; the preview box messages.
- `test_spot_focus_widget.py`: settings (defaults, round trip, old and
  malformed calibrations, old `mode: image` files) and the Spot panel (buttons
  Calibrate / Zero here / Lock / Stop, offset, edge boxes 1 and 2, sweep spins,
  camera, LED, ROI, a lock run, Calibrate and Zero here off the GUI thread,
  Lock without a calibration, two-box calibration with both slope signs, the
  rig-shaped single-edge calibration, the telemetry note, the preview error
  after Calibrate) and its dock action.
- `test_spot_live_view.py`: the live view (zoom, ROI and edge-box drawing,
  the second box, the edge2 draw mode, the second cross and ring, overlays,
  levels).
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
- `calibration`, `edge_box`, `edge_box_2`, `sweep_span_um`, `sweep_steps`,
  `sweep_frames` as described above. `params` also holds
  `max_edge_disagreement_um` (1.0) and `hold_band_um` (0.15).

### GUI

- **Spot panel** (`widgets/_spot_focus_panel.py`, hosted by the "Spot Focus
  Lock" action). A vertical splitter holds the live view on top (docked by
  default, minimum height 240 px) and, below it, a scroll area with the
  controls, so the image does not scroll away with them. Top to bottom in the
  controls:
  - Buttons **Calibrate** (tooltip: focus on the cells first; sweeps P around
    here), **Zero here**, **Lock** and **Stop**. Lock is enabled whenever the
    panel is idle, because Lock without a calibration calibrates first.
  - **Offset (um)** spin (-5 to 5): live, persisted, reset to 0 by Zero here
    and Calibrate.
  - State, fault and calibration labels. The calibration label has one line
    per edge (axis, signed slope px/um, R2, residual, noise in um from
    `edge_noise_um`), then the shared zero P, range and created time. A line in
    the attention colour shows the warnings (for example "edge 2 dropped:
    ..."), and "edge box changed since the calibration; recalibrate" when a
    saved box snaps to a different box than the calibration's (checked for
    both boxes).
  - Group **Edge box and calibration sweep**: a **Draw edge box** toggle and
    the saved box as x, y, w, h ("none" until drawn); **Draw edge box 2** and
    **Clear box 2** with box 2 shown as x, y, w, h or "none (optional)"; then
    spins for span (um), steps and frames per step.
  - Group **Camera**:
    - SDK folder: a line edit plus Browse, saved to
      `spot_focus.source.sdk_dir`.
    - A camera info label: model, serial, sensor, bits.
    - A **Preview** toggle and a **Pop out** / **Dock** toggle. Preview lights
      the CRISP LED (CRISP State Ready) and turning it off restores CRISP,
      unless the lock is running. Pop out floats the live view as its own
      window; Dock (or closing that window) puts it back above the controls.
      There is no Show view button: docked, the view is always on screen, and
      the floating window is owned by the panel so it stays above the app.
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
  - Readouts: Moves / min, Move time (mean / max), Edge 1 (px), Edge 2 (px)
    (a dash when there is no second edge), Disagreement (um), Contrast, Frame
    rate, Error (along). The error is the fused one and shows a dash when
    `along_um` is None, else the error in um. A note line shows the telemetry
    `note` or `reason`, for example "edge 2 lost; using edge 1". When a frame
    holds on one edge (`held_edge` in the telemetry), the Disagreement
    readout turns orange and reads for example "1.230 (holding on edge 2)",
    and the note line gives the full text.
  - Group **Parameters**: Deadband, Kp, Ki, Max step, Max rate, Min contrast
    (counts), Min valid rows (fraction), Max angle change (degrees), Max
    edge residual (px), Edge disagreement limit (um, default 1.0) and Hold
    band (um, default 0.15; 0 turns holding off). `avg_frames` and
    `max_saturated_frac` have no spin.
- Panel behaviour:
  - **Calibrate** needs a saved edge box, else the fault label says "Draw an
    edge box across the edge that moves with focus". It passes `[box1]` or
    `[box1, box2]` to the service and runs the sweep on a thread. On success it
    saves the calibration, resets the offset to 0 and moves the setpoint ring
    (one per edge) to the zero edge point. A failure shows "Calibration
    failed: ...".
  - **Zero here** is enabled with a valid calibration and the lock stopped. A
    failure shows "Zero failed: ...".
  - After a Calibrate, a Zero here, loading the settings at service start, and
    Stop, the panel calls `svc.set_calibration(cal)` and
    `svc.set_edge_boxes(boxes)`, so the preview shows the fused error (and the
    chart fills) before Lock.
  - **Draw edge box 2** and **Clear box 2** work like the box 1 button; box 2
    is optional and is saved in `spot_focus.edge_box_2`. The panel advises
    drawing it across a different edge, wide along its normal.
  - **Draw edge box** puts the live view in edge-box draw mode, makes it
    visible (raises it when floating) and starts Preview if it is not on. The
    panel tells the user to draw the box wide along the edge normal (about 14 x
    the edge sigma). The box is saved in `spot_focus.edge_box` and sent to the
    worker with `set_edge_box` (also when the service is created, and again
    after Stop).
  - While the lock runs, Zero here, Calibrate and the Draw edge box buttons
    are disabled, and a box drawn then is refused ("Stop the lock before
    moving the edge box.").
  - An old or malformed saved calibration shows its message and never crashes
    the constructor.
- **Live view** (`widgets/_spot_live_view.py`), titled "Spot camera". It is
  docked in the panel's splitter by default and can be popped out into a
  resizable top-level window and docked again (the choice is kept in memory
  only). The scene and zoom survive both moves:
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
  - **Draw ROI**, **Draw edge box** and **Draw edge box 2** buttons are mutually
    exclusive draw modes (`draw_target` is `"roi"`, `"edge"`, `"edge2"` or
    None).
    - Draw ROI: dragging a rectangle (snapped to even ints, clamped to the
      sensor) emits `roiDrawn(x, y, w, h)`. The panel fills its ROI spins and
      applies them.
    - Draw edge box: the same drag (minimum 16 px) emits
      `edgeBoxDrawn(x, y, w, h)`. The panel stores the box and sends it to the
      worker.
    - Draw edge box 2: the same drag emits `edgeBox2Drawn(x, y, w, h)`.
    - A valid drag turns its button off (one box per click); a click or a
      sliver keeps the mode. The panel can switch the mode with
      `set_edge_draw_mode(on)` and follows it through `edgeDrawModeChanged`.
  - Overlays: the sensor outline, the current ROI, the edge box (dashed
    violet), the edge point (a cross at the box centre on the measured edge)
    and the zero edge point (a ring). For the optional second edge there is a
    second dashed box in its own colour (`set_edge_box2(box | None)`), a
    second cross (`set_secondary_point(xy | None)`, fed by the panel from
    telemetry `edges[1]`) and a second ring (`set_secondary_setpoint(xy |
    None)`). The edge boxes are user data: frames and `clear()` do not remove
    them. The status text reads "edge (x, y)", plus "(not ok)" for a failed
    fix.
  - Levels are linear. The fixed range is 0 to 2^bits-1, with an **Auto
    levels** option (min to max). Auto levels is display only; the measurement
    always uses raw counts.
  - Under the cursor it shows x, y and the pixel value.
  - A 30 Hz timer polls `latest_frame` and repaints only on a new `seq`. It
    runs while the view is visible and polling is on (`set_polling`); the panel
    turns polling on with Preview and off when Preview stops, docked or
    floating.
  - Turning Preview off clears the image. A docked view keeps its space (no
    layout jump); a floating window is hidden.
  - Closing the floating window docks the view again and keeps Preview
    running (`windowClosed` is still emitted). The floating window is hidden
    with the panel (a closed dock, another tab) and with `shutdown()`, and
    deleted with the panel.

## First rig results (2026-10-28)

The first P sweep on the rig (a 9-step, +-1 um calibration on the top edge,
saved as an npz) was analysed offline. The lock itself has not been run yet.

- **Top edge** (box 762, 590, 176, 114): +1.92 px/um, R2 0.993, residual 0.106
  px. The box takes in the rounded top-left corner of the patch, so the line
  straightness is 2.1 px and the angle -5.9 degrees.
- **Left edge:** +2.10 px/um, R2 0.9986, residual 0.027 um, monotonic. It is
  blurrier (10-90 % width 25-28 px against 13-15 px for the top edge) and
  curved.
- **Right edge:** only +0.76 px/um (noisier in um). **Bottom edge:** does not
  respond (slope -0.25, not monotonic).
- **The motion is not a rigid shift.** The left, right, top and bottom edges
  move by different amounts and both x and y change; the patch also shrinks.
  That is why each edge gets its own slope, and why a lateral shift can be told
  from a focus change.
- **Noise.** The strip chart range of +-0.15 um is consistent with about 0.05
  um of single-frame noise (3 sigma). It is not a bug: the chart autoscales
  with a minimum of +-0.05 um. The `noise_px` stored in the calibration (6
  static frames, 0.017 um) understates the single-frame noise during a sweep.
- **Range fault, fixed.** The first Lock faulted because
  `p_min_um` / `p_max_um` defaulted to 0..150 um while piezo P sits at about
  -45.5 um on this rig. They now default to None; the excursion limit is the
  main guard, and the service reads the piezo `Position` limits at Start only
  if they contain the start P. Fault messages print the range and the start P.
- **Advice.** Move box 1 off the rounded corner: a clean top box (about x
  850-970) has noise 0.022 um and straightness 0.8 px, although its slope is
  lower (+1.53 px/um). Put box 2 across the left edge, wide along x. Fusing
  the two gives the lowest noise (0.024 um against 0.048 um for the saved top
  box alone) and the lateral-drift check.

## Rig test plan (GO / NO-GO)

Do these in order. Stop at the first NO-GO and fix it before going on.

1. **Find the edge.** Preview on, LED on, exposure so the patch is about
   100-180 counts with 0 % saturated. Watch the patch while jogging P by +-1 um
   by hand (lock stopped) and identify the edge that moves most.
   - GO: the edge visibly moves. Otherwise try another edge.
2. **Draw the edge box** across that edge: about 60 % of the edge length, about
   2x the expected travel plus a margin on each side along the normal, ghost
   out. Keep the rounded corner out of the box (on the top edge, start at about
   x 850). With P still, the cross should sit on the edge and stay still
   (noise below 0.2 px).
3. **Draw edge box 2 (optional).** Press Draw edge box 2 and draw across a
   second edge that moves, for example the left edge, wide along x. The second
   cross should sit on that edge and stay still.
   - GO: both crosses are steady. Otherwise Clear box 2 and use one edge.
4. **Calibrate.** Focus on the cells by eye (OPM image), then press Calibrate
   (default +-1 um, 9 steps).
   - GO: for each edge `|slope| >= 1 px/um`, R2 `>= 0.98`, residual below 0.3
     px, no "left the box", no hysteresis warning. With two boxes the label
     shows one line per edge and no "edge 2 dropped" warning.
   - NO-GO: shrink the span, move or enlarge the box, or pick another edge. If
     only box 2 is dropped, the lock still works on edge 1, but fix box 2
     before relying on the disagreement check.
5. **Preview error.** Without locking, with the calibration loaded, check that
   the chart and the Error readout are live (the worker holds the calibration
   in preview), and jog P by hand by +-0.3 um.
   - GO: the Error follows the jog with the right sign, and the two edge
     readouts move together.
6. **Lock** with offset 0. Set the offset to +0.3 um and back to 0.
   - GO: P follows within 2-3 s with at most 3 moves and settles inside the
     dead band.
   - Check that the cells stay sharp over 30 min with moves/min low (below 10).
7. **Disagreement readout (two edges).** Lateral beam walk cannot be produced
   safely on the rig, so check the readout instead.
   - While locked, the Disagreement (um) readout stays below 0.1 um.
   - Cover one edge region (for example block the light so box 2 loses its
     edge): the lock continues on the other edge, the note line says "edge 2
     lost; using edge 1", and P keeps being held. Uncover it and the note
     disappears.
   - If the edges disagree by more than the limit (1.0 um) while one edge is
     within the hold band (0.15 um) of its target, the Disagreement readout
     turns orange with "holding on edge K" and the note line says so; the lock
     keeps running on that edge. If neither edge is in the band, the frames
     are strikes and the lock faults after 5.
   - GO: both behaviours as described. NO-GO: any move toward a wrong target
     while one edge is covered, or a persistent disagreement above 1.0 um
     (or a "holding on edge K" that stays on for minutes).
8. **Fault drills.**
   - LED off: fault "edge lost" after 5 bad frames, P held.
   - Exposure doubled: no move (the contrast changes, the edge position does
     not).
   - Start an MDA Z-stack while locked: no serial traffic is added during
     volumes.
