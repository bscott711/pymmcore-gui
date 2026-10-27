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
  - Reads P once with `getPosition`, sets CRISP to Ready (property `"CRISP State"`) and remembers the previous state.
  - Sets the LED intensity if one is configured, then sends `start`.
- `stop()` sends `stop` and restores the previous CRISP state. It never moves P.
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
