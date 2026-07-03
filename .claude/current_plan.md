# Spectral-Channel Splitter Cropping + Per-Channel OME-Zarr Save

## Context

The microscope has 2 physical cameras, each preceded by an image splitter that
puts 2 fixed sub-regions ("spectral channels") on one sensor — 4 total:

| Region | Excitation | Fluorophore class |
|---|---|---|
| Camera-1 top | 488nm | GFP-like |
| Camera-1 bottom | 405nm | Calcein Violet-like |
| Camera-2 top | 561nm | mScarlet-like |
| Camera-2 bottom | 642nm (config preset `638nm`) | CF647-like |

Today, `MultiCameraHandler` (`_multi_camera_handler.py`) writes one **full-sensor**
file per physical camera. The downstream HPC preprocessing pipeline instead needs
**one file per spectral channel** (up to 4), pre-cropped to the splitter's fixed
sub-region, and in OME-Zarr (fast to read on the HPC side) rather than the
combined full-frame TIFF/Zarr. There's also a `"AllLasers"` Micro-Manager config
preset that fires all 4 lasers at once — when that preset is active, all 4
regions carry real signal and should be saved; when a single-laser preset (e.g.
`"488nm"`) is active, only its one matching region does.

The crop must happen in software immediately after the frame comes off the
buffer — it is **not** achievable via the camera's hardware ROI, since both
splitter sub-regions live on one camera's full sensor readout. The 4 rectangles
are fixed for this microscope's optical path but must be re-drawable (camera
alignment can drift), persisted, and applied automatically thereafter. Users
also need the rectangles overlaid on the live and snap views to visually verify
alignment when recalibrating the splitters/cameras.

**Confirmed with user:**
- Crop selection is **auto-derived from the active laser preset per MDA event**
  (not a static manual enable list) — `"AllLasers"` → all 4 channels;
  a single-laser preset → only its one matching channel.
- **Snap stays display-only** (ROI overlay for alignment only). Confirmed via
  `core_actions.py` that Snap currently has zero save-to-disk behavior — this
  feature does not add one. Only MDA runs crop-and-save.

## 1. Settings model — `src/pymmcore_gui/_settings.py`

Add a new nested settings model next to `WindowSettingsV1`, following the exact
same `BaseMMSettings` nesting pattern already used for `window: WindowSettingsV1`:

```python
class SpectralChannelConfig(BaseModel):
    """One splitter sub-region tied to one laser preset on one physical camera."""
    name: str                     # per-file label, e.g. "GFP_488"
    camera: str                   # physical camera device label, e.g. "Camera-1"
    laser_preset: str             # single-laser preset this region maps to, e.g. "488nm"
    rect: tuple[int, int, int, int] | None = None   # (x, y, w, h), full-sensor pixels
    enabled: bool = False
    writer_format: Literal["ome-zarr", "ome-tiff", "tiff-sequence"] = "ome-zarr"

    @property
    def is_ready(self) -> bool:
        return self.enabled and self.rect is not None


def _default_spectral_channels() -> list[SpectralChannelConfig]:
    return [
        SpectralChannelConfig(name="GFP_488",           camera="Camera-1", laser_preset="488nm"),
        SpectralChannelConfig(name="CalceinViolet_405", camera="Camera-1", laser_preset="405nm"),
        SpectralChannelConfig(name="mScarlet_561",      camera="Camera-2", laser_preset="561nm"),
        SpectralChannelConfig(name="CF647_638",         camera="Camera-2", laser_preset="638nm"),
    ]


class SpectralChannelSettingsV1(BaseMMSettings):
    """Splitter-crop feature configuration."""
    enabled: bool = False                       # master on/off for the whole feature
    laser_config_group: str = "Lasers"
    all_lasers_preset: str = "AllLasers"
    channels: list[SpectralChannelConfig] = Field(default_factory=_default_spectral_channels)
```

Add to `SettingsV1` (next to `window: WindowSettingsV1 = Field(...)`):

```python
    spectral: SpectralChannelSettingsV1 = Field(default_factory=SpectralChannelSettingsV1)
```

Notes (verified against the actual `_settings.py` source):
- `rect=None` + `enabled=False` by default so nothing crops/saves until a user
  has drawn and enabled a region — `is_ready` is the single gate used everywhere.
- `laser_config_group`/`all_lasers_preset` are **hardcoded here**, not imported
  from `asi_z_stack.common.HardwareConstants` (which defines the same two
  concepts for the separate `microscope-control` entrypoint) — this respects
  the existing module boundary between the main GUI app and `asi_z_stack/`.
  Flagged as a minor duplication; not worth a shared-constants module for two
  strings.
- Verified `_good_data_only()` behavior: because `channels` is a bare
  `list[SpectralChannelConfig]` (not a `BaseModel` field itself), a malformed
  persisted entry causes the **entire list** to be dropped and defaults
  repopulate (whole-list `TypeAdapter` validation, not per-item). This is
  acceptable — defaults are safe — just don't expect partial salvage.

Persistence follows the existing pattern exactly: mutate
`SettingsV1.instance().spectral...` then call `.flush()`.

## 2. Cropping + multi-file-writer MDA handler — new `src/pymmcore_gui/_spectral_channel_handler.py`

Mirrors `MultiCameraHandler`'s structure (the informal `sequenceStarted` /
`frameReady` / `sequenceFinished` listener protocol, auto-connected by
`mda_listeners_connected()`), reusing `_sanitize` and `without_cam_index` from
`_multi_camera_handler.py`.

**Critical correctness detail, verified against `pymmcore_plus`'s
`_5DWriterBase.frameReady`:** a writer's store shape comes from
`position_sizes(seq)` (derived from `seq.sizes`, which has a `"c"` axis sized
by `len(seq.channels)` whenever the sequence has multiple channels), and it
indexes as `tuple(event.index[k] for k in pos_sizes)`. If we handed a
per-channel writer the *original* 4-channel sequence, its store would be
shaped `c=4` but only ever receive `c=1` worth of frames — 3 empty channel
slots per file, violating the "own file, correctly shaped" requirement.

**Fix:** in `sequenceStarted`, build a channel-collapsed sequence once for all
spectral writers:
```python
self._writer_seq = self._sequence.model_copy(update={"channels": ()})
```
so every per-channel store is a clean single-channel array. `without_cam_index`
is still applied before forwarding (harmless/consistent with the existing
pattern; the writer only iterates keys present in `pos_sizes` anyway).

Sketch:
```python
def _channel_output_path(base: str | Path, ch: SpectralChannelConfig) -> str:
    ...  # strip any known suffix from base, append _<sanitized-name> + format-specific suffix
         # (.ome.zarr default, .ome.tiff, or bare dir for tiff-sequence)

class SpectralChannelHandler:
    def __init__(self, output, channels, laser_group, all_lasers_preset, *, mmcore=None): ...

    def _active_channels_for_event(self, event: useq.MDAEvent) -> list[SpectralChannelConfig]:
        ch = event.channel
        preset = ch.config if (ch is not None and ch.group == self._laser_group) else None
        if preset is None:
            return list(self._channels)                 # fallback: don't silently drop data
        if preset == self._all_lasers_preset:
            return list(self._channels)
        return [c for c in self._channels if c.laser_preset == preset]

    def sequenceStarted(self, seq, meta=None):
        self._sequence = seq
        self._writer_seq = seq.model_copy(update={"channels": ()})
        ...

    def frameReady(self, frame, event, meta):
        label = meta.get("camera_device") or self._mmc.getCameraDevice()
        clean_event = without_cam_index(event)
        for ch in self._active_channels_for_event(event):
            if ch.camera != label:
                continue
            x, y, w, h = ch.rect
            crop = np.ascontiguousarray(frame[y : y + h, x : x + w])
            self._get_writer(ch).frameReady(crop, clean_event, meta)

    def sequenceFinished(self, seq): ...   # call sequenceFinished on every writer
```
`_get_writer`/`_call_sequence_started` mirror `MultiCameraHandler`'s
implementation (creates via `handler_for_path`, passes `self._writer_seq` not
the original sequence, handles varying `sequenceStarted` arity).

**Substitution point** — `src/pymmcore_gui/widgets/_mda_widget.py`,
`GuiMDAWidget.execute_mda` (the exact existing hook, verified by direct read):
```python
def execute_mda(self, output):
    if isinstance(output, (str, Path)):
        spectral = SettingsV1.instance().spectral
        active_cams = set(self._physical_camera_labels())
        ready = [c for c in spectral.channels if c.is_ready and c.camera in active_cams]
        if spectral.enabled and ready:
            output = SpectralChannelHandler(
                output, ready, spectral.laser_config_group,
                spectral.all_lasers_preset, mmcore=self._mmc,
            )
        elif self._mmc.getNumberOfCameraChannels() > 1:
            output = MultiCameraHandler(output, mmcore=self._mmc)
    sequence = self.value()
    self._mmc.run_mda(sequence, output=output)
```
Feature disabled or no ready channels → behavior is unchanged from today.
Extract the physical-camera-label lookup (currently private inside
`MultiCameraHandler._physical_camera_labels`) into one shared free function in
`_multi_camera_handler.py` and import it from both the MDA widget and the new
handler, rather than a third copy (`NDVViewersManager` already has its own
independent copy too — leave that one, out of scope here).

## 3. ROI overlay rendering on live/snap previews — `widgets/image_preview/_pygfx_image.py`

The live/snap previews are a **custom pygfx scene** (`PygfxImagePreview`:
`self._scene`, `self._image_node`, `self._camera` with `scale_y = -1`) — not
ndv's `ArrayViewer` (which only supports one `RectangularROIModel` per viewer).
Because it's a custom scene, we can add up to 4 independent rectangle overlays
freely as extra `pygfx.WorldObject`s:

```python
def set_roi_overlays(self, rois: list[tuple[str, tuple[int, int, int, int]]]) -> None:
    """rois: (label, (x, y, w, h)) in full-sensor pixel coords."""
    self.clear_roi_overlays()
    for label, (x, y, w, h) in rois:
        pts = np.array([[x, y, 0], [x+w, y, 0], [x+w, y+h, 0], [x, y+h, 0], [x, y, 0]], np.float32)
        line = pygfx.Line(pygfx.Geometry(positions=pts),
                           pygfx.LineMaterial(thickness=2.0, color="#00E5FF"))
        self._scene.add(line)
        self._roi_overlays.append(line)
    self._canvas.request_draw(self._draw_function)

def clear_roi_overlays(self) -> None:
    for obj in getattr(self, "_roi_overlays", []):
        self._scene.remove(obj)
    self._roi_overlays = []
```

**Refresh triggers:**
- In `NDVViewersManager._create_camera_preview(camera_label)` (`_ndv_viewers.py`),
  right after constructing `preview`, read `SettingsV1.instance().spectral`,
  filter `is_ready` channels with `camera == camera_label`, call
  `preview.set_roi_overlays(...)`. Since previews are created identically for
  live and snap (both go through `_get_or_create_camera_preview`), this single
  hook covers both surfaces (requirement 4).
- Add `NDVViewersManager.refresh_roi_overlays()` that loops `self._camera_previews`
  and reapplies from settings; called by the config widget after Save.

**Open risk (flag before implementing):** the exact pixel→world-unit mapping
for the pygfx `Image` node (offset, whether `scale_y=-1` needs a corresponding
translation) is **not yet verified**. Before trusting overlay placement, draw a
test rect at `(0, 0, W, H)` and confirm it exactly frames the full image.

## 4. Interactive ROI drawing + config UI

**Action registry** — `src/pymmcore_gui/actions/widget_actions.py`, following
the existing `show_camera_roi` pattern exactly:
```python
class WidgetAction(ActionKey):
    ...
    SPECTRAL_CHANNELS = "pymmcore_gui.spectral_channel_config"

def create_spectral_channel_config(parent: QWidget) -> QWidget:
    from pymmcore_gui.widgets._spectral_channel_config import SpectralChannelConfigWidget
    return SpectralChannelConfigWidget(parent=parent, mmcore=_get_core(parent))

show_spectral_channels = WidgetActionInfo(
    key=WidgetAction.SPECTRAL_CHANNELS,
    text="Spectral Channels",
    icon="mdi:grid-large",
    create_widget=create_spectral_channel_config,
    dock_area=DockWidgetArea.LeftDockWidgetArea,
)
```
Register it in `_main_window.py`'s `MENUS[Menu.PLUGINS]` list (verified —
currently `[WidgetAction.CRISP]`, the same "instrument-specific dock widget"
category this belongs to): add `WidgetAction.SPECTRAL_CHANNELS` alongside it.
No other `_main_window.py` changes needed — `ActionInfo.for_key(...).to_qaction(...)`
wires it generically.

**Config dock widget** — new `src/pymmcore_gui/widgets/_spectral_channel_config.py`,
a `QWidget(parent, *, mmcore)` with:
- Master "Enable spectral cropping" checkbox → `spectral.enabled`.
- One row per `SpectralChannelConfig`: camera `QComboBox` (physical camera
  labels), laser-preset `QComboBox` (from `mmc.getAvailableConfigs("Lasers")`),
  writer-format `QComboBox`, enabled `QCheckBox`, and 4 `QSpinBox`es (x, y, w, h)
  as the numeric fine-tune / fallback path.
- A **"Draw on live view"** button per row.
- **Save** button → writes rows back into `SettingsV1.instance().spectral`,
  calls `.flush()`, then `refresh_roi_overlays()` on the main window's
  `NDVViewersManager`.

**Draw interaction — important finding:** `pymmcore_widgets`' existing
ROI-drawing code (`control/_rois/{roi_manager,canvas_event_filter,_vispy}.py`)
is built on **vispy** `SceneCanvas` (`canvas_to_world` via
`view.scene.transform.imap`). This app's live/snap previews are **pygfx**, so
that code is **not reusable as-is** — only its shape (install an event filter,
invert the scene transform screen→world) is instructive. Implement a small
pygfx-native drag handler on `PygfxImagePreview`:
```python
def begin_roi_draw(self, on_done: Callable[[tuple[int, int, int, int]], None]) -> None:
    # disable self._controller (pan/zoom) while drawing
    # register pointer_down/move/up handlers on self._renderer
    # draw a live rubber-band pygfx.Line while dragging
    # on release: convert screen coords -> pixel coords (same mapping as §3,
    # verify empirically), clamp to frame shape, re-enable controller, call on_done(rect)
```
Flow: row's "Draw" button → get/create that channel's camera preview via
`NDVViewersManager` → `preview.begin_roi_draw(callback)` → callback fills the
row's spin boxes and calls `preview.set_roi_overlays(...)` for live feedback →
user clicks **Save** to persist.

## 5. Implementation order

1. **Settings model (§1).** Verify: defaults round-trip through
   `model_dump_json()`/reload; corrupt one persisted channel and confirm
   graceful fallback to defaults (per the whole-list validation caveat above).
2. **Handler + headless unit test (§2).** Build a fake `useq.MDASequence` with
   a `"Lasers"`-group channel list (e.g. `["405nm", "488nm", "561nm", "638nm", "AllLasers"]`)
   and feed synthetic frames through `SpectralChannelHandler.frameReady`.
   Assert: `AllLasers` event → 4 crops written; single-preset event → exactly 1
   crop to the right file; crop shape matches the configured rect; each
   resulting store has **no leftover `c` axis** (the channel-collapse
   correctness check) and is fully populated, not sparse. No hardware needed.
3. **Substitute into `execute_mda` (§2, end).** Extract the shared
   `_physical_camera_labels` helper. Verify the gate: feature off → identical
   to current behavior; on with ready channels → new handler selected.
4. **Overlay rendering (§3).** Manual: open a live view, hardcode a test rect,
   confirm it exactly frames the image and top/bottom splits land correctly —
   this is where the pixel↔world mapping gets pinned down for real.
5. **Config UI + draw interaction (§4), last.** End-to-end manual check: draw
   all 4 regions across both cameras' live views, Save, restart the app,
   confirm the overlays reappear (persistence roundtrip). Run a short MDA
   using the `"AllLasers"` preset and one using a single-laser preset;
   confirm the correct number of separate `.ome.zarr` files exist with
   correctly-cropped, correctly-populated contents.

Run `just lint` and `just test` after each stage; add the handler unit test to
`tests/` alongside existing MDA-handler tests.

## 6. Risks to keep visible during implementation

1. **pygfx pixel↔world coordinate mapping is unverified** — the single
   biggest unknown; blocks trusting both the overlay (§3) and the draw
   interaction (§4) until empirically pinned down.
2. **Channel-axis collapse assumes a "clean" laser acquisition** — correct
   only if a sequence uses *either* `AllLasers` *or* a set of distinct,
   non-repeating single-laser presets in the `"Lasers"` group. A sequence
   mixing presets unusually, repeating one, or using a different multi-entry
   channel group could make two source frames target the same spectral
   writer at the same `(t, z, p)` and silently overwrite. Worth a guard/warning
   in `sequenceStarted` if this turns out to matter in practice.
3. **Camera label matching** relies on the settings' `camera` string exactly
   matching `meta["camera_device"]` (`getPhysicalCameraDevice` label). The
   config UI's camera dropdown must be sourced from the same helper the
   handler uses, or a renamed/mismatched camera silently drops a channel.
4. **Redrawing a rect for an already-used dataset path** changes the crop
   shape; since each MDA run should write to a fresh output path (the
   existing norm), this is a non-issue as long as users don't reuse an old
   output path after recalibrating — worth a one-line note in the UI, not
   code.

## Critical files

- `src/pymmcore_gui/_settings.py` — new `SpectralChannelConfig` / `SpectralChannelSettingsV1`, new `spectral` field on `SettingsV1`
- `src/pymmcore_gui/_spectral_channel_handler.py` **(new)** — mirrors `src/pymmcore_gui/_multi_camera_handler.py`
- `src/pymmcore_gui/widgets/_mda_widget.py` — `GuiMDAWidget.execute_mda` substitution point
- `src/pymmcore_gui/widgets/image_preview/_pygfx_image.py` — `set_roi_overlays`/`clear_roi_overlays`/`begin_roi_draw`
- `src/pymmcore_gui/_ndv_viewers.py` — overlay refresh hook in `_create_camera_preview`, new `refresh_roi_overlays()`
- `src/pymmcore_gui/actions/widget_actions.py` — new `WidgetAction.SPECTRAL_CHANNELS` + factory + `WidgetActionInfo`
- `src/pymmcore_gui/widgets/_spectral_channel_config.py` **(new)** — config dock widget
- `src/pymmcore_gui/_main_window.py` — add `WidgetAction.SPECTRAL_CHANNELS` to `MENUS[Menu.PLUGINS]`

## Verification

- `uv run pytest tests/ -q` (add the new handler unit test from step 2 here)
- `just lint`
- `just run` → open both camera live-view docks, confirm ROI overlays appear
  once drawn/saved and persist across restart
- Run a real (or demo-config) MDA with an `AllLasers` step and a single-laser
  step; inspect the output directory for the expected `*_<channel-name>.ome.zarr`
  files and confirm each opens correctly and contains only its cropped region
