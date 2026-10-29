import json
import os
import threading
import warnings
from dataclasses import asdict
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Any, Literal, cast

from platformdirs import user_data_dir
from pydantic import (
    Base64Bytes,
    BaseModel,
    Field,
    TypeAdapter,
    ValidationError,
    WrapSerializer,
    model_validator,
)
from pydantic.fields import FieldInfo
from pydantic_settings import (
    BaseSettings,
    EnvSettingsSource,
    PydanticBaseSettingsSource,
    SettingsConfigDict,
)

from pymmcore_gui.actions.widget_actions import WidgetAction

if TYPE_CHECKING:
    from pymmcore_gui.asi_z_stack.image_focus import (
        FocusBox,
        FocusCalibration,
        LockLimits,
    )
    from pymmcore_gui.asi_z_stack.spot_focus import (
        FocusCalibration as SpotFocusCalibration,
    )
    from pymmcore_gui.asi_z_stack.spot_focus import SpotLoopParams

APP_NAME = "pymmcore-gui"
USER_DATA_DIR = Path(user_data_dir(appname=APP_NAME))
USER_DATA_DIR.mkdir(parents=True, exist_ok=True)
SETTINGS_FILE_NAME = USER_DATA_DIR / "pmm_settings.json"
TESTING = "PYTEST_VERSION" in os.environ
_GLOBAL_SETTINGS: "SettingsV1 | None" = None


class BaseMMSettings(BaseSettings):
    """Config shared across all settings subclasses."""

    model_config = SettingsConfigDict(
        # don't fail if an extra key is present.  just ignore it
        # (this is important for backwards compatibility)
        # note: this could also be "include", if we want an older version of the app
        # to be able to open a newer version's settings file, without losing data
        extra="ignore",
    )


class MMGuiUserPrefsSource(PydanticBaseSettingsSource):
    """Loads variables from file json file persisted to disk."""

    @staticmethod
    def exists() -> bool:
        """Return True if the settings file exists."""
        return SETTINGS_FILE_NAME.exists()

    @staticmethod
    def content() -> str:
        """Return the contents of the settings file."""
        return SETTINGS_FILE_NAME.read_text(errors="ignore")

    @staticmethod
    def values() -> dict[str, Any]:
        """Return the contents of the settings file."""
        if not MMGuiUserPrefsSource.exists():
            return {}

        if not (content := MMGuiUserPrefsSource.content()):
            # file exists but is empty
            return {}

        values = json.loads(content)
        if not isinstance(values, dict):
            raise ValueError("Settings file does not contain a dictionary.")
        return values

    def _read_settings(self) -> dict[str, Any]:
        """Return the settings values from the source."""
        try:
            return MMGuiUserPrefsSource.values()
        except Exception as e:
            # Never block the application from starting because of a settings file
            warnings.warn(
                f"Failed to read settings from {SETTINGS_FILE_NAME}: {e}",
                RuntimeWarning,
                stacklevel=2,
            )
        return {}

    def __call__(self) -> dict[str, Any]:
        """Return Settings values for this source."""
        if os.getenv("MMGUI_NO_SETTINGS"):  # pragma: no cover
            return {}
        values = self._read_settings()
        return _good_data_only(SettingsV1, values, warn=True)

    def get_field_value(
        self, field: FieldInfo, field_name: str
    ) -> tuple[Any, str, bool]:
        """Return the value for a field (required by ABC)."""
        # Nothing to do here. Only implement the return statement to make mypy happy
        return None, "", False  # pragma: no cover


def _good_data_only(
    cls: type[BaseModel], data: dict[str, Any], warn: bool = True
) -> dict[str, Any]:
    """Attempt to extract only the good fields from `data` that are valid for `cls`."""
    cleaned: dict[str, Any] = {}
    model_fields = cls.model_fields
    for key, value in data.items():
        if key in model_fields:
            # check whether the value is valid for the field
            field = model_fields[key]
            annotation = field.annotation
            if isinstance(annotation, type) and issubclass(annotation, BaseModel):
                cleaned[key] = _good_data_only(annotation, value, warn=warn)
            else:
                try:
                    TypeAdapter(field.annotation).validate_python(value)
                    cleaned[key] = value
                except ValidationError as e:
                    if warn:
                        warnings.warn(
                            f"Could not validate key {key!r} from settings file: {e}",
                            RuntimeWarning,
                            stacklevel=2,
                        )
        elif warn:
            # user supplied something that doesn't exist in the model
            # ignore it, but warn the user
            warnings.warn(
                f"Key {key!r} from settings file not found in model.",
                RuntimeWarning,
                stacklevel=2,
            )
            # we still include it for backwards compatibility
            # it could be an additional function that "cleans" settings.
            cleaned[key] = value
    return cleaned


def _default_widgets() -> set[str]:
    """The default set widgets that open on launch."""
    return {WidgetAction.CONFIG_GROUPS, WidgetAction.MDA_WIDGET}


# set of widgets that are sorted when serialized
WidgetNames = Annotated[set[str], WrapSerializer(lambda v, h: sorted(h(v)))]


class WindowSettingsV1(BaseMMSettings):
    """Settings related to window positioning and geometry."""

    geometry: Base64Bytes | None = None
    """Position and size of the main window. Restored with .restoreGeometry()"""
    window_state: Base64Bytes | None = None
    """State of main window's toolbars and dockwidgets. Restored with .restoreState()"""
    dock_manager_state: Base64Bytes | None = None
    """State of dock_manager dockwidgets"""
    open_widgets: WidgetNames = Field(default_factory=_default_widgets)
    """Set of widgets to load on startup, or when restoring settings."""

    @model_validator(mode="before")
    @classmethod
    def _migrate_names(cls, values: dict[str, Any]) -> dict[str, Any]:
        """Migrate old settings to new ones.

        This is a temporary solution until we have a proper migration system.
        """
        # migrate from old settings
        if "initial_widgets" in values:
            values["open_widgets"] = values.pop("initial_widgets")
        return values


class SpectralChannelConfig(BaseModel):
    """One splitter sub-region tied to one laser preset on one physical camera.

    Which regions are actually saved during an MDA is derived from the laser
    presets used by the running sequence (see
    :func:`~pymmcore_gui._spectral_channel_handler.channels_for_sequence`), not
    from a per-channel flag: exciting a laser always saves its region, and
    unused regions are skipped. The output file format follows the MDA save
    widget's choice rather than a per-channel setting.
    """

    # TODO: add an optional per-channel "always save this region even when its
    # laser isn't in the MDA" override for power users capturing extra regions.

    name: str
    """Per-file label, e.g. ``"mNG_488nm"``."""
    camera: str
    """Physical camera device label, e.g. ``"Camera-1"``."""
    laser_preset: str
    """The single-laser config preset this region maps to, e.g. ``"488nm"``."""
    position: Literal["top", "bottom"] = "top"
    """Which half of the image splitter's sensor this region occupies.

    Used only to auto-populate sibling regions when the first region on any
    camera is drawn (see :class:`~pymmcore_gui.widgets._spectral_channel_config.
    SpectralChannelConfigWidget`) -- it is not otherwise consulted for saving.
    """
    rect: tuple[int, int, int, int] | None = None
    """``(x, y, w, h)`` in full-sensor pixel coordinates. ``None`` until drawn."""

    @property
    def is_ready(self) -> bool:
        """Whether this channel has a drawn rect and can be saved."""
        return self.rect is not None


def _default_spectral_channels() -> list[SpectralChannelConfig]:
    return [
        SpectralChannelConfig(
            name="mNG_488nm", camera="Camera-1", laser_preset="488nm", position="top"
        ),
        SpectralChannelConfig(
            name="Electra_405nm",
            camera="Camera-1",
            laser_preset="405nm",
            position="bottom",
        ),
        SpectralChannelConfig(
            name="mScarlet_561nm",
            camera="Camera-2",
            laser_preset="561nm",
            position="top",
        ),
        SpectralChannelConfig(
            name="CF647_638nm",
            camera="Camera-2",
            laser_preset="638nm",
            position="bottom",
        ),
    ]


class SpectralChannelSettingsV1(BaseMMSettings):
    """Image-splitter crop-and-save-per-channel feature configuration."""

    enabled: bool = False
    """Master on/off switch for the whole feature."""
    crop_camera_roi: bool = False
    """Crop each camera's hardware ROI to the bounding box of its regions.

    Shorter rolling readout (faster slices) and less data per frame; the
    regions stay in full-sensor coordinates (see ``pymmcore_gui._camera_crop``).
    """
    laser_config_group: str = "Lasers"
    all_lasers_preset: str = "AllLasers"
    channels: list[SpectralChannelConfig] = Field(
        default_factory=_default_spectral_channels
    )


class ArgusStreamSettingsV1(BaseMMSettings):
    """Real-time frame-streaming-to-Argus feature configuration.

    See ``pymmcore_gui._argus_stream`` for the streaming client this
    configures, and ``opym_local/src/opym/stream/`` (Argus side, canonical)
    for the wire protocol and receiver these fields feed. v1 only supports
    single-position MDA sequences (Argus's ``(T, C)`` grid has no position
    axis). Multi-camera sequences are supported *only* when spectral-channel
    cropping (see :class:`SpectralChannelSettingsV1`) resolves each camera's
    regions into distinct channel identities -- an uncropped multi-camera
    acquisition is skipped, same as multi-position (see
    ``_argus_stream._session.ArgusStreamSession``).
    """

    enabled: bool = False
    """Master on/off switch for the whole feature."""
    ssh_host: str = "Argus"
    """``Host`` alias from the user's ``~/.ssh/config`` to tunnel through."""
    direct_endpoint: str = ""
    """Argus's direct 10 GbE endpoint, e.g. ``tcp://137.216.250.14:5556``.

    Empty: stream through the SSH tunnel only. Set: each run connects here
    first and falls back to the tunnel if Argus doesn't answer within a few
    seconds. The tunnel moved ~33 MB/s; this link runs at 10 GbE. It is plain
    TCP, accepted by Argus only from this PC's IP (``opym-receive``'s
    ``OPYM_STREAM_ALLOW_IPS``) and allowed through the site firewall only
    from here.
    """
    compress_over_tunnel: bool = True
    """Compress frames (blosc: lz4 + bitshuffle, ~2.5x on camera data) when
    streaming through the SSH tunnel, if Argus accepts them. The tunnel moves
    ~33 MB/s, so this is ~2.5x the frames per second; the direct link is
    faster than one thread can compress, so frames always go to it raw.
    """
    local_port: int = 5555
    """Local end of the SSH port-forward that the ZMQ DEALER connects to.

    With several ``stream_links``, link k's forward listens on
    ``local_port + k``.
    """
    stream_links: int = 4
    """Parallel connections each run streams over.

    Through the tunnel, each link is its own SSH forward (and ssh process).
    One SSH connection is capped by sshd's fixed 2 MB window per round trip,
    however fast the network is; measured from the rig on 2026-09-26: 1 link
    63 MB/s, 2: 128, 4: 255, 8: 514. 4 links carry ~600 MB/s of camera data
    after compression. Only used once Argus advertises "links"; otherwise
    link 0 carries everything. Measure with
    ``python -m pymmcore_gui._argus_stream.linkbench``. Read at app launch.
    """
    ssh_processes: int = 0
    """How many ``ssh`` processes carry the links; 0 means one per link.

    Separate processes are separate TCP connections (and encrypt in
    parallel); several links in one process share its connection.
    """
    remote_port: int = 5555
    """Port ``opym-receive`` binds to on ``127.0.0.1`` on the Argus side.

    Matches ``opym.stream.receiver.DEFAULT_BIND_ADDR`` (``tcp://127.0.0.1:5555``).
    """
    buffer_budget_mb: int = 4096
    """RAM budget for buffered-but-unacked volumes before raising an alarm.

    Sized generously (several volumes deep) since the policy on exceeding it
    is to warn, not to drop data -- see ``_argus_stream._session``.
    """
    buffer_hard_cap_mb: int = 0
    """RAM the stream may hold unsent, across runs, before it gives up.

    Past it (Argus unreachable for too long), the current run stops
    streaming: its unsent volumes are dropped, Argus is told not to keep
    the partial copy, and the run goes to Argus by Globus instead.
    Acquisition and the local save are never affected. 0 means a quarter of
    this PC's RAM.
    """
    gpfs_scratch_root: str = ""
    """GPFS root the local save directory structure is mirrored under.

    Each run's actual ``raw_root`` is this root plus the MDA save widget's
    own ``save_dir``, drive letter stripped -- e.g. a local save to
    ``S:/20260922-SVO-YG_0.1umBead_PSF/...`` streams to
    ``<gpfs_scratch_root>/20260922-SVO-YG_0.1umBead_PSF/...``, matching
    what a Globus transfer of the same local tree would have produced.
    Each channel's raw store is then named
    ``<base_name>_<channel_name>.ome.zarr`` directly under THAT (see
    ``opym.stream.rawmirror.store_path_for_channel`` on the Argus side),
    not nested under a further ``base_name`` subdirectory. Streaming
    refuses to start if this is unset.
    """
    output_format: Literal["both", "ome-zarr", "tiff"] = "both"
    """Format the processed (deskewed/deconvolved) result is kept in on Argus.

    ``"tiff"``: per-frame OME-TIFFs, which ChimeraX opens directly.
    ``"ome-zarr"``: one pyramidal OME-Zarr per run (napari); the TIFF frames
    are removed once the zarr copy is verified. ``"both"``: keep both, at
    roughly twice the disk. PetaKit5D's MIPs are kept in every mode.
    """


class MdaWriterSettingsV1(BaseMMSettings):
    """Local MDA save-writer tuning.

    See ``pymmcore_gui._async_writer`` and
    ``pymmcore_gui._vendored.mda_handlers._ome_zarr_writer``.
    """

    zarr_compression: bool = False
    """Compress OME-Zarr chunks (blosc/lz4).

    Off by default: for fast multi-camera / SPIM acquisitions the compress
    step is the write-throughput bottleneck and raw camera frames compress
    poorly. Turn on only if disk space matters more than acquisition-keep-up.
    """
    backlog_budget_mb: int = 32768
    """Per-writer cap on frame bytes buffered in RAM waiting to be written.

    Each per-camera / per-channel writer drains its own queue on its own
    thread; if the disk can't keep up and a writer's backlog would exceed
    this, further frames for that writer are dropped (with a visible alarm)
    rather than growing until the process runs out of memory. A healthy run
    never approaches this -- the backlog stays near zero.
    """


class FocusOffsetSettingsV1(BaseMMSettings):
    """Per-excitation-wavelength focus (Z) offset configuration.

    Corrects axial chromatic aberration on the ASI SPIM rig: CRISP holds one
    physical plane, but each excitation wavelength focuses slightly
    differently. When enabled, a small per-wavelength Z offset is applied at
    the per-volume channel switch by shifting the CRISP lock setpoint (see
    ``pymmcore_gui.asi_z_stack.engine.ASISPIMEngine``); the offset travels
    into the sequence as ``useq.Channel.z_offset`` (see
    ``pymmcore_gui.widgets._mda_widget``).

    Offsets are stored **absolute** -- each measured against one calibration
    datum -- and ``locked_preset`` picks the runtime zero, so re-locking CRISP
    on a different channel never needs the offsets re-measured.
    """

    enabled: bool = False
    """Master on/off switch for the whole feature."""
    laser_config_group: str = "Lasers"
    """MM config group whose presets name the excitation wavelengths."""
    all_lasers_preset: str = "AllLasers"
    """Simultaneous-multi-wavelength preset -- never given an offset."""
    crisp_label: str = "CRISPAFocus:P:34"
    """CRISP AutoFocus device servoing the focus piezo. Empty => auto-discover."""
    counts_per_um: float | None = None
    """Signed CRISP lock-offset counts per micron of focus shift, from the
    bench probe (``crisp_focus_offset_tuning.probe_lock_offset_response``).
    ``None`` falls back to the device's own
    ``|Calibration Gain| / Calibration Range(um)`` sensitivity."""
    locked_preset: str = ""
    """Which wavelength CRISP is currently focused/locked on -- the runtime
    zero that gets no net move. Empty => use the running sequence's first
    channel."""
    apply_live: bool = False
    """Auto-apply the active channel's offset during ordinary live preview
    (opt-in; off by default -- otherwise it silently fights the CRISPy panel)."""
    offsets: dict[str, float] = Field(default_factory=dict)
    """Per-preset focus offset in microns, absolute vs the calibration datum,
    keyed by laser-preset name (e.g. ``{"561nm": 0.35}``)."""


class SpimTimingSettingsV1(BaseMMSettings):
    """Advanced per-slice timing for the ASI galvo SPIM MDA.

    See ``pymmcore_gui.asi_z_stack.spim_timing`` for the timing model; the
    engine writes these to the galvo card and PLogic for every MDA.
    """

    mode: Literal["legacy", "all_rows"] = "legacy"
    """``"all_rows"``: pseudo-global exposure -- the camera's Level Trigger
    pulse covers readout + exposure and PLogic delays the laser until every
    row is integrating, so every row gets the full exposure, matching Live.
    ``"legacy"``: the proven setup, camera and laser share one PLogic pulse
    (later-read rows get less light)."""
    galvo_settle_ms: float = 25.75
    """Galvo settle between each slice step and the camera trigger (written to
    both ``SPIMDelayBeforeCamera`` and ``SPIMDelayBeforeLaser``)."""
    volume_start_settle_ms: float = 0.0
    """Extra settle before each volume's first slice, for the galvo's jump
    back to the start of the sweep (``SPIMDelayBeforeSide``)."""
    readout_ms_override: float | None = None
    """Camera rolling readout time in ms; ``None`` uses the value the cameras
    report (PVCAM ``Timing-ReadoutTimeNs``, maximum over cameras)."""
    fallback_readout_ms: float = 13.5
    """Readout time assumed when the cameras haven't reported one yet (e.g.
    the first MDA of a session before any Snap/Live) and no override is set."""
    margin_ms: float = 0.5
    """All Rows safety margin: the laser starts this long after the last row
    has started, and the camera pulse ends this long after the laser."""
    level_trigger_overlap: bool = False
    """All Rows with the cameras in Level Trigger Overlap: the next frame
    starts exposing while the previous one reads out, so a slice costs about
    readout + exposure instead of 2 x readout + exposure."""


class FocusBoxSettings(BaseModel):
    """Persisted form of :class:`~pymmcore_gui.asi_z_stack.image_focus.FocusBox`."""

    camera: str
    """Physical camera device label, e.g. ``"Camera-1"``."""
    preset: str
    """Single-laser preset whose frames are measured, e.g. ``"488nm"``."""
    slice_index: int = 0
    """Which slice of the volume is measured."""
    rect: tuple[int, int, int, int]
    """``(x, y, w, h)`` in full-sensor pixel coordinates (x = column, y = row)."""

    def to_box(self) -> "FocusBox":
        """Return the numpy-only box the focus-lock logic works with."""
        from pymmcore_gui.asi_z_stack.image_focus import FocusBox

        return FocusBox(self.camera, self.preset, self.slice_index, self.rect)


class CalibrationRecord(BaseModel):
    """Persisted form of ``image_focus.FocusCalibration`` (see that module)."""

    rows_per_um: float
    """Signed rows of content shift per +1 um of piezo."""
    intercept_rows: float = 0.0
    r2: float
    """Fit quality; the lock refuses calibrations below ``min_r2``."""
    range_um: tuple[float, float]
    """Linear range of the sweep that was kept (min, max piezo)."""
    n_points: int
    date: str
    """ISO timestamp the calibration was made."""

    def to_calibration(self) -> "FocusCalibration":
        """Return the numpy-only calibration the focus-lock logic works with."""
        from pymmcore_gui.asi_z_stack.image_focus import FocusCalibration

        return FocusCalibration(**self.model_dump())

    @classmethod
    def from_calibration(cls, cal: "FocusCalibration") -> "CalibrationRecord":
        """Build a record from a fresh :func:`fit_calibration` result."""
        return cls(**asdict(cal))


class ImageFocusSettingsV1(BaseMMSettings):
    """Image-based focus lock configuration (legacy; Spot is the only mode).

    Per volume, the row-intensity profile inside ``box`` is compared with a
    reference and the piezo is moved to cancel the measured row shift (see
    ``pymmcore_gui.asi_z_stack.image_focus``). The limits mirror
    :class:`~pymmcore_gui.asi_z_stack.image_focus.LockLimits`.
    """

    mode: Literal["spot"] = "spot"
    """The focus hold: ``"spot"``, the standalone spot lock
    (:class:`SpotFocusSettingsV1`), is the only one exposed."""
    box: FocusBoxSettings | None = None
    """The measurement box; ``None`` until the user draws one."""
    calibrations: dict[str, CalibrationRecord] = Field(default_factory=dict)
    """Piezo-to-row-shift calibrations keyed by
    :func:`~pymmcore_gui.asi_z_stack.image_focus.calibration_key`
    (``"camera|preset|window"``)."""
    end_of_run: Literal["restore", "stay"] = "restore"
    """After the run, return the piezo to where it started or leave it."""

    loop_gain: float = 0.5
    """Fraction of the measured error corrected per volume."""
    deadband_rows: float = 0.5
    """Errors up to this size are left alone."""
    max_step_um: float = 0.3
    """Largest piezo move in a single volume."""
    max_excursion_um: float = 3.0
    """Largest distance the piezo may end up from where the run started."""
    min_confidence: float = 0.5
    """Minimum peak correlation to trust a measurement."""
    max_centroid_disagreement_rows: float = 3.0
    """Largest allowed gap between the correlation and centroid shifts."""
    min_signal: float = 1.0
    """Minimum background-subtracted profile mean."""
    max_strikes: int = 3
    """Consecutive rejected measurements before the lock falls back."""
    max_saturated: int = 3
    """Consecutive clipped moves before the lock falls back."""
    min_r2: float = 0.9
    """Calibrations fitting worse than this are refused."""

    def to_limits(self) -> "LockLimits":
        """Return the numpy-only limits the focus-lock logic works with."""
        from pymmcore_gui.asi_z_stack.image_focus import LockLimits

        return LockLimits(
            loop_gain=self.loop_gain,
            deadband_rows=self.deadband_rows,
            max_step_um=self.max_step_um,
            max_excursion_um=self.max_excursion_um,
            min_confidence=self.min_confidence,
            max_centroid_disagreement_rows=self.max_centroid_disagreement_rows,
            min_signal=self.min_signal,
            max_strikes=self.max_strikes,
            max_saturated=self.max_saturated,
            min_r2=self.min_r2,
        )


class SpotSourceSettingsV1(BaseModel):
    """Where the spot lock's frames come from (see ``toupcam_source.make_source``)."""

    kind: Literal["toupcam", "fake"] = "toupcam"
    """``"toupcam"`` is the AmScope camera; ``"fake"`` is a simulated spot."""
    serial: str | None = None
    """Camera serial (``None`` takes the first camera found)."""
    sdk_dir: str | None = None
    """Folder holding the vendor's ``toupcam.py``; ``None`` uses the
    ``TOUPCAM_SDK_DIR`` environment variable."""
    fake: dict[str, Any] = Field(default_factory=dict)
    """Keyword arguments of ``FakeSpotSource`` (``kind == "fake"`` only)."""


class SpotLoopParamsSettings(BaseModel):
    """Persisted form of ``asi_z_stack.spot_focus.SpotLoopParams``."""

    kp: float = 0.6
    ki: float = 0.05
    deadband_um: float = 0.03
    max_step_um: float = 0.5
    max_rate_hz: float = 2.0
    avg_frames: int = 4
    offset_um: float = 0.0
    min_contrast: float = 10.0
    max_saturated_frac: float = 0.002
    min_valid_fraction: float = 0.5
    max_angle_change_deg: float = 5.0
    max_edge_residual_px: float = 3.0
    max_edge_disagreement_um: float = 1.0
    hold_band_um: float = 0.15
    """When the two edges disagree, hold on an edge this close to the target."""
    p_min_um: float | None = None
    p_max_um: float | None = None
    max_excursion_um: float = 5.0
    strikes: int = 5

    @model_validator(mode="before")
    @classmethod
    def _drop_legacy_range(cls, data: Any) -> Any:
        """Files written with the old 0..150 um default load as "no range".

        The real piezo P runs negative, so that default faulted a healthy lock.
        """
        if (
            isinstance(data, dict)
            and data.get("p_min_um") == 0.0
            and data.get("p_max_um") == 150.0
        ):
            data = {**data, "p_min_um": None, "p_max_um": None}
        return data

    def to_params(self) -> "SpotLoopParams":
        """Return the numpy-only parameters the spot loop works with."""
        from pymmcore_gui.asi_z_stack.spot_focus import SpotLoopParams

        return SpotLoopParams.from_dict(self.model_dump())


class SpotFocusSettingsV1(BaseMMSettings):
    """Spot focus lock: the CRISP LED spot on a side camera, held by piezo P.

    The loop runs from Start until Stop, independent of any acquisition (see
    ``docs/spot_focus_lock.md``).
    """

    source: SpotSourceSettingsV1 = Field(default_factory=SpotSourceSettingsV1)
    exposure_ms: float = 5.0
    gain: int = 100
    """Camera gain in percent (ToupTek convention, 100 = unity)."""
    roi: tuple[int, int, int, int] | None = None
    """Camera ROI ``(x, y, w, h)``; ``None`` uses the full frame."""
    channel: Literal["sum", "red"] = "red"
    """Which Bayer sites the spot is measured on. The LED is 720 nm on a colour
    sensor, so only the red sites carry it."""
    bayer: Literal["RGGB", "BGGR", "GRBG", "GBRG"] = "RGGB"
    led_intensity: float | None = None
    """CRISP LED intensity set on Start; ``None`` leaves it as it is."""
    params: SpotLoopParamsSettings = Field(default_factory=SpotLoopParamsSettings)
    calibration: dict[str, Any] | None = None
    """``FocusCalibration.to_dict()`` of the last calibration (a single-edge
    ``EdgeCalibration`` dict from older versions also loads)."""
    edge_box: tuple[int, int, int, int] | None = None
    """The edge box ``(x, y, w, h)`` in sensor pixels; ``None`` until drawn."""
    edge_box_2: tuple[int, int, int, int] | None = None
    """The optional second edge box, same convention as ``edge_box``."""
    sweep_span_um: float = 2.0
    """Total piezo span of the calibration sweep, um."""
    sweep_steps: int = 9
    """Number of points of the calibration sweep."""
    sweep_frames: int = 3
    """Frames averaged per sweep point."""

    def calibration_status(self) -> "tuple[SpotFocusCalibration | None, str]":
        """Return ``(calibration, message)``; never raises.

        ``message`` is empty for a valid calibration, else says why there is none
        (nothing stored yet, or an old / malformed calibration ignored).
        """
        from pymmcore_gui.asi_z_stack.spot_focus import load_calibration

        return load_calibration(self.calibration)

    def to_calibration(self) -> "SpotFocusCalibration | None":
        """Return the stored calibration, ``None`` if there is none or it is bad."""
        return self.calibration_status()[0]


class SnapshotSettingsV1(BaseMMSettings):
    """Where the Save Image action writes Snap/Live frames."""

    save_dir: Path | None = None
    """Destination folder; ``None`` means ask on the next save."""


class SettingsV1(BaseMMSettings):
    """Global settings for the PyMMCore GUI."""

    model_config = SettingsConfigDict(
        # Prefix for all environment variables. e.g. PMM_<key>=<value>
        env_prefix="PMM_",
    )

    version: Literal["1.0"] = "1.0"
    window: WindowSettingsV1 = Field(default_factory=WindowSettingsV1)
    spectral: SpectralChannelSettingsV1 = Field(
        default_factory=SpectralChannelSettingsV1
    )
    argus_stream: ArgusStreamSettingsV1 = Field(default_factory=ArgusStreamSettingsV1)
    mda_writer: MdaWriterSettingsV1 = Field(default_factory=MdaWriterSettingsV1)
    focus: FocusOffsetSettingsV1 = Field(default_factory=FocusOffsetSettingsV1)
    image_focus: ImageFocusSettingsV1 = Field(default_factory=ImageFocusSettingsV1)
    spot_focus: SpotFocusSettingsV1 = Field(default_factory=SpotFocusSettingsV1)
    snapshot: SnapshotSettingsV1 = Field(default_factory=SnapshotSettingsV1)
    spim_timing: SpimTimingSettingsV1 = Field(default_factory=SpimTimingSettingsV1)

    send_error_reports: bool | None = None
    """Whether to send error reports to the developers, None means undecided."""

    last_config: Path | None = None
    """Path to the last used config file."""
    auto_load_last_config: bool | None = None
    """Load the last used config on startup. None means undecided."""
    fallback_to_demo_config: bool = False
    """Load demo config if no config is found."""

    @property
    def version_tuple(self) -> tuple[int, int, str]:
        """Return the version as a tuple of integers.

        The first two are guaranteed to be integers.  Any additional parts are joined
        with a period and returned as a string.
        """
        major, minor, *rest = self.version.split(".")
        return int(major), int(minor), ".".join(rest)

    # ----------------------- Configurations -----------------------

    @classmethod
    def instance(cls) -> "SettingsV1":
        """Return the singleton instance of the settings."""
        global _GLOBAL_SETTINGS
        if _GLOBAL_SETTINGS is None:
            _GLOBAL_SETTINGS = SettingsV1()
        return _GLOBAL_SETTINGS

    @classmethod
    def settings_customise_sources(
        cls,
        settings_cls: type[BaseSettings],
        init_settings: PydanticBaseSettingsSource,
        env_settings: PydanticBaseSettingsSource,
        dotenv_settings: PydanticBaseSettingsSource,
        file_secret_settings: PydanticBaseSettingsSource,
    ) -> tuple[PydanticBaseSettingsSource, ...]:
        """Define the sources and their order for loading the settings values.

        Values are loaded in the order of the sources. Priority is given to earlier
        sources (i.e. values explicitly passed to the constructor take priority,
        then environment variables, then dotenv files, then the user settings from
        SETTINGS_FILE, then file secrets).
        """
        if TESTING:
            # we're running in tests...
            # don't load the user settings, and change env-prefix
            # I started by using a fixture in conftest.py, to patch this method
            # but it's difficult to ensure that it always gets patched in time
            # this is more guaranteed to work
            cast("EnvSettingsSource", env_settings).env_prefix = "PMM_TEST_"
            return (init_settings, env_settings)

        return (
            init_settings,
            env_settings,
            dotenv_settings,
            MMGuiUserPrefsSource(settings_cls),
            file_secret_settings,
        )

    def flush(self, timeout: float | None = None) -> None:
        """Write the settings to disk.

        If `timeout` is not None, block until the write is complete, or until the
        timeout is reached.
        """
        if TESTING or os.getenv("MMGUI_NO_SETTINGS"):  # pragma: no cover
            return
        # write in another thread, so we don't block the main thread
        thread = threading.Thread(target=self._write_settings)
        thread.start()
        if timeout:
            thread.join(timeout)

    def _write_settings(self) -> None:
        json_str = self.model_dump_json(indent=2, exclude_defaults=True)
        SETTINGS_FILE_NAME.write_text(json_str, errors="ignore")


Settings = SettingsV1


def reset_to_defaults() -> None:
    """Erase user settings and reset to defaults."""
    global _GLOBAL_SETTINGS
    if TESTING or os.getenv("MMGUI_NO_SETTINGS"):  # pragma: no cover
        return
    SETTINGS_FILE_NAME.unlink(missing_ok=True)
    _GLOBAL_SETTINGS = SettingsV1()
