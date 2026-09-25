import json
import os
import threading
import warnings
from pathlib import Path
from typing import Annotated, Any, Literal, cast

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

APP_NAME = "pymmcore-gui"
USER_DATA_DIR = Path(user_data_dir(appname=APP_NAME))
USER_DATA_DIR.mkdir(parents=True, exist_ok=True)
SETTINGS_FILE_NAME = USER_DATA_DIR / "pmm_settings.json"
TESTING = "PYTEST_VERSION" in os.environ
_GLOBAL_SETTINGS: "None | SettingsV1" = None


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
    """Per-file label, e.g. ``"GFP_488"``."""
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
            name="GFP_488", camera="Camera-1", laser_preset="488nm", position="top"
        ),
        SpectralChannelConfig(
            name="CalceinViolet_405",
            camera="Camera-1",
            laser_preset="405nm",
            position="bottom",
        ),
        SpectralChannelConfig(
            name="mScarlet_561", camera="Camera-2", laser_preset="561nm", position="top"
        ),
        SpectralChannelConfig(
            name="CF647_638",
            camera="Camera-2",
            laser_preset="638nm",
            position="bottom",
        ),
    ]


class SpectralChannelSettingsV1(BaseMMSettings):
    """Image-splitter crop-and-save-per-channel feature configuration."""

    enabled: bool = False
    """Master on/off switch for the whole feature."""
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
    local_port: int = 5555
    """Local end of the SSH port-forward that the ZMQ DEALER connects to."""
    remote_port: int = 5555
    """Port ``opym-receive`` binds to on ``127.0.0.1`` on the Argus side.

    Matches ``opym.stream.receiver.DEFAULT_BIND_ADDR`` (``tcp://127.0.0.1:5555``).
    """
    deploy_reverse_port: int = 0
    """Argus-side loopback port for an SSH *reverse* forward back to this
    machine's own local SSH server (``-R <port>:localhost:22``), so code
    changes here (this repo) can be deployed from Argus without anyone at
    this console. ``0`` disables it -- the default, since it needs Windows
    OpenSSH Server running locally (loopback only; it is never exposed to
    the network -- only reachable back through this same outbound SSH
    connection) and Argus's own key in this account's ``authorized_keys``.
    Piggybacks on the one persistent tunnel connection
    :class:`~pymmcore_gui._argus_stream._tunnel.ArgusTunnelManager` already
    keeps open, rather than a second connection someone has to remember to
    start by hand.
    """
    buffer_budget_mb: int = 4096
    """RAM budget for buffered-but-unacked volumes before raising an alarm.

    Sized generously (several volumes deep) since the policy on exceeding it
    is to warn, not to drop data -- see ``_argus_stream._session``.
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
