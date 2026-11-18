"""The EPI (Orca) camera worker service and the EPI Snap / Live helpers.

The Orca lives in its own :class:`EpiCameraWorkerService`, a subclass of the OPM
:class:`~pymmcore_gui.asi_z_stack.camera_worker_service.CameraWorkerService` with
its own ``_active`` slot, so ``CameraWorkerService.get_active()`` keeps meaning "the
OPM service" in both modes. It hosts exactly one worker built from
``settings.epi.camera`` (the Orca is in no cfg), and never touches the main core's
cameras or the OPM pool.

Lifecycle: created and spawned lazily on the first successful EPI entry (the mode
controller's ``epi_service_provider`` hook), reused on every later entry, never
spawned per use, left alone by config reloads, and shut down once at app exit
(:func:`shutdown_epi_service`).

The Snap / Live helpers light only the LED of the current ``EPI LEDs`` preset at its
stored percent (D12) and close it again; they never call ``open_selected_lasers``.
The preset only sets the ``EPI LED Selector`` (a Multi Shutter used as a label
store), so applying it never lights or darkens an LED.
"""

from __future__ import annotations

import logging
import weakref
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar

import numpy as np

from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.asi_z_stack.camera_handoff import (
    CameraHandoffSnapshot,
    _CameraSnapshot,
)
from pymmcore_gui.asi_z_stack.camera_worker import CameraWorkerConfig
from pymmcore_gui.asi_z_stack.camera_worker_service import (
    CameraWorkerService,
    CameraWorkerServiceState,
)
from pymmcore_gui.asi_z_stack.worker_pool import CameraWorkerHandle
from pymmcore_gui.epi._led import apply_led, channel_for_preset, resolve_led_channels
from pymmcore_gui.epi._mode import (
    EpiModeRefusal,
    ScopeMode,
    _is_deleted,
    current_mode,
)
from pymmcore_gui.epi._tiger_guard import EPI_CHANNEL_GROUP

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_plus import CMMCorePlus
    from PyQt6.QtCore import QObject

    from pymmcore_gui._settings import EpiCameraSettingsV1
    from pymmcore_gui.asi_z_stack.worker_pool import CameraWorkerPool

logger = logging.getLogger("pymmcore_gui")

#: Shown when EPI entry is refused because the Orca settings are empty.
NOT_CONFIGURED = "Set the Orca camera in settings (epi.camera) first"


class EpiCameraWorkerService(CameraWorkerService):
    """Owns the one persistent Orca camera worker for the whole session.

    Built from :class:`~pymmcore_gui._settings.EpiCameraSettingsV1` by :meth:`begin`
    instead of from the main core's released cameras. Everything else (Live, Snap,
    exposure, ROI, shutdown) is the base class, which is allowed in EPI mode here.
    """

    # The base class uses ``type(self)._active``, so this slot is separate and
    # ``CameraWorkerService.get_active()`` still returns the OPM service.
    _active: ClassVar[EpiCameraWorkerService | None] = None
    _ALLOW_IN_EPI: ClassVar[bool] = True

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._camera: EpiCameraSettingsV1 | None = None
        self.liveStateChanged.connect(self._on_live_state_changed)

    # ------------------------------------------------------------------ start

    def begin(self, camera: EpiCameraSettingsV1) -> None:
        """Describe the camera from settings and spawn its worker (async).

        A no-op unless the service is ``INACTIVE`` (first entry, or after a failed
        spawn), so a later EPI entry reuses the running worker.

        Parameters
        ----------
        camera : EpiCameraSettingsV1
            The Orca identity and geometry.

        Raises
        ------
        ValueError
            *camera* is not fully configured, or its dtype is not a numpy dtype.
        """
        if self._state is not CameraWorkerServiceState.INACTIVE:
            return
        if not camera.is_configured():
            raise ValueError(NOT_CONFIGURED)
        try:
            itemsize = np.dtype(camera.dtype).itemsize
        except TypeError as exc:
            raise ValueError(f"Unknown Orca dtype {camera.dtype!r}") from exc
        self._camera = camera.model_copy(deep=True)
        self._snapshot = CameraHandoffSnapshot(
            n_cameras=1,
            camera_labels=(camera.label,),
            core_camera_role="",
            image_width=camera.width,
            image_height=camera.height,
            bytes_per_pixel=itemsize,
            n_components=1,
            per_camera={camera.label: _CameraSnapshot(None, None, {})},
        )
        self._roi = {}
        with self._lock:
            self._state = CameraWorkerServiceState.SPAWNING
        type(self)._active = self
        self.spawn_async()

    def _build_worker_handle(self, label: str) -> CameraWorkerHandle:
        camera = self._camera
        snapshot = self._snapshot
        assert camera is not None
        assert snapshot is not None
        cam = snapshot.per_camera.get(label)
        return CameraWorkerHandle.from_config(
            CameraWorkerConfig(
                camera_label=label,
                adapter_module=camera.adapter_module,
                adapter_device_name=camera.device_name,
                pre_init_props=tuple(
                    (str(p), str(v)) for p, v in camera.pre_init_props
                ),
                geometry=(camera.height, camera.width, camera.dtype),
                # The Orca runs on its own default (internal/software) trigger:
                # the worker never writes TriggerMode, expose-out or timing
                # properties, whatever the camera exposes (user rule, 2026-11-18).
                manage_trigger=False,
                property_snapshot=cam.property_values if cam else {},
                circular_buffer_mb=self.hw.worker_circular_buffer_mb,
                live_max_fps=self.hw.live_max_fps,
            ),
            n_slots=self.hw.frame_ring_slots_per_camera,
        )

    # ----------------------------------------------- not the OPM release/reload

    def begin_release(self, mmc: CMMCorePlus, hw: object) -> bool:
        """Not applicable: the Orca is never released from the main core."""
        return False

    @classmethod
    def prepare_for_reload(cls, mmc: CMMCorePlus) -> None:
        """Do nothing: a config reload must not touch the cfg-independent Orca."""

    # ------------------------------------------------------------------- Live

    def _on_live_state_changed(self, live: bool) -> None:
        """Close the Live LED when Live ends on its own (error, dead worker)."""
        if not live and self.state is not CameraWorkerServiceState.LIVE:
            close_live_led()

    # ------------------------------------------------------------ the EPI MDA

    def claim_for_run(self) -> CameraWorkerPool:
        """Take the pool for one EPI MDA run (state ``MDA``), stopping Live first.

        Not ``acquire_for_mda``: the EPI engine never calls the OPM service's
        method. :meth:`release_from_mda` ends the claim. While claimed, Snap, Live
        and camera-property changes are refused, so nothing else reads the pipes.

        Raises
        ------
        RuntimeError
            A worker is dead, the pool is missing, or the service is busy/not ready.
        """
        self.check_workers_alive()
        with self._lock:
            was_live = self._state is CameraWorkerServiceState.LIVE
        if was_live:
            self.stop_live()
            close_live_led()
        with self._lock:
            pool = self._pool
            if self._state is not CameraWorkerServiceState.IDLE or pool is None:
                raise RuntimeError(
                    "The Orca camera is not ready for an MDA "
                    f"(state={self._state.name})."
                )
            self._state = CameraWorkerServiceState.MDA
        return pool


# ---------------------------------------------------------------- the registry

_epi_service: EpiCameraWorkerService | None = None


def get_epi_service() -> EpiCameraWorkerService | None:
    """The EPI camera service, or ``None`` before the first EPI entry."""
    svc = _epi_service
    if svc is None or _is_deleted(svc):
        return None
    return svc


def active_camera_service() -> CameraWorkerService | None:
    """The service Snap / Live / exposure / ROI act on in the current mode.

    The EPI service in EPI mode (once it exists), else the OPM service.
    """
    if current_mode() is ScopeMode.EPI and (svc := get_epi_service()) is not None:
        return svc
    return CameraWorkerService.get_active()


def epi_camera_precondition() -> str | None:
    """EPI-entry check: a refusal reason while the Orca settings are incomplete."""
    camera = SettingsV1.instance().epi.camera
    if not camera.is_configured():
        return NOT_CONFIGURED
    try:
        np.dtype(camera.dtype)
    except TypeError:
        return f"Unknown Orca dtype {camera.dtype!r} in settings (epi.camera.dtype)"
    return None


def ensure_epi_service(parent: QObject) -> tuple[EpiCameraWorkerService, bool]:
    """Create the EPI service if needed and make sure its worker is (being) spawned.

    Returns the service and whether it was created by this call. A service whose
    spawn failed (``INACTIVE``) is spawned again; one that is spawning or running
    is reused untouched.

    Parameters
    ----------
    parent : QObject
        Qt parent of a newly created service (the main window).
    """
    global _epi_service
    svc = get_epi_service()
    created = svc is None or svc.parent() is not parent
    if created:
        svc = EpiCameraWorkerService(parent)
        _epi_service = svc
    assert svc is not None
    if svc.state is CameraWorkerServiceState.INACTIVE:
        svc.begin(SettingsV1.instance().epi.camera)
    return svc, created


def epi_service_provider(
    parent: QObject,
    on_created: Callable[[EpiCameraWorkerService], None] | None = None,
) -> Callable[[], EpiCameraWorkerService | None]:
    """Build the mode controller's ``epi_service_provider`` hook.

    Holds *parent* weakly (it owns the controller that holds the hook).

    Parameters
    ----------
    parent : QObject
        The main window.
    on_created : Callable[[EpiCameraWorkerService], None] | None
        Called once with a newly created service (connect the preview to it).
    """
    ref = weakref.ref(parent)

    def provider() -> EpiCameraWorkerService | None:
        if (owner := ref()) is None:
            return None
        svc, created = ensure_epi_service(owner)
        if created and on_created is not None:
            on_created(svc)
        return svc

    return provider


def shutdown_epi_service(timeout: float | None = None) -> None:
    """Shut the EPI service down (app exit). Safe when there is none; never raises."""
    global _epi_service
    svc = get_epi_service()
    _epi_service = None
    if svc is None:
        return
    try:
        svc.shutdown(timeout=timeout)
    except Exception:
        logger.exception("Error shutting down the EPI camera worker service")


# ------------------------------------------------------------- Snap / Live (D12)


@dataclass(frozen=True)
class EpiLight:
    """The LED an EPI Snap / Live lights."""

    label: str
    """Device label of the LED channel."""
    percent: int
    """Stored intensity (1-100, the card's native unit)."""
    name: str
    """Band name, e.g. ``"488"``."""


def resolve_epi_light(mmc: CMMCorePlus) -> EpiLight:
    """Return the LED of the current ``EPI LEDs`` preset at its stored percent.

    Raises
    ------
    EpiModeRefusal
        No preset is selected, it names no configured channel, or that channel's
        LED is not loaded. Nothing is lit or snapped then.
    """
    epi = SettingsV1.instance().epi
    try:
        preset = str(mmc.getCurrentConfig(EPI_CHANNEL_GROUP))
    except Exception:
        preset = ""
    if not preset:
        raise EpiModeRefusal(
            f"Select an LED in the {EPI_CHANNEL_GROUP!r} channel group first "
            "(an LED left open in the LED panel also hides the selection)."
        )
    led = channel_for_preset(preset, epi.leds)
    if led is None:
        raise EpiModeRefusal(
            f"Preset {preset!r} is not an LED channel set in settings (epi.leds)."
        )
    label = resolve_led_channels(mmc, epi.card_address).get(led.channel)
    if label is None:
        raise EpiModeRefusal(
            f"The LED for preset {preset!r} (channel {led.channel}) is not loaded."
        )
    return EpiLight(label=label, percent=led.percent, name=led.name)


def _ready_service(*, allow_live: bool) -> EpiCameraWorkerService:
    svc = get_epi_service()
    if svc is None:
        raise EpiModeRefusal("The Orca camera service is not running.")
    state = svc.state
    ok = state is CameraWorkerServiceState.IDLE or (
        allow_live and state is CameraWorkerServiceState.LIVE
    )
    if not ok:
        raise EpiModeRefusal(
            f"The Orca camera is not ready (state={state.name}); try again shortly."
        )
    return svc


def _close_led(mmc: CMMCorePlus, label: str) -> None:
    try:
        mmc.setShutterOpen(label, False)
    except Exception:
        logger.exception("Could not close LED %s", label)


#: ``(core, LED label)`` of the LED that Live opened, so stopping Live closes it.
_live_led: tuple[CMMCorePlus, str] | None = None


def close_live_led() -> None:
    """Close the LED Live opened, if any (idempotent)."""
    global _live_led
    led, _live_led = _live_led, None
    if led is not None:
        _close_led(*led)


def epi_snap(mmc: CMMCorePlus) -> dict[str, np.ndarray]:
    """Snap one Orca frame lit by the current preset's LED, then close the LED.

    Live, if running, is stopped first (and its LED closed).

    Raises
    ------
    EpiModeRefusal
        The Orca is not ready, or no LED is selected (no light, no snap).
    """
    svc = _ready_service(allow_live=True)
    if svc.state is CameraWorkerServiceState.LIVE:
        epi_stop_live()
    light = resolve_epi_light(mmc)
    try:
        apply_led(mmc, light.label, light.percent)
        return svc.snap()
    finally:
        _close_led(mmc, light.label)


def epi_start_live(mmc: CMMCorePlus) -> None:
    """Light the current preset's LED and start Orca Live (LED closed on failure).

    Raises
    ------
    EpiModeRefusal
        The Orca is not ready, or no LED is selected.
    """
    global _live_led
    svc = _ready_service(allow_live=True)
    if svc.state is CameraWorkerServiceState.LIVE:
        return
    light = resolve_epi_light(mmc)
    try:
        apply_led(mmc, light.label, light.percent)
        _live_led = (mmc, light.label)
        svc.start_live()
    except BaseException:
        _live_led = None
        _close_led(mmc, light.label)
        raise


def epi_stop_live() -> None:
    """Stop Orca Live, then close the LED Live opened."""
    svc = get_epi_service()
    try:
        if svc is not None:
            svc.stop_live()
    finally:
        close_live_led()
