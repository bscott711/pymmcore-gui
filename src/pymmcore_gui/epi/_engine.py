"""The EPI (Orca + TGLED) MDA engine and the shared EPI sequence validation.

:class:`EpiMDAEngine` runs a v1 EPI acquisition: a single-plane time series of
``EPI LEDs`` channels, one Orca frame per event. It is installed by the mode
controller on entering EPI mode and replaced by the OPM engine on leaving it.

Per event it applies the ``EPI LEDs`` preset (which closes that LED), writes the
run's LED percent, opens the LED, snaps one frame through the EPI camera
service's pool (the same arm as ``CameraWorkerService.snap``), and closes the
LED again. It never touches the lasers, PLogic, the BNC outputs, the galvo, the
OPM pool, any CRISP write or ``setZPosition``.

:func:`check_epi_sequence` is the one validation, used by the engine
(``setup_sequence``, which also covers Console ``run_mda``) and by the MDA
widget's friendly pre-check.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, NamedTuple

from pymmcore_plus.mda import MDAEngine
from useq import AcquireImage

from pymmcore_gui._settings import SettingsV1
from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerServiceState
from pymmcore_gui.epi._camera_service import NOT_CONFIGURED, get_epi_service
from pymmcore_gui.epi._led import all_leds_off as _all_leds_off
from pymmcore_gui.epi._led import apply_led, channel_for_preset, resolve_led_channels
from pymmcore_gui.epi._mode import ScopeMode, current_mode
from pymmcore_gui.epi._tiger_guard import (
    CRISP_Z_LABEL,
    EPI_CHANNEL_GROUP,
    STAGE_Z_LABEL,
)

if TYPE_CHECKING:
    from collections.abc import Iterable

    import numpy as np
    from pymmcore_plus import CMMCorePlus
    from pymmcore_plus.metadata import FrameMetaV1, SummaryMetaV1
    from useq import MDAEvent, MDASequence

    from pymmcore_gui.asi_z_stack.worker_pool import CameraWorkerPool
    from pymmcore_gui.epi._camera_service import EpiCameraWorkerService

logger = logging.getLogger("pymmcore_gui")

#: The CRISP that servos the EPI objective's stage Z (``ZStage:Z:32``), taken from
#: the cfg (T06). This is NOT ``settings.focus.crisp_label``, which is the piezo CRISP.
EPI_CRISP_LABEL = CRISP_Z_LABEL
#: ``CRISP State`` values meaning the servo is actively holding focus.
CRISP_LOCKED_STATES = frozenset({"Lock", "In Focus"})
#: The CRISP property that holds the servo state.
_CRISP_STATE_PROP = "CRISP State"
#: Key under ``MDASequence.metadata`` that holds the per-run EPI block (D11).
SEQUENCE_METADATA_KEY = "pymmcore_gui"
#: What the recorded LED percent means (U6: linearity is not measured).
PCT_UNITS = "card native, uncalibrated"


class EpiSequenceRefused(RuntimeError):
    """The sequence cannot run as a v1 EPI acquisition (sticky toast in the GUI)."""

    persistent_notice = True


class LedRun(NamedTuple):
    """One preset's LED, frozen for a run: ``(label, channel, name, pct)``."""

    label: str
    """Device label of the LED channel, e.g. ``LED:L:37:1``."""
    channel: int
    """Card channel number (1-4)."""
    name: str
    """Band name, e.g. ``"488"``."""
    pct: int
    """Percent (1-100, the card's native unit) written before each open."""


# ------------------------------------------------------------------ validation


def _service_problem() -> str | None:
    """Why the Orca service cannot take a run now, or ``None``."""
    if not SettingsV1.instance().epi.camera.is_configured():
        return NOT_CONFIGURED
    svc = get_epi_service()
    if svc is None:
        return "The Orca camera service is not running: switch to EPI mode first."
    state = svc.state
    if state not in (CameraWorkerServiceState.IDLE, CameraWorkerServiceState.LIVE):
        return f"The Orca camera is not ready (state={state.name}); try again shortly."
    return None


def check_epi_sequence(mmc: CMMCorePlus, sequence: MDASequence) -> str | None:
    """Return why *sequence* cannot run as a v1 EPI acquisition, or ``None``.

    v1 is a single-plane time series of ``EPI LEDs`` channels, so a Z plan, an
    autofocus plan, a grid or more than one XY position are refused, as is any
    channel from another group or whose preset names no loaded, visible LED. A
    single injected stage position (the MDA widget adds the current one, with
    ``z``) is fine. The CRISP Z lock is deliberately not checked here (D13).

    Parameters
    ----------
    mmc : CMMCorePlus
        The main core (to find the loaded LED devices).
    sequence : MDASequence
        The sequence about to run.
    """
    if sequence.z_plan:
        return "EPI v1 is single-plane: remove the Z plan (z-stacks are not supported)."
    if sequence.autofocus_plan:
        return "EPI v1 does not support an autofocus plan: remove it."
    if sequence.grid_plan or len(sequence.stage_positions) > 1:
        return (
            "EPI v1 does not support a grid or more than one XY position: "
            "use a single position (or none)."
        )
    if not sequence.channels:
        return f"Pick at least one channel from the {EPI_CHANNEL_GROUP!r} group."
    epi = SettingsV1.instance().epi
    loaded = resolve_led_channels(mmc, epi.card_address)
    for ch in sequence.channels:
        if ch.group != EPI_CHANNEL_GROUP:
            return (
                f"Channel {ch.config!r} is from the {ch.group!r} group; EPI runs "
                f"only acquire {EPI_CHANNEL_GROUP!r} channels."
            )
        led = channel_for_preset(ch.config, epi.leds)
        if led is None:
            return (
                f"Preset {ch.config!r} is not an LED channel set in settings "
                "(epi.leds)."
            )
        if not led.visible:
            return f"The LED for preset {ch.config!r} is hidden in settings (epi.leds)."
        if led.channel not in loaded:
            return (
                f"The LED for preset {ch.config!r} (channel {led.channel}) is not "
                "loaded: load 20261118-OPM_EPI.cfg."
            )
    return _service_problem()


def snapshot_led_map(mmc: CMMCorePlus, sequence: MDASequence) -> dict[str, LedRun]:
    """Freeze ``{preset: LedRun}`` for the presets *sequence* uses, from settings.

    Call after :func:`check_epi_sequence` passed. Later edits to the settings (or
    the LED panel) do not change a run that already took this snapshot.
    """
    epi = SettingsV1.instance().epi
    loaded = resolve_led_channels(mmc, epi.card_address)
    out: dict[str, LedRun] = {}
    for ch in sequence.channels:
        led = channel_for_preset(ch.config, epi.leds)
        if led is None or led.channel not in loaded:
            raise EpiSequenceRefused(f"No loaded LED for preset {ch.config!r}.")
        out[ch.config] = LedRun(loaded[led.channel], led.channel, led.name, led.percent)
    return out


def led_map_metadata(led_map: dict[str, LedRun]) -> dict[str, dict[str, Any]]:
    """The LED map as plain, JSON-friendly dicts (for metadata)."""
    return {
        preset: {
            "led": run.label,
            "led_channel": run.channel,
            "led_name": run.name,
            "led_pct": run.pct,
        }
        for preset, run in led_map.items()
    }


def with_epi_metadata(sequence: MDASequence, mmc: CMMCorePlus) -> MDASequence:
    """Return *sequence* with the per-run LED map in its metadata (D11).

    Writes ``metadata["pymmcore_gui"]["epi"]`` = ``{"leds": {preset: ...},
    "pct_units": ...}`` on a copy; the sequence is immutable. Call only in EPI,
    after :func:`check_epi_sequence` passed.
    """
    block = {
        "leds": led_map_metadata(snapshot_led_map(mmc, sequence)),
        "pct_units": PCT_UNITS,
    }
    existing = dict(sequence.metadata.get(SEQUENCE_METADATA_KEY, {}))
    existing["epi"] = block
    return sequence.model_copy(
        update={"metadata": {**sequence.metadata, SEQUENCE_METADATA_KEY: existing}}
    )


# ---------------------------------------------------------------------- engine


class EpiMDAEngine(MDAEngine):
    """MDA engine for v1 EPI: single-plane time series, Orca frames, TGLED light.

    Stock behaviour everywhere except: ``setup_sequence`` validates first and adds
    ``extra["epi"]`` to the summary metadata; ``_set_event_z`` does nothing (the MDA
    widget injects the current z into every event, and the CRISP holds focus);
    exposure goes to the Orca worker with each arm, not to ``core.setExposure``;
    ``exec_event`` lights the LED and snaps through the EPI service's pool;
    ``teardown_sequence`` also closes every LED.

    Hardware sequencing is off and XY is only re-sent when it changes. The initial
    state is never restored (that would call ``setZPosition``).
    """

    def __init__(self, mmc: CMMCorePlus) -> None:
        super().__init__(
            mmc,
            use_hardware_sequencing=False,
            force_set_xy_position=False,
            restore_initial_state=False,
        )
        self._led_map: dict[str, LedRun] = {}
        self._card_address = SettingsV1.instance().epi.card_address
        self._svc: EpiCameraWorkerService | None = None
        self._pool: CameraWorkerPool | None = None
        self._pixel_size_um = 0.0
        self._event_exposure_ms: float | None = None
        self._crisp_state: str | None = None

    # ------------------------------------------------------------ CRISP (read only)

    def _read_crisp_state(self) -> str | None:
        """``CRISP State`` of the EPI CRISP, or ``None`` if not loaded or unreadable."""
        mmc = self.mmcore
        try:
            if EPI_CRISP_LABEL not in mmc.getLoadedDevices():
                return None
            return str(mmc.getProperty(EPI_CRISP_LABEL, _CRISP_STATE_PROP))
        except Exception:
            logger.debug("Could not read CRISP State", exc_info=True)
            return None

    def _read_stage_z(self) -> float | None:
        """Stage-Z position (read only), or ``None`` if not loaded or unreadable."""
        mmc = self.mmcore
        try:
            if STAGE_Z_LABEL not in mmc.getLoadedDevices():
                return None
            return float(mmc.getPosition(STAGE_Z_LABEL))
        except Exception:
            logger.debug("Could not read stage Z", exc_info=True)
            return None

    def _poll_crisp(self) -> str | None:
        """Read the CRISP state for this time point; log it only when it changed.

        Never stops the run (D13).
        """
        state = self._read_crisp_state()
        if state != self._crisp_state:
            logger.warning(
                "EPI run: CRISP Z (%s) state changed: %s -> %s",
                EPI_CRISP_LABEL,
                self._crisp_state,
                state,
            )
            self._crisp_state = state
        return state

    # ------------------------------------------------------------------- sequence

    def setup_sequence(self, sequence: MDASequence) -> SummaryMetaV1 | None:
        """Validate, freeze the LED map, then run the stock setup.

        Raises
        ------
        EpiSequenceRefused
            *sequence* is not a v1 EPI acquisition (before any hardware moves).
        """
        mmc = self.mmcore
        if current_mode() is not ScopeMode.EPI:
            raise EpiSequenceRefused("The EPI engine only runs in EPI mode.")
        if (problem := check_epi_sequence(mmc, sequence)) is not None:
            raise EpiSequenceRefused(problem)
        svc = get_epi_service()
        assert svc is not None  # check_epi_sequence verified it
        svc.check_workers_alive()

        self._card_address = SettingsV1.instance().epi.card_address
        self._led_map = snapshot_led_map(mmc, sequence)
        self._svc = svc
        self._pool = None
        self._event_exposure_ms = None

        # D13: an unlocked CRISP Z is one warning, never a refusal.
        crisp_state = self._read_crisp_state()
        self._crisp_state = crisp_state
        if crisp_state not in CRISP_LOCKED_STATES:
            logger.warning(
                "EPI run: CRISP Z (%s) is %s, not locked; continuing "
                "(a lost lock will not stop the run).",
                EPI_CRISP_LABEL,
                "not loaded" if crisp_state is None else repr(crisp_state),
            )
        stage_z = self._read_stage_z()
        try:
            self._pixel_size_um = float(mmc.getPixelSizeUm())
        except Exception:
            self._pixel_size_um = 0.0

        summary = super().setup_sequence(sequence)
        if summary is not None:
            summary["extra"] = {
                **summary.get("extra", {}),
                "epi": {
                    "leds": led_map_metadata(self._led_map),
                    "crisp_z_label": EPI_CRISP_LABEL,
                    "crisp_z_state": crisp_state,
                    "stage_z_um": stage_z,
                    "pct_units": PCT_UNITS,
                },
            }
        return summary

    def teardown_sequence(self, sequence: MDASequence) -> None:
        """Stock teardown, then every LED off and the Orca pool released."""
        try:
            super().teardown_sequence(sequence)
        finally:
            try:
                _all_leds_off(self.mmcore, self._card_address)
            finally:
                svc, self._svc, self._pool = self._svc, None, None
                if svc is not None:
                    svc.release_from_mda()

    # ---------------------------------------------------------------------- event

    def _set_event_z(self, event: MDAEvent) -> None:
        """No-op: v1 never moves Z.

        The MDA widget injects the current position, including Core Focus ``z``
        (the piezo here), into every event. The stock call would ``setZPosition``
        the piezo, or command a move against the stage-Z CRISP lock.
        """

    def setup_single_event(self, event: MDAEvent) -> None:
        """Stock event setup (XY, channel preset) with exposure sent to the Orca.

        The stock method would ``core.setExposure`` on a main core that has no
        Orca (and warn every event), and may open the Core shutter for
        ``keep_shutter_open``; neither applies here. The exposure is passed to the
        Orca worker with each arm in :meth:`exec_event`.
        """
        super().setup_single_event(
            event.model_copy(update={"exposure": None, "keep_shutter_open": False})
        )
        self._event_exposure_ms = event.exposure

    def _entry_for(self, event: MDAEvent) -> LedRun:
        ch = event.channel
        if ch is None or ch.group != EPI_CHANNEL_GROUP:
            raise RuntimeError("EPI event without an 'EPI LEDs' channel.")
        try:
            return self._led_map[ch.config]
        except KeyError:
            raise RuntimeError(
                f"Preset {ch.config!r} was not part of this run's LED map."
            ) from None

    def _claim_pool(self) -> tuple[EpiCameraWorkerService, CameraWorkerPool]:
        """Take the Orca pool for the run on the first event (stops Live)."""
        svc = self._svc
        if svc is None:
            raise RuntimeError("EPI engine used before setup_sequence.")
        if self._pool is None:
            self._pool = svc.claim_for_run()
        return svc, self._pool

    def _close_led(self, label: str) -> None:
        try:
            self.mmcore.setShutterOpen(label, False)
        except Exception:
            logger.exception("Could not close LED %s", label)

    def exec_event(
        self, event: MDAEvent
    ) -> Iterable[tuple[np.ndarray, MDAEvent, FrameMetaV1]]:
        """Light the event's LED, snap one Orca frame, close the LED, yield it.

        The LED is closed and the pool stopped and drained on every exit path
        (a failed arm, a stalled frame, an exception), before the frame is
        handed to the runner. A generator ``send("cancel")`` ends the event.
        """
        action = getattr(event, "action", None)
        if not isinstance(action, AcquireImage | type(None)):
            return
        entry = self._entry_for(event)
        svc, pool = self._claim_pool()
        crisp_state = self._poll_crisp()
        exposure_ms = (
            self._event_exposure_ms
            if self._event_exposure_ms is not None
            else svc.last_known_exposure_ms
        )

        frame: tuple[str, int, np.ndarray, dict[str, Any], int] | None = None
        try:
            # LED first, then arm: an internally triggered arm starts exposing at
            # once (the same order as ``epi_snap``).
            apply_led(self.mmcore, entry.label, entry.pct)
            pool.arm_all(
                1,
                armed_timeout=svc.hw.worker_arm_timeout_s,
                external_trigger=False,
                exposure_ms=exposure_ms,
            )
            for item in pool.iter_frames(stall_timeout_s=5.0 + exposure_ms / 1000.0):
                frame = item
                break
        finally:
            try:
                self._close_led(entry.label)
            finally:
                pool.stop_and_drain(timeout=svc.hw.worker_shutdown_timeout_s)
        if frame is None:
            raise RuntimeError("The Orca returned no frame for this event.")

        camera_label, _slice, img, camera_metadata, remaining = frame
        runner_t0 = event.metadata.get("runner_t0")
        runner_time_ms = (
            (time.perf_counter() - runner_t0) * 1000.0 if runner_t0 else 0.0
        )
        meta: FrameMetaV1 = {
            "format": "frame-dict",
            "version": "1.0",
            "pixel_size_um": self._pixel_size_um,
            "camera_device": camera_label,
            "exposure_ms": exposure_ms,
            "property_values": (),
            "runner_time_ms": runner_time_ms,
            "mda_event": event,
            "hardware_triggered": False,
            "images_remaining_in_buffer": remaining,
            "camera_metadata": camera_metadata,
            "extra": {
                "epi": {
                    "led": entry.label,
                    "led_channel": entry.channel,
                    "led_name": entry.name,
                    "led_pct": entry.pct,
                    "crisp_z_state": crisp_state,
                }
            },
        }
        received = yield img, event, meta
        if received == "cancel":
            logger.info("EPI MDA cancelled.")
