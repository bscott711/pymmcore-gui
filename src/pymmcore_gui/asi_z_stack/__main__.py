# src/microscope/ui/__main__.py
import sys
import time
import traceback
from dataclasses import dataclass
from threading import Thread

import useq
from pymmcore_plus import CMMCorePlus
from pymmcore_plus.metadata import frame_metadata
from qtpy.QtWidgets import QApplication

from pymmcore_gui import MicroManagerGUI, WidgetAction
from pymmcore_gui.asi_z_stack.asi_controller import (
    close_global_shutter,
    configure_plogic_for_dual_nrt_pulses,
    open_global_shutter,
    set_camera_trigger_mode,
)
from pymmcore_gui.asi_z_stack.common import (
    AcquisitionSettings,
)
from pymmcore_gui.asi_z_stack.common import (
    HardwareConstants as BaseHardwareConstants,
)


@dataclass
class HardwareConstants(BaseHardwareConstants):
    """Extend the base hardware constants with additional parameters."""

    slice_calibration_slope_um_per_deg: float = 100.0
    line_scans_per_slice: int = 1
    delay_before_scan_ms: float = 0.0
    line_scan_duration_ms: float = 1.0
    delay_before_side_ms: float = 0.0


def main():
    """Launch the Microscope Control GUI and connect hardware events."""
    app = QApplication(sys.argv)

    mmc = CMMCorePlus.instance()
    HW = HardwareConstants()
    original_autoshutter_state = mmc.getAutoShutter()

    try:
        mmc.loadSystemConfiguration(HW.cfg_path)
        print(f"Successfully loaded system configuration: {HW.cfg_path}")
    except Exception as e:
        print(f"\n--- CONFIGURATION ERROR ---\n{e}\n---------------------------\n")
        print("Loading demo configuration instead. The GUI will still open.")
        mmc.loadSystemConfiguration()

    def _prepare_for_acquisition(settings: AcquisitionSettings):
        """Configure PLogic and Camera for the sequence."""
        print("--- SEQUENCE STARTED: Preparing PLogic and Camera ---")
        mmc.setAutoShutter(False)
        set_camera_trigger_mode(HW.camera_a_label)
        settings.laser_trig_duration_ms = settings.camera_exposure_ms
        open_global_shutter(
            HW.plogic_label,
            HW.tiger_comm_hub_label,
            HW.plogic_always_on_cell,
            HW.plogic_bnc3_addr,
        )
        configure_plogic_for_dual_nrt_pulses(
            settings,
            HW.plogic_label,
            HW.tiger_comm_hub_label,
            HW.plogic_laser_preset_num,
            HW.plogic_camera_cell,
            HW.pulses_per_ms,
            HW.plogic_4khz_clock_addr,
            HW.plogic_trigger_ttl_addr,
            HW.plogic_laser_on_cell,
        )
        print("--- PLogic and Camera ready ---")

    def _cleanup_after_acquisition(sequence=None):
        """Clean up hardware after the sequence."""
        print("--- SEQUENCE FINISHED: Cleaning up hardware ---")
        if HW.galvo_a_label in mmc.getLoadedDevices():
            print("Resetting ASI SPIM state to Idle.")
            mmc.setProperty(HW.galvo_a_label, "SPIMState", "Idle")
            mmc.setProperty(HW.galvo_a_label, "BeamEnabled", "No")

        close_global_shutter(
            HW.plogic_label, HW.tiger_comm_hub_label, HW.plogic_bnc3_addr
        )
        time.sleep(0.1)
        mmc.setProperty(HW.camera_a_label, "TriggerMode", "Internal Trigger")
        mmc.setAutoShutter(original_autoshutter_state)
        print("--- Hardware cleanup complete ---")

    mmc.mda.events.sequenceFinished.connect(_cleanup_after_acquisition, thread="main")

    main_win = MicroManagerGUI()

    # FIX: Reconnect the frameReady signal to execute in the main GUI thread.
    # This prevents the application from crashing when the viewer tries to update
    # its display from the background acquisition thread.
    viewer = getattr(main_win, "viewer", None)
    if viewer:
        # The `psygnal` library allows disconnecting a slot by providing the function
        mmc.mda.events.frameReady.disconnect(viewer._on_frame_ready)
        # Reconnect with `thread="main"` to force the update to happen in the GUI thread
        mmc.mda.events.frameReady.connect(viewer._on_frame_ready, thread="main")
    else:
        print(
            "\nWARNING: Could not find the 'viewer' attribute on MicroManagerGUI."
            " The frameReady signal could not be safely reconnected."
            " The application may still crash.\n"
        )

    mda_widget = main_win.get_widget(WidgetAction.MDA_WIDGET)
    if not mda_widget:
        raise RuntimeError("Could not find MDA widget in the GUI.")

    def _run_asi_spim_acquisition(sequence: useq.MDASequence):
        """
        Run a Z-stack using the ASI SPIM hardware triggering capabilities.

        This is based on the logic from the provided tkinter script.
        """
        if not sequence.z_plan:
            print("No Z-plan in sequence. Nothing to do.")
            mmc.mda.events.sequenceFinished.emit(sequence)
            return

        z_positions = list(sequence.z_plan)
        if not z_positions:
            print("Z-plan is empty. Nothing to do.")
            mmc.mda.events.sequenceFinished.emit(sequence)
            return

        try:
            exposure = sequence.channels[0].exposure if sequence.channels else None
            settings = AcquisitionSettings(camera_exposure_ms=(exposure or 10.0))

            _prepare_for_acquisition(settings)

            num_slices = len(z_positions)

            step_size_um = 0.0
            if isinstance(sequence.z_plan, useq.ZRangeAround | useq.ZAboveBelow):
                step_size_um = sequence.z_plan.step
            elif num_slices > 1:
                step_size_um = abs(z_positions[1] - z_positions[0])

            amplitude_um = (num_slices - 1) * step_size_um
            galvo_amplitude_deg = amplitude_um / HW.slice_calibration_slope_um_per_deg

            print("Configuring ASI galvo for SPIM scan...")
            mmc.setProperty(HW.galvo_a_label, "SPIMNumSlices", num_slices)
            mmc.setProperty(
                HW.galvo_a_label,
                "SingleAxisYAmplitude(deg)",
                round(galvo_amplitude_deg, 4),
            )
            mmc.setProperty(HW.galvo_a_label, "BeamEnabled", "Yes")
            mmc.setProperty(HW.galvo_a_label, "SPIMNumRepeats", 1)
            mmc.setProperty(HW.galvo_a_label, "SPIMNumSides", 1)
            mmc.setProperty(HW.galvo_a_label, "SPIMFirstSide", "A")
            mmc.setProperty(HW.galvo_a_label, "SPIMAlternateDirectionsEnable", "No")
            mmc.setProperty(
                HW.galvo_a_label, "SPIMScanDuration(ms)", HW.line_scan_duration_ms
            )

            mmc.mda.events.sequenceStarted.emit(sequence, {})
            mmc.initializeCircularBuffer()
            mmc.startSequenceAcquisition(HW.camera_a_label, num_slices, 0, True)
            print(f"Camera sequence started, waiting for {num_slices} triggers.")

            mmc.setProperty(HW.galvo_a_label, "SPIMState", "Running")
            print("ASI SPIM scan triggered.")

            images_collected = 0
            events = list(sequence)
            timeout_s = (num_slices * settings.camera_exposure_ms / 1000) + 5

            start_time = time.time()
            while images_collected < num_slices:
                if time.time() - start_time > timeout_s:
                    raise TimeoutError(f"Acquisition timed out after {timeout_s:.1f}s.")

                if mmc.getRemainingImageCount() > 0:
                    tagged_img = mmc.popNextTaggedImage()
                    event = events[images_collected]
                    print(
                        f"  - Acquired image {images_collected + 1}/{num_slices} "
                        f"for event {event.index}"
                    )
                    meta = frame_metadata(mmc, mda_event=event)
                    mmc.mda.events.frameReady.emit(tagged_img.pix, event, meta)
                    images_collected += 1
                elif not mmc.isSequenceRunning() and images_collected < num_slices:
                    raise RuntimeError(
                        "Sequence stopped unexpectedly after acquiring "
                        f"{images_collected} images."
                    )
                else:
                    time.sleep(0.005)

            print("All expected images collected.")

        except Exception as e:
            print(f"FATAL: Acquisition thread failed: {e}")
            traceback.print_exc()
        finally:
            print("Acquisition thread finishing.")
            if mmc.isSequenceRunning():
                mmc.stopSequenceAcquisition()
            mmc.mda.events.sequenceFinished.emit(sequence)

    def _intercept_mda_run(output=None):
        """
        Intercept the 'Run' button to execute a hardware-timed Z-stack.

        This creates a Z-stack only sequence and executes it with our
        custom hardware-triggered runner.
        """
        full_sequence = mda_widget.value()
        z_stack_sequence = useq.MDASequence(
            z_plan=full_sequence.z_plan,
            channels=tuple(full_sequence.channels) if full_sequence.channels else (),
        )
        print("\n--- INTERCEPTED MDA RUN ---")
        print(
            f"Executing ASI SPIM hardware-triggered Z-stack: {z_stack_sequence.z_plan}"
        )
        thread = Thread(target=_run_asi_spim_acquisition, args=(z_stack_sequence,))
        thread.start()

    mda_widget.execute_mda = _intercept_mda_run

    main_win.show()
    app.exec()


if __name__ == "__main__":
    main()
