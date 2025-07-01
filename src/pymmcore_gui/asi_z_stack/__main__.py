# src/microscope/ui/__main__.py
import sys
import time

import useq
from pymmcore_plus import CMMCorePlus
from qtpy.QtWidgets import QApplication

from pymmcore_gui import MicroManagerGUI, WidgetAction
from pymmcore_gui.asi_z_stack.asi_controller import (
    close_global_shutter,
    configure_plogic_for_dual_nrt_pulses,
    open_global_shutter,
    set_camera_trigger_mode,
)
from pymmcore_gui.asi_z_stack.common import AcquisitionSettings, HardwareConstants


def main():
    """Launch the Microscope Control GUI and connect hardware events."""
    app = QApplication(sys.argv)

    mmc = CMMCorePlus.instance()
    HW = HardwareConstants()
    original_autoshutter_state = mmc.getAutoShutter()

    try:
        mmc.loadSystemConfiguration(HW.cfg_path)
        print(f"Successfully loaded system configuration: {HW.cfg_path}")
        # The FocusDevice should be set in the .cfg file, not here.

    except Exception as e:
        print(f"\n--- CONFIGURATION ERROR ---\n{e}\n---------------------------\n")
        print("Loading demo configuration instead. The GUI will still open.")
        mmc.loadSystemConfiguration()

    # --- PLogic Hardware Preparation ---
    def _prepare_for_acquisition():
        """Configure hardware for the sequence."""
        print("--- SEQUENCE STARTED: Preparing hardware ---")
        mmc.setAutoShutter(False)
        set_camera_trigger_mode(HW.camera_a_label)

        settings = AcquisitionSettings(camera_exposure_ms=mmc.getExposure())
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
        print("--- Hardware ready for acquisition ---")

    def _cleanup_after_acquisition(sequence=None):
        """Clean up hardware after the sequence."""
        print("--- SEQUENCE FINISHED: Cleaning up hardware ---")
        close_global_shutter(
            HW.plogic_label, HW.tiger_comm_hub_label, HW.plogic_bnc3_addr
        )
        # Add a small delay to allow the camera to settle before changing mode
        time.sleep(0.1)
        mmc.setProperty(HW.camera_a_label, "TriggerMode", "Internal Trigger")
        mmc.setAutoShutter(original_autoshutter_state)
        print("--- Hardware cleanup complete ---")

    # We do NOT connect the prepare function to the sequenceStarted signal anymore.
    # It will be called synchronously before the MDA run.
    mmc.mda.events.sequenceFinished.connect(
        _cleanup_after_acquisition, thread="main"
    )

    # --- GUI and MDA Override ---
    main_win = MicroManagerGUI()
    mda_widget = main_win.get_widget(WidgetAction.MDA_WIDGET)
    if not mda_widget:
        raise RuntimeError("Could not find MDA widget in the GUI.")

    def _run_z_stack_only(output=None):
        """Intercept the 'Run' button to execute ONLY a hardware-timed Z-stack."""
        full_sequence = mda_widget.value()  # type: ignore
        z_stack_sequence = useq.MDASequence(
            z_plan=full_sequence.z_plan,
            time_plan=full_sequence.time_plan,
            channels=tuple(full_sequence.channels) if full_sequence.channels else (),
        )
        print("\n--- INTERCEPTED MDA RUN ---")

        # Call the preparation function synchronously in the main GUI thread
        # BEFORE starting the MDA worker thread.
        _prepare_for_acquisition()

        print(f"Executing purified Z-stack: {z_stack_sequence.z_plan}")
        mmc.run_mda(z_stack_sequence)

    mda_widget.execute_mda = _run_z_stack_only

    main_win.show()
    app.exec()


if __name__ == "__main__":
    main()
