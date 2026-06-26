# src/pymmcore_gui/asi_z_stack/engine.py
import time
from typing import Iterable
import numpy as np
from useq import MDAEvent, MDASequence
from pymmcore_plus import CMMCorePlus
from pymmcore_plus.mda import MDAEngine
from pymmcore_plus.metadata import summary_metadata, frame_metadata, FrameMetaV1, SummaryMetaV1

from .common import HardwareConstants, AcquisitionSettings
from .asi_controller import (
    set_camera_trigger_mode,
    open_global_shutter,
    configure_plogic_for_dual_nrt_pulses,
    close_global_shutter,
)

class ASISPIMEngine(MDAEngine):
    def __init__(self, mmc: CMMCorePlus, hw: HardwareConstants):
        # We disable pymmcore-plus hardware sequencing because the ASI TGGALVO 
        # handles the Z-stack sequencing internally via TTL triggers.
        super().__init__(mmc, use_hardware_sequencing=False)
        self.hw = hw
        self._num_slices = 0
        self._exposure_ms = 10.0
        self._original_autoshutter = True

    def setup_sequence(self, sequence: MDASequence) -> SummaryMetaV1 | None:
        # 1. Calculate Z-stack parameters
        if sequence.z_plan:
            z_positions = list(sequence.z_plan)
            self._num_slices = len(z_positions)
            step_size_um = abs(z_positions[1] - z_positions[0]) if self._num_slices > 1 else 0.0
            amplitude_um = (self._num_slices - 1) * step_size_um
            galvo_amplitude_deg = amplitude_um / self.hw.slice_calibration_slope_um_per_deg
        else:
            self._num_slices = 1
            galvo_amplitude_deg = 0.0

        # 2. Determine exposure
        if sequence.channels and len(sequence.channels) > 0 and sequence.channels[0].exposure:
            self._exposure_ms = sequence.channels[0].exposure
        else:
            self._exposure_ms = self._mmc.getExposure()

        # 3. Prepare Hardware
        print("--- SEQUENCE STARTED: Preparing PLogic and Camera ---")
        self._original_autoshutter = self._mmc.getAutoShutter()
        self._mmc.setAutoShutter(False)
        
        active_cam = self._mmc.getCameraDevice()
        set_camera_trigger_mode(active_cam)
        
        settings = AcquisitionSettings(
            camera_exposure_ms=self._exposure_ms,
            laser_trig_duration_ms=self._exposure_ms
        )
        configure_plogic_for_dual_nrt_pulses(
            settings, self.hw.plogic_label, self.hw.tiger_comm_hub_label,
            self.hw.plogic_laser_preset_num, self.hw.plogic_camera_cell,
            self.hw.pulses_per_ms, self.hw.plogic_4khz_clock_addr,
            self.hw.plogic_trigger_ttl_addr, self.hw.plogic_laser_on_cell,
        )
        
        open_global_shutter(
            self.hw.plogic_label, self.hw.tiger_comm_hub_label,
            self.hw.plogic_always_on_cell, self.hw.plogic_bnc3_addr,
        )

        # 4. Configure Galvo
        print("Configuring ASI galvo for SPIM scan...")
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMNumSlices", str(self._num_slices))
        self._mmc.setProperty(self.hw.galvo_a_label, "SingleAxisYAmplitude(deg)", f"{galvo_amplitude_deg:.4f}")
        self._mmc.setProperty(self.hw.galvo_a_label, "BeamEnabled", "Yes")
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMNumRepeats", "1")
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMNumSides", "1")
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMFirstSide", "A")
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMAlternateDirectionsEnable", "No")
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMScanDuration(ms)", str(self.hw.line_scan_duration_ms))
        
        print("--- PLogic and Camera ready ---")
        
        # Return summary metadata. This automatically queries getNumberOfCameraChannels()
        # and informs the OmeWritersSink that it needs to expect interleaved multi-camera data.
        return summary_metadata(self._mmc, mda_sequence=sequence)

    def exec_event(self, event: MDAEvent) -> Iterable[tuple[np.ndarray, MDAEvent, FrameMetaV1]]:
        active_cam = self._mmc.getCameraDevice()
        n_cameras = self._mmc.getNumberOfCameraChannels()
        total_images = self._num_slices * n_cameras
        
        # Arm the buffer and trigger the hardware
        self._mmc.startSequenceAcquisition(active_cam, total_images, 0, True)
        self._mmc.setProperty(self.hw.galvo_a_label, "SPIMState", "Running")
        
        # Pull images as they arrive in the circular buffer
        images_collected = 0
        timeout_s = (total_images * self._exposure_ms / 1000.0) + 5.0
        start_time = time.time()
        
        while images_collected < total_images:
            if time.time() - start_time > timeout_s:
                raise TimeoutError(f"Acquisition timed out after {timeout_s:.1f}s.")
                
            if self._mmc.getRemainingImageCount() > 0:
                img = self._mmc.popNextImage()
                meta = frame_metadata(self._mmc, mda_event=event)
                yield img, event, meta
                images_collected += 1
            elif not self._mmc.isSequenceRunning() and images_collected < total_images:
                raise RuntimeError(f"Sequence stopped unexpectedly after {images_collected} images.")
            else:
                time.sleep(0.005)

    def teardown_sequence(self, sequence: MDASequence) -> None:
        print("--- SEQUENCE FINISHED: Cleaning up hardware ---")
        if self.hw.galvo_a_label in self._mmc.getLoadedDevices():
            self._mmc.setProperty(self.hw.galvo_a_label, "SPIMState", "Idle")
            self._mmc.setProperty(self.hw.galvo_a_label, "BeamEnabled", "No")
            
        close_global_shutter(
            self.hw.plogic_label, self.hw.tiger_comm_hub_label, self.hw.plogic_bnc3_addr
        )
        time.sleep(0.1)
        self._mmc.setAutoShutter(self._original_autoshutter)
        print("--- Hardware cleanup complete ---")
