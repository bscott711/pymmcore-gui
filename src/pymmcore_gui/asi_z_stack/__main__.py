# src/pymmcore_gui/asi_z_stack/__main__.py
import sys
from pymmcore_gui import create_mmgui
from pymmcore_gui._qt.QtWidgets import QApplication

from .common import HardwareConstants
from .engine import ASISPIMEngine

def main():
    # 1. Initialize the GUI (exec_app=False allows us to configure before showing)
    window = create_mmgui(exec_app=False)
    mmc = window.mmcore
    hw = HardwareConstants()

    # 2. Load Hardware Configuration
    try:
        mmc.loadSystemConfiguration(hw.cfg_path)
        print(f"Successfully loaded system configuration: {hw.cfg_path}")
    except Exception as e:
        print(f"\n--- CONFIGURATION ERROR ---\n{e}\n---------------------------\n")
        print("Loading demo configuration instead. The GUI will still open.")
        mmc.loadSystemConfiguration()

    # 3. Register the Custom Engine
    # This replaces the default MDAEngine with our ASI SPIM logic
    mmc.mda.set_engine(ASISPIMEngine(mmc, hw))
    print("ASISPIMEngine registered successfully.")

    # 4. Show Window and Start Event Loop
    window.show()
    QApplication.instance().exec()

if __name__ == "__main__":
    main()
