# src/pymmcore_gui/asi_z_stack/__main__.py
import argparse
import logging

from pymmcore_gui import create_mmgui
from pymmcore_gui._qt.QtWidgets import QApplication

from ._logging import configure_asi_logging
from .common import HardwareConstants
from .engine import ASISPIMEngine

logger = logging.getLogger(__name__)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="microscope-control")
    parser.add_argument(
        "--loglevel",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Logging level for the ASI hardware layer (default: INFO).",
    )
    parser.add_argument(
        "--debug",
        action="store_const",
        dest="loglevel",
        const="DEBUG",
        help="Shortcut for --loglevel DEBUG.",
    )
    return parser.parse_args()


def main() -> None:
    """Initialize the GUI, load configuration, and register the ASI SPIM Engine."""
    args = _parse_args()
    configure_asi_logging(args.loglevel)

    # 1. Initialize the GUI (exec_app=False allows us to configure before showing)
    window = create_mmgui(exec_app=False)
    mmc = window.mmcore
    hw = HardwareConstants()

    # 2. Load Hardware Configuration
    try:
        mmc.loadSystemConfiguration(hw.cfg_path)
        logger.info(f"Successfully loaded system configuration: {hw.cfg_path}")
    except Exception:
        logger.error("Failed to load system configuration.", exc_info=True)
        logger.info("Loading demo configuration instead. The GUI will still open.")
        mmc.loadSystemConfiguration()

    # 3. Register the Custom Engine
    # This replaces the default MDAEngine with our ASI SPIM logic
    mmc.mda.set_engine(ASISPIMEngine(mmc, hw))
    logger.info("ASISPIMEngine registered successfully.")

    # 4. Show Window and Start Event Loop
    window.show()
    if app := QApplication.instance():
        app.exec()


if __name__ == "__main__":
    main()
