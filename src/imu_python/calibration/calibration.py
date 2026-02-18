"""Calibration for IMU sensors."""

import argparse
import time
from pathlib import Path

from loguru import logger

from imu_python.base_classes import IMUSensorTypes
from imu_python.calibration.ellipsoid_fitting import (
    FittingAlgorithmNames,
)
from imu_python.calibration.mag_calibration import MagCalibration
from imu_python.definitions import (
    CAL_DIR,
    DEFAULT_LOG_LEVEL,
    IMU_FILENAME_KEY,
    I2CBusID,
    LogLevel,
)
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from imu_python.utils import setup_logger


def has_magnetometer(manager: IMUManager) -> bool:  # pragma: no cover
    """Check if the IMUManager has a magnetometer sensor.

    :param manager: IMUManager instance
    :return: True if magnetometer is present, False otherwise
    """
    return manager.imu_wrapper._read_plans[IMUSensorTypes.mag] is not None


def collect_calibration_data() -> None:  # pragma: no cover
    """Collect magnetometer calibration data of all connected IMUs and save to files."""
    sensor_managers_l = IMUFactory.detect_and_create(
        i2c_id=I2CBusID.bus_1, log_data=True
    )
    sensor_managers_r = IMUFactory.detect_and_create(
        i2c_id=I2CBusID.bus_7, log_data=True
    )
    for manager in sensor_managers_l:
        if has_magnetometer(manager):
            manager.file_writer.calibration_mode = True
            manager.start()
    for manager in sensor_managers_r:
        if has_magnetometer(manager):
            manager.file_writer.calibration_mode = True
            manager.start()

    logger.info("Started magnetometer calibration. Press ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        for manager in sensor_managers_l:
            manager.stop()
        for manager in sensor_managers_r:
            manager.stop()

        time.sleep(1.0)  # wait for file write


def main(
    log_level: str,
    stderr_level: str,
    algorithm: str,
    filepath: Path | None = None,
):  # pragma: no cover
    """Run magnetometer calibration.

    :param log_level: Log level for logger.
    :param stderr_level: Std err level for logger.
    :param filepath: Optional path to IMU data file for calibration.
    :param algorithm: The ellipsoid fitting algorithm to use.
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    if filepath:
        MagCalibration(filepath=filepath, algorithm=algorithm)
    else:
        collect_calibration_data()

        # Iterate through files in the calibration directory and calculate calibration
        for cal_file in CAL_DIR.iterdir():
            if cal_file.is_file() and cal_file.name.startswith(IMU_FILENAME_KEY):
                MagCalibration(filepath=cal_file, algorithm=algorithm)


if __name__ == "__main__":  # pragma: no cover
    parser = argparse.ArgumentParser("Run the pipeline.")
    parser.add_argument(
        "--log-level",
        "-l",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the log level.",
        type=str,
    )
    parser.add_argument(
        "--stderr-level",
        "-s",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the std err level.",
        type=str,
    )
    parser.add_argument(
        "--filepath",
        "-f",
        type=Path,
        help="Path to the IMU data file for calibration.",
    )
    parser.add_argument(
        "--algorithm",
        "-a",
        type=str,
        default=FittingAlgorithmNames.LS,
        choices=list(a for a in FittingAlgorithmNames),
        help="The ellipsoid fitting algorithm to use",
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        filepath=args.filepath,
        algorithm=args.algorithm,
    )
