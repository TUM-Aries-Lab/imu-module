"""Sample doc string."""

import argparse
import time

from loguru import logger

from imu_python.definitions import DEFAULT_LOG_LEVEL, I2CBusID, LogLevel
from imu_python.factory import IMUFactory
from imu_python.utils import setup_logger


def main(
    log_level: str, stderr_level: str, freq: float, record_imu: bool
) -> None:  # pragma: no cover
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :param freq: The frequency to use.
    :param record_imu: Flag to record the IMU data
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    sensor_managers_l = IMUFactory.detect_and_create(
        i2c_id=I2CBusID.bus_1, log_data=record_imu
    )
    sensor_managers_r = IMUFactory.detect_and_create(
        i2c_id=I2CBusID.bus_7, log_data=record_imu
    )
    for manager in sensor_managers_l:
        manager.start()
    for manager in sensor_managers_r:
        manager.start()
    try:
        while True:
            for manager in sensor_managers_l:
                data = manager.get_data()
                logger.info(f"Data for {manager}: {data.quat.to_euler(seq='xyz')}")
            for manager in sensor_managers_r:
                manager.get_data()
            time.sleep(1 / freq)
    except KeyboardInterrupt:
        for manager in sensor_managers_l:
            manager.stop()
        for manager in sensor_managers_r:
            manager.stop()


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
        "--freq",
        "-f",
        type=float,
        help="Frequency to use.",
        default=1.0,
    )
    parser.add_argument("--record", "-r", help="Record IMU data.", action="store_true")
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        freq=args.freq,
        record_imu=args.record,
    )
