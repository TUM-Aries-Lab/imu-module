"""Sample doc string."""

import argparse
import time

import board
from loguru import logger

from imu_python.definitions import DEFAULT_LOG_LEVEL, IMUFrequency, LogLevel
from imu_python.factory import IMUFactory
from imu_python.utils import setup_logger


def main(
    log_level: str = DEFAULT_LOG_LEVEL, stderr_level: str = DEFAULT_LOG_LEVEL
) -> None:  # pragma: no cover
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    i2c = board.I2C()

    sensor_managers = IMUFactory.detect_and_create(i2c_bus=i2c)
    for manager in sensor_managers:
        manager.start()

    try:
        while True:
            for manager in sensor_managers:
                manager.log_data()
            time.sleep(IMUFrequency.IMU_READ_FREQUENCY)
    except KeyboardInterrupt:
        logger.info("Stopping...")
        for manager in sensor_managers:
            manager.stop()


if __name__ == "__main__":  # pragma: no cover
    parser = argparse.ArgumentParser("Run the pipeline.")
    parser.add_argument(
        "--log-level",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the log level.",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--stderr-level",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the std err level.",
        required=False,
        type=str,
    )
    args = parser.parse_args()

    main(log_level=args.log_level, stderr_level=args.stderr_level)
