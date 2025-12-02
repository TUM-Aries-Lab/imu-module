"""Sample doc string."""

import argparse
import time

import board
from loguru import logger

from imu_python.base_classes import IMUType
from imu_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from imu_python.imu_factory import IMUFactory
from imu_python.sensor_manager import SensorManager
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
    imu = IMUFactory.create(imu_type=IMUType.MOCK, i2c=i2c)
    logger.info(imu)
    sensor_manager = SensorManager(imu=imu)
    sensor_manager.start()
    try:
        while True:
            data = sensor_manager.get_data()  # Returns IMUData dataclass
            if data:
                logger.info(
                    f"IMU: acc={data.accel.as_array()}, "
                    f"mag={data.mag.as_array()}, "
                    f"gyro={data.gyro.as_array()}, "
                )
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping...")
        sensor_manager.stop()


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

    main(log_level=args.log_level)
