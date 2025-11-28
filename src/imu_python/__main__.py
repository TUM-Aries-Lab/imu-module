"""Sample doc string."""

import argparse
import time

import adafruit_bno055
import board
from loguru import logger

from imu_python.config.definitions import DEFAULT_LOG_LEVEL, LogLevel
from imu_python.utils import setup_logger


def main(
    log_level: str = DEFAULT_LOG_LEVEL, stderr_level: str = DEFAULT_LOG_LEVEL
) -> None:
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)
    logger.info("Hello, world!")

    # modified sample code from
    # https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    while True:
        logger.info(f"Temperature: {sensor.temperature} degrees C")
        logger.info(f"Accelerometer (m/s^2): {sensor.acceleration}")
        logger.info(f"Magnetometer (microteslas): {sensor.magnetic}")
        logger.info(f"Gyroscope (rad/sec): {sensor.gyro}")
        logger.info(f"Euler angle: {sensor.euler}")
        logger.info(f"Quaternion: {sensor.quaternion}")
        logger.info(f"Linear acceleration (m/s^2): {sensor.linear_acceleration}")
        logger.info(f"Gravity (m/s^2): {sensor.gravity}")
        logger.info("Hello, I'm testing git")
        time.sleep(1)


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
