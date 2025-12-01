"""Sample doc string."""

import time

import adafruit_bno055
import board
from loguru import logger


def test_imu() -> None:
    """Test the imu sensor.

    :return: None
    """
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
