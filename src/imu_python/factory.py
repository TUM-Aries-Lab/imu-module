"""Factory that creates IMU object from given IMU type."""

from typing import Any

from loguru import logger

from imu_python.definitions import I2CBusID
from imu_python.devices import IMUDevices
from imu_python.sensor_manager import IMUManager
from imu_python.wrapper import IMUWrapper

try:
    from adafruit_extended_bus import ExtendedI2C

    I2C_BUS_L = ExtendedI2C(I2CBusID.left)  # pin 27 (SDA) & 28 (SCL)
    I2C_BUS_R = ExtendedI2C(I2CBusID.right)  # pin 3 (SDA) & 5 (SCL)
except ValueError as device_err:
    logger.warning(f"{device_err}. Are you running without the Jetson?")
    I2C_BUS_L = None  # allow desktop environments to import this file
    I2C_BUS_R = None


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def detect_and_create(i2c_id: int | None = None) -> list[IMUManager]:
        """Automatically detect addresses and create sensor managers.

        :param i2c_id: I2C bus identifier. If None, attempt to use board.I2C().
        :return: list of SensorManager instances.
        """
        imu_managers: list[IMUManager] = []

        if i2c_id is None:
            i2c_bus = None
        elif i2c_id == I2CBusID.left:
            i2c_bus = I2C_BUS_L
        elif i2c_id == I2CBusID.right:
            i2c_bus = I2C_BUS_R
        else:
            logger.error(f"Bus ID {i2c_id} not supported.")
            return imu_managers

        addresses = IMUFactory.scan_i2c_bus(i2c=i2c_bus)
        for addr in addresses:
            config = IMUDevices.from_address(addr)
            if config:
                imu_wrapper = IMUWrapper(config=config, i2c_bus=i2c_bus)
                imu_managers.append(IMUManager(imu_wrapper=imu_wrapper, i2cid=i2c_id))
                logger.info(f"Detected {config} at I2C address '{addr}'.")

        return imu_managers

    @staticmethod
    def scan_i2c_bus(i2c: Any) -> list[int]:
        """Scan the I2C bus for sensor addresses."""
        try:
            while not i2c.try_lock():
                pass
            addresses = i2c.scan()
            i2c.unlock()
            return addresses
        except Exception as err:
            logger.warning(f"I2C scan failed: {err}. Returning Mock address.")
            return IMUDevices.BASE.config.addresses
