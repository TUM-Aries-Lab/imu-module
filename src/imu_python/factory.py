"""Factory that creates IMU object from given IMU type."""

from typing import Any

from loguru import logger

from imu_python.definitions import I2CBusID
from imu_python.devices import IMUDevices
from imu_python.i2c_bus import JetsonBus
from imu_python.sensor_manager import IMUManager
from imu_python.wrapper import IMUWrapper


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def detect_and_create(
        i2c_id: I2CBusID | None = None, log_data: bool = False
    ) -> list[IMUManager]:
        """Automatically detect addresses and create sensor managers.

        :param i2c_id: I2C bus identifier. If None, attempt to use board.I2C().
        :return: list of SensorManager instances.
        :param log_data: Flag to record the IMU data
        """
        imu_managers: list[IMUManager] = []

        i2c_bus = JetsonBus.get(bus_id=i2c_id)

        addresses = IMUFactory.scan_i2c_bus(i2c=i2c_bus)
        for addr in addresses:
            config = IMUDevices.from_address(addr)
            if config:
                imu_wrapper = IMUWrapper(config=config, i2c_bus=i2c_bus)
                imu_managers.append(
                    IMUManager(
                        imu_wrapper=imu_wrapper, i2c_id=i2c_id, log_data=log_data
                    )
                )
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
            logger.warning(
                f"I2C scan failed: {err}. Returning {IMUDevices.MOCK.config} addresses."
            )
            return IMUDevices.MOCK.config.addresses
