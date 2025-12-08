"""Factory that creates IMU object from given IMU type."""

import board
from loguru import logger

from imu_python.devices import IMU_DEVICES
from imu_python.sensor_manager import SensorManager
from imu_python.wrapper import IMUWrapper


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def detect_and_create(i2c_bus=None) -> list[SensorManager]:
        """Automatically detect addresses and create matched sensors and their managers."""
        if i2c_bus is None:
            i2c_bus = board.I2C()
        imu_managers = []
        detected_addresses = IMUFactory.scan_i2c_bus(i2c_bus)

        for cfg in IMU_DEVICES:
            if address := IMUFactory.compare_addresses(
                cfg.addresses, detected_addresses
            ):
                logger.info(f"Detected address {address}")
                # Create wrapper with config and i2c
                imu_wrapper = IMUWrapper(cfg, i2c_bus)
                # Create manager for the imu
                imu_manager = SensorManager(imu_wrapper)
                # Add the manager to the list of returned managers
                imu_managers.append(imu_manager)

        return imu_managers

    @staticmethod
    def scan_i2c_bus(i2c) -> list[int]:
        """Scan the I2C bus for sensor addresses."""
        while not i2c.try_lock():
            pass
        try:
            return i2c.scan()
        finally:
            i2c.unlock()

    @staticmethod
    def compare_addresses(
        imu_address: list[int], detected_addresses: list[int]
    ) -> int | None:
        """Compare the IMU addresses against a list of detected addresses."""
        matches = set(detected_addresses) & set(imu_address)

        if len(matches) == 0:
            # No address for this IMU found → continue checking next IMUConfig
            return None

        elif len(matches) == 1:
            # Normal case: exactly one address matched
            actual_address = next(iter(matches))
            return actual_address

        elif len(matches) > 1:
            # Unusual case: multiple of this IMU`s possible addresses detected, skip.
            logger.warning("Multiple possible addresses detected. ")
            return None
        return None
