"""Enum registry of IMU device configurations."""

from dataclasses import replace
from enum import Enum

from imu_python.base_classes import IMUConfig


class IMUDevices(Enum):
    """Enumeration containing configuration for all supported IMU devices."""

    BNO055 = IMUConfig(
        name="BNO055",
        addresses=[0x28, 0x29],
        library="adafruit_bno055",  # module import path
        module_class="BNO055_I2C",  # driver class inside the module
    )

    LSM6DSOX = IMUConfig(
        name="LSM6DSOX",
        addresses=[0x6A, 0x6B],
        library="adafruit_lsm6ds.lsm6dsox",
        module_class="LSM6DSOX",
    )

    MOCK = IMUConfig(
        name="MOCK",
        addresses=[0x00, 0x01],  # fake I2C addresses for testing
        library="imu_python.mock_imu",  # module path (corrected)
        module_class="MockIMU",  # driver class
    )

    @property
    def config(self) -> IMUConfig:
        """Return the IMUConfig stored inside the enum member."""
        return self.value

    @staticmethod
    def from_address(addr: int) -> IMUConfig | None:
        """Return the enum member matching this I2C address, or None if unknown."""
        for device in IMUDevices:
            if addr in device.config.addresses:
                config = replace(device.value)
                config.addresses = [addr]
                return config
        return None
