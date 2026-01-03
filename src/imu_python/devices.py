"""Enum registry of IMU device configurations."""

from dataclasses import replace
from enum import Enum

from imu_python.base_classes import IMUConfig, PreConfigStep


class IMUDevices(Enum):
    """Enumeration containing configuration for all supported IMU devices."""

    BNO055 = IMUConfig(
        name="BNO055",
        addresses=[0x28, 0x29],
        library="adafruit_bno055",  # module import path
        module_class="BNO055_I2C",  # driver class inside the module
        i2c_param="i2c",
        filter_gain=0.01149,
        pre_config=[
            # Switch to CONFIG mode
            PreConfigStep(
                name="mode",
                args=("CONFIG_MODE",),
                step_type="set",
            ),
            # Wait for sensor to switch modes
            PreConfigStep(
                name="time.sleep",
                args=(0.025,),
                step_type="call",
            ),
            # Set sensor ranges and bandwidths
            PreConfigStep(
                name="accel_range",
                args=("ACCEL_4G",),
                step_type="set",
            ),
            PreConfigStep(
                name="gyro_range",
                args=("GYRO_250_DPS",),
                step_type="set",
            ),
            PreConfigStep(
                name="accel_bandwidth",
                args=("ACCEL_125HZ",),
                step_type="set",
            ),
            PreConfigStep(
                name="gyro_bandwidth",
                args=("GYRO_116HZ",),
                step_type="set",
            ),
            # Wait for settings to take effect
            PreConfigStep(
                name="time.sleep",
                args=(0.025,),
                step_type="call",
            ),
            # Switch back to NDOF mode
            PreConfigStep(
                name="mode",
                args=("NDOF_MODE",),
                step_type="set",
            ),
        ],
    )

    LSM6DSOX = IMUConfig(
        name="LSM6DSOX",
        addresses=[0x6A, 0x6B],
        library="adafruit_lsm6ds.lsm6dsox",
        module_class="LSM6DSOX",
        i2c_param="i2c_bus",
        filter_gain=0.00087,
        constants_module="adafruit_lsm6ds",
        pre_config=[
            PreConfigStep(
                name="accelerometer_range",
                args=("AccelRange.RANGE_4G",),
                step_type="set",
            ),
            PreConfigStep(
                name="gyro_range",
                args=("GyroRange.RANGE_250_DPS",),
                step_type="set",
            ),
            PreConfigStep(
                name="accelerometer_data_rate",
                args=("Rate.RATE_104_HZ",),
                step_type="set",
            ),
            PreConfigStep(
                name="gyro_data_rate",
                args=("Rate.RATE_104_HZ",),
                step_type="set",
            ),
        ],
    )

    BNO08x = IMUConfig(
        name="BNO08x",
        addresses=[0x4A, 0x4B],
        library="adafruit_bno08x.i2c",  # module import path
        module_class="BNO08X_I2C",  # driver class inside the module
        i2c_param="i2c_bus",
        filter_gain=0.01149,
        constants_module="adafruit_bno08x",
        pre_config=[
            PreConfigStep(
                name="enable_feature",
                args=("BNO_REPORT_ACCELEROMETER",),
                step_type="call",
            ),
            PreConfigStep(
                name="enable_feature",
                args=("BNO_REPORT_GYROSCOPE",),
                step_type="call",
            ),
        ],
    )

    MOCK = IMUConfig(
        name="MOCK",
        addresses=[0x00, 0x01],  # fake I2C addresses for testing
        library="imu_python.base_classes",  # module path (corrected)
        module_class="AdafruitIMU",  # driver class
        i2c_param="i2c",
    )

    @property
    def config(self) -> IMUConfig:
        """Return the IMUConfig stored inside the enum member."""
        return self.value

    @staticmethod
    def from_address(addr: int) -> IMUConfig | None:
        """Return the enum member matching this I2C address, or None if unknown.

        :param addr: I2C address of the device
        :return: IMUConfig matching the I2C address
        """
        for device in IMUDevices:
            if addr in device.config.addresses:
                config = replace(device.value)
                config.addresses = [addr]
                return config
        return None
