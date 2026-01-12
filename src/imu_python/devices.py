"""Enum registry of IMU device configurations."""

from dataclasses import replace
from enum import Enum

from imu_python.base_classes import IMUConfig, PreConfigStep, PreConfigStepType


class IMUDevices(Enum):
    """Enumeration containing configuration for all supported IMU devices."""

    BNO055 = IMUConfig(
        name="BNO055",
        addresses=[0x28, 0x29],
        library="adafruit_bno055",  # module import path
        module_class="BNO055_I2C",  # driver class inside the module
        i2c_param="i2c",
        accel_range_g=4.0,
        gyro_range_dps=2000.0,
        filter_gain=0.002250,
        # Range setting does not actually work on the BNO055
        pre_config=[
            # Switch to CONFIG mode
            PreConfigStep(
                name="mode",
                args=("CONFIG_MODE",),
                step_type=PreConfigStepType.SET,
            ),
            # Wait for sensor to switch modes
            PreConfigStep(
                name="time.sleep",
                args=(0.025,),
                step_type=PreConfigStepType.CALL,
            ),
            # Set sensor ranges and bandwidths
            PreConfigStep(
                name="accel_range",
                args=("ACCEL_4G",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="gyro_range",
                args=("GYRO_500_DPS",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="accel_bandwidth",
                args=("ACCEL_125HZ",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="gyro_bandwidth",
                args=("GYRO_116HZ",),
                step_type=PreConfigStepType.SET,
            ),
            # Wait for settings to take effect
            PreConfigStep(
                name="time.sleep",
                args=(0.025,),
                step_type=PreConfigStepType.CALL,
            ),
            # Switch to ACCGYRO mode
            PreConfigStep(
                name="mode",
                args=("ACCGYRO_MODE",),
                step_type=PreConfigStepType.SET,
            ),
        ],
    )

    LSM6DSOX = IMUConfig(
        name="LSM6DSOX",
        addresses=[0x6A, 0x6B],
        library="adafruit_lsm6ds.lsm6dsox",
        module_class="LSM6DSOX",
        i2c_param="i2c_bus",
        accel_range_g=4.0,
        gyro_range_dps=500.0,
        filter_gain=0.000573,
        constants_module="adafruit_lsm6ds",
        pre_config=[
            PreConfigStep(
                name="accelerometer_range",
                args=("AccelRange.RANGE_4G",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="gyro_range",
                args=("GyroRange.RANGE_500_DPS",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="accelerometer_data_rate",
                args=("Rate.RATE_104_HZ",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="gyro_data_rate",
                args=("Rate.RATE_104_HZ",),
                step_type=PreConfigStepType.SET,
            ),
        ],
    )

    BNO08x = IMUConfig(
        name="BNO08x",
        addresses=[0x4A, 0x4B],
        library="adafruit_bno08x.i2c",
        module_class="BNO08X_I2C",
        i2c_param="i2c_bus",
        accel_range_g=8.0,  # default 8g, not settable in driver
        gyro_range_dps=2000.0,  # default 2000dps, not settable in driver
        filter_gain=0.001538,
        constants_module="adafruit_bno08x",
        pre_config=[
            PreConfigStep(
                name="enable_feature",
                args=("BNO_REPORT_ACCELEROMETER",),
                step_type=PreConfigStepType.CALL,
            ),
            PreConfigStep(
                name="enable_feature",
                args=("BNO_REPORT_GYROSCOPE",),
                step_type=PreConfigStepType.CALL,
            ),
        ],
    )

    MOCK = IMUConfig(
        name="MOCK",
        addresses=[0x00, 0x01],  # fake I2C addresses for testing
        library="imu_python.base_classes",  # module path (corrected)
        module_class="AdafruitIMU",  # driver class
        i2c_param="i2c",
        accel_range_g=8.0,
        gyro_range_dps=2000.0,
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
