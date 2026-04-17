"""IMUConfig for known supported IMUs.

Current configs: BNO055, BNO08x, LSM6DSOX+LIS3MDL
"""

from imu_python.base_classes import (
    IMUConfig,
    IMUParamNames,
    IMUSensorTypes,
    PreConfigStep,
    PreConfigStepType,
    SensorConfig,
)
from imu_python.definitions import MOCK_NAME, FilterConfig, IMUDeviceID

BNO055 = IMUConfig(
    devices={
        IMUDeviceID.IMU0: SensorConfig(
            name="BNO055",
            addresses=[0x28, 0x29],
            library="adafruit_bno055",  # module import path
            module_class="BNO055_I2C",  # driver class inside the module
            param_names=IMUParamNames(i2c="i2c", address="address"),
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
                    args=("GYRO_2000_DPS",),
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
                # Switch to AMG mode where accel, gyro and mag are on
                PreConfigStep(
                    name="mode",
                    args=("AMG_MODE",),
                    step_type=PreConfigStepType.SET,
                ),
            ],
        ),
    },
    roles={
        IMUSensorTypes.accel: IMUDeviceID.IMU0,
        IMUSensorTypes.gyro: IMUDeviceID.IMU0,
        IMUSensorTypes.mag: IMUDeviceID.IMU0,
    },
    accel_range_g=4.0,
    gyro_range_dps=2000.0,
    filter_config=FilterConfig(freq_hz=100.0, gain=0.002250),
    # Note: Gyro range setting does not actually work on the BNO055
)

LSM6DSOX_LIS3MDL = IMUConfig(
    devices={
        IMUDeviceID.IMU0: SensorConfig(
            name="LSM6DSOX",
            addresses=[0x6A, 0x6B],
            library="adafruit_lsm6ds.lsm6dsox",
            module_class="LSM6DSOX",
            param_names=IMUParamNames(i2c="i2c_bus", address="address"),
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
                    args=("Rate.RATE_416_HZ",),
                    step_type=PreConfigStepType.SET,
                ),
                PreConfigStep(
                    name="gyro_data_rate",
                    args=("Rate.RATE_416_HZ",),
                    step_type=PreConfigStepType.SET,
                ),
            ],
        ),
        IMUDeviceID.IMU1: SensorConfig(
            name="LIS3MDL",
            addresses=[0x1C, 0x1E],
            library="adafruit_lis3mdl",
            module_class="LIS3MDL",
            param_names=IMUParamNames(i2c="i2c_bus", address="address"),
            pre_config=[
                PreConfigStep(
                    name="range",
                    args=("Range.RANGE_4_GAUSS",),
                    step_type=PreConfigStepType.SET,
                ),
                PreConfigStep(
                    name="data_rate",
                    args=("Rate.RATE_40_HZ",),
                    step_type=PreConfigStepType.SET,
                ),
            ],
        ),
    },
    roles={
        IMUSensorTypes.accel: IMUDeviceID.IMU0,
        IMUSensorTypes.gyro: IMUDeviceID.IMU0,
        IMUSensorTypes.mag: IMUDeviceID.IMU1,
    },
    accel_range_g=4.0,
    gyro_range_dps=500.0,
    filter_config=FilterConfig(freq_hz=104.0, gain=0.000573),
)

BNO08X = IMUConfig(
    devices={
        IMUDeviceID.IMU0: SensorConfig(
            name="BNO08x",
            addresses=[0x4A, 0x4B],
            library="adafruit_bno08x.i2c",
            module_class="BNO08X_I2C",
            param_names=IMUParamNames(i2c="i2c_bus", address="address"),
            constants_module="adafruit_bno08x",
            pre_config=[
                PreConfigStep(
                    name="enable_feature",
                    args=("BNO_REPORT_ACCELEROMETER", 10000),
                    step_type=PreConfigStepType.CALL,
                ),
                PreConfigStep(
                    name="enable_feature",
                    args=("BNO_REPORT_GYROSCOPE", 10000),
                    step_type=PreConfigStepType.CALL,
                ),
                PreConfigStep(
                    name="enable_feature",
                    args=("BNO_REPORT_MAGNETOMETER",),
                    step_type=PreConfigStepType.CALL,
                ),
                PreConfigStep(
                    name="enable_feature",
                    args=("BNO_REPORT_ROTATION_VECTOR", 10000),
                    step_type=PreConfigStepType.CALL,
                ),
            ],
        ),
    },
    roles={
        IMUSensorTypes.accel: IMUDeviceID.IMU0,
        IMUSensorTypes.gyro: IMUDeviceID.IMU0,
        IMUSensorTypes.mag: IMUDeviceID.IMU0,
        IMUSensorTypes.quat: IMUDeviceID.IMU0,
    },
    accel_range_g=8.0,  # default 8g, not settable in driver
    gyro_range_dps=2000.0,  # default 2000dps, not settable in driver
)

MOCK = IMUConfig(
    devices={
        IMUDeviceID.IMU0: SensorConfig(
            name=MOCK_NAME,
            addresses=[0x00, 0x01],  # fake I2C addresses for testing
            library="imu_python.base_classes",  # module path (corrected)
            module_class="AdafruitIMU",  # driver class
            param_names=IMUParamNames(i2c="i2c", address="address"),
        ),
    },
    roles={
        IMUSensorTypes.accel: IMUDeviceID.IMU0,
        IMUSensorTypes.gyro: IMUDeviceID.IMU0,
    },
    accel_range_g=8.0,
    gyro_range_dps=2000.0,
)
