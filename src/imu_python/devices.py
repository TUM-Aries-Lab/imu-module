"""Enum registry of IMU device configurations."""

from dataclasses import replace
from enum import Enum

from imu_python.base_classes import (
    IMUConfig,
    IMUParamNames,
    IMUSensorTypes,
    PreConfigStep,
    PreConfigStepType,
    SensorConfig,
)
from imu_python.definitions import FilterConfig


class IMUDevices(Enum):
    """Enumeration containing configuration for all supported IMU devices."""

    BNO055 = IMUConfig(
        devices={
            "imu0": SensorConfig(
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
            IMUSensorTypes.accel: "imu0",
            IMUSensorTypes.gyro: "imu0",
            IMUSensorTypes.mag: "imu0",
        },
        accel_range_g=4.0,
        gyro_range_dps=2000.0,
        filter_config=FilterConfig(freq_hz=100.0, gain=0.002250),
        # Note: Gyro range setting does not actually work on the BNO055
    )

    LSM6DSOX_LIS3MDL = IMUConfig(
        devices={
            "imu0": SensorConfig(
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
                        args=("Rate.RATE_104_HZ",),
                        step_type=PreConfigStepType.SET,
                    ),
                    PreConfigStep(
                        name="gyro_data_rate",
                        args=("Rate.RATE_104_HZ",),
                        step_type=PreConfigStepType.SET,
                    ),
                ],
            ),
            "imu1": SensorConfig(
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
                ],
            ),
        },
        roles={
            IMUSensorTypes.accel: "imu0",
            IMUSensorTypes.gyro: "imu0",
            IMUSensorTypes.mag: "imu1",
        },
        accel_range_g=4.0,
        gyro_range_dps=500.0,
        filter_config=FilterConfig(freq_hz=104.0, gain=0.000573),
    )

    BNO08x = IMUConfig(
        devices={
            "imu0": SensorConfig(
                name="BNO08x",
                addresses=[0x4A, 0x4B],
                library="adafruit_bno08x.i2c",
                module_class="BNO08X_I2C",
                param_names=IMUParamNames(i2c="i2c_bus", address="address"),
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
                    PreConfigStep(
                        name="enable_feature",
                        args=("BNO_REPORT_MAGNETOMETER",),
                        step_type=PreConfigStepType.CALL,
                    ),
                ],
            ),
        },
        roles={
            IMUSensorTypes.accel: "imu0",
            IMUSensorTypes.gyro: "imu0",
            IMUSensorTypes.mag: "imu0",
        },
        accel_range_g=8.0,  # default 8g, not settable in driver
        gyro_range_dps=2000.0,  # default 2000dps, not settable in driver
        filter_config=FilterConfig(
            freq_hz=20.0, gain=0.001538
        ),  # 50 ms update interval by default
    )

    MOCK = IMUConfig(
        devices={
            "imu0": SensorConfig(
                name="MOCK",
                addresses=[0x00, 0x01],  # fake I2C addresses for testing
                library="imu_python.base_classes",  # module path (corrected)
                module_class="AdafruitIMU",  # driver class
                param_names=IMUParamNames(i2c="i2c", address="address"),
            ),
        },
        roles={
            IMUSensorTypes.accel: "imu0",
            IMUSensorTypes.gyro: "imu0",
        },
        accel_range_g=8.0,
        gyro_range_dps=2000.0,
    )

    @property
    def config(self) -> IMUConfig:
        """Return the IMUConfig stored inside the enum member."""
        return self.value

    @staticmethod
    def _from_address(addr: int) -> tuple[tuple[str, int], IMUConfig] | None:
        """Return a tuple containing IMU anchor information and IMU config based on the given address, or None if unknown.

        :param addr: I2C address of the device
        :return: Information of the IMU ((imu name, device index), IMUConfig) of the matched device or None
        """
        for device in IMUDevices:
            base_config: IMUConfig = device.value

            for dev_id, sensor_cfg in base_config.devices.items():
                if addr not in sensor_cfg.addresses:
                    continue

                # Compute instance index to be addr index
                instance_idx = sensor_cfg.addresses.index(addr)
                # narrow down to a single address
                narrowed_sensor = replace(sensor_cfg, addresses=[addr])
                # narrow down the device dict to contain only this device
                devices = {dev_id: narrowed_sensor}
                # narrow down the role dict to contain only this device
                roles = {
                    role: target
                    for role, target in base_config.roles.items()
                    if target == dev_id
                }
                # combine the changes
                partial_config = replace(
                    base_config,
                    devices=devices,
                    roles=roles,
                )

                # anchor = device name and index of the address in address list
                key = (device.name, instance_idx)

                return key, partial_config

        return None

    @staticmethod
    def get_config(addresses: list[int]) -> dict[tuple[str, int], IMUConfig]:
        """Return a dictionary mapping imu name and device index to IMU Configs based on a list of addresses.

        The device index is the same as its address index on the address list, which is used to distinguish between multiple IMU devices of the same model.
        It is assumed that a split IMU has either high or low addresses across all of its devices. i.e.
        LSM6DSOX+LIS3MDL has the addresses 0x6A and 0x1C or 0x6B and 0x1E.

        :param addresses: list of detected addresses
        :type addresses: list[int]
        :return: A dictionary of IMU names as keys and IMUConfigs as values.
        :rtype: dict[str, IMUConfig]
        """
        detected: dict[tuple[str, int], IMUConfig] = {}

        for addr in addresses:
            result = IMUDevices._from_address(addr)
            if not result:
                continue

            key, partial = result

            if key not in detected:
                detected[key] = partial
            else:
                # merge partial IMUConfigs if devices belong to the same IMU
                base = detected[key]
                # add this device to the device list of IMUConfig with the same key
                merged_devices = dict(base.devices)
                merged_devices.update(partial.devices)
                # update roles to include roles of this device
                merged_roles = dict(base.roles)
                merged_roles.update(partial.roles)
                # update IMUConfig
                detected[key] = replace(
                    base,
                    devices=merged_devices,
                    roles=merged_roles,
                )

        return detected
