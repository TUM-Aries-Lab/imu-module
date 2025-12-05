"""Stores all config info of IMU devices."""

from .base_classes import IMUConfig

IMUDevices = [
    IMUConfig(
        name="BNO055",
        addresses=[0x28, 0x29],
        library="adafruit_bno055",
        driver_class="BNO055_I2C",
    ),
    IMUConfig(
        name="LSM6DSOX",
        addresses=[0x6A, 0x6B],
        library="adafruit_lsm6ds.lsm6dsox",
        driver_class="LSM6DSOX",
    ),
]
