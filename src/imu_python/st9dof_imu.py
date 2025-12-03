"""ST 9-DoF IMU implementation."""

from __future__ import annotations

from adafruit_lis3mdl import LIS3MDL
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS

from imu_python.base_classes import VectorXYZ
from imu_python.imu_base import IMUBase


class ST9DOFIMU(IMUBase):
    """IMU implementation using Adafruit 9-DoF."""

    def __init__(self, i2c) -> None:
        self.i2c = i2c
        self.sensor_accel_gyro = None
        self.sensor_mag = None

    def initialize(self):
        """Initialize the 9-DoF sensor."""
        self.sensor_accel_gyro = LSM6DS(self.i2c)
        self.sensor_mag = LIS3MDL(self.i2c)
        self.started = True

    def acceleration(self) -> VectorXYZ:
        """9-DoF sensor's acceleration information as a VectorXYZ."""
        return VectorXYZ.from_tuple(
            tuple(self.sensor_accel_gyro.acceleration or (0.0, 0.0, 0.0))  # type: ignore
        )

    def gyro(self) -> VectorXYZ:
        """9-DoF sensor's gyro information as a VectorXYZ."""
        return VectorXYZ.from_tuple(
            tuple(self.sensor_accel_gyro.gyro or (0.0, 0.0, 0.0))  # type: ignore
        )

    def magnetic(self) -> VectorXYZ:
        """9-DoF sensor's magnetic information as a VectorXYZ."""
        return VectorXYZ.from_tuple(tuple(self.sensor_mag.magnetic or (0.0, 0.0, 0.0)))  # type: ignore
