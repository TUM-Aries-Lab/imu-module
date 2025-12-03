"""BNO055 IMU implementation."""

from __future__ import annotations

import adafruit_bno055

from imu_python.base_classes import VectorXYZ
from imu_python.imu_base import IMUBase


class BNO055IMU(IMUBase):
    """IMU implementation using Adafruit BNO055."""

    def __init__(self, i2c) -> None:
        self.i2c = i2c
        self.sensor = None

    def initialize(self):
        """Initialize the BNO055 sensor."""
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def acceleration(self) -> VectorXYZ:
        """BNO055 sensor's acceleration information as a VectorXYZ."""
        return VectorXYZ.from_tuple(tuple(self.sensor.acceleration or (0.0, 0.0, 0.0)))  # type: ignore

    def gyro(self) -> VectorXYZ:
        """BNO055 sensor's gyro information as a VectorXYZ."""
        return VectorXYZ.from_tuple(tuple(self.sensor.gyro or (0.0, 0.0, 0.0)))  # type: ignore

    def magnetic(self) -> VectorXYZ:
        """BNO055 sensor's magnetic information as a VectorXYZ."""
        return VectorXYZ.from_tuple(tuple(self.sensor.magnetic or (0.0, 0.0, 0.0)))  # type: ignore
