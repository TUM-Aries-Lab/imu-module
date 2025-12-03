"""Abstract IMU base class."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod

from imu_python.base_classes import IMUData, VectorXYZ


class IMUBase(ABC):
    """Abstract base class for an IMU device."""

    @abstractmethod
    def initialize(self):
        """Initialize the sensor object."""
        pass

    @abstractmethod
    def acceleration(self) -> VectorXYZ:
        """Return acceleration (m/s^2)."""
        pass

    @abstractmethod
    def gyro(self) -> VectorXYZ:
        """Return gyroscope reading (rad/s)."""
        pass

    @abstractmethod
    def magnetic(self) -> VectorXYZ:
        """Return magnetometer reading (microteslas)."""
        pass

    def all(self) -> IMUData:
        """Return acceleration, magnetic and gyro information as an IMUData."""
        accel = self.acceleration()
        gyro = self.gyro()
        mag = self.magnetic()
        return IMUData(
            timestamp=time.time(),
            accel=accel,
            gyro=gyro,
            mag=mag,
        )
