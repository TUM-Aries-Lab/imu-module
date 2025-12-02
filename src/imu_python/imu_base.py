"""Abstract IMU base class."""

from __future__ import annotations

from abc import ABC, abstractmethod

from imu_python.imu_data import IMUData


class IMUBase(ABC):
    """Abstract base class for an IMU device."""

    @abstractmethod
    def initialize(self):
        """Initialize the sensor object."""
        pass

    @abstractmethod
    def acceleration(self) -> tuple[float, float, float]:
        """Return acceleration (m/s^2)."""
        pass

    @abstractmethod
    def magnetic(self) -> tuple[float, float, float]:
        """Return magnetometer reading (microteslas)."""
        pass

    @abstractmethod
    def gyro(self) -> tuple[float, float, float]:
        """Return gyroscope reading (rad/s)."""
        pass

    def all(self) -> IMUData:
        """Return acceleration, magnetic and gyro information as an IMUData."""
        accel = self.acceleration()
        mag = self.magnetic()
        gyro = self.gyro()
        return IMUData(acceleration=accel, magnetic=mag, gyro=gyro)
