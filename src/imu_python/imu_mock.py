"""Mock IMU for testing."""

from __future__ import annotations

from imu_python.base_classes import VectorXYZ
from imu_python.imu_base import IMUBase


class FakeIMU(IMUBase):
    """Mock IMU that returns fixed test values."""

    def __init__(
        self,
        accel: tuple[float, float, float] = (0.0, 0.0, 0.0),
        mag: tuple[float, float, float] = (0.0, 0.0, 0.0),
        gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._started = False
        self._accel = accel
        self._mag = mag
        self._gyro = gyro

    def initialize(self):
        """Initialize the mock IMU."""
        self._started = True

    def acceleration(self) -> VectorXYZ:
        """Return mock acceleration information."""
        return VectorXYZ.from_tuple(self._accel)

    def magnetic(self) -> VectorXYZ:
        """Return mock magnetic information."""
        return VectorXYZ.from_tuple(self._mag)

    def gyro(self) -> VectorXYZ:
        """Return mock gyro information."""
        return VectorXYZ.from_tuple(self._gyro)
