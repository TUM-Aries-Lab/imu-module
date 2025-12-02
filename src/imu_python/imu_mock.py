"""Mock IMU for testing."""

from __future__ import annotations

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

    def acceleration(self) -> tuple[float, float, float]:
        """Return mock acceleration information."""
        return self._accel

    def magnetic(self) -> tuple[float, float, float]:
        """Return mock magnetic information."""
        return self._mag

    def gyro(self) -> tuple[float, float, float]:
        """Return mock gyro information."""
        return self._gyro
