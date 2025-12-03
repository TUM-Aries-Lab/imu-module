"""Mock IMU for testing."""

from __future__ import annotations

import random

from imu_python.base_classes import VectorXYZ
from imu_python.imu_base import IMUBase


class FakeIMU(IMUBase):
    """Mock IMU that returns random values."""

    def __init__(self) -> None:
        self._disconnect = False
        self._accel = (VectorXYZ.from_tuple((0.0, 0.0, 0.0)),)
        self._mag = (VectorXYZ.from_tuple((0.0, 0.0, 0.0)),)
        self._gyro = (VectorXYZ.from_tuple((0.0, 0.0, 0.0)),)

    def initialize(self):
        """Initialize the mock IMU."""
        self.started = True
        self._disconnect = False

    def acceleration(self) -> VectorXYZ:
        """Return mock acceleration information."""
        if self._disconnect:
            raise OSError(121, "Simulated I2C disconnect")  # errno 121 is EIO on Linux
        self._accel = self._random_vector()
        return self._accel

    def gyro(self) -> VectorXYZ:
        """Return mock gyro information."""
        if self._disconnect:
            raise OSError(121, "Simulated I2C disconnect")  # errno 121 is EIO on Linux
        self._gyro = self._random_vector()
        return self._gyro

    def magnetic(self) -> VectorXYZ:
        """Return mock magnetic information."""
        if self._disconnect:
            raise OSError(121, "Simulated I2C disconnect")  # errno 121 is EIO on Linux
        self._mag = self._random_vector()
        return self._mag

    @staticmethod
    def _random_vector() -> VectorXYZ:
        return VectorXYZ(random.random(), random.random(), random.random())  # noqa: S311

    def disconnect(self):
        """Simulate IMU disconnection."""
        self._disconnect = True

    def reconnect(self):
        """Simulate IMU reconnection."""
        self._disconnect = False
