"""Mock IMU module used for testing without hardware."""

import time

from imu_python.base_classes import IMUData, VectorXYZ


class MockIMU:
    """A hardware-free IMU class that mimics Adafruit sensor APIs.

    Matches the interface expected by IMUWrapper:
    .acceleration
    .gyro
    .magnetic
    """

    def __init__(self, i2c=None):
        """Initialize the mock IMU.

        :param i2c: Ignored. Present only for API compatibility.
        """
        self._accel = (0.0, 0.0, 0.0)
        self._gyro = (0.0, 0.0, 0.0)

    @property
    def acceleration(self):
        """Return mock acceleration data."""
        return self._accel

    @property
    def gyro(self):
        """Return mock acceleration data."""
        return self._gyro

    @property
    def all(self) -> IMUData:
        """Return acceleration, magnetic and gyro information as an IMUData."""
        accel = VectorXYZ(
            self.acceleration[0],
            self.acceleration[1],
            self.acceleration[2],
        )
        gyro = VectorXYZ(
            self.gyro[0],
            self.gyro[1],
            self.gyro[2],
        )
        return IMUData(
            timestamp=time.time(),
            accel=accel,
            gyro=gyro,
        )
