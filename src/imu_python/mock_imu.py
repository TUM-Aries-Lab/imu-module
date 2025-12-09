"""Mock IMU module used for testing without hardware."""


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
