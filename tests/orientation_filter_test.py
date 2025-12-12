"""Test the orientation filter class."""

import numpy as np

from src.imu_python.definitions import (
    ACCEL_GRAVITY_MSEC2,
    DEFAULT_QUAT_POSE,
    FilterConfig,
)
from src.imu_python.orientation_filter import OrientationFilter


def test_orientation_filter():
    """Test orientation filter."""
    # Arrange
    config = FilterConfig()
    accel = np.array([0.0, 0.0, ACCEL_GRAVITY_MSEC2])
    gyro = np.array([0.00, 0.00, 0.00])

    # Act
    magdwick_filter = OrientationFilter(gain=config.gain, frequency=config.freq_hz)
    for t in range(10):
        magdwick_filter.update(accel=accel, gyro=gyro, timestamp=t)

    # Assert
    for i in range(4):
        assert magdwick_filter.pose[i] == DEFAULT_QUAT_POSE[i]
