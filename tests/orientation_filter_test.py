"""Test the orientation filter class."""

import numpy as np
import pytest
from numpy.typing import NDArray

from src.imu_python.definitions import (
    ACCEL_GRAVITY_MSEC2,
    DEFAULT_QUAT_POSE,
    FilterConfig,
)
from src.imu_python.orientation_filters import MadgwickFilterAHRS, MadgwickFilterPyImu


def test_ahrs_filter():
    """Test orientation filter."""
    # Arrange
    config = FilterConfig()
    accel = np.array([0.0, 0.0, ACCEL_GRAVITY_MSEC2])
    gyro = np.array([0.00, 0.00, 0.00])

    # Act
    magdwick_filter = MadgwickFilterAHRS(config=config)
    for t in range(10):
        magdwick_filter.update(accel=accel, gyro=gyro, timestamp=t)

    # Assert
    for i in range(4):
        assert magdwick_filter.quat[i] == DEFAULT_QUAT_POSE[i]


@pytest.mark.parametrize(
    "gyro",
    [
        np.array([0.01, 0.01, 0.01]),
        np.array([-0.01, -0.01, -0.01]),
        np.array([0.01, 0.02, 0.03]),
    ],
)
def test_py_imu_filter(gyro: NDArray):
    """Test orientation filter."""
    # Arrange
    config = FilterConfig()
    accel = np.array([0.0, 0.0, ACCEL_GRAVITY_MSEC2])

    # Act
    magdwick_filter = MadgwickFilterPyImu(config=config)
    for t in range(10):
        magdwick_filter.update(accel=accel, gyro=gyro, timestamp=t)

    # Assert
    for i in range(4):
        np.testing.assert_almost_equal(
            magdwick_filter.quat[i], DEFAULT_QUAT_POSE[i], decimal=2
        )
