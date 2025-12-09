"""Test the factory and manager for the imu sensor objects."""

import numpy as np

from src.imu_python.base_classes import VectorXYZ
from src.imu_python.devices import IMUDevices
from src.imu_python.wrapper import IMUWrapper


def test_imu_wrapper() -> None:
    """Test the imu wrapper class."""
    # Arrange
    config = IMUDevices.MOCK.config

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()

    # Assert
    assert wrapper.imu.gyro[0] == 0.0
    assert wrapper.imu.gyro[1] == 0.0
    assert wrapper.imu.gyro[2] == 0.0

    assert wrapper.imu.gyro[0] == 0.0
    assert wrapper.imu.gyro[1] == 0.0
    assert wrapper.imu.gyro[2] == 0.0


def test_imu_wrapper_bad_attr() -> None:
    """Test the imu wrapper class."""
    # Arrange
    config = IMUDevices.MOCK.config
    nan_vector_xyz = VectorXYZ(np.nan, np.nan, np.nan)

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()
    data = wrapper.read_imu_vector("magnetic")

    # Assert
    assert data.x is nan_vector_xyz.x
    assert data.y is nan_vector_xyz.y
    assert data.z is nan_vector_xyz.z
