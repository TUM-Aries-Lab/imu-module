"""Test the IMUFactory class."""

import pytest

from src.imu_python.devices import IMUDevices
from src.imu_python.sensor_manager import IMUManager
from src.imu_python.wrapper import IMUWrapper


def test_manager():
    """Mock scan_i2c_bus so it returns [0x00] using MagicMock."""
    # Arrange
    wrapper = IMUWrapper(config=IMUDevices.MOCK.config, i2c_bus=None)

    # Act
    sensor_manager = IMUManager(imu_wrapper=wrapper, i2c_id=None)
    sensor_manager.start()
    data = sensor_manager.get_data()
    sensor_manager.stop()

    # Assert
    assert data.device_data.accel is not None
    assert data.device_data.gyro is not None
    assert data.device_data.mag is None  # TODO: Not implemented yet


def test_manager_apply_remapping():
    """Test if manager applies a rotation_matrix to wrapper."""
    import numpy as np

    # Arrange
    wrapper = IMUWrapper(config=IMUDevices.MOCK.config, i2c_bus=None)
    rotation_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    wrong_rotation_matrix = np.eye(4)  # 4x4 identity, incorrect size

    # Act
    sensor_manager = IMUManager(imu_wrapper=wrapper, i2c_id=None)
    sensor_manager.set_rotation_matrix(rotation_matrix)

    # Assert
    assert np.array_equal(sensor_manager.imu_wrapper.rotation_matrix, rotation_matrix)
    with pytest.raises(ValueError):
        sensor_manager.set_rotation_matrix(wrong_rotation_matrix)
