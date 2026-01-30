"""Test the IMUFactory class."""

import time

import pytest

from src.imu_python.devices import IMUDevices
from src.imu_python.sensor_manager import IMUManager
from src.imu_python.wrapper import IMUWrapper


@pytest.fixture
def imu_setup() -> IMUManager:
    """Fixture providing sensor_manager for tests."""
    wrapper = IMUWrapper(config=IMUDevices.MOCK.config, i2c_bus=None)
    sensor_manager = IMUManager(imu_wrapper=wrapper, i2c_id=None, imu_id=("MOCK", 0))
    return sensor_manager


def test_manager(imu_setup: IMUManager) -> None:
    """Mock scan_i2c_bus so it returns [0x00] using MagicMock."""
    # Arrange
    sensor_manager = imu_setup

    # Act
    sensor_manager.start()
    time.sleep(0.01)  # wait for data to be read
    data = sensor_manager.get_data()
    sensor_manager.stop()

    # Assert
    assert data is not None
    assert data.device_data.accel is not None
    assert data.device_data.gyro is not None
    assert data.device_data.mag is None  # TODO: Not implemented yet


def test_manager_apply_remapping(imu_setup: IMUManager) -> None:
    """Test if manager applies a rotation_matrix to wrapper."""
    import numpy as np

    # Arrange
    sensor_manager = imu_setup
    rotation_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    # Act
    sensor_manager.set_rotation_matrix(rotation_matrix)

    # Assert
    assert np.array_equal(sensor_manager.imu_wrapper.rotation_matrix, rotation_matrix)


def test_manager_apply_wrong_rotation(imu_setup: IMUManager) -> None:
    """Test if manager rejects a wrong rotation matrix."""
    import numpy as np

    # Arrange
    sensor_manager = imu_setup
    wrong_rotation_matrix = np.eye(4)  # 4x4 identity, incorrect size

    # Act & Assert
    with pytest.raises(ValueError):
        sensor_manager.set_rotation_matrix(wrong_rotation_matrix)


def test_manager_pauses_during_disconnect(imu_setup: IMUManager) -> None:
    """Test if manager pauses data reading during disconnection."""
    # Arrange
    sensor_manager = imu_setup
    sensor_manager.start()
    time.sleep(0.01)  # wait for data to be read

    # Act: simulate disconnection
    sensor_manager.imu_wrapper._devices["imu0"].disconnect()
    time.sleep(0.01)  # wait for disconnection to be registered

    # Assert: ensure no new data is read
    assert not sensor_manager.imu_wrapper.started
    assert sensor_manager.get_data() is None

    # clean up
    sensor_manager.stop()
