"""Test the IMUFactory class."""

from src.imu_python.devices import IMUDevices
from src.imu_python.sensor_manager import IMUManager
from src.imu_python.wrapper import IMUWrapper


def test_manager():
    """Mock scan_i2c_bus so it returns [0x00] using MagicMock."""
    # Arrange
    wrapper = IMUWrapper(config=IMUDevices.MOCK.config, i2c_bus=None)

    # Act
    sensor_manager = IMUManager(imu_wrapper=wrapper)
    sensor_manager.start()
    data = sensor_manager.get_data()
    sensor_manager.stop()

    # Assert
    assert data.accel.x == 0.0
    assert data.accel.y == 0.0
    assert data.accel.z == 0.0
    assert data.gyro.x == 0.0
    assert data.gyro.y == 0.0
    assert data.gyro.z == 0.0
    assert data.mag is None
