"""Test the IMUFactory class."""

from src.imu_python.devices import IMUDevices
from src.imu_python.sensor_manager import IMUManager
from src.imu_python.wrapper import IMUWrapper


def test_manager():
    """Mock scan_i2c_bus so it returns [0x00] using MagicMock."""
    # Arrange
    wrapper = IMUWrapper(config=IMUDevices.BASE.config, i2c_bus=None)

    # Act
    sensor_manager = IMUManager(imu_wrapper=wrapper, i2cid=None)
    sensor_manager.start()
    data = sensor_manager.get_data()
    sensor_manager.stop()

    # Assert
    assert data.accel is not None
    assert data.gyro is not None
    assert data.mag is None  # TODO: Not implemented yet
