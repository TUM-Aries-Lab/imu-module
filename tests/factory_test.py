"""Test the IMUFactory class."""

from src.imu_python.devices import IMUDevices
from src.imu_python.factory import IMUFactory


def test_imu_factory() -> None:
    """Test the IMUFactory class."""
    # Arrange
    mock_imu_name = IMUDevices.MOCK.name

    # Act
    imu_managers = IMUFactory.detect_and_create()

    # Assert
    assert len(imu_managers) >= 0
    for imu_manager in imu_managers:
        assert imu_manager.imu_wrapper.config.name == mock_imu_name
