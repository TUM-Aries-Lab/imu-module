"""Test the available IMU devices."""

from imu_python.base_classes import IMUConfig
from imu_python.devices import IMUDevices


def test_device_addresses() -> None:
    """Test the IMU device addresses."""
    for device in IMUDevices:
        assert len(device.config.addresses) == 2


def test_device_from_bad_address() -> None:
    """Test the IMU device addresses."""
    # Arrange
    invalid_addr = 0xAA

    # Act
    config = IMUDevices.from_address(addr=invalid_addr)

    # Assert
    assert config is None


def test_device_from_good_address() -> None:
    """Test the IMU device addresses."""
    # Arrange
    valid_addr = IMUDevices.MOCK.config.addresses[0]

    # Act
    config = IMUDevices.from_address(addr=valid_addr)

    # Assert
    assert isinstance(config, IMUConfig)
