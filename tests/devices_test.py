"""Test the available IMU devices."""

import pytest

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


@pytest.mark.parametrize("valid_address", IMUDevices.MOCK.config.addresses)
def test_device_from_valid_address(valid_address: int) -> None:
    """Test the IMU device addresses."""
    # Act
    config = IMUDevices.from_address(addr=valid_address)

    # Assert
    assert isinstance(config, IMUConfig)
    assert len(config.addresses) == 1
