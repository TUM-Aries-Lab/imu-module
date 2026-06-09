"""Test the IMUFactory class."""

from unittest.mock import MagicMock

from src.imu_python.devices import IMU_DEVICES, get_mock
from src.imu_python.factory import IMUFactory


def test_imu_factory() -> None:
    """Test the IMUFactory class."""
    # Arrange
    mock_imu_name, mock_imu_config = get_mock()

    # Act
    imu_managers = IMUFactory.detect_and_create(create_mock=True)

    # Assert
    assert len(imu_managers) > 0
    for imu_manager in imu_managers:
        config = imu_manager.imu_wrapper.config
        # populate the wrapper device dict
        imu_manager.imu_wrapper.reload()

        # The manager should have at least one device
        assert len(config.devices) > 0

        # All roles must point to an actual device
        for role, device_id in config.roles.items():
            assert device_id in config.devices

            # Each role attribute should exist in the device driver
            device = imu_manager.imu_wrapper._devices.get(device_id)
            attr_name = role.value
            # getattr should succeed without error
            getattr(device, attr_name, None)

        # check that config matches the expected IMU name
        assert mock_imu_name in IMU_DEVICES
        assert mock_imu_config == IMU_DEVICES[mock_imu_name]


def test_scan_i2c_bus_success() -> None:
    """Test scan_i2c_bus with successful scan."""
    # Arrange
    mock_i2c = MagicMock()
    mock_i2c.try_lock.return_value = True
    expected_addresses = [0x28, 0x6A, 0x1C]
    mock_i2c.scan.return_value = expected_addresses

    # Act
    addresses = IMUFactory.scan_i2c_bus(i2c=mock_i2c)

    # Assert
    assert addresses == expected_addresses
    mock_i2c.try_lock.assert_called_once()
    mock_i2c.scan.assert_called_once()
    mock_i2c.unlock.assert_called_once()


def test_scan_i2c_bus_exception_returns_mock_addresses() -> None:
    """Test scan_i2c_bus returns mock addresses on exception."""
    # Arrange
    mock_i2c = MagicMock()
    mock_i2c.try_lock.return_value = True
    mock_i2c.scan.side_effect = OSError("I2C communication error")

    # Act
    addresses = IMUFactory.scan_i2c_bus(i2c=mock_i2c)

    # Assert
    # Should return mock addresses
    _, mock_config = get_mock()
    expected_addresses = [a for d in mock_config.devices.values() for a in d.addresses]
    assert addresses == expected_addresses
    mock_i2c.try_lock.assert_called()
    mock_i2c.scan.assert_called_once()
    # unlock should not be called when scan() raises exception
    mock_i2c.unlock.assert_not_called()
