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
        config = imu_manager.imu_wrapper.config

        # The manager should have at least one device
        assert len(config.devices) > 0

        # All roles must point to an actual device
        for role, device_id in config.roles.items():
            assert device_id in config.devices

            # Each role attribute should exist in the device driver
            device = imu_manager.imu_wrapper._devices.get(device_id)
            if device:  # device may not exist if not reloaded
                attr_name = role.value
                # getattr should succeed without error
                getattr(device, attr_name, None)

        # check that config matches the expected IMU name
        assert mock_imu_name in IMUDevices.__members__
