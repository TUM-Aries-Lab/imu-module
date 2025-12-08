"""Test the available IMU devices."""

from imu_python.devices import IMUDevices


def test_device_addresses() -> None:
    """Test the IMU device addresses."""
    for device in IMUDevices:
        assert len(device.config.addresses) >= 2
