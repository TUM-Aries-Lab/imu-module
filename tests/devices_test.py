"""Test the available IMU devices."""

from imu_python.devices import IMU_DEVICES


def test_device_addresses() -> None:
    """Test the IMU device addresses."""
    for device in IMU_DEVICES:
        assert len(device.addresses) >= 2
