"""Test the available IMU devices."""

from unittest.mock import patch

import pytest

from imu_python.base_classes import IMUConfig, IMUSensorTypes
from imu_python.definitions import IMUDeviceID
from imu_python.devices import IMUDevices


def test_device_addresses() -> None:
    """Test the IMU device addresses."""
    for device in IMUDevices:
        assert (len(a.addresses) == 2 for a in device.config.devices.values())


def test_device_from_bad_address() -> None:
    """Test the IMU device addresses."""
    # Arrange
    invalid_addr = 0xAA

    # Act
    config = IMUDevices._from_address(addr=invalid_addr)

    # Assert
    assert config is None


@pytest.mark.parametrize(
    "valid_address",
    [addr for dev in IMUDevices.MOCK.config.devices.values() for addr in dev.addresses],
)
def test_device_from_valid_address(valid_address: int) -> None:
    """Test the IMU device addresses."""
    # Act
    imu_info = IMUDevices._from_address(addr=valid_address)

    # Assert
    assert imu_info is not None
    assert isinstance(imu_info[1], IMUConfig)
    assert len(imu_info[1].devices) == 1
    assert (len(a.addresses) == 1 for a in imu_info[1].devices.values())


def test_merge_partial_configs() -> None:
    """Test get_config merges partial configs."""
    from dataclasses import replace

    # Build a mutated MOCK config with a second device that shares
    # the same address index (1) for its address list so merging occurs.
    config = IMUDevices.MOCK.config
    imu0_sensor = config.devices[IMUDeviceID.IMU0]
    # create a second sensor with a different address pair
    imu1_sensor = replace(imu0_sensor, name=IMUDeviceID.IMU1, addresses=[0x10, 0x11])
    mutated_devices = dict(config.devices)
    mutated_devices[IMUDeviceID.IMU1] = imu1_sensor
    mutated_config = replace(config, devices=mutated_devices)
    mutated_config.roles.update({IMUSensorTypes.mag: IMUDeviceID.IMU1})

    # Patch the MOCK member so that there are two devices in MOCK
    with patch.object(IMUDevices.MOCK, "_value_", mutated_config):
        # Choose addresses that map to index 1 in both device address lists
        detected = IMUDevices.get_config([0x01, 0x11])

    assert (IMUDevices.MOCK.name, 1) in detected
    cfg = detected[(IMUDevices.MOCK.name, 1)]
    assert isinstance(cfg, IMUConfig)
    # merged devices should contain both entries
    assert len(cfg.devices) == 2
    assert IMUDeviceID.IMU0 in cfg.devices and IMUDeviceID.IMU1 in cfg.devices
