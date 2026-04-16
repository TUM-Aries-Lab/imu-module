"""Test the available IMU devices."""

from unittest.mock import patch

import pytest

from imu_python.base_classes import IMUConfig, IMUSensorTypes
from imu_python.definitions import MOCK_NAME, IMUDescriptor, IMUDeviceID
from imu_python.devices import IMU_DEVICES, _from_address, get_config


def test_device_addresses() -> None:
    """Test the IMU device addresses."""
    for device in IMU_DEVICES:
        assert all(len(a.addresses) == 2 for a in IMU_DEVICES[device].devices.values())


def test_device_from_bad_address() -> None:
    """Test the IMU device addresses."""
    # Arrange
    invalid_addr = 0xAA

    # Act
    config = _from_address(addr=invalid_addr)

    # Assert
    assert config is None


@pytest.mark.parametrize(
    "valid_address",
    [addr for dev in IMU_DEVICES[MOCK_NAME].devices.values() for addr in dev.addresses],
)
def test_device_from_valid_address(valid_address: int) -> None:
    """Test the IMU device addresses."""
    # Act
    imu_info = _from_address(addr=valid_address)

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
    config = IMU_DEVICES[MOCK_NAME]
    imu0_sensor = config.devices[IMUDeviceID.IMU0]
    # create a second sensor with a different address pair
    imu1_sensor = replace(imu0_sensor, name=IMUDeviceID.IMU1, addresses=[0x10, 0x11])
    mutated_devices = dict(config.devices)
    mutated_devices[IMUDeviceID.IMU1] = imu1_sensor
    mutated_config = replace(config, devices=mutated_devices)
    mutated_config.roles.update({IMUSensorTypes.mag: IMUDeviceID.IMU1})

    # Patch the MOCK member so that there are two devices in MOCK
    with patch.dict(IMU_DEVICES, {MOCK_NAME: mutated_config}):
        # Choose addresses that map to index 1 in both device address lists
        detected = get_config([0x01, 0x11])

    dsc = IMUDescriptor(MOCK_NAME, index=1)
    cfg = detected[dsc]
    assert dsc in detected
    assert isinstance(cfg, IMUConfig)
    # merged devices should contain both entries
    assert len(cfg.devices) == 2
    assert IMUDeviceID.IMU0 in cfg.devices and IMUDeviceID.IMU1 in cfg.devices
