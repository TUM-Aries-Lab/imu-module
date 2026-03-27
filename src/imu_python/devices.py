"""Enum registry of IMU device configurations."""

from dataclasses import replace

from loguru import logger

from imu_python.base_classes import (
    IMUConfig,
)
from imu_python.definitions import MOCK_NAME, IMUDescriptor
from imu_python.registry import IMU_DEVICES


def get_mock() -> tuple[str, IMUConfig]:
    """Return a MOCK IMU with name and IMUConfig.

    :return: tuple with the name and IMUConfig of the MOCK IMU.
    """
    mock_name = MOCK_NAME
    try:
        return mock_name, IMU_DEVICES[mock_name]
    except KeyError:
        raise


def _from_address(addr: int) -> tuple[IMUDescriptor, IMUConfig] | None:
    """Return a tuple containing IMU anchor information and IMU config based on the given address, or None if unknown.

    :param addr: I2C address of the device
    :return: Information of the IMU (IMUDescriptor, IMUConfig) of the matched device or None
    """
    for device in IMU_DEVICES:
        base_config = IMU_DEVICES[device]

        for dev_id, sensor_cfg in base_config.devices.items():
            if addr not in sensor_cfg.addresses:
                continue

            # Compute instance index to be addr index
            instance_idx = sensor_cfg.addresses.index(addr)
            # narrow down to a single address
            narrowed_sensor = replace(sensor_cfg, addresses=[addr])
            # narrow down the device dict to contain only this device
            devices = {dev_id: narrowed_sensor}
            # narrow down the role dict to contain only this device
            roles = {
                role: target
                for role, target in base_config.roles.items()
                if target == dev_id
            }
            # combine the changes
            partial_config = replace(
                base_config,
                devices=devices,
                roles=roles,
            )

            # anchor = device name and index of the address in address list
            key = IMUDescriptor(name=device, index=instance_idx)

            logger.trace(
                f"Address 0x{addr:02X} matched to device {device} index {instance_idx}"
            )
            return key, partial_config

    return None


def get_config(addresses: list[int]) -> dict[IMUDescriptor, IMUConfig]:
    """Return a dictionary mapping imu name and device index to IMU Configs based on a list of addresses.

    The device index is the same as its address index on the address list, which is used to distinguish between multiple IMU devices of the same model.
    It is assumed that a split IMU has either high or low addresses across all of its devices. i.e.
    LSM6DSOX+LIS3MDL has the addresses 0x6A and 0x1C or 0x6B and 0x1E.

    :param addresses: list of detected addresses
    :return: A dictionary of IMUDescriptor as keys and IMUConfigs as values.
    """
    detected: dict[IMUDescriptor, IMUConfig] = {}

    for addr in addresses:
        result = _from_address(addr)
        if not result:
            continue

        key, partial = result

        if key not in detected:
            detected[key] = partial
        else:
            # merge partial IMUConfigs if devices belong to the same IMU
            base = detected[key]
            # add this device to the device list of IMUConfig with the same key
            merged_devices = dict(base.devices)
            merged_devices.update(partial.devices)
            # update roles to include roles of this device
            merged_roles = dict(base.roles)
            merged_roles.update(partial.roles)
            # update IMUConfig
            detected[key] = replace(
                base,
                devices=merged_devices,
                roles=merged_roles,
            )

    return detected
