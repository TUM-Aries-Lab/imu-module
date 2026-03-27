"""Registry for device entry points."""

from importlib.metadata import entry_points

from loguru import logger

from imu_python.base_classes import IMUConfig
from imu_python.builtin_devices import MOCK
from imu_python.definitions import MOCK_NAME

IMU_DEVICES: dict[str, IMUConfig] = {}


def _load_registry() -> dict[str, IMUConfig]:
    registry = {}

    registry[MOCK_NAME] = MOCK

    for ep in entry_points(group="imu_module.devices"):
        config: IMUConfig = ep.load()
        registry[ep.name] = config
        logger.info(f"loaded IMU config {ep.name}")

    for ep in entry_points(group="imu_module.device_overrides"):
        registry[ep.name] = ep.load()
        logger.info(f"overrode IMU config {ep.name}")

    return registry


def reload_registry() -> None:
    """Reload all registered IMU devices and overrides.

    Call this after installing or uninstalling packages that register
    devices, or to pick up changes to override configs without restarting.
    """
    IMU_DEVICES.clear()
    IMU_DEVICES.update(_load_registry())


# Build registry at import time
reload_registry()
