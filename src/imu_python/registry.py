"""Registry for device entry points."""

from importlib.metadata import entry_points

from loguru import logger

from imu_python.base_classes import IMUConfig
from imu_python.builtin_devices import BNO055, BNO08X, LSM6DSOX_LIS3MDL, MOCK
from imu_python.definitions import MOCK_NAME

IMU_DEVICES: dict[str, IMUConfig] = {}


def _load_registry() -> dict[str, IMUConfig]:
    """Load all registered IMU devices and overrides from entry points.

    :return: dict of IMU names and IMUConfigs
    """
    registry: dict[str, IMUConfig] = {}

    # register built-in IMUs
    registry[MOCK_NAME] = MOCK
    registry["BNO08X"] = BNO08X
    registry["BNO055"] = BNO055
    registry["LSM6DSOX_LIS3MDL"] = LSM6DSOX_LIS3MDL

    for ep in entry_points(group="imu_module.devices"):
        config = ep.load()
        if not isinstance(config, IMUConfig):
            logger.warning(
                f"Entry point {ep.name} did not return an IMUConfig, skipping"
            )
            continue
        registry[ep.name] = config
        logger.info(f"loaded IMU config {ep.name}")

    for ep in entry_points(group="imu_module.device_overrides"):
        config = ep.load()
        if not isinstance(config, IMUConfig):
            logger.warning(
                f"Entry point {ep.name} did not return an IMUConfig, skipping"
            )
            continue
        registry[ep.name] = config
        logger.info(f"overrode IMU config {ep.name}")

    return registry


def reload_registry() -> None:
    """Reload all registered IMU devices and overrides.

    IMU config changes are applied only during init time.
    :return: None
    """
    IMU_DEVICES.clear()
    IMU_DEVICES.update(_load_registry())


# Build registry at import time
reload_registry()
