"""Wrapper class for the IMUs."""

import importlib
import time
import types

import numpy as np
from loguru import logger

from imu_python.base_classes import AdafruitIMU, IMUConfig, IMUData, VectorXYZ
from imu_python.definitions import FilterConfig
from imu_python.orientation_filter import OrientationFilter


class IMUWrapper:
    """Wrapper class for the IMU sensors."""

    def __init__(self, config: IMUConfig, i2c_bus):
        """Initialize the wrapper.

        :param config: IMU configuration object.
        :param i2c_bus: i2c bus this device is connected to.
        """
        self.config: IMUConfig = config
        self.i2c = i2c_bus
        self.started: bool = False
        self.imu: AdafruitIMU = AdafruitIMU()
        self.filter: OrientationFilter = OrientationFilter(
            gain=FilterConfig.gain, frequency=FilterConfig.freq_hz
        )  # TODO: set gain for each IMU

    def reload(self) -> None:
        """Initialize the sensor object."""
        try:
            module = self._import_imu_module()
            imu_class = self._load_class(module=module)
            self.imu = imu_class(self.i2c)
            self.started = True
        except Exception as err:
            logger.error(f"Failed to load imu: {err}")

    def read_imu_vector(self, attr: str) -> VectorXYZ:
        """Read the IMU attributes."""
        data = getattr(self.imu, attr, None)
        if data:
            return VectorXYZ.from_tuple(data)
        else:
            logger.warning(f"IMU:{self.config.name} - No {attr} data.")
            return VectorXYZ(np.nan, np.nan, np.nan)

    def get_data(self) -> IMUData:
        """Return acceleration and gyro information as an IMUData."""
        accel_vector = self.read_imu_vector("acceleration")
        gyro_vector = self.read_imu_vector("gyro")
        pose_quat = self.filter.update(accel_vector.as_array(), accel_vector.as_array())

        return IMUData(
            timestamp=time.time(),
            accel=accel_vector,
            gyro=gyro_vector,
            pose=pose_quat,
        )

    def _import_imu_module(self) -> types.ModuleType:
        """Dynamically import the IMU driver module.

        Example: "adafruit_bno055" -> <module 'adafruit_bno055'>
        """
        try:
            module = importlib.import_module(self.config.library)
            return module
        except ImportError as err:
            raise RuntimeError(
                f"{err} - Failed to import IMU driver '{self.config.library}'."
            ) from err

    def _load_class(self, module) -> type[AdafruitIMU]:
        imu_class = getattr(module, self.config.module_class, None)
        if imu_class is None:
            raise RuntimeError(
                f"Module '{module}' has no class '{self.config.module_class}'"
            )
        return imu_class
