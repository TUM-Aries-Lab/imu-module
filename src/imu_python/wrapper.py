"""Wrapper class for the IMUs."""

import importlib
import time
import types

import numpy as np
from loguru import logger

from imu_python.base_classes import AdafruitIMU, IMUConfig, IMUData, VectorXYZ


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

    def reload(self) -> None:
        """Initialize the sensor object."""
        # Dynamically import the IMU library
        module = self._import_imu_module(self.config.library)

        # Instantiate the driver class
        imu_class = getattr(module, self.config.module_class, None)
        if imu_class is None:
            raise RuntimeError(
                f"Module '{self.config.library}' has no class '{self.config.module_class}'"
            )
        self.imu = imu_class(self.i2c)
        self.started = True

    def acceleration(self) -> VectorXYZ:
        """Sensor's acceleration information as a VectorXYZ."""
        accel_data = self.imu.acceleration
        if accel_data:
            return VectorXYZ.from_tuple(accel_data)
        else:
            logger.warning(f"IMU:{self.config.name} - No acceleration data.")
            return VectorXYZ(np.nan, np.nan, np.nan)

    def gyro(self) -> VectorXYZ:
        """Sensor's gyro information as a VectorXYZ."""
        gyro_data = self.imu.gyro
        if gyro_data:
            return VectorXYZ.from_tuple(gyro_data)
        else:
            logger.warning(f"IMU:{self.config.name} - No gyro data.")
            return VectorXYZ(np.nan, np.nan, np.nan)

    def all(self) -> IMUData:
        """Return acceleration, magnetic and gyro information as an IMUData."""
        accel = self.acceleration()
        gyro = self.gyro()
        return IMUData(
            timestamp=time.time(),
            accel=accel,
            gyro=gyro,
        )

    @staticmethod
    def _import_imu_module(library_path: str) -> types.ModuleType:
        """Dynamically import the IMU driver module.

        Example: "adafruit_bno055" -> <module 'adafruit_bno055'>
        """
        try:
            module = importlib.import_module(library_path)
            return module
        except ImportError as err:
            raise RuntimeError(
                f"{err} - Failed to import IMU driver '{library_path}'."
            ) from err
