"""Wrapper class for the IMUs."""

import importlib
import time
import types

from loguru import logger

from imu_python.base_classes import (
    AdafruitIMU,
    IMUConfig,
    IMUData,
    IMUSensorTypes,
    VectorXYZ,
)
from imu_python.definitions import FilterConfig
from imu_python.i2c_bus import ExtendedI2C
from imu_python.orientation_filter import OrientationFilter


class IMUWrapper:
    """Wrapper class for the IMU sensors."""

    def __init__(self, config: IMUConfig, i2c_bus: ExtendedI2C | None):
        """Initialize the wrapper.

        :param config: IMU configuration object.
        :param i2c_bus: i2c bus this device is connected to.
        """
        self.config: IMUConfig = config
        self.i2c_bus: ExtendedI2C | None = i2c_bus
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
            self.imu = imu_class(i2c=self.i2c_bus)
            self.started = True
        except Exception:
            raise

    def read_sensor(self, attr: str) -> VectorXYZ:
        """Read the IMU attributes."""
        data = getattr(self.imu, attr, None)

        if data is None:
            msg = f"IMU attribute {attr} not found."
            logger.warning(msg)
            raise AttributeError(msg)
        elif isinstance(data, float):
            raise TypeError(f"IMU attribute {attr} is a float.")
        else:
            return VectorXYZ.from_tuple(data)

    def get_data(self) -> IMUData:
        """Return acceleration and gyro information as an IMUData."""
        timestamp = time.monotonic()
        accel_vector = self.read_sensor(IMUSensorTypes.accel)
        gyro_vector = self.read_sensor(IMUSensorTypes.gyro)
        pose_quat = self.filter.update(
            timestamp=timestamp,
            accel=accel_vector.as_array(),
            gyro=gyro_vector.as_array(),
        )

        return IMUData(
            timestamp=timestamp,
            accel=accel_vector,
            gyro=gyro_vector,
            quat=pose_quat,
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
