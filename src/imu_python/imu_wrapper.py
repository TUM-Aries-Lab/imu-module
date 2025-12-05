"""Wrapper class for the IMUs."""

import importlib
import time

from .base_classes import IMUConfig, IMUData, VectorXYZ


class IMUWrapper:
    """Wrapper class for the IMU sensors."""

    config: IMUConfig
    imu = None
    started: bool = False
    i2c = None

    def __init__(self, config: IMUConfig, i2c_bus):
        self.config = config
        self.i2c = i2c_bus

    def initialize(self):
        """Initialize the sensor object."""
        # Dynamically import the IMU library
        module = IMUWrapper._import_imu_driver(self.config.library)

        # Instantiate the driver class
        imu_class = getattr(module, self.config.driver_class, None)
        if imu_class is None:
            raise RuntimeError(
                f"Module '{self.config.library}' has no class '{self.config.name}'"
            )
        self.imu = imu_class(self.i2c)
        self.started = True

    def acceleration(self) -> VectorXYZ:
        """BNO055 sensor's acceleration information as a VectorXYZ."""
        return VectorXYZ.from_tuple(tuple(self.imu.acceleration or (0.0, 0.0, 0.0)))  # type: ignore

    def gyro(self) -> VectorXYZ:
        """BNO055 sensor's gyro information as a VectorXYZ."""
        return VectorXYZ.from_tuple(tuple(self.imu.gyro or (0.0, 0.0, 0.0)))  # type: ignore

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
    def _import_imu_driver(library_path: str):
        """Dynamically import the IMU driver module.

        Example: "adafruit_bno055" -> <module 'adafruit_bno055'>
        """
        try:
            module = importlib.import_module(library_path)
            return module
        except ImportError as e:
            raise RuntimeError(f"Failed to import IMU driver '{library_path}'") from e
