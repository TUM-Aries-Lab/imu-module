"""Wrapper class for the IMUs."""

import importlib
import types

from loguru import logger

from imu_python.base_classes import (
    AdafruitIMU,
    IMUConfig,
    IMURawData,
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
            gain=self.config.filter_gain, frequency=FilterConfig.freq_hz
        )

    def reload(self) -> None:
        """Initialize the sensor object."""
        try:
            module = self._import_imu_module()
            imu_class = self._load_class(module=module)
            # use the parameter name defined in config
            kwargs = {self.config.i2c_param: self.i2c_bus}
            self.imu = imu_class(**kwargs)
            self._preconfigure_sensor()
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

    def get_data(self) -> IMURawData:
        """Return acceleration and gyro information as an IMUData."""
        accel_vector = self.read_sensor(IMUSensorTypes.accel)
        gyro_vector = self.read_sensor(IMUSensorTypes.gyro)
        return IMURawData(
            accel=accel_vector,
            gyro=gyro_vector,
        )

    def _import_imu_module(self) -> types.ModuleType:
        """Dynamically import the IMU driver module.

        Example: "adafruit_bno055" -> <module 'adafruit_bno055'>
        """
        return self._import_module(self.config.library)

    @staticmethod
    def _import_module(module_path: str) -> types.ModuleType:
        """Dynamically import a Python module by path."""
        try:
            return importlib.import_module(module_path)
        except ImportError as err:
            raise RuntimeError(
                f"{err} - Failed to import module '{module_path}'."
            ) from err

    def _load_class(self, module) -> type[AdafruitIMU]:
        imu_class = getattr(module, self.config.module_class, None)
        if imu_class is None:
            raise RuntimeError(
                f"Module '{module}' has no class '{self.config.module_class}'"
            )
        return imu_class

    def _resolve_arg(self, arg):
        """Resolve string-based symbolic constants."""
        if not isinstance(arg, str):
            return arg
        module_path = self.config.constants_module or self.config.library
        module = self._import_module(module_path=module_path)
        if "." in arg:
            value = module
            for part in arg.split("."):
                try:
                    value = getattr(value, part)
                except AttributeError as err:
                    raise RuntimeError(
                        f"Failed to resolve '{arg}' in module '{module_path}'"
                    ) from err
            return value
        if hasattr(module, arg):
            return getattr(module, arg)
        return arg

    def _preconfigure_sensor(self) -> None:
        for step in self.config.pre_config:
            # resolve all args
            resolved_args = [self._resolve_arg(a) for a in step.args]
            resolved_kwargs = {k: self._resolve_arg(v) for k, v in step.kwargs.items()}

            attr = getattr(self.imu, step.name)

            if step.step_type == "call":
                attr(*resolved_args, **resolved_kwargs)
            elif step.step_type == "set":
                setattr(self.imu, step.name, resolved_args[0])
