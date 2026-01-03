"""Wrapper class for the IMUs."""

import importlib
import types
from collections.abc import Callable
from typing import Any

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
        """Load the IMU class inside the given Adafruit library."""
        imu_class = getattr(module, self.config.module_class, None)
        if imu_class is None:
            raise RuntimeError(
                f"Module '{module}' has no class '{self.config.module_class}'"
            )
        return imu_class

    def _resolve_arg(self, arg: Any) -> Any:
        """Resolve string-based symbolic constants."""
        if not isinstance(arg, str):
            return arg

        module_path = self.config.constants_module or self.config.library
        module = self._import_module(module_path=module_path)

        if "." not in arg:
            if hasattr(module, arg):
                return getattr(module, arg)
            return arg

        value = module
        for part in arg.split("."):
            try:
                # try to resolve as attribute of the current value
                value = getattr(value, part)
            except AttributeError as err:
                raise RuntimeError(
                    f"Failed to resolve attribute '{part}' in module '{module.__name__}'"
                ) from err

        return value

    def _resolve_func(self, func_name: str) -> Callable:
        """Resolve a function name to a callable."""
        if not isinstance(func_name, str):
            raise TypeError("Function name must be a string.")
        if "." not in func_name:
            raise RuntimeError(
                f"Function name '{func_name}' must include module path, e.g., 'time.sleep'."
            )
        root, *rest = func_name.split(".")
        part = rest[0]
        try:
            module = self._import_module(root)
            value = module
            for part in rest:
                value = getattr(value, part)
            func = value
        except RuntimeError as err:
            raise RuntimeError(f"Failed to import '{root}'") from err
        except AttributeError as err:
            raise RuntimeError(
                f"Failed to resolve attribute '{part}' in module '{root}'"
            ) from err

        if not callable(func):
            raise RuntimeError(
                f"Failed to resolve function '{func_name}' in module '{module}'"
            )
        return func

    def _preconfigure_sensor(self) -> None:
        """Perform method calls and variable assignments listed in the IMUConfig."""
        for step in self.config.pre_config:
            # resolve all args
            resolved_args = [self._resolve_arg(a) for a in step.args]
            resolved_kwargs = {k: self._resolve_arg(v) for k, v in step.kwargs.items()}

            if step.step_type == "call":
                try:
                    func = getattr(self.imu, step.name)
                    if not callable(func):
                        raise RuntimeError(
                            f"'{step.name}' in module '{self.config.name}' is not a callable"
                        )
                except AttributeError:
                    # try to resolve function globally
                    func = self._resolve_func(step.name)
                # call the function with resolved args and kwargs
                func(*resolved_args, **resolved_kwargs)
                continue

            elif step.step_type == "set":
                if len(resolved_args) != 1:
                    raise ValueError(
                        f"Set step '{step.name}' must have exactly 1 positional argument"
                    )
                try:
                    getattr(self.imu, step.name)
                    setattr(self.imu, step.name, resolved_args[0])
                except AttributeError as err:
                    raise RuntimeError(
                        f"Failed to set '{step.name}' in module '{self.config.name}'"
                    ) from err
