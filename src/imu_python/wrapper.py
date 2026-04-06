"""Wrapper class for the IMUs."""

import importlib
import time
import types
from collections.abc import Callable, Iterable
from numbers import Number
from typing import Any

from loguru import logger
from numpy.typing import NDArray

from imu_python.base_classes import (
    AdafruitIMU,
    IMUConfig,
    IMUData,
    IMUDeviceData,
    IMUSensorTypes,
    Quaternion,
    SensorConfig,
    VectorXYZ,
)
from imu_python.calibration.mag_calibration import apply_mag_cal, load_calibration
from imu_python.definitions import (
    DEFAULT_HARD_IRON,
    DEFAULT_INV_SOFT_IRON,
    DEFAULT_ROTATION_MATRIX,
    I2CBusID,
    IMUDescriptor,
    IMUDeviceID,
    IMUNameFormat,
    PreConfigStepType,
)
from imu_python.i2c_bus import ExtendedI2C, I2CBusDescriptor
from imu_python.orientation_filters import (
    BaseIMUFilter,
    MadgwickFilterPyImu,
)


class IMUWrapper:
    """Wrapper class for the IMU sensors."""

    def __init__(
        self,
        config: IMUConfig,
        imu_descriptor: IMUDescriptor,
        i2c_bus_descriptor: I2CBusDescriptor,
        calibration_mode: bool = False,
    ) -> None:
        """Initialize the wrapper.

        :param config: IMU configuration object.
        :param imu_descriptor: IMUDescriptor containing IMU name and IMU index.
        :param i2c_bus_descriptor: i2c bus information this device is connected to.
        :param calibration_mode: Flag to ignore calibration requirement.
        """
        self.config: IMUConfig = config
        self.imu_descriptor = imu_descriptor
        self.i2c_bus_instance: ExtendedI2C | None = i2c_bus_descriptor.bus_instance
        self.i2c_bus_id: I2CBusID | None = i2c_bus_descriptor.bus_id
        self.started: bool = False
        self.rotation_matrix: NDArray = DEFAULT_ROTATION_MATRIX
        self._devices: dict[
            IMUDeviceID, AdafruitIMU
        ] = {}  # device ID to device instance
        self.role_to_device_map: dict[
            IMUSensorTypes, IMUDeviceID
        ] = {}  # sensor type to device id

        # map roles to devices for sensor reads
        for role, device_id in config.roles.items():
            self.role_to_device_map[role] = device_id

        if calibration_mode:
            logger.debug("Calibration mode - using original mag readings.")
            self.mag_calibration = (DEFAULT_HARD_IRON, DEFAULT_INV_SOFT_IRON)
        elif IMUSensorTypes.mag in self.role_to_device_map:
            name = IMUNameFormat(
                imu_descriptor=self.imu_descriptor,
                bus_id=self.i2c_bus_id,
            ).get_name()
            mag_calibration = load_calibration(sensor_name=name)
            if mag_calibration is None:
                logger.warning(
                    f"No magnetometer calibration found for {name}. Magnetometer readings will not be used."
                )
                self.role_to_device_map.pop(IMUSensorTypes.mag)
            else:
                self.mag_calibration: tuple[NDArray, NDArray] = mag_calibration
                logger.info(f"Loaded magnetometer calibration for {name}.")

        if IMUSensorTypes.quat in self.role_to_device_map:
            self.scalar_first = config.scalar_first
        else:
            self.filter: BaseIMUFilter = MadgwickFilterPyImu(
                config=self.config.filter_config
            )

    def reload(self) -> None:
        """(Re)Initialize the IMU object."""
        try:
            self._devices.clear()
            for device_id, sensor_config in self.config.devices.items():
                self._devices[device_id] = self._initialize_sensor(sensor_config)
            self.started = True
        except Exception:
            self.started = False
            raise

    def _initialize_sensor(self, sensor_config: SensorConfig) -> AdafruitIMU:
        """Initialize sensor and return the initialized sensor object."""
        module = self._import_module(sensor_config.library)
        imu_class = self._load_class(
            module=module, module_class=sensor_config.module_class
        )
        # use the parameter names defined in config
        kwargs = {
            sensor_config.param_names.i2c: self.i2c_bus_instance,
            sensor_config.param_names.address: sensor_config.addresses[
                0
            ],  # only one address should be present here
        }
        sensor = imu_class(**kwargs)
        self._preconfigure_sensor(sensor=sensor, sensor_config=sensor_config)
        return sensor

    def get_imu_data(self) -> IMUDeviceData | IMUData:
        """Return acceleration, gyro and magnetic information as an IMUData."""
        accel_vector = self.read_sensor(IMUSensorTypes.accel)
        gyro_vector = self.read_sensor(IMUSensorTypes.gyro)
        mag_vector = self.read_sensor(IMUSensorTypes.mag)
        quat = self.read_sensor(IMUSensorTypes.quat)
        if not isinstance(accel_vector, VectorXYZ):
            raise ValueError("Accel reading is invalid.")
        if not isinstance(gyro_vector, VectorXYZ):
            raise ValueError("Gyro reading is invalid.")
        accel_vector.rotate(self.rotation_matrix)
        gyro_vector.rotate(self.rotation_matrix)
        if isinstance(mag_vector, VectorXYZ):
            mag_vector = apply_mag_cal(
                mag_vector=mag_vector, mag_calibration=self.mag_calibration
            )
            mag_vector.rotate(self.rotation_matrix)
        elif isinstance(mag_vector, Quaternion):
            raise ValueError("Mag reading is invalid.")
        device_data = IMUDeviceData(
            accel=accel_vector,
            gyro=gyro_vector,
            mag=mag_vector,
        )
        if isinstance(quat, Quaternion):
            quat.rotate(self.rotation_matrix)
            return IMUData(
                timestamp=time.monotonic(),
                device_data=device_data,
                quat=quat,
            )
        else:
            return device_data

    def read_sensor(self, attr: IMUSensorTypes) -> VectorXYZ | Quaternion | None:
        """Read the IMU attribute and return it as a VectorXYZ.

        :param attr: attribute defined as IMUSensorType
        :return: VectorXYZ data or None if not valid or available.
        """
        device_id = self.role_to_device_map.get(attr)
        if not device_id:
            msg = f"IMU attribute {attr} has no role associated with it."
            logger.debug(msg)
            return None

        device = self._devices.get(device_id)
        if not device:
            msg = f"IMU attribute {attr} has no device associated with it."
            logger.debug(msg)
            return None

        value = getattr(device, attr, None)
        vector = self._vectorize(value)
        if vector is None:
            return None
        return vector

    def _vectorize(self, value: Any) -> VectorXYZ | Quaternion | None:
        """Vectorize sensor output into a VectorXYZ object.

        :param value: The raw sensor output value.
        :return: VectorXYZ or None if the value is invalid or unavailable.
        """
        if value is None:
            return None

        # Catch invalid iterables
        if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
            try:
                values = tuple(value)
            except TypeError:
                return None
            if (
                not values
                or any(v is None for v in values)
                or not all(isinstance(v, Number) for v in values)
            ):
                return None
            if len(values) == 4:
                if self.scalar_first:
                    quat = Quaternion(
                        w=values[0], x=values[1], y=values[2], z=values[3]
                    )
                else:
                    quat = Quaternion(
                        w=values[3], x=values[0], y=values[1], z=values[2]
                    )
                return quat
            if len(values) == 3:
                return VectorXYZ.from_tuple(values)

        return None

    @staticmethod
    def _import_module(module_path: str) -> types.ModuleType:
        """Dynamically import a Python module by path."""
        try:
            return importlib.import_module(module_path)
        except ImportError as err:
            raise RuntimeError(
                f"{err} - Failed to import module '{module_path}'."
            ) from err

    def _load_class(self, module: Any, module_class: str) -> type[AdafruitIMU]:
        """Load the IMU class inside the given Adafruit library."""
        imu_class = getattr(module, module_class, None)
        if imu_class is None:
            raise RuntimeError(f"Module '{module}' has no class '{module_class}'")
        return imu_class

    def _resolve_arg(
        self, arg: Any, sensor: AdafruitIMU, sensor_config: SensorConfig
    ) -> Any:
        """Resolve string-based symbolic constants.

        :param arg: argument to be resolved for pre-config function call or attribute assignment.
        :param sensor: the sensor object that arg might belong to.
        :param sensor_config: the sensor configuration that defines the location of arg.
        :return: the resolved arg.
        """
        # Return as-is for non string arg
        if not isinstance(arg, str):
            return arg

        # Retrieve module from either constants_module if defined or IMU library
        module_path = sensor_config.constants_module or sensor_config.library
        module = self._import_module(module_path=module_path)

        if "." not in arg:
            if hasattr(module, arg):
                return getattr(module, arg)
            if hasattr(sensor, arg):
                return getattr(sensor, arg)
            return arg

        # Iteratively retrieving and modules
        value = module
        for part in arg.split("."):
            try:
                # try to resolve as attribute of the current value
                value = getattr(value, part)
            except AttributeError as err:
                msg = (
                    f"Failed to resolve argument '{arg}' in module '{module.__name__}'"
                )
                logger.error(msg)
                raise RuntimeError(msg) from err
        # Fallback: string arg as-is
        return value

    def _resolve_func(self, func_name: str) -> Callable:
        """Resolve a function name from python library to a callable.

        :param func_name: the function name as a string, e.g., "time.sleep"
        """
        if not isinstance(func_name, str):
            raise RuntimeError("Function name must be a string.")
        if "." not in func_name:
            raise RuntimeError(
                f"Function name '{func_name}' must include module path, e.g., 'time.sleep'."
            )
        # Splitting module path
        root, *rest = func_name.split(".")
        # Assign part so that it's not possibly unbound
        part = rest[0]
        # Iteratively retrieving modules
        try:
            module = self._import_module(root)
            value = module
            for part in rest:
                value = getattr(value, part)
            func = value
        except AttributeError as err:
            msg = f"Failed to resolve attribute '{part}' in module '{root}'"
            logger.error(msg)
            raise RuntimeError(msg) from err

        if not callable(func):
            msg = f"Attribute '{func_name}' in module '{module}' is not a callable"
            logger.error(msg)
            raise RuntimeError(msg)
        return func

    def _preconfigure_sensor(
        self, sensor: AdafruitIMU, sensor_config: SensorConfig
    ) -> None:
        """Perform method calls and variable assignments listed in the IMUConfig.

        :param sensor: the sensor object to configure.
        :param sensor_config: the sensor configuration data.
        :return: None
        """
        for step in sensor_config.pre_config:
            # resolve all args
            resolved_args = [
                self._resolve_arg(a, sensor=sensor, sensor_config=sensor_config)
                for a in step.args
            ]
            resolved_kwargs = {
                k: self._resolve_arg(v, sensor=sensor, sensor_config=sensor_config)
                for k, v in step.kwargs.items()
            }

            if step.step_type == PreConfigStepType.CALL:
                try:
                    func = getattr(sensor, step.name)
                    if not callable(func):
                        raise RuntimeError(
                            f"'{step.name}' in module '{sensor_config.name}' is not a callable"
                        )
                except AttributeError:
                    # try to resolve function globally
                    func = self._resolve_func(step.name)
                # call the function with resolved args and kwargs
                func(*resolved_args, **resolved_kwargs)
                continue

            elif step.step_type == PreConfigStepType.SET:
                if len(resolved_args) != 1:
                    raise RuntimeError(
                        f"Set step '{step.name}' must have exactly 1 positional argument"
                    )
                if hasattr(sensor, step.name) is False:
                    raise RuntimeError(
                        f"Attribute '{step.name}' not found in module '{sensor_config.name}'"
                    )
                setattr(sensor, step.name, resolved_args[0])
