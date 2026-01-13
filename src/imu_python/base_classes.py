"""IMU data classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from loguru import logger
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as Rot

from imu_python.definitions import (
    ACCEL_GRAVITY_MSEC2,
    CLIP_MARGIN,
    FilterConfig,
    PreConfigStepType,
)


@dataclass
class VectorXYZ:
    """Represent a 3D vector."""

    x: float
    y: float
    z: float

    @classmethod
    def from_tuple(cls, values: tuple[float, float, float]) -> VectorXYZ:
        """Create a VectorXYZ from a 3-tuple."""
        if len(values) != 3:
            msg = f"Expected 3 floats, got {len(values)}"
            logger.error(msg)
            raise ValueError(msg)
        return cls(x=values[0], y=values[1], z=values[2])

    def as_array(self) -> NDArray:
        """Return the vector as a NumPy array with shape (3,)."""
        return np.array([self.x, self.y, self.z], dtype=float)

    def rotate(self, rotation_matrix: NDArray):
        """Rotate the vector using a 3x3 rotation matrix.

        :param rotation_matrix: A 3x3 rotation matrix.
        """
        logger.debug(f"Rotating {self}")
        if rotation_matrix.shape != (3, 3):
            msg = f"Expected 3x3 rotation matrix, got {rotation_matrix.shape}"
            logger.error(msg)
            raise ValueError(msg)

        new_vec = rotation_matrix @ self.as_array()
        self.x = new_vec[0]
        self.y = new_vec[1]
        self.z = new_vec[2]

    def __repr__(self) -> str:
        """Return a string representation of the object."""
        return f"VectorXYZ(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

    def is_clipped(
        self, sensor_range: float, sensor_type: str, margin: float = CLIP_MARGIN
    ) -> bool:
        """Check if any component is close to clipping the specified range.

        :param range: hardware full scale (e.g. 500 for ±500 dps)
        :param margin: fraction of range to consider as clipping threshold
        :param type: sensor type for logging purposes
        :return: True if any component is close to clipping
        """
        threshold = sensor_range * margin
        if (
            abs(self.x) >= threshold
            or abs(self.y) >= threshold
            or abs(self.z) >= threshold
        ):
            logger.warning(
                f"{sensor_type} reading {self} is close to clipping limit ±{range}"
            )
            return True
        return False


@dataclass
class Quaternion:
    """Represent a Quaternion (w, x, y, z)."""

    w: float
    x: float
    y: float
    z: float

    def __repr__(self) -> str:
        """Return a string representation of the object."""
        return f"Quaternion(w={self.w:.3f}, x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

    def to_euler(self, seq: str) -> VectorXYZ:
        """Convert the Quaternion to an Euler angle (x, y, z)."""
        rot = Rot.from_quat(quat=[self.x, self.y, self.z, self.w], scalar_first=False)
        euler = rot.as_euler(seq=seq, degrees=False)
        return VectorXYZ.from_tuple(euler)


@dataclass(frozen=True)
class IMURawData:
    """Represent raw sensor data."""

    accel: VectorXYZ
    gyro: VectorXYZ
    mag: VectorXYZ | None = None


@dataclass(frozen=True)
class IMUData:
    """Represent parsed IMU sensor data."""

    timestamp: float
    quat: Quaternion
    raw_data: IMURawData


@dataclass
class IMUConfig:
    """Configuration data for sensor models.

    Attributes:
        name: Name of the IMU.
        addresses: List of possible I2C addresses.
        library: Module import path for the driver.
        module_class: Name of the class inside the module.
        i2c_param: Name of the I2C parameter in the class constructor.
        constants_module: Location of the constants/enums (if any) for the PreconfigStep.
        filter_gain: Gain value for the IMU filter.
        pre_config: List of pre-configuration steps to initialize/calibrate the IMU.

    """

    name: str
    addresses: list[int]
    library: str
    module_class: str
    i2c_param: str
    accel_range_g: float
    gyro_range_dps: float
    constants_module: str | None = None
    filter_gain: float = FilterConfig.gain
    pre_config: list[PreConfigStep] = field(default_factory=list)


@dataclass
class PreConfigStep:
    """Class representing a single IMU pre-configuration step.

    Attributes:
        name: Name of the method or property to configure.
        args: Positional arguments to pass if it's a callable method.
        kwargs: Keyword arguments to pass if it's a callable method.
        step_type: PreConfigStepType.CALL for a method or .SET for a property assignment.

    """

    name: str
    args: tuple[Any, ...] = ()
    kwargs: dict[str, Any] = field(default_factory=dict)
    step_type: PreConfigStepType = PreConfigStepType.CALL


@dataclass
class IMUDataFile:
    """IMU data reading with Pandas."""

    time: np.ndarray
    gyros: list[VectorXYZ]
    accels: list[VectorXYZ]
    mags: list[VectorXYZ]
    quats: list[Quaternion]

    def __iter__(self):
        """Iterate row-by-row, yielding IMUData instances."""
        n = len(self.time)
        for i in range(n):
            imu_data = []
            data = IMUData(
                timestamp=float(self.time[i]),
                quat=self.quats[i],
                raw_data=IMURawData(
                    accel=self.accels[i],
                    gyro=self.gyros[i],
                    mag=self.mags[i],
                ),
            )
            imu_data.append(data)
            yield imu_data


class AdafruitIMU:
    """Interface for Adafruit IMU sensors."""

    def __init__(self, i2c=None):
        """Initialize the mock IMU.

        :param i2c: I2C interface.
        """
        self.i2c = i2c

    @property
    def acceleration(self) -> tuple[float, float, float]:
        """Get the acceleration vector."""
        x, y, z = np.random.normal(loc=0, scale=0.2, size=(3,))
        return x, y, z + ACCEL_GRAVITY_MSEC2

    @property
    def gyro(self) -> tuple[float, float, float]:
        """Get the gyro vector."""
        x, y, z = np.random.normal(loc=0, scale=0.1, size=(3,))
        return x, y, z


@dataclass
class IMUSensorTypes:
    """Represent IMU sensor types."""

    accel = "acceleration"
    gyro = "gyro"
    mag = "magnetic"  # TODO: not implemented
