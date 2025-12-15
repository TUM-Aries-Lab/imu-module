"""IMU data classes."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from loguru import logger
from numpy.typing import NDArray

from imu_python.definitions import ACCEL_GRAVITY_MSEC2


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
        return cls(values[0], values[1], values[2])

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


@dataclass(frozen=True)
class IMUData:
    """Represent parsed IMU sensor data."""

    timestamp: float
    accel: VectorXYZ
    gyro: VectorXYZ
    pose: Quaternion
    mag: VectorXYZ | None = None


@dataclass
class IMUConfig:
    """Configuration data for sensor models."""

    name: str
    addresses: list[int]
    library: str
    module_class: str  # name of the class inside the module


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
                gyro=self.gyros[i],
                accel=self.accels[i],
                mag=self.mags[i],
                pose=self.quats[i],
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
