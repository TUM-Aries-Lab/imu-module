"""IMU data classes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from loguru import logger
from numpy.typing import NDArray


@dataclass
class VectorXYZ:
    """Represent a 3D vector."""

    x: float | NDArray
    y: float | NDArray
    z: float | NDArray

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


@dataclass(frozen=True)
class IMUData:
    """Represent parsed IMU sensor data."""

    timestamp: float
    accel: VectorXYZ
    gyro: VectorXYZ
    mag: VectorXYZ | None = None


class IMUType(Enum):
    """Enum of IMU sensor types."""

    BNO055 = 1
    ST9DOF = 2
    MOCK = 3
