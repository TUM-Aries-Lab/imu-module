"""IMU data classes."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from loguru import logger
from numpy.typing import NDArray


@dataclass(frozen=True)
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

    def rotate(self, rotation_matrix: NDArray) -> VectorXYZ:
        """Rotate the vector using a 3x3 rotation matrix.

        :param rotation_matrix: A 3x3 rotation matrix.
        :return VectorXYZ: The rotated vector.
        """
        if rotation_matrix.shape != (3, 3):
            msg = f"Expected 3x3 rotation matrix, got {rotation_matrix.shape}"
            logger.error(msg)
            raise ValueError(msg)

        new_vec = rotation_matrix @ self.as_array()
        return VectorXYZ(new_vec[0], new_vec[1], new_vec[2])


@dataclass(frozen=True)
class IMUData:
    """Represent parsed IMU sensor data."""

    timestamp: float
    accel: VectorXYZ
    gyro: VectorXYZ
    mag: VectorXYZ | None = None
