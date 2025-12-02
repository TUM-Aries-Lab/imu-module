"""Data class that contains IMU reading information."""

from dataclasses import dataclass


@dataclass
class IMUData:
    """Acceleration, magnetic and gyro information as tuples."""

    acceleration: tuple[float, float, float]
    magnetic: tuple[float, float, float]
    gyro: tuple[float, float, float]
