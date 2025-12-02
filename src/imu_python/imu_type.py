"""Types of IMU sensors."""

from enum import Enum


class IMUType(Enum):
    """Enum of IMU sensor types."""

    BNO055 = 1
    MOCK = 2
