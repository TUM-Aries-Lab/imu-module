"""Factory that creates IMU object from given IMU type."""

from .bno055_imu import BNO055IMU
from .imu_mock import FakeIMU
from .imu_type import IMUType


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def create(imu_type: IMUType, i2c):
        """Create IMU object from given IMU type."""
        if imu_type == IMUType.BNO055:
            return BNO055IMU(i2c)
        elif imu_type == IMUType.MOCK:
            return FakeIMU(i2c)
        else:
            raise ValueError(f"Unsupported IMU type: {imu_type}")
