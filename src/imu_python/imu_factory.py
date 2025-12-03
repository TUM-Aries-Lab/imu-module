"""Factory that creates IMU object from given IMU type."""

from .base_classes import IMUType
from .bno055_imu import BNO055IMU
from .imu_mock import FakeIMU
from .st9dof_imu import ST9DOFIMU


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def create(imu_type: IMUType, i2c):
        """Create IMU object from given IMU type."""
        if imu_type == IMUType.BNO055:
            return BNO055IMU(i2c)
        elif imu_type == IMUType.ST9DOF:
            return ST9DOFIMU(i2c)
        elif imu_type == IMUType.MOCK:
            return FakeIMU()
        else:
            raise ValueError(f"Unsupported IMU type: {imu_type}")
