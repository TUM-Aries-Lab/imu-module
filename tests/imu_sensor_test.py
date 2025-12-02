"""Test the mock IMU."""

import time

from imu_python.base_classes import IMUData, VectorXYZ
from imu_python.imu_mock import FakeIMU


def test_initialize_sets_started_flag():
    """Test initialization started flag."""
    imu = FakeIMU()
    assert imu._started is False

    imu.initialize()
    assert imu._started is True


def test_acceleration_returns_given_tuple():
    """Test acceleration from given tuple."""
    imu = FakeIMU(accel=(1.0, 2.0, 3.0))
    assert imu.acceleration() == (1.0, 2.0, 3.0)


def test_magnetic_returns_given_tuple():
    """Test magnetic from given tuple."""
    imu = FakeIMU(mag=(4.0, 5.0, 6.0))
    assert imu.magnetic() == (4.0, 5.0, 6.0)


def test_gyro_returns_given_tuple():
    """Test gyro from given tuple."""
    imu = FakeIMU(gyro=(7.0, 8.0, 9.0))
    assert imu.gyro() == (7.0, 8.0, 9.0)


def test_all_returns_valid_imu_data():
    """Test all imu data from given tuples."""
    accel = (1.0, 2.0, 3.0)
    mag = (0.1, 0.2, 0.3)
    gyro = (9.0, 8.0, 7.0)

    imu = FakeIMU(accel=accel, mag=mag, gyro=gyro)
    imu.initialize()

    data = imu.all()

    # Check type
    assert isinstance(data, IMUData)

    # Timestamp should be close to "now"
    assert abs(time.time() - data.timestamp) < 0.1

    # Check accelerometer is converted correctly
    assert isinstance(data.accel, VectorXYZ)
    assert (data.accel.x, data.accel.y, data.accel.z) == accel

    # Check magnetometer
    assert isinstance(data.mag, VectorXYZ)
    assert (data.mag.x, data.mag.y, data.mag.z) == mag

    # Check gyro
    assert isinstance(data.gyro, VectorXYZ)
    assert (data.gyro.x, data.gyro.y, data.gyro.z) == gyro
