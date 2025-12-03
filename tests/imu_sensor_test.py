"""Test the mock IMU."""

import errno

import pytest

from imu_python.base_classes import VectorXYZ
from imu_python.imu_mock import FakeIMU


def test_initialize_sets_started_flag():
    """Test initialization started flag."""
    imu = FakeIMU()
    assert imu.started is False

    imu.initialize()
    assert imu.started is True


def test_random_acceleration_changes():
    """Test randomization of acceleration between readings."""
    imu = FakeIMU()
    imu.initialize()

    v1 = imu.acceleration()
    v2 = imu.acceleration()

    assert isinstance(v1, VectorXYZ)
    assert isinstance(v2, VectorXYZ)
    # Values should be different most of the time
    assert (v1.x, v1.y, v1.z) != (v2.x, v2.y, v2.z)


def test_random_gyro_changes():
    """Test randomization of gyro between readings."""
    imu = FakeIMU()
    imu.initialize()

    v1 = imu.gyro()
    v2 = imu.gyro()

    assert isinstance(v1, VectorXYZ)
    assert isinstance(v2, VectorXYZ)
    assert (v1.x, v1.y, v1.z) != (v2.x, v2.y, v2.z)


def test_random_magnetic_changes():
    """Test randomization of magnetic between readings."""
    imu = FakeIMU()
    imu.initialize()

    v1 = imu.magnetic()
    v2 = imu.magnetic()

    assert isinstance(v1, VectorXYZ)
    assert isinstance(v2, VectorXYZ)
    assert (v1.x, v1.y, v1.z) != (v2.x, v2.y, v2.z)


def test_disconnect_raises_error():
    """Test simulation of disconnection raising error."""
    imu = FakeIMU()
    imu.initialize()

    imu.disconnect()

    # All three reading functions should raise the I/O error
    with pytest.raises(OSError) as excinfo:
        imu.acceleration()

    assert excinfo.value.errno in (errno.EIO, 121)


def test_reconnect_restores_functionality():
    """Test simulation of reconnection restores functionality."""
    imu = FakeIMU()
    imu.initialize()

    imu.disconnect()

    # Confirm that disconnected IMU raises
    with pytest.raises(OSError):
        imu.gyro()

    imu.reconnect()
    assert imu.started is True

    # Should now return valid values again
    vec = imu.acceleration()
    assert isinstance(vec, VectorXYZ)
