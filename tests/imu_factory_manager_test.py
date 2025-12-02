"""Test the factory and manager for the imu sensor objects."""

import time

import pytest

from imu_python.base_classes import IMUData, IMUType
from imu_python.imu_factory import IMUFactory
from imu_python.imu_mock import FakeIMU
from imu_python.sensor_manager import SensorManager


def test_create_mock_imu():
    """Test creating a mock IMU."""
    imu = IMUFactory.create(imu_type=IMUType.MOCK, i2c=None)
    assert isinstance(imu, FakeIMU)


def test_unsupported_imu_type():
    """Factory should raise ValueError for unsupported sensor types."""
    with pytest.raises(ValueError):
        IMUFactory.create("UNKNOWN_TYPE", i2c=None)


def test_imu_manager_start_thread():
    """Test if imu manager starts thread."""
    imu = FakeIMU()
    manager = SensorManager(imu)

    manager.start()

    # initialization should have been called
    assert imu._started is True

    # thread should be running
    assert manager.thread is not None
    assert manager.thread.is_alive()

    manager.stop()


def test_manager_updates_latest_data():
    """The manager should update the latest IMU data."""
    imu = FakeIMU(accel=(1, 2, 3), mag=(4, 5, 6), gyro=(7, 8, 9))
    manager = SensorManager(imu)

    manager.start()

    # wait for the loop to run at least once
    time.sleep(1.2)

    data = manager.get_data()
    assert isinstance(data, IMUData)

    # Check converted values
    assert (data.accel.x, data.accel.y, data.accel.z) == (1, 2, 3)
    assert (data.mag.x, data.mag.y, data.mag.z) == (4, 5, 6)
    assert (data.gyro.x, data.gyro.y, data.gyro.z) == (7, 8, 9)

    manager.stop()


def test_get_data_thread_safe():
    """Test thread safety of the manager."""
    imu = FakeIMU(accel=(10, 20, 30))
    manager = SensorManager(imu)
    manager.start()

    # let thread generate data
    time.sleep(1.2)

    # get data twice ensuring lock doesn't break
    d1 = manager.get_data()
    d2 = manager.get_data()

    assert d1 is not None
    assert d2 is not None
    assert isinstance(d1, IMUData)
    assert isinstance(d2, IMUData)

    manager.stop()


def test_stop_stops_thread_cleanly():
    """Ensure the thread is not alive after manager stops."""
    imu = FakeIMU()
    manager = SensorManager(imu)

    manager.start()
    time.sleep(0.3)
    manager.stop()

    assert manager.running is False

    # thread should not be alive after stop
    assert manager.thread.is_alive() is False


def test_shutdown_called_if_exists(monkeypatch):
    """Ensures that stop() calls imu.shutdown() if present."""

    class ShutdownIMU(FakeIMU):
        def __init__(self):
            super().__init__()
            self.shutdown_called = False

        def shutdown(self):
            self.shutdown_called = True

    imu = ShutdownIMU()
    manager = SensorManager(imu)

    manager.start()
    time.sleep(0.2)
    manager.stop()

    assert imu.shutdown_called is True
