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
    assert imu.started is True

    # thread should be running
    assert manager.thread is not None
    assert manager.thread.is_alive()

    manager.stop()


def test_manager_updates_latest_data():
    """The manager should update the latest IMU data."""
    imu = FakeIMU()
    manager = SensorManager(imu)

    manager.start()

    time.sleep(1.5)

    data1 = manager.get_data()
    time.sleep(1.5)
    data2 = manager.get_data()
    assert isinstance(data1, IMUData)
    assert isinstance(data2, IMUData)

    # Check converted values
    assert (data1.accel.x, data1.accel.y, data1.accel.z) != (
        data2.accel.x,
        data2.accel.y,
        data2.accel.z,
    )
    assert (data1.mag.x, data1.mag.y, data1.mag.z) != (
        data2.mag.x,
        data2.mag.y,
        data2.mag.z,
    )
    assert (data1.gyro.x, data1.gyro.y, data1.gyro.z) != (
        data2.gyro.x,
        data2.gyro.y,
        data2.gyro.z,
    )

    manager.stop()


def test_get_data_thread_safe():
    """Test thread safety of the manager."""
    imu = FakeIMU()
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


def test_manager_handles_disconnect_and_reconnect():
    """Test if manager correctly handles disconnection and reconnection of IMU."""
    """
    End-to-end test:
    - Start manager with FakeIMU
    - Confirm normal data stream
    - Simulate disconnect (OSError)
    - Manager should auto-reinitialize
    - Simulate reconnect
    - Data stream should resume
    """
    imu = FakeIMU()
    manager = SensorManager(imu)
    manager.start()

    # --- Step 1: ensure valid data is produced ---
    data1 = manager.get_data()
    assert data1 is not None, "Manager did not produce initial data"

    # Save this to compare later
    initial_accel = data1.accel

    # --- Step 2: simulate disconnection ---
    imu.disconnect()

    # Trigger a read that will throw (the thread does this naturally)
    # Allow the manager loop time to detect the failure
    time.sleep(2.5)

    # During disconnect, latest_data should not update
    data_after_disconnect = manager.latest_data
    assert data_after_disconnect is not None  # still old
    assert data_after_disconnect.accel.x == initial_accel.x

    # --- Step 3: simulate reconnect ---
    imu.reconnect()
    assert imu.started is True

    # Give manager time to reinitialize and resume reads
    time.sleep(2.5)

    # --- Step 4: verify the data stream has resumed with new values ---
    data2 = manager.get_data()
    assert data2 is not None
    assert data2.accel.x != initial_accel.x, "Data did not update after reconnect"

    # Cleanup
    manager.stop()
