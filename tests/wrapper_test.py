"""Test the factory and manager for the imu sensor objects."""

from src.imu_python.devices import IMUConfig
from src.imu_python.wrapper import IMUWrapper


def test_mock_imu_wrapper() -> None:
    """Test the imu wrapper class."""
    cfg = IMUConfig(
        name="MOCK",
        addresses=[0x00],
        library="imu_python.mock_imu",
        driver_class="MockIMU",
    )

    wrapper = IMUWrapper(cfg, i2c_bus=None)
    wrapper.reload()

    imu = wrapper.imu
    data = imu.all

    assert data.accel.x == 0
    assert data.accel.y == 0
    assert data.accel.z == 0
