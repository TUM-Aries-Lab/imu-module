"""Test the JetsonBus class."""

from src.imu_python.definitions import I2CBusID
from src.imu_python.i2c_bus import JetsonBus


def test_initialize() -> None:
    """Test if JetsonBus is initialized."""
    assert JetsonBus._initialized is False
    JetsonBus.get(bus_id=I2CBusID.right)
    assert JetsonBus._initialized is True


def test_bus_get() -> None:
    """Test output of JetsonBus class get() function."""
    assert JetsonBus.get(bus_id=None) is None
    assert JetsonBus.get(bus_id=-1) is None  # type: ignore[arg-type]
