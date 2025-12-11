"""Test the JetsonBus class."""

import pytest

from src.imu_python.definitions import I2CBusID
from src.imu_python.i2c_bus import JetsonBus


def test_initialize() -> None:
    """Test if JetsonBus is initialized."""
    assert JetsonBus._initialized is False
    JetsonBus.get(bus_id=I2CBusID.right)
    assert JetsonBus._initialized is True


@pytest.mark.parametrize("bus_id", [None, -1])
def test_bus_get_none(bus_id) -> None:
    """Test output of JetsonBus class get() function."""
    assert JetsonBus.get(bus_id=bus_id) is None
