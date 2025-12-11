"""Test the factory and manager for the imu sensor objects."""

import pytest

from imu_python.base_classes import (
    IMUSensorTypes,
)
from src.imu_python.devices import IMUDevices
from src.imu_python.wrapper import IMUWrapper


def test_imu_wrapper() -> None:
    """Test the imu wrapper class."""
    # Arrange
    config = IMUDevices.MOCK.config

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()

    # Assert
    assert wrapper.started


def test_imu_wrapper_bad_attr() -> None:
    """Test the imu wrapper class."""
    # Arrange
    config = IMUDevices.MOCK.config

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()

    with pytest.raises(AttributeError):
        wrapper.read_sensor(IMUSensorTypes.mag)
