"""Test the factory and manager for the imu sensor objects."""

from unittest.mock import MagicMock, patch

import pytest

from imu_python.base_classes import (
    IMUSensorTypes,
    PreConfigStep,
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


def test_pre_config() -> None:
    """Test if the IMU is pre-configured properly."""
    # Arrange
    config = IMUDevices.MOCK.config
    config.pre_config = [
        PreConfigStep(name="gyro_range", args=("RANGE_125_DPS",), step_type="set"),
        PreConfigStep(name="enable_feature", args=("MOCK.FEATURE",), step_type="call"),
        PreConfigStep(
            name="another_feature", args=("ANOTHER_FEATURE",), step_type="call"
        ),
    ]

    wrapper = IMUWrapper(config=config, i2c_bus=None)
    imu = MagicMock()
    # Patch _import_module so that it returns a mock module
    mock_module = MagicMock()
    mock_module.MOCK = MagicMock()

    mock_module.RANGE_125_DPS = "RANGE_125_DPS"
    mock_module.MOCK.FEATURE = "MOCK.FEATURE"
    mock_module.ANOTHER_FEATURE = "ANOTHER_FEATURE"

    wrapper.imu = imu

    with patch.object(wrapper, "_import_module", return_value=mock_module):
        # Act
        wrapper._preconfigure_sensor()

    # Assert
    assert wrapper.imu.gyro_range == "RANGE_125_DPS"
    wrapper.imu.enable_feature.assert_called_once_with("MOCK.FEATURE")
    wrapper.imu.another_feature.assert_called_once_with("ANOTHER_FEATURE")
