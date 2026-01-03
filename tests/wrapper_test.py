"""Test the factory and manager for the imu sensor objects."""

import copy
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


@pytest.mark.parametrize(
    "reason, mutate_config",
    [
        ("invalid library", lambda c: setattr(c, "library", "Wrong Library")),
        ("invalid module class", lambda c: setattr(c, "module_class", "Wrong Module")),
        (
            "invalid pre-config attribute: attribute does not exist",
            lambda c: setattr(
                c,
                "pre_config",
                [PreConfigStep(name="wrong_attribute", args=(1.0,), step_type="set")],
            ),
        ),
        (
            "invalid pre-config argument: not resolvable",
            lambda c: setattr(
                c,
                "pre_config",
                [PreConfigStep(name="i2c", args=("Wrong.value",), step_type="set")],
            ),
        ),
        (
            "invalid pre-config function: no module path",
            lambda c: setattr(
                c,
                "pre_config",
                [PreConfigStep(name="wrong_function", args=(1.0,), step_type="call")],
            ),
        ),
        (
            "invalid pre-config function: function does not exist",
            lambda c: setattr(
                c,
                "pre_config",
                [
                    PreConfigStep(
                        name="time.wrong_function", args=(1.0,), step_type="call"
                    )
                ],
            ),
        ),
        (
            "invalid pre-config function: not callable",
            lambda c: setattr(
                c,
                "pre_config",
                [PreConfigStep(name="time.timezone", args=(1.0,), step_type="call")],
            ),
        ),
    ],
)
def test_imu_wrapper_reload_fails(reason, mutate_config):
    """Test if wrapper raises runtime error with bad IMU Configs."""
    config = copy.deepcopy(IMUDevices.MOCK.config)
    mutate_config(config)

    wrapper = IMUWrapper(config=config, i2c_bus=None)

    with pytest.raises(RuntimeError):
        wrapper.reload()


def test_pre_config() -> None:
    """Test if the IMU is pre-configured properly."""
    # Arrange
    config = IMUDevices.MOCK.config
    config.pre_config = [
        PreConfigStep(name="param", args=(1.0,), step_type="set"),
        PreConfigStep(name="gyro_range", args=("RANGE_125_DPS",), step_type="set"),
        PreConfigStep(name="enable_feature", args=("MOCK.FEATURE",), step_type="call"),
        PreConfigStep(
            name="another_feature", args=("ANOTHER_FEATURE",), step_type="call"
        ),
        PreConfigStep(
            name="func_with_2_kwargs",
            kwargs={"param1": 10, "param2": 0.5},
            step_type="call",
        ),
    ]

    wrapper = IMUWrapper(config=config, i2c_bus=None)
    imu = MagicMock()
    # Patch _import_module so that it returns a mock module
    mock_module = MagicMock()
    mock_module.MOCK = MagicMock()

    feature_1 = object()
    feature_2 = object()
    mock_module.RANGE_125_DPS = "RANGE_125_DPS"
    mock_module.MOCK.FEATURE = feature_1
    mock_module.ANOTHER_FEATURE = feature_2

    wrapper.imu = imu

    with patch.object(wrapper, "_import_module", return_value=mock_module):
        # Act
        wrapper._preconfigure_sensor()

    # Assert
    assert wrapper.imu.param == 1.0
    assert wrapper.imu.gyro_range == "RANGE_125_DPS"
    wrapper.imu.enable_feature.assert_called_once_with(mock_module.MOCK.FEATURE)
    wrapper.imu.another_feature.assert_called_once_with(mock_module.ANOTHER_FEATURE)
    wrapper.imu.func_with_2_kwargs.assert_called_once_with(
        param1=10,
        param2=0.5,
    )


def test_preconfig_time_sleep():
    """Test if time.sleep can be called in pre-configuration."""
    # Arrange
    config = IMUDevices.MOCK.config
    config.pre_config = [
        PreConfigStep(name="time.sleep", args=(0.25,), step_type="call"),
    ]

    wrapper = IMUWrapper(config=config, i2c_bus=None)

    fake_time = MagicMock()
    fake_time.sleep = MagicMock()

    # patch _import_module to return fake_time
    with patch.object(wrapper, "_import_module", return_value=fake_time):
        # Act
        wrapper._preconfigure_sensor()

    # Assert
    fake_time.sleep.assert_called_once_with(0.25)
