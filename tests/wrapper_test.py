"""Test the factory and manager for the imu sensor objects."""

import copy
from dataclasses import replace
from unittest.mock import MagicMock, patch

import pytest

from imu_python.base_classes import (
    IMUConfig,
    IMUSensorTypes,
    PreConfigStep,
    PreConfigStepType,
    VectorXYZ,
)
from src.imu_python.devices import IMUDevices
from src.imu_python.wrapper import IMUWrapper


def mutate_sensor(
    cfg: IMUConfig,
    device_id: str,
    **changes,
) -> IMUConfig:
    """Return a new IMUConfig with modified SensorConfig for device_id."""
    dev = cfg.devices[device_id]
    new_dev = replace(dev, **changes)

    new_devices = dict(cfg.devices)
    new_devices[device_id] = new_dev

    return replace(cfg, devices=new_devices)


def test_imu_wrapper() -> None:
    """Test the imu wrapper class."""
    # Arrange
    config = IMUDevices.MOCK.config

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()

    # Assert
    assert wrapper.started


def test_imu_wrapper_attr_with_no_role() -> None:
    """Test the imu wrapper class read attribute with no role."""
    # Arrange
    config = IMUDevices.MOCK.config

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()

    assert wrapper.read_sensor(IMUSensorTypes.mag) is None


def test_imu_wrapper_attr_with_no_device() -> None:
    """Test the imu wrapper class read attribute with no device."""
    # Arrange
    config = IMUDevices.MOCK.config
    config.roles.update({IMUSensorTypes.mag: "imu1"})

    # Act
    wrapper = IMUWrapper(config=config, i2c_bus=None)
    wrapper.reload()

    assert wrapper.read_sensor(IMUSensorTypes.mag) is None


@pytest.mark.parametrize(
    "reason, mutate_config",
    [
        (
            "invalid library",
            lambda c: mutate_sensor(c, "imu0", library="Wrong Library"),
        ),
        (
            "invalid module class",
            lambda c: mutate_sensor(c, "imu0", module_class="Wrong Module"),
        ),
        (
            "invalid pre-config attribute: attribute does not exist",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="wrong_attribute",
                        args=(1.0,),
                        step_type=PreConfigStepType.SET,
                    )
                ],
            ),
        ),
        (
            "invalid pre-config argument: not resolvable",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="i2c",
                        args=("Wrong.value",),
                        step_type=PreConfigStepType.SET,
                    )
                ],
            ),
        ),
        (
            "invalid pre-config argument: not exactly one argument",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="i2c",
                        args=(
                            1.0,
                            2.0,
                        ),
                        step_type=PreConfigStepType.SET,
                    )
                ],
            ),
        ),
        (
            "invalid pre-config function: no module path",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="sleep", args=(1.0,), step_type=PreConfigStepType.CALL
                    )
                ],
            ),
        ),
        (
            "invalid pre-config function: python library does not exist",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="wrong_time.sleep",
                        args=(1.0,),
                        step_type=PreConfigStepType.CALL,
                    )
                ],
            ),
        ),
        (
            "invalid pre-config function: function does not exist",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="time.wrong_function",
                        args=(1.0,),
                        step_type=PreConfigStepType.CALL,
                    )
                ],
            ),
        ),
        (
            "invalid pre-config function: not callable from python library",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="time.timezone",
                        args=(1.0,),
                        step_type=PreConfigStepType.CALL,
                    )
                ],
            ),
        ),
        (
            "invalid pre-config function: not callable from module library",
            lambda c: mutate_sensor(
                c,
                "imu0",
                pre_config=[
                    PreConfigStep(
                        name="i2c", args=(1.0,), step_type=PreConfigStepType.CALL
                    )
                ],
            ),
        ),
    ],
)
def test_imu_wrapper_reload_fails(reason, mutate_config):
    """Test if wrapper raises runtime error with bad IMU Configs."""
    config = copy.deepcopy(IMUDevices.MOCK.config)
    config = mutate_config(config)

    wrapper = IMUWrapper(config=config, i2c_bus=None)

    with pytest.raises(RuntimeError):
        wrapper.reload()


def test_pre_config_with_mock() -> None:
    """Test if the IMU is pre-configured properly with mock."""
    # Arrange
    config = IMUDevices.MOCK.config
    config = mutate_sensor(
        cfg=config,
        device_id="imu0",
        pre_config=[
            PreConfigStep(name="param", args=(1.0,), step_type=PreConfigStepType.SET),
            PreConfigStep(
                name="gyro_range",
                args=("RANGE_125_DPS",),
                step_type=PreConfigStepType.SET,
            ),
            PreConfigStep(
                name="enable_feature",
                args=("MOCK.FEATURE",),
                step_type=PreConfigStepType.CALL,
            ),
            PreConfigStep(
                name="another_feature",
                args=("ANOTHER_FEATURE",),
                step_type=PreConfigStepType.CALL,
            ),
            PreConfigStep(
                name="func_with_2_kwargs",
                kwargs={"param1": 10, "param2": 0.5},
                step_type=PreConfigStepType.CALL,
            ),
        ],
    )

    wrapper = IMUWrapper(config=config, i2c_bus=None)
    imu = MagicMock()
    # Patch _import_module so that it returns a mock module
    mock_module = MagicMock()
    mock_module.MOCK = MagicMock()

    feature_1 = object()
    feature_2 = object()
    mock_module.RANGE_125_DPS = 125
    mock_module.input = MagicMock()
    mock_module.MOCK.FEATURE = feature_1
    mock_module.ANOTHER_FEATURE = feature_2

    wrapper._devices["imu0"] = imu

    with patch.object(wrapper, "_import_module", return_value=mock_module):
        # Act
        wrapper._preconfigure_sensor(sensor=imu, sensor_config=config.devices["imu0"])

    # Assert
    assert wrapper._devices["imu0"].param == 1.0
    assert wrapper._devices["imu0"].gyro_range == 125
    wrapper._devices["imu0"].enable_feature.assert_called_once_with(
        mock_module.MOCK.FEATURE
    )
    wrapper._devices["imu0"].another_feature.assert_called_once_with(
        mock_module.ANOTHER_FEATURE
    )
    wrapper._devices["imu0"].func_with_2_kwargs.assert_called_once_with(
        param1=10,
        param2=0.5,
    )


def test_pre_config_string():
    """Test if the IMU is pre-configured properly with a string argument."""
    # Arrange
    config = IMUDevices.MOCK.config
    config = mutate_sensor(
        cfg=config,
        device_id="imu0",
        pre_config=[
            PreConfigStep(
                name="i2c", args=("some_string",), step_type=PreConfigStepType.SET
            ),
        ],
    )

    wrapper = IMUWrapper(config=config, i2c_bus=None)

    # Act
    wrapper.reload()

    # Assert
    assert wrapper._devices["imu0"].i2c == "some_string"


def test_pre_config_time_sleep():
    """Test if time.sleep can be called in pre-configuration."""
    # Arrange
    config = IMUDevices.MOCK.config
    config = mutate_sensor(
        cfg=config,
        device_id="imu0",
        pre_config=[
            PreConfigStep(
                name="time.sleep", args=(0.25,), step_type=PreConfigStepType.CALL
            ),
        ],
    )

    wrapper = IMUWrapper(config=config, i2c_bus=None)

    fake_time = MagicMock()
    fake_time.sleep = MagicMock()

    import importlib

    def import_side_effect(module_path):
        if module_path == "time":
            return fake_time
        else:
            return importlib.import_module(module_path)

    # patch _import_module to return fake_time only for the 'time' module
    with patch.object(wrapper, "_import_module", side_effect=import_side_effect):
        wrapper.reload()

    # Assert
    fake_time.sleep.assert_called_once_with(0.25)


class BadIterable:
    """Iterable whose iterator raises TypeError when iterated."""

    def __iter__(self):
        """Return an iterator that raises TypeError."""
        raise TypeError("broken iterator")


@pytest.mark.parametrize(
    "value, expected",
    [
        (None, None),
        (5, None),
        ("1,2,3", None),
        (BadIterable(), None),
        ([], None),
        ((1.0, None, 3.0), None),
        ((1.0, "a", 3.0), None),
        ((1.0, 2.0), None),
        ((1.0, 2.0, 3.0, 4.0), None),
        ((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)),
    ],
)
def test_vectorize_parametrized(value, expected) -> None:
    """Parametrized tests covering all None-return cases and a positive case."""
    v = IMUWrapper._vectorize(value)
    if expected is None:
        assert v is None
    else:
        assert isinstance(v, VectorXYZ)
        assert (v.x, v.y, v.z) == expected
