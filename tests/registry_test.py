"""Tests for the device registry loading and overriding mechanism."""

from dataclasses import replace
from importlib.metadata import EntryPoint
from unittest.mock import MagicMock, patch

from imu_python.base_classes import IMUConfig
from imu_python.builtin_devices import MOCK
from imu_python.definitions import MOCK_NAME
from imu_python.registry import _load_registry


def test_builtin_devices_are_loaded():
    """Test that the built-in devices are loaded into the registry."""
    registry = _load_registry()
    assert len(registry) > 0
    for name, config in registry.items():
        assert isinstance(config, IMUConfig), f"{name} did not load as IMUConfig"


def test_known_builtin_present():
    """Test that known built-in devices are present in the registry."""
    registry = _load_registry()
    assert MOCK_NAME in registry


def test_override_replaces_builtin():
    """Test that device overrides replace built-in devices."""
    builtin_mock_config = MOCK
    modified = replace(builtin_mock_config, accel_range_g=16.0)

    mock_override = MagicMock(spec=EntryPoint)
    mock_override.name = MOCK_NAME
    mock_override.load.return_value = modified

    with patch("imu_python.registry.entry_points") as mock_ep:
        mock_ep.side_effect = lambda group: (
            []
            if group == "imu_module.devices"
            else [mock_override]
            if group == "imu_module.device_overrides"
            else []
        )
        registry = _load_registry()

    assert registry[MOCK_NAME].accel_range_g == 16.0
