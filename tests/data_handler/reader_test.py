"""Test the IMU data file reader."""

from pathlib import Path

from imu_python.base_classes import (
    IMUDataFile,
)
from imu_python.data_handler.data_reader import load_imu_data
from imu_python.definitions import IMU_FILENAME_KEY


def test_load_imu_data() -> None:
    """Test the loading of IMU data."""
    filepath = Path("tests", "data", f"{IMU_FILENAME_KEY}_test.csv")
    imu_data_file = load_imu_data(filepath=filepath)
    assert isinstance(imu_data_file, IMUDataFile)
