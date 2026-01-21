"""Test the IMU data file reader."""

import math
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from imu_python.base_classes import (
    IMUData,
    IMUDataFile,
    IMUDeviceData,
    Quaternion,
    VectorXYZ,
)
from imu_python.data_handler.data_reader import load_imu_data
from imu_python.data_handler.data_writer import IMUFileWriter
from imu_python.data_handler.gain_calculator import calculate_gain
from imu_python.definitions import IMU_FILENAME_KEY, IMUDataFileColumns
from imu_python.devices import IMUDevices


def test_load_imu_data() -> None:
    """Test the loading of IMU data."""
    filepath = Path("tests", "data", f"{IMU_FILENAME_KEY}_test.csv")
    imu_data_file = load_imu_data(filepath=filepath)
    assert isinstance(imu_data_file, IMUDataFile)


@pytest.mark.parametrize(
    "data, expected_rows",
    [
        (
            [
                IMUData(
                    timestamp=1.0,
                    device_data=IMUDeviceData(
                        accel=VectorXYZ(1.0, 2.0, 3.0),
                        gyro=VectorXYZ(4.0, 5.0, 6.0),
                        mag=None,
                    ),
                    quat=Quaternion(1.0, 0.0, 0.0, 0.0),
                ),
            ],
            1,
        ),
        (
            [
                IMUData(
                    timestamp=1.0,
                    device_data=IMUDeviceData(
                        accel=VectorXYZ(1.0, 2.0, 3.0),
                        gyro=VectorXYZ(4.0, 5.0, 6.0),
                        mag=None,
                    ),
                    quat=Quaternion(1.0, 0.0, 0.0, 0.0),
                ),
                IMUData(
                    timestamp=2.0,
                    device_data=IMUDeviceData(
                        accel=VectorXYZ(7.0, 8.0, 9.0),
                        gyro=VectorXYZ(10.0, 11.0, 12.0),
                        mag=VectorXYZ(0.1, 0.2, 0.3),
                    ),
                    quat=Quaternion(0.0, 1.0, 0.0, 0.0),
                ),
            ],
            2,
        ),
    ],
)
def test_imu_writer(data, expected_rows) -> None:
    """Test data writer."""
    writer = IMUFileWriter()

    # Act
    writer.append_imu_data(data=data)
    writer.append_imu_data(data=None)
    input_df = writer.data_frame
    with tempfile.TemporaryDirectory() as tmpdir:
        file = writer.save_dataframe(
            imu_config=IMUDevices.MOCK.config, bus_id=None, output_dir=Path(tmpdir)
        )
        columns = [col.value for col in IMUDataFileColumns]
        read_df = pd.read_csv(file, usecols=lambda c: c in columns)

    def normalize_nulls(df: pd.DataFrame) -> pd.DataFrame:
        return df.where(df.notna(), np.nan)

    # Assert
    assert len(input_df) == expected_rows
    assert input_df.loc[0, IMUDataFileColumns.TIMESTAMP.value] == data[0].timestamp
    pd.testing.assert_frame_equal(
        normalize_nulls(input_df),
        normalize_nulls(read_df),
        check_dtype=False,  # CSV round-trip changes dtypes
        rtol=1e-9,
        atol=1e-12,
    )


def test_gain_calculator() -> None:
    """Test the gain calculator."""
    filepath = Path("tests", "data", f"{IMU_FILENAME_KEY}_test.csv")

    gain = calculate_gain(filepath=filepath)
    hand_calculated_gain = 0.01194646478  # Pre-calculated gain for this test data

    # Compare with a precalculated gain
    assert math.isclose(gain, hand_calculated_gain, abs_tol=0.00001)
