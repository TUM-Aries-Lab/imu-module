"""Test the IMU data file reader."""

from pathlib import Path

import pytest

from imu_python.base_classes import (
    IMUData,
    IMUDataFile,
    IMURawData,
    Quaternion,
    VectorXYZ,
)
from imu_python.data_handler.data_reader import load_imu_data
from imu_python.data_handler.data_writer import IMUFileWriter
from imu_python.definitions import IMU_FILENAME_KEY, IMUDataFileColumns


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
                    raw_data=IMURawData(
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
                    raw_data=IMURawData(
                        accel=VectorXYZ(1.0, 2.0, 3.0),
                        gyro=VectorXYZ(4.0, 5.0, 6.0),
                        mag=None,
                    ),
                    quat=Quaternion(1.0, 0.0, 0.0, 0.0),
                ),
                IMUData(
                    timestamp=2.0,
                    raw_data=IMURawData(
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
def test_imu_writer(data, expected_rows):
    """Test data writer."""
    writer = IMUFileWriter()

    # Act
    writer.append_imu_data(data)
    df = writer.data_frame

    # Assert
    assert len(df) == expected_rows
    assert df.loc[0, IMUDataFileColumns.TIMESTAMP.value] == data[0].timestamp
