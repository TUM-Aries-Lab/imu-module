"""IMU data reading."""

from pathlib import Path

import pandas as pd
from loguru import logger

from imu_python.base_classes import IMUDataFile, Quaternion, VectorXYZ
from imu_python.definitions import IMUDataFileColumns


def load_imu_data(filepath: Path) -> IMUDataFile:
    """Load IMU data from CSV file."""
    logger.debug(f"Loading IMU data from '{filepath}'.")
    columns = [col.value for col in IMUDataFileColumns]
    data_frame = pd.read_csv(filepath, usecols=lambda c: c in columns)

    time = data_frame[IMUDataFileColumns.TIMESTAMP.value].to_numpy()

    accels = [
        VectorXYZ(x, y, z)
        for x, y, z in zip(
            data_frame[IMUDataFileColumns.ACCEL_X.value],
            data_frame[IMUDataFileColumns.ACCEL_Y.value],
            data_frame[IMUDataFileColumns.ACCEL_Z.value],
            strict=True,
        )
    ]

    gyros = [
        VectorXYZ(x, y, z)
        for x, y, z in zip(
            data_frame[IMUDataFileColumns.GYRO_X.value],
            data_frame[IMUDataFileColumns.GYRO_Y.value],
            data_frame[IMUDataFileColumns.GYRO_Z.value],
            strict=True,
        )
    ]

    mags = [
        VectorXYZ(x, y, z)
        for x, y, z in zip(
            data_frame[IMUDataFileColumns.MAG_X.value],
            data_frame[IMUDataFileColumns.MAG_Y.value],
            data_frame[IMUDataFileColumns.MAG_Z.value],
            strict=True,
        )
    ]

    quats = [
        Quaternion(w, x, y, z)
        for w, x, y, z in zip(
            data_frame[IMUDataFileColumns.POSE_W.value],
            data_frame[IMUDataFileColumns.POSE_X.value],
            data_frame[IMUDataFileColumns.POSE_Y.value],
            data_frame[IMUDataFileColumns.POSE_Z.value],
            strict=True,
        )
    ]

    return IMUDataFile(time=time, gyros=gyros, accels=accels, mags=mags, quats=quats)
