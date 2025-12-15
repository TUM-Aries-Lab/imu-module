"""IMU gain calculation."""

import math
from pathlib import Path

import numpy as np
from loguru import logger

from imu_python.data_handler.data_reader import load_imu_data
from imu_python.definitions import IMU_FILENAME_KEY, RECORDINGS_DIR


def calculate_gain(filepath: Path) -> float:
    """Calculate the gyro gain value based on recorded IMU data.

    :param filepath: Path and name of the IMU data file.
    """
    data = load_imu_data(filepath).gyros
    xs = np.fromiter((v.x for v in data), dtype=float)
    ys = np.fromiter((v.y for v in data), dtype=float)
    zs = np.fromiter((v.z for v in data), dtype=float)

    std_x = xs.std()
    std_y = ys.std()
    std_z = zs.std()

    return math.sqrt(3 / 4) * math.sqrt(std_x**2 + std_y**2 + std_z**2)


def main() -> None:
    """Run calculation of gain."""
    filepath = Path(RECORDINGS_DIR, f"{IMU_FILENAME_KEY}_bno055.csv")
    logger.info(calculate_gain(filepath=filepath))  # 0.011491877696095883


if __name__ == "__main__":  # pragma: no cover
    main()
