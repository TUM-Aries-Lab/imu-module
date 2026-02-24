"""IMU gain calculation."""

import argparse
import math
from pathlib import Path

import numpy as np
from loguru import logger

from imu_python.data_handler.data_reader import load_imu_data


def calculate_gain(filepath: Path) -> float:
    """Calculate the gyro gain value based on recorded IMU data.

    :param filepath: Path and name of the IMU data file.
    :return: Calculated gain value.
    """
    data = load_imu_data(filepath=filepath).gyros
    xs = np.fromiter((v.x for v in data), dtype=float)
    ys = np.fromiter((v.y for v in data), dtype=float)
    zs = np.fromiter((v.z for v in data), dtype=float)

    std_x = xs.std()
    std_y = ys.std()
    std_z = zs.std()

    return math.sqrt(3 / 4) * math.sqrt(std_x**2 + std_y**2 + std_z**2)


def main() -> None:  # pragma: no cover
    """Run calculation of gain."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", "-f", type=Path, required=True)
    args = parser.parse_args()

    logger.info(calculate_gain(filepath=args.filepath))


if __name__ == "__main__":  # pragma: no cover
    main()
