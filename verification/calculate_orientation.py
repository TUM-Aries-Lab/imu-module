import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from imu_python.data_handler.data_reader import load_imu_data
from imu_python.base_classes import Quaternion
from imu_python.definitions import FilterConfig
from imu_python.orientation_filters import MadgwickFilterPyImu, MadgwickFilterAHRS

def calculate_orientation(filepath: str | Path, gain: float, trim: float = 0.0) -> tuple[NDArray, list[Quaternion]]:
    """Calculate orientation from file using orientation filter in imu_python.

    Timestamps (and the corresponding sensor samples) before the `trim`
    threshold are discarded.
    """
    imu_data_raw = load_imu_data(filepath=Path(filepath))

    time = np.asarray(imu_data_raw.time)
    trim_mask = time >= trim
    time = time[trim_mask]

    accels = [a for i, a in enumerate(imu_data_raw.accels) if trim_mask[i]]
    gyros = [g for i, g in enumerate(imu_data_raw.gyros) if trim_mask[i]]
    mags = [m for i, m in enumerate(imu_data_raw.mags) if trim_mask[i]]

    quats: list[Quaternion] = []
    filter = MadgwickFilterPyImu(config=FilterConfig(gain=gain, freq_hz=1))

    for i, timestamp in enumerate(time):
        accel = accels[i].as_array()
        gyro = gyros[i].as_array()
        mag = mags[i].as_array() if not math.isnan(mags[i].x) else None

        quats.append(filter.update(
            timestamp=timestamp,
            accel=accel,
            gyro=gyro,
            mag=mag,
        ))

    assert len(time) == len(quats)
    return time, quats
