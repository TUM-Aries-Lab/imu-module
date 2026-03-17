from pathlib import Path

from numpy.typing import NDArray

from imu_python.data_handler.data_reader import load_imu_data
from imu_python.base_classes import Quaternion
from imu_python.orientation_filter import OrientationFilter

def calculate_orientation(filepath: str | Path, gain: float) -> tuple[NDArray, list[Quaternion]]:
    """Calculate orientation from file using orientation filter in imu_python."""
    imu_data_raw = load_imu_data(filepath=Path(filepath))
    time = imu_data_raw.time
    accels = imu_data_raw.accels
    gyros = imu_data_raw.gyros
    mags = imu_data_raw.mags
    quats: list[Quaternion] = []
    filter = OrientationFilter(gain=gain, frequency=0.01)
    for i in range(len(time)):
        timestamp = time[i]
        accel = accels[i].as_array()
        gyro = gyros[i].as_array()
        mag = mags[i].as_array()

        quats.append(filter.update(
            timestamp=timestamp,
            accel=accel,
            gyro=gyro,
            mag=mag,
        ))
    assert len(time) == len(quats)
    return time, quats
