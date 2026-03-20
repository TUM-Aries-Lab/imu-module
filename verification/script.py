from dataclasses import dataclass
from enum import Enum
import os
from pathlib import Path

from numpy.typing import NDArray
import pandas as pd
from loguru import logger
from scipy.spatial.transform import Rotation, Slerp
import numpy as np
from calculate_orientation import calculate_orientation
from imu_python.base_classes import Quaternion
from imu_python.utils import setup_logger

def parse_mocap_csv(filepath: str | Path, trim: int) -> pd.DataFrame:
    """Parse a Vicon-style mocap CSV:
    """
    df = pd.read_csv(
        filepath,
        names=["Frame", "SubFrame", "RX", "RY", "RZ", "TX", "TY", "TZ"]
    )
    df = df.apply(pd.to_numeric, errors="coerce").dropna(subset=["Frame", "RX", "RY", "RZ"])
    df.rename(columns={'Frame': 'time'}, inplace=True)
    df = df.drop(df[df['time'] < trim].index)
    df['time'] = df['time'] / 200
    df['time'] = df['time'] - df['time'].iloc[0]

    df = df.drop(columns=["SubFrame", "TX", "TY", "TZ"])
    df = df.reset_index(drop=True)
    return df

def parse_matlab_csv(filepath: str | Path, trim: float) -> tuple[NDArray, list[Quaternion]]:
    df = pd.read_csv(
        filepath,
        names=["time", "w", "x", "y", "z"]
    )
    df = df.apply(pd.to_numeric, errors="coerce").dropna()
    df = df.drop(df[df['time'] < trim].index)
    df['time'] = df['time'] - df['time'].iloc[0]
    quats: list[Quaternion] = []
    for _index, row in df.iterrows():
        quaternion = Quaternion(
            w=float(row['w']),
            x=float(row['x']),
            y=float(row['y']),
            z=float(row['z'])
        )
        quats.append(quaternion)
    return np.asarray(df['time']), quats


def quats_to_rotations(quats: list[Quaternion]) -> Rotation:
    """Convert a DataFrame with columns w, x, y, z to a scipy Rotation object.

    Returns a Rotation instance containing one rotation per row in the DataFrame.
    """
    quats_array = np.array([[q.w, q.x, q.y, q.z] for q in quats])
    return Rotation.from_quat(quats_array, scalar_first=True)

def rotvec_to_rotations(df: pd.DataFrame) -> Rotation:
    """Convert a DataFrame with columns RX, RY, RZ to a scipy Rotation object.

    Returns a Rotation instance containing one rotation per row in the DataFrame.
    """
    eulers = df[["RX", "RY", "RZ"]].to_numpy()
    return Rotation.from_rotvec(eulers, degrees=True)

def calculate_and_apply_rotation(data: Rotation, first_rotation: Rotation | None = None) -> Rotation:
    """Extract rotation from the first data point and apply inverse rotation to all data points.

    :param data: A scipy Rotation object.
    :param first_rotation: The first rotation to use for alignment. If None, uses the first rotation in data.
    :return: A scipy Rotation object.
    """
    if first_rotation is None:
        first_rotation = data[0]
    inverse_first_rotation = first_rotation.inv()
    return inverse_first_rotation * data

def stretch_arduino(timestamps: NDArray) -> NDArray:
    return timestamps * 1.0046

def process_imu_data(filepath: str | Path, trim: float, gain: float, offset: float | None = None, is_arduino: bool = False) -> tuple[np.ndarray, Rotation]:
    """Process IMU data from a CSV file and return timestamps and aligned rotations."""
    if is_arduino:
        timestamps, quats = parse_matlab_csv(filepath=filepath, trim=trim)
        timestamps = stretch_arduino(timestamps=timestamps)
    else:
        timestamps, quats = calculate_orientation(filepath=filepath, gain=gain, trim=trim)
        timestamps = timestamps - timestamps[0]  # normalize to start at 0
    imu_rotations = quats_to_rotations(quats=quats)
    # first 0.5 seconds -> filter converge and stablize
    # 0.5 to 2 seconds: average rotation as reference
    mask = (timestamps > 0.5) & (timestamps < 2)
    reference_rotation = imu_rotations[mask].mean() # type: ignore
    logger.info(imu_rotations[0])
    logger.info(reference_rotation)
    if offset is not None:
        offset_rotation = Rotation.from_euler('z', offset, degrees=True)
        reference_rotation = reference_rotation * offset_rotation
    imu_rotations_aligned = calculate_and_apply_rotation(imu_rotations, first_rotation=reference_rotation)
    return timestamps, imu_rotations_aligned

def process_mocap_data(filepath: str | Path, trim: int) -> tuple[np.ndarray, Rotation]:
    """Process mocap data from a CSV file and return timestamps and aligned rotations."""
    mocap_df = parse_mocap_csv(filepath=filepath, trim=trim)
    timestamps = mocap_df["time"].to_numpy()
    mocap_rotations = rotvec_to_rotations(mocap_df)
    mocap_rotations_aligned = calculate_and_apply_rotation(mocap_rotations)
    return timestamps, mocap_rotations_aligned

def interpolate_imu_to_mocap_grid(
    imu_timestamps: np.ndarray,
    imu_rotations: Rotation,
    mocap_timestamps: np.ndarray,
    trim_mocap_end: float,
) -> tuple[Rotation, np.ndarray]:
    """
    Interpolate IMU rotations onto the mocap timestamp grid using SLERP.

    Parameters
    ----------
    imu_timestamps : np.ndarray
        1-D array of IMU timestamps in seconds, shape (N,). Must be sorted.
    imu_rotations : Rotation
        Scipy Rotation object containing N rotations (one per IMU timestamp).
    mocap_timestamps : np.ndarray
        1-D array of mocap timestamps in seconds, shape (M,). Must be sorted.
    mocap_rotations : Rotation
        Scipy Rotation object containing M rotations (one per mocap timestamp).
        Returned as-is — included so the caller can keep both outputs together.

    Returns
    -------
    imu_interpolated : Rotation
        IMU rotations resampled at every mocap timestamp, shape (M,).
    valid_mask : np.ndarray
        Boolean mask of shape (M,) that is True where a mocap timestamp falls
        within the IMU time range [imu_timestamps[0], imu_timestamps[-1]].
        Timestamps outside that range are excluded from interpolation to avoid
        extrapolation artefacts.
    """
    if not np.all(np.diff(imu_timestamps) > 0):
        raise ValueError("imu_timestamps must be strictly increasing.")
    if not np.all(np.diff(mocap_timestamps) > 0):
        raise ValueError("mocap_timestamps must be strictly increasing.")
    if len(imu_timestamps) != len(imu_rotations):
        raise ValueError(
            f"imu_timestamps length ({len(imu_timestamps)}) must match "
            f"imu_rotations length ({len(imu_rotations)})."
        )


    # Build a SLERP interpolator over the full IMU sequence.
    # Scipy's Slerp accepts a Rotation object directly — no manual
    # conversion to quaternions required.
    slerp = Slerp(imu_timestamps, imu_rotations)

    # Only interpolate at mocap timestamps that lie inside the IMU window.
    # Extrapolation (clamping) would silently introduce constant-rotation
    # artefacts at the edges, so we expose a mask instead.
    valid_mask: np.ndarray = (mocap_timestamps >= imu_timestamps[0] + 2.0) & (
        mocap_timestamps <= trim_mocap_end
    )

    valid_mocap_timestamps = mocap_timestamps[valid_mask]
    imu_interpolated = slerp(valid_mocap_timestamps)

    return imu_interpolated, valid_mask

root  = Path(__file__).resolve().parents[1]

@dataclass
class TrialConfig:
    mocap_filepath: str | Path
    imu_filepath: str | Path
    trim_mocap_start: int
    trim_mocap_end: float
    trim_imu: float
    gain: float
    offset: float | None = None
    is_arduino: bool = False

class Trials(Enum):
    BNO055_nomag = TrialConfig(
        mocap_filepath= root /"data/mocap/test rig run 03 bno055jetson 01.csv",
        imu_filepath= root / "data/test_recordings/imu_data_BNO055_0_7_nomag.csv",
        trim_mocap_start=880,
        trim_mocap_end=150.0,
        trim_imu=3798.022202,
        gain=0.002250,
    )
    BNO055_test01 = TrialConfig(
        mocap_filepath= root /"data/mocap/test rig run 03 bno055jetson 01.csv",
        imu_filepath= root / "data/test_recordings/imu_data_BNO055_0_7_test01mag.csv",
        trim_mocap_start=880,
        trim_mocap_end=175.0,
        trim_imu=3169.238798,
        gain=0.002250,
    )
    LSM = TrialConfig(
        mocap_filepath= root /"data/mocap/test rig run 06 lsmJetson.csv",
        imu_filepath= root / "data/test_recordings/imu_data_LSM6DSOX_LIS3MDL_1_7_test.csv",
        trim_mocap_start=5631,
        trim_mocap_end=180.0,
        trim_imu=479.105010643,
        gain=0.002250,
        offset = -2.0,
    )
    ARDUINO1 = TrialConfig(
        is_arduino=True,
        mocap_filepath= root / "data/mocap/test rig run 01 arduino01.csv",
        imu_filepath= root / "data/test_recordings/arduino.csv",
        trim_mocap_start=899,
        trim_mocap_end=160.0,
        gain=0.0,
        trim_imu=6.62,
    )


if __name__ == "__main__":
    setup_logger(log_level="INFO")
    ## Change trial here ##
    trial_config = Trials.ARDUINO1
    ## --------------------##
    trim = trial_config.value.trim_imu
    gain = trial_config.value.gain
    mocap_timestamps, mocap_rotations = process_mocap_data(filepath=trial_config.value.mocap_filepath,
                                                           trim=trial_config.value.trim_mocap_start)
    imu_timestamps, imu_rotations = process_imu_data(trial_config.value.imu_filepath,
                                                     trim=trim,
                                                     gain=gain,
                                                     offset=trial_config.value.offset,
                                                     is_arduino=trial_config.value.is_arduino)
    imu_interpolated, valid_mask = interpolate_imu_to_mocap_grid(
        imu_timestamps=imu_timestamps,
        imu_rotations=imu_rotations,
        mocap_timestamps=mocap_timestamps,
        trim_mocap_end=trial_config.value.trim_mocap_end,
    )

    # now IMU timestamp can be ignored
    from plot import plot_rotation_comparison
    output_dir = "./trials"
    os.makedirs(output_dir, exist_ok=True)
    fig = plot_rotation_comparison(
        imu_rotations=imu_interpolated,
        mocap_rotations=mocap_rotations[valid_mask], # type: ignore
        mocap_timestamps=mocap_timestamps[valid_mask],
        output_path=output_dir,
        name = trial_config.name
    )
