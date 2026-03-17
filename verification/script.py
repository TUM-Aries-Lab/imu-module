import os
from pathlib import Path

import pandas as pd
from loguru import logger
from scipy.spatial.transform import Rotation, Slerp
import numpy as np
from calculate_orientation import calculate_orientation
from imu_python.base_classes import Quaternion
from imu_python.utils import setup_logger

def parse_mocap_csv(filepath: str | Path) -> pd.DataFrame:
    """Parse a Vicon-style mocap CSV:
    """
    df = pd.read_csv(
        filepath,
        names=["frame", "RX", "RY", "RZ","time"]
    )
    df = df.apply(pd.to_numeric, errors="coerce").dropna(subset=["time", "RX", "RY", "RZ"])
    df = df.reset_index(drop=True)
    return df


def parse_imu_csv(filepath: str | Path) -> pd.DataFrame:

    df = pd.read_csv(
        filepath,
        names=["ignore", "w", "x", "y","z","time"]
    )
    df = df.apply(pd.to_numeric, errors="coerce").dropna(subset=["time", "w", "x", "y", "z"])
    df = df.reset_index(drop=True)

    return df

def quats_to_rotations(quats: list[Quaternion]) -> Rotation:
    """Convert a DataFrame with columns w, x, y, z to a scipy Rotation object.

    Returns a Rotation instance containing one rotation per row in the DataFrame.
    """
    quats_array = np.array([[q.w, q.x, q.y, q.z] for q in quats])
    return Rotation.from_quat(quats_array, scalar_first=True)

def euler_to_rotations(df: pd.DataFrame) -> Rotation:
    """Convert a DataFrame with columns RX, RY, RZ to a scipy Rotation object.

    Returns a Rotation instance containing one rotation per row in the DataFrame.
    """
    eulers = df[["RX", "RY", "RZ"]].to_numpy()
    return Rotation.from_euler("xyz", eulers, degrees=True)

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

def process_imu_data(filepath: str | Path) -> tuple[np.ndarray, Rotation]:
    """Process IMU data from a CSV file and return timestamps and aligned rotations."""
    # imu_df = parse_imu_csv(filepath)
    # timestamps = imu_df["time"].to_numpy()
    timestamps, quats = calculate_orientation(filepath=filepath, gain=0.002250, trim=3800.022202)
    timestamps = timestamps - timestamps[0]  # normalize to start at 0
    imu_rotations = quats_to_rotations(quats=quats)
    imu_rotations_aligned = calculate_and_apply_rotation(imu_rotations)
    return timestamps, imu_rotations_aligned

def process_mocap_data(filepath: str | Path) -> tuple[np.ndarray, Rotation]:
    """Process mocap data from a CSV file and return timestamps and aligned rotations."""
    mocap_df = parse_mocap_csv(filepath)
    timestamps = mocap_df["time"].to_numpy()
    mocap_rotations = euler_to_rotations(mocap_df)
    mocap_rotations_aligned = calculate_and_apply_rotation(mocap_rotations)
    return timestamps, mocap_rotations_aligned

def interpolate_imu_to_mocap_grid(
    imu_timestamps: np.ndarray,
    imu_rotations: Rotation,
    mocap_timestamps: np.ndarray,
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
    valid_mask: np.ndarray = (mocap_timestamps >= imu_timestamps[0]) & (
        mocap_timestamps <= imu_timestamps[-1]
    )

    valid_mocap_timestamps = mocap_timestamps[valid_mask]
    imu_interpolated = slerp(valid_mocap_timestamps)

    return imu_interpolated, valid_mask


if __name__ == "__main__":
    setup_logger(log_level="INFO")
    root  = Path(__file__).resolve().parents[1]
    mocap_timestamps, mocap_rotations = process_mocap_data(root / "data/mocap/trimmed/bno055_01_mocap.csv")
    imu_timestamps, imu_rotations = process_imu_data(root / "data/test_recordings/imu_data_BNO055_0_7_nomag.csv")
    imu_interpolated, valid_mask = interpolate_imu_to_mocap_grid(
        imu_timestamps, imu_rotations, mocap_timestamps
    )
    # now IMU timestamp can be ignored
    from plot import plot_rotation_comparison
    output_dir = "./trials"
    os.makedirs(output_dir, exist_ok=True)
    fig = plot_rotation_comparison(
        imu_rotations=imu_interpolated,
        mocap_rotations=mocap_rotations[valid_mask],
        mocap_timestamps=mocap_timestamps[valid_mask],
        output_path=output_dir
    )
