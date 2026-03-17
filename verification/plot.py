import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def plot_rotation_comparison(
    imu_rotations: Rotation,
    mocap_rotations: Rotation,
    mocap_timestamps: np.ndarray,
    output_path: str
) -> None:
    """
    Plot IMU vs mocap Euler angles (XYZ) and per-axis errors.

    Parameters
    ----------
    imu_rotations : Rotation
        IMU rotations interpolated onto the mocap grid, shape (N,).
    mocap_rotations : Rotation
        Mocap rotations, shape (N,).
    mocap_timestamps : np.ndarray
        Timestamps in seconds, shape (N,).
    """
    imu_euler = np.degrees(imu_rotations.as_euler("xyz"))
    mocap_euler = np.degrees(mocap_rotations.as_euler("zyx"))
    error = imu_euler - mocap_euler

    axis_labels = ["X (Roll)", "Y (Pitch)", "Z (Yaw)"]
    t = mocap_timestamps

    fig, axes = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
    fig.suptitle("IMU vs Mocap Rotation Comparison")

    for i, label in enumerate(axis_labels):
        # Left column: signal comparison
        ax = axes[i, 0]
        ax.plot(t, mocap_euler[:, i], label="Mocap", linewidth=1)
        ax.plot(t, imu_euler[:, i], label="IMU", linewidth=1)
        ax.set_ylabel(f"{label} (°)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # Right column: error
        ax = axes[i, 1]
        ax.plot(t, error[:, i], color="red", linewidth=1)
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_ylabel(f"{label} error (°)")
        ax.grid(True, alpha=0.3)

    axes[0, 0].set_title("Euler Angles")
    axes[0, 1].set_title("Error (IMU − Mocap)")

    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()
    plt.savefig(output_path+"/rotation_comparison.png", dpi=300)
