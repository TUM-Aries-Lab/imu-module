import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def plot_rotation_comparison(
    imu_rotations: Rotation,
    mocap_rotations: Rotation,
    mocap_timestamps: np.ndarray,
    output_path: str,
    name: str
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
    # imu_euler = np.degrees(imu_rotations.as_euler("xyz"))
    # mocap_euler = np.degrees(mocap_rotations.as_euler("zyx"))
    imu_rotvec = np.degrees(imu_rotations.as_rotvec())
    mocap_rotvec = np.degrees(mocap_rotations.as_rotvec())
    mocap_rotvec[:, [0, 2]] = mocap_rotvec[:, [2, 0]] # swap x and z axis of mocap
    error = imu_rotvec - mocap_rotvec
    rms = np.sqrt(np.mean(error**2, axis=0))

    axis_labels = ["X (Yaw)", "Y (Roll)", "Z (Pitch)"]
    t = mocap_timestamps

    fig, axes = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
    fig.suptitle("IMU vs Mocap Rotation Comparison")

    for i, label in enumerate(axis_labels):
        # Left column: signal comparison
        ax = axes[i, 0]
        ax.plot(t, mocap_rotvec[:, i], label="Mocap", linewidth=1)
        ax.plot(t, imu_rotvec[:, i], label="IMU", linewidth=1)
        ax.set_ylabel(f"{label} (°)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # Right column: error
        ax = axes[i, 1]
        ax.plot(t, error[:, i], color="red", linewidth=1)
        ax.axhline(0, color="black", linewidth=0.8)

        # Fit linear line to error (drift)
        coeffs = np.polyfit(t, error[:, i], 1)
        slope = coeffs[0]  # slope in degrees/second
        fit_line = np.polyval(coeffs, t)
        ax.plot(t, fit_line, color="darkred", linewidth=2, linestyle="--", label=f"Linear fit (slope: {slope:.4f}°/s)")

        ax.set_ylabel(f"{label} error (°)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)

        # Add RMS and drift text with semi-transparent background
        text_str = f"RMS: {rms[i]:.2f}°\nDrift: {slope:.4f}°/s"
        ax.text(0.95, 0.95, text_str, transform=ax.transAxes, ha='right', va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    axes[0, 0].set_title("Rotation vectors")
    axes[0, 1].set_title("Error (IMU − Mocap)")

    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.savefig(output_path+f"/{name}_rotation_comparison.png", dpi=300)
    plt.show()
