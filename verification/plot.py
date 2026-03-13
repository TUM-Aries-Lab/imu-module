import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.spatial.transform import Rotation


def plot_rotation_comparison(
    imu_timestamps: np.ndarray,
    imu_rotations: Rotation,
    mocap_timestamps: np.ndarray,
    mocap_rotations: Rotation,
    imu_interpolated: Rotation | None = None,
    valid_mask: np.ndarray | None = None,
    euler_seq: str = "xyz",
    degrees: bool = True,
    figsize: tuple[float, float] = (14, 10),
    title: str = "IMU vs MoCap Rotation Comparison",
) -> plt.Figure:
    """
    Plot all 3 Euler axes for IMU and MoCap rotations, plus per-axis errors.

    The function accepts two modes:
      1. Raw (unaligned) — pass raw imu_timestamps/imu_rotations and
         mocap_timestamps/mocap_rotations.  The IMU data is then interpolated
         internally onto the mocap grid before plotting.
      2. Pre-interpolated — pass the output of `interpolate_imu_to_mocap_grid`
         as `imu_interpolated` + `valid_mask` to skip recomputation.

    Parameters
    ----------
    imu_timestamps : np.ndarray        Raw IMU timestamps, shape (N,).
    imu_rotations  : Rotation          Raw IMU rotations, length N.
    mocap_timestamps : np.ndarray      MoCap timestamps, shape (M,).
    mocap_rotations  : Rotation        MoCap rotations, length M.
    imu_interpolated : Rotation | None Pre-interpolated IMU rotations on the
                                       mocap grid (length = valid_mask.sum()).
                                       If None, computed internally.
    valid_mask : np.ndarray | None     Boolean mask of shape (M,) indicating
                                       which mocap frames are inside the IMU
                                       time window.  If None, computed
                                       internally.
    euler_seq  : str                   Euler angle convention passed to
                                       Rotation.as_euler(), e.g. "xyz", "zyx".
    degrees    : bool                  If True, plot angles in degrees.
    figsize    : tuple                 Matplotlib figure size.
    title      : str                   Super-title for the figure.

    Returns
    -------
    fig : plt.Figure
        The completed matplotlib Figure (caller can save or further customise).
    """
    angle_unit = "°" if degrees else " rad"
    axis_labels = [f"{seq.upper()} {angle_unit}" for seq in euler_seq]

    # ------------------------------------------------------------------ #
    # 1.  Interpolate IMU onto mocap grid (if not pre-supplied)           #
    # ------------------------------------------------------------------ #
    if imu_interpolated is None or valid_mask is None:
        from scipy.spatial.transform import Slerp

        valid_mask = (mocap_timestamps >= imu_timestamps[0]) & (
            mocap_timestamps <= imu_timestamps[-1]
        )
        slerp = Slerp(imu_timestamps, imu_rotations)
        imu_interpolated = slerp(mocap_timestamps[valid_mask])

    t_common = mocap_timestamps[valid_mask]          # shared time axis
    mocap_trimmed = Rotation.from_matrix(
        mocap_rotations.as_matrix()[valid_mask]
    )

    # ------------------------------------------------------------------ #
    # 2.  Convert to Euler angles                                         #
    # ------------------------------------------------------------------ #
    imu_euler   = imu_interpolated.as_euler(euler_seq, degrees=degrees)   # (M, 3)
    mocap_euler = mocap_trimmed.as_euler(euler_seq, degrees=degrees)       # (M, 3)

    # Per-axis signed error and geodesic (axis-independent) error
    euler_error = imu_euler - mocap_euler                                  # (M, 3)

    # Geodesic (true angular) error via relative rotation
    rel_rot      = imu_interpolated.inv() * mocap_trimmed
    geodesic_err = np.degrees(rel_rot.magnitude()) if degrees else rel_rot.magnitude()

    # Raw IMU euler for background reference trace
    imu_euler_raw = imu_rotations.as_euler(euler_seq, degrees=degrees)     # (N, 3)

    # ------------------------------------------------------------------ #
    # 3.  Layout: 3 rotation rows + 1 error row + 1 geodesic row         #
    # ------------------------------------------------------------------ #
    fig = plt.figure(figsize=figsize, constrained_layout=True)
    fig.suptitle(title, fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(
        5, 3, figure=fig,
        height_ratios=[2, 2, 2, 1.5, 1.2],
        hspace=0.55, wspace=0.35,
    )

    COLOR_MOCAP = "#2196F3"   # blue
    COLOR_IMU   = "#F44336"   # red
    COLOR_RAW   = "#BDBDBD"   # light grey for raw uninterpolated IMU
    COLOR_ERR   = "#FF9800"   # orange
    COLOR_GEO   = "#9C27B0"   # purple

    rotation_axes = []

    # -- Rows 0-2: one column per Euler axis ----------------------------- #
    for col, (ax_name, ax_label) in enumerate(zip(euler_seq.upper(), axis_labels)):
        ax = fig.add_subplot(gs[col, :])          # full-width row per axis
        rotation_axes.append(ax)

        # Raw (uninterpolated) IMU as a faint reference
        ax.plot(
            imu_timestamps, imu_euler_raw[:, col],
            color=COLOR_RAW, lw=0.8, alpha=0.45,
            label="IMU (raw)", zorder=1,
        )
        # MoCap
        ax.plot(
            t_common, mocap_euler[:, col],
            color=COLOR_MOCAP, lw=1.6, label="MoCap", zorder=3,
        )
        # Interpolated IMU
        ax.plot(
            t_common, imu_euler[:, col],
            color=COLOR_IMU, lw=1.2, ls="--", label="IMU (interp.)", zorder=4,
        )

        ax.set_ylabel(ax_label, fontsize=9)
        ax.set_title(f"Axis {ax_name}", fontsize=10, pad=3)
        ax.grid(True, lw=0.4, alpha=0.5)
        ax.tick_params(labelsize=8)

        if col == 0:
            ax.legend(loc="upper right", fontsize=8, ncol=3,
                      framealpha=0.85, edgecolor="#cccccc")

    # -- Row 3: per-axis Euler error ------------------------------------- #
    ax_err = fig.add_subplot(gs[3, :])
    for col, ax_name in enumerate(euler_seq.upper()):
        ax_err.plot(
            t_common, euler_error[:, col],
            lw=1.2, label=f"Δ{ax_name}", alpha=0.85,
        )
    ax_err.axhline(0, color="black", lw=0.6, ls="--")
    ax_err.set_ylabel(f"Error {angle_unit}", fontsize=9)
    ax_err.set_title("Per-axis Euler error  (IMU − MoCap)", fontsize=10, pad=3)
    ax_err.legend(loc="upper right", fontsize=8, ncol=3,
                  framealpha=0.85, edgecolor="#cccccc")
    ax_err.grid(True, lw=0.4, alpha=0.5)
    ax_err.tick_params(labelsize=8)

    # -- Row 4: geodesic error + summary stats --------------------------- #
    ax_geo = fig.add_subplot(gs[4, :])
    ax_geo.fill_between(t_common, geodesic_err, alpha=0.25, color=COLOR_GEO)
    ax_geo.plot(t_common, geodesic_err, color=COLOR_GEO, lw=1.2,
                label="Geodesic error")
    ax_geo.set_ylabel(f"Error {angle_unit}", fontsize=9)
    ax_geo.set_xlabel("Time (s)", fontsize=9)
    ax_geo.set_title("Geodesic (true angular) error", fontsize=10, pad=3)
    ax_geo.grid(True, lw=0.4, alpha=0.5)
    ax_geo.tick_params(labelsize=8)

    # Annotate summary statistics in the geodesic panel
    stats_text = (
        f"mean={geodesic_err.mean():.2f}{angle_unit}  "
        f"std={geodesic_err.std():.2f}{angle_unit}  "
        f"max={geodesic_err.max():.2f}{angle_unit}  "
        f"RMSE={np.sqrt((geodesic_err**2).mean()):.2f}{angle_unit}"
    )
    ax_geo.text(
        0.01, 0.93, stats_text,
        transform=ax_geo.transAxes,
        fontsize=7.5, va="top", ha="left",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.8, ec="#cccccc"),
    )

    # Sync all x-axes
    x_min, x_max = t_common[0], t_common[-1]
    for ax in [*rotation_axes, ax_err, ax_geo]:
        ax.set_xlim(x_min, x_max)

    return fig
