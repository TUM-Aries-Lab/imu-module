"""imu_to_angle.py
---------------
Extracts Euler angle traces from a list of IMU quaternions.

All three Euler axes (X, Y, Z) are returned so that the comparison pipeline
can find the axis pair that best matches the mocap, rather than each sensor
independently picking its own dominant axis (which can differ due to
coordinate-frame misalignment).

Usage:

    from imu_to_angle import imu_to_angle

    results = imu_to_angle(
        timestamps  = my_timestamps,    # list of time.monotonic() floats
        quaternions = my_quaternions,   # list of Quaternion(w, x, y, z) dataclass
        ref_window  = (6.0, 9.0),       # stationary window in 0-based seconds
        trim_end    = 95.0,             # cut off chaotic/unwanted end (seconds)
        output_dir  = "./results",
        label       = "imu_trial_01",
        euler_order = "xyz",
    )

    # results is a dict with keys:
    #   'time_s'     : np.ndarray          — 0-based seconds
    #   'eulers_deg' : np.ndarray (N, 3)   — zero-referenced Euler angles
    #   'euler_order': str                 — e.g. 'xyz'
    #   'region'     : list[str]           — 'ramp_up', 'clean', or 'chaotic'
    #
    # For convenience the old single-axis keys are also present:
    #   'angle_deg'  : np.ndarray  — angle on the highest-variance axis
    #   'axis'       : str         — which axis that was, e.g. 'X'
"""

import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from loguru import logger
from matplotlib import gridspec
from scipy.spatial.transform import Rotation

from imu_python.base_classes import Quaternion


# ── Conversion ─────────────────────────────────────────────────────────────────

def quaternions_to_euler_angles(quaternions: list[Quaternion],
                                euler_order: str = "xyz"
                                ) -> np.ndarray:
    """Convert a list of Quaternion(w,x,y,z) to a (N, 3) Euler-angle array.

    Args:
        quaternions:  List of Quaternion(w, x, y, z) instances
        euler_order:  Euler decomposition order (default 'xyz')

    Returns:
        eulers_deg:  Shape (N, 3) array of Euler angles in degrees,
                     columns ordered to match euler_order.
    """
    wxyz = np.array([[q.w, q.x, q.y, q.z] for q in quaternions])
    xyzw = wxyz[:, [1, 2, 3, 0]]           # scipy expects scalar-last

    R_all      = Rotation.from_quat(xyzw)
    eulers_deg = R_all.as_euler(euler_order, degrees=True)   # (N, 3)

    variances = eulers_deg.var(axis=0)
    logger.trace("Euler axis variances ({}):  {}",
                 euler_order.upper(),
                 "  ".join(f"{euler_order[i].upper()}={variances[i]:.2f}"
                           for i in range(3)))

    return eulers_deg


# ── Reference Zero ─────────────────────────────────────────────────────────────

def compute_reference_offsets(eulers: np.ndarray,
                               ref_start_idx: int,
                               ref_end_idx: int) -> np.ndarray:
    """Return per-axis mean angles over the stationary reference window.
    Subtracting these sets the rest position to (0, 0, 0).
    """
    offsets = eulers[ref_start_idx:ref_end_idx].mean(axis=0)
    logger.debug("Reference offsets (mean over window):  "
                 "X={:.4f}  Y={:.4f}  Z={:.4f} deg".format(*offsets))
    return offsets


# ── Region Labelling ───────────────────────────────────────────────────────────

def label_regions(time: np.ndarray,
                  ref_start: float,
                  trim_end: float) -> list[str]:
    regions = []
    for t in time:
        if t < ref_start:
            regions.append("ramp_up")
        elif t <= trim_end:
            regions.append("clean")
        else:
            regions.append("chaotic")
    return regions


# ── Output: CSV ────────────────────────────────────────────────────────────────

def save_csv(time: np.ndarray, eulers: np.ndarray,
             euler_order: str, regions: list[str], output_path: str):
    cols = {ax.upper(): eulers[:, i] for i, ax in enumerate(euler_order)}
    df = pd.DataFrame({"time_s": time, **cols, "region": regions})
    df.to_csv(output_path, index=False)
    logger.info(f"Saved angle CSV  -> {output_path}")


# ── Output: Plot ───────────────────────────────────────────────────────────────

def save_plot(time: np.ndarray, eulers: np.ndarray,
              euler_order: str,
              ref_start: float, ref_end: float,
              trim_end: float, sample_rate: float,
              label: str, output_path: str):
    """Plot all three Euler axes; highlight the highest-variance axis."""
    colours    = ["steelblue", "darkorange", "seagreen"]
    clean_mask = (time >= ref_end) & (time <= trim_end)
    t_clean    = time[clean_mask]

    variances  = eulers[clean_mask].var(axis=0)
    best_col   = int(np.argmax(variances))
    axis_label = euler_order[best_col].upper()

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(f"IMU Euler Angles vs Time  —  {label}  (order: {euler_order.upper()})",
                 fontsize=13, fontweight="bold")

    for i, ax in enumerate(axes):
        col_label = euler_order[i].upper()
        highlight = (i == best_col)

        ramp_mask  = time < ref_end
        chaos_mask = time > trim_end

        if np.any(ramp_mask):
            ax.plot(time[ramp_mask], eulers[ramp_mask, i],
                    color="lightskyblue", linewidth=0.8, alpha=0.7)

        ax.axvspan(ref_start, ref_end, color="mediumorchid", alpha=0.12)
        ax.axvline(ref_start, color="mediumorchid", linewidth=1.0, linestyle="--", alpha=0.7)
        ax.axvline(ref_end,   color="mediumorchid", linewidth=1.0, linestyle="--", alpha=0.7)

        lw = 1.4 if highlight else 0.9
        ax.plot(t_clean, eulers[clean_mask, i],
                color=colours[i], linewidth=lw,
                label=f"Axis {col_label}  (var={variances[i]:.1f} deg²)"
                      + ("  ← highest variance" if highlight else ""))

        if np.any(chaos_mask):
            ax.plot(time[chaos_mask], eulers[chaos_mask, i],
                    color="lightgrey", linewidth=0.8)

        ax.axhline(0, color="grey", linewidth=0.4, linestyle=":")
        ax.set_ylabel(f"{col_label} (deg)", fontsize=10)
        ax.legend(fontsize=9, loc="upper right")
        ax.grid(True, alpha=0.3)
        if highlight:
            ax.set_facecolor("#f7fff0")

    axes[-1].set_xlabel("Time (s)", fontsize=10)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    logger.info(f"Saved plot       -> {output_path}")
    plt.close()


# ── Main Function ──────────────────────────────────────────────────────────────

def imu_to_angle(
    timestamps:   list[float],
    quaternions:  list[Quaternion],
    ref_window:   tuple[float, float],
    trim_end:     float,
    output_dir:   str = ".",
    label:        str = "imu",
    euler_order:  str = "xyz",
) -> dict:
    """Extract Euler angle traces from IMU quaternion data.

    Returns a dict with keys:
        'time_s'      — 0-based seconds (np.ndarray)
        'eulers_deg'  — (N, 3) zero-referenced Euler angles (np.ndarray)
        'euler_order' — decomposition order string, e.g. 'xyz'
        'region'      — per-frame region labels (list[str])
        'angle_deg'   — single-axis convenience: highest-variance axis (np.ndarray)
        'axis'        — which axis that was, e.g. 'X'
    """
    if len(timestamps) != len(quaternions):
        raise ValueError(
            f"timestamps ({len(timestamps)}) and quaternions ({len(quaternions)}) "
            "must have the same length."
        )

    ref_start, ref_end = ref_window
    if ref_start >= ref_end:
        raise ValueError(f"ref_window start ({ref_start}) must be less than end ({ref_end})")
    if ref_end >= trim_end:
        raise ValueError(f"ref_window end ({ref_end}s) must be before trim_end ({trim_end}s)")

    t = np.array(timestamps, dtype=float)
    t = t - t[0]

    dt          = float(np.median(np.diff(t)))
    sample_rate = 1.0 / dt
    logger.info(f"[imu_to_angle] {label}")
    logger.info(f"  Frames: {len(t)}  |  Duration: {t[-1]:.2f}s  |  "
                f"Sample rate: ~{sample_rate:.1f} Hz  (median dt = {dt*1000:.2f} ms)")

    if ref_start < 0 or ref_end > t[-1]:
        raise ValueError(
            f"ref_window ({ref_start}-{ref_end}s) is outside recording duration "
            f"(0-{t[-1]:.2f}s)"
        )
    if trim_end > t[-1]:
        logger.warning(f"trim_end ({trim_end}s) is beyond recording end "
                       f"({t[-1]:.2f}s) — using full recording.")
        trim_end = t[-1]

    ref_start_idx = int(np.searchsorted(t, ref_start))
    ref_end_idx   = int(np.searchsorted(t, ref_end))
    logger.info(f"  Ref window: {ref_start}-{ref_end}s  "
                f"(frames {ref_start_idx}-{ref_end_idx}, "
                f"{ref_end_idx - ref_start_idx} frames)")

    logger.info("  Converting quaternions → Euler angles...")
    eulers_raw = quaternions_to_euler_angles(quaternions, euler_order)

    logger.info("  Zero-referencing all axes over reference window...")
    offsets    = compute_reference_offsets(eulers_raw, ref_start_idx, ref_end_idx)
    eulers     = eulers_raw - offsets

    # Convenience: single dominant-axis output (kept for backward compat)
    variances  = eulers[(t >= ref_end) & (t <= trim_end)].var(axis=0)
    best_col   = int(np.argmax(variances))
    axis_label = euler_order[best_col].upper()
    logger.info(f"  Highest-variance IMU axis: {axis_label}  "
                f"(var = {variances[best_col]:.2f} deg²)")

    regions = label_regions(t, ref_start, trim_end)

    os.makedirs(output_dir, exist_ok=True)
    csv_path  = os.path.join(output_dir, f"{label}_euler.csv")
    plot_path = os.path.join(output_dir, f"{label}_euler.png")

    save_csv(t, eulers, euler_order, regions, csv_path)
    save_plot(t, eulers, euler_order, ref_start, ref_end, trim_end,
              sample_rate, label, plot_path)

    logger.info("  Done.")

    return {
        "time_s":      t,
        "eulers_deg":  eulers,
        "euler_order": euler_order,
        "region":      regions,
        # backward-compat single-axis keys
        "angle_deg":   eulers[:, best_col],
        "axis":        axis_label,
    }
