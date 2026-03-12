"""mocap_to_angle.py
-----------------
Converts a Vicon-style mocap CSV file into Euler angle vs time data.

All three Euler axes are returned so that the comparison pipeline can find
the axis pair that best matches the IMU, rather than each sensor independently
choosing its own dominant axis (which may differ due to coordinate-frame
misalignment between Vicon and IMU).

The mocap plot does NOT show a drift fit — optical mocap has no gyro
integration so there is no sensor drift by definition.
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from loguru import logger
from matplotlib import gridspec
from scipy.spatial.transform import Rotation


# ── CSV Parsing ────────────────────────────────────────────────────────────────

def parse_mocap_csv(filepath: str) -> tuple[pd.DataFrame, float]:
    """Parse a Vicon-style mocap CSV:
        Line 1: 'Segments'
        Line 2: sample rate (Hz)
        Line 3: ,,segment_name,,,,,,
        Line 4: Frame,Sub Frame,RX,RY,RZ,TX,TY,TZ
        Line 5: ,,deg,deg,deg,mm,mm,mm
        Line 6+: data rows
    Returns (dataframe, sample_rate_hz).
    """
    with open(filepath) as f:
        lines = f.readlines()

    try:
        sample_rate = float(lines[1].strip())
    except ValueError:
        logger.warning("Could not read sample rate from line 2, defaulting to 200 Hz.")
        sample_rate = 200.0

    header_row_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith("Frame"):
            header_row_idx = i
            break

    if header_row_idx is None:
        raise ValueError("Could not find 'Frame' header row in CSV.")

    df = pd.read_csv(
        filepath,
        skiprows=header_row_idx + 2,
        header=None,
        names=["Frame", "SubFrame", "RX", "RY", "RZ", "TX", "TY", "TZ"]
    )
    df = df.apply(pd.to_numeric, errors="coerce").dropna(subset=["Frame", "RX", "RY", "RZ"])
    df = df.reset_index(drop=True)

    return df, sample_rate


def euler_to_angles(df: pd.DataFrame, order: str,
                    input_format: str = "helical") -> np.ndarray:
    """Convert Vicon RX/RY/RZ columns to a (N, 3) Euler-angle array.

    Vicon's default "Global Angle" export format is HELICAL (rotation-vector /
    axis-angle), NOT XYZ Euler angles.  The three values are the components of
    the rotation vector r = θ·n̂ in degrees, where θ is the rotation magnitude
    and n̂ is the unit axis.  scipy expects radians for rotvec, so we convert.

    Steps:
      1. Read RX/RY/RZ as a rotation-vector (helical) in degrees
      2. Convert to radians and build Rotation objects via from_rotvec()
      3. Re-decompose as Euler angles with the requested order

    If your Vicon project is configured to export XYZ Euler angles instead of
    the default helical format, set MOCAP_INPUT_FORMAT = "euler_xyz" in config
    and the from_euler() path will be used.

    Zero-referencing is done separately by the caller.

    Returns:
        eulers_deg:  Shape (N, 3) array in degrees, columns matching `order`.
    """
    raw         = df[["RX", "RY", "RZ"]].values
    scipy_order = order.lower()

    if input_format == "helical":
        # Vicon default: rotation-vector (axis-angle) in degrees → radians for scipy
        R_all = Rotation.from_rotvec(np.deg2rad(raw))
    elif input_format == "euler_xyz":
        # Vicon project configured to export XYZ Euler angles directly
        R_all = Rotation.from_euler(scipy_order, raw, degrees=True)
    else:
        raise ValueError(f"Unknown MOCAP_INPUT_FORMAT {input_format!r}. "
                         f"Use 'helical' or 'euler_xyz'.")

    eulers_deg = R_all.as_euler(scipy_order, degrees=True)   # (N, 3)

    variances = eulers_deg.var(axis=0)
    logger.trace("  Euler axis variances ({}):  {}",
                 order,
                 "  ".join(f"{order[i].upper()}={variances[i]:.2f}" for i in range(3)))

    return eulers_deg


def compute_reference_offsets(eulers: np.ndarray, time: np.ndarray,
                               ref_start: float, ref_end: float) -> np.ndarray:
    """Return per-axis mean angles over the stationary reference window."""
    mask = (time >= ref_start) & (time <= ref_end)
    if not np.any(mask):
        raise ValueError(
            f"Reference window {ref_start}-{ref_end}s contains no data. "
            f"Check --ref-window values (recording is {time[-1]:.1f}s long)."
        )
    offsets   = eulers[mask][-1]
    return offsets


# Keep old single-value signature for any callers that haven't updated yet.
def compute_reference_angle(angles: np.ndarray, time: np.ndarray,
                             ref_start: float, ref_end: float) -> float:
    """Scalar version — computes the mean of a 1D angle array over the window."""
    mask = (time >= ref_start) & (time <= ref_end)
    if not np.any(mask):
        raise ValueError(
            f"Reference window {ref_start}-{ref_end}s contains no data."
        )
    return float(np.mean(angles[mask]))


# ── Output: CSV ────────────────────────────────────────────────────────────────

def save_angle_csv(time: np.ndarray, eulers: np.ndarray,
                   euler_order: str,
                   ref_end: float, trim_end_time: float,
                   output_path: str):
    def region(t: float) -> str:
        if t < ref_end:            return "ramp_up"
        elif t > trim_end_time:    return "chaotic"
        else:                      return "clean"

    cols = {ax.upper(): eulers[:, i] for i, ax in enumerate(euler_order)}
    out_df = pd.DataFrame({
        "time_s": time,
        **cols,
        "region": [region(t) for t in time],
    })
    out_df.to_csv(output_path, index=False)
    logger.info(f"Saved angle CSV  -> {output_path}")


# ── Output: Plot ───────────────────────────────────────────────────────────────

def plot_final_angle(time: np.ndarray, eulers: np.ndarray,
                     euler_order: str,
                     ref_start: float, ref_end: float,
                     trim_end_time: float,
                     sample_rate: float, output_path: str):
    """Plot all three Euler axes; highlight the highest-variance axis."""
    colours    = ["steelblue", "darkorange", "seagreen"]
    clean_mask = (time >= ref_start) & (time <= trim_end_time)
    t_clean    = time[clean_mask]

    variances  = eulers[clean_mask].var(axis=0)
    best_col   = int(np.argmax(variances))
    axis_label = euler_order[best_col].upper()

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(
        f"Mocap Euler Angles vs Time  —  order: {euler_order.upper()}",
        fontsize=13, fontweight="bold"
    )

    for i, ax in enumerate(axes):
        col_label = euler_order[i].upper()
        highlight = (i == best_col)

        ramp_mask  = time < ref_start
        chaos_mask = time > trim_end_time

        if np.any(ramp_mask):
            ax.plot(time[ramp_mask], eulers[ramp_mask, i],
                    color="lightskyblue", linewidth=0.8, alpha=0.7)

        ax.axvspan(ref_start, ref_end, color="mediumorchid", alpha=0.12,
                   label=f"Ref window ({ref_start}-{ref_end}s)")
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
            ax.axvline(trim_end_time, color="tomato", linewidth=1.0,
                       linestyle="--", alpha=0.7)

        ax.axhline(0, color="grey", linewidth=0.4, linestyle=":")
        ax.set_ylabel(f"{col_label} (deg)", fontsize=10)
        ax.legend(fontsize=9, loc="upper right")
        ax.grid(True, alpha=0.3)
        if highlight:
            ax.set_facecolor("#f7fff0")

    axes[-1].set_xlabel("Time (s)", fontsize=10)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    logger.info(f"Saved final plot -> {output_path}")
    plt.close()
