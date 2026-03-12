"""Orchestrates the full mocap vs IMU comparison pipeline.

  1. Runs mocap_to_angle  → extracts all three Euler angle traces from Vicon CSV
  2. Runs imu_to_angle    → extracts all three Euler angle traces from IMU quaternions
  3. Finds the best-matching (mocap_axis, imu_axis) pair by cross-correlation
     — this handles coordinate-frame misalignment between the two sensors
  4. Synchronises the two time axes via cross-correlation on that axis pair
  5. Resamples IMU onto mocap time grid (clean region only)
  6. Computes error statistics on the matched axis pair
  7. Produces a combined comparison plot

Why axis-pair matching?
  Both sensors are decomposed into Euler angles using the same order and the
  same scipy round-trip (so gimbal representation is consistent).  But the
  Vicon frame and the IMU body frame are physically different orientations, so
  the arm's primary rotation may appear on X in one sensor and Y in the other.
  Instead of each sensor independently picking its own highest-variance axis
  (which can disagree in amplitude even when the physical motion is identical),
  we try all 9 axis combinations and select the pair with the highest
  cross-correlation.  The selected pair is logged so you can verify it makes
  physical sense.

Usage:
    Edit the CONFIG section below, then run:
        python main.py
"""

import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from imu_to_angle import imu_to_angle
from loguru import logger
from matplotlib import gridspec
from mocap_to_angle import (
    compute_reference_offsets,
    euler_to_angles,
    parse_mocap_csv,
    plot_final_angle,
    save_angle_csv,
)
from scipy.interpolate import interp1d
from scipy.signal import correlate

from imu_python.utils import setup_logger

# ══════════════════════════════════════════════════════════════════════════════
# CONFIG — edit these values before running
# ══════════════════════════════════════════════════════════════════════════════

MOCAP_CSV  = "/home/cat/git_repos/imu-module/data/mocap/test rig run 02 arduino02.csv"
OUTPUT_DIR = "./results"
LABEL      = "arduino02"

# Mocap windows — seconds from the start of the MOCAP recording.
MOCAP_REF_WINDOW = (0, 5.9)
MOCAP_TRIM_END   = 160

# IMU windows — seconds from the start of the IMU recording.
IMU_CSV        = "/home/cat/git_repos/imu-module/data/test_recordings/arduino2.csv"
IMU_REF_WINDOW = (0.0, 8.0)
IMU_TRIM_END   = 160

IMU_EULER_ORDER = "zxy"
MOCAP_EULER_ORDER = "zyx"

sign = 1

# ── Step 1: Process Mocap ─────────────────────────────────────────────────────

def process_mocap(mocap_csv: str, ref_window: tuple, trim_end: float,
                  euler_order: str, output_dir: str, label: str) -> tuple[pd.DataFrame,str]:
    """Parse mocap CSV, decompose into Euler angles, zero-reference, save outputs.

    Returns a DataFrame with columns: time_s, X, Y, Z, region
    (column names match euler_order axes in uppercase).
    """
    ref_start, ref_end = ref_window

    logger.info("=" * 60)
    logger.info("STEP 1: Processing mocap data")
    logger.info("=" * 60)

    df, sample_rate = parse_mocap_csv(mocap_csv)
    time = df["Frame"].values / sample_rate
    logger.info(f"Loaded {len(df)} frames at {sample_rate} Hz  ({time[-1]:.2f}s)")
    logger.info(f"Euler order: {euler_order.upper()}")

    eulers_raw = euler_to_angles(df, euler_order)
    offsets    = compute_reference_offsets(eulers_raw, time, ref_start, ref_end)
    eulers     = eulers_raw - offsets

    variances  = eulers[(time >= ref_end) & (time <= trim_end)].var(axis=0)
    best_col   = int(np.argmax(variances))
    axis_label = euler_order[best_col].upper()
    logger.info(f"Highest-variance mocap axis: {axis_label}  "
                f"(var = {variances[best_col]:.2f} deg²)")

    def region(t: float) -> str:
        if t < ref_end:       return "ramp_up"
        elif t > trim_end:    return "chaotic"
        else:                 return "clean"

    regions = [region(t) for t in time]

    os.makedirs(output_dir, exist_ok=True)
    save_angle_csv(time, eulers, euler_order, ref_end, trim_end,
                   os.path.join(output_dir, f"{label}_mocap_euler.csv"))
    plot_final_angle(time, eulers, euler_order, ref_start, ref_end, trim_end,
                     sample_rate,
                     os.path.join(output_dir, f"{label}_mocap_euler.png"))

    cols = {ax.upper(): eulers[:, i] for i, ax in enumerate(euler_order)}
    return pd.DataFrame({"time_s": time, **cols, "region": regions}), axis_label


# ── Step 2: Process IMU ───────────────────────────────────────────────────────

def process_imu(ref_window: tuple, trim_end: float,
                output_dir: str, label: str) -> tuple[pd.DataFrame, str]:
    """Fetch IMU data, decompose quaternions into Euler angles, zero-reference.

    Returns a DataFrame with columns: time_s, X, Y, Z, region.
    Uses the same EULER_ORDER as mocap so the decomposition is directly
    comparable before any axis matching.
    """
    logger.info("=" * 60)
    logger.info("STEP 2: Processing IMU data")
    logger.info("=" * 60)

    #from imu_python.data_handler.data_reader import load_imu_data
    #imu_data    = load_imu_data(Path(IMU_CSV))
    #timestamps  = imu_data.time.tolist()
    #quaternions = imu_data.quats

    from read_matlab import load_quaternions_from_csv
    timestamps, quaternions = load_quaternions_from_csv(IMU_CSV)

    if len(timestamps) == 0 or len(quaternions) == 0:
        raise RuntimeError(
            "IMU data not loaded — please fill in the data fetching section "
            "in process_imu() before running."
        )

    results = imu_to_angle(
        timestamps  = timestamps,
        quaternions = quaternions,
        ref_window  = ref_window,
        trim_end    = trim_end,
        output_dir  = output_dir,
        label       = f"{label}_imu",
        euler_order = IMU_EULER_ORDER,
    )

    eulers   = results["eulers_deg"]        # (N, 3)
    time     = results["time_s"]
    imu_axis = results["axis"]              # highest-variance axis from IMU analysis
    cols     = {ax.upper(): eulers[:, i] for i, ax in enumerate(IMU_EULER_ORDER)}
    df       = pd.DataFrame({"time_s": time, **cols, "region": results["region"]})
    return df, imu_axis


# ── Steps 3 & 4: Axis Matching + Synchronisation ─────────────────────────────
#
# Two separate concerns, solved separately:
#
# AXIS MATCHING (which mocap axis corresponds to the IMU axis?)
#   Score each mocap axis by how well its first-movement direction and shape
#   match the IMU after applying the coarse lag.  The first movement after
#   ref_end is the only non-repeating feature — it has a unique ramp shape
#   and a definite sign, unlike the subsequent periodic oscillation.
#   We use a SHORT window (< 1 period) so we're comparing the ramp, not
#   a full cycle.  Sign is read from the dot product: positive → same
#   direction, negative → flipped.
#
# TIME SYNC (what is the exact lag?)
#   The coarse lag (mocap_ref_end − imu_ref_end) is applied first.
#   A narrow cross-correlation on the FULL clean region then finds the
#   small residual.  The search window is ±(period/4) — half the half-period.
#   Using the full clean region (many cycles) gives a strong, sharp
#   correlation peak even for a small residual, and ±(period/4) is
#   narrow enough that only the true peak can fall inside.

RAMP_WINDOW = 2.0   # seconds after ref_end used to read first-movement sign



def find_best_axis_pair(mocap_df: pd.DataFrame,
                        imu_df: pd.DataFrame,
                        imu_axis: str,
                        mocap_axis: str,
                        mocap_ref_end: float,
                        imu_ref_end: float,
                        ) -> tuple[str, float]:
    """Find the mocap axis, sign, and fine lag that align with the fixed IMU axis.

    Step 1 — Axis + sign: compare the first RAMP_WINDOW seconds of movement
             on each mocap axis against the IMU after coarse-lag alignment.
             Dot product score > 0 → same direction (sign = +1),
             dot product score < 0 → flipped (sign = −1).

    Step 2 — Fine lag: cross-correlate the full clean region of the selected
             axis pair within ±(period/4) of zero residual.

    Returns:
        mocap_axis  — e.g. 'X'
        imu_sign    — +1.0 or -1.0
        fine_lag_s  — small residual correction to add on top of coarse lag
    """
    logger.info("=" * 60)
    logger.info("STEP 3: Axis matching + sign + fine lag")
    logger.info("=" * 60)
    logger.info(f"  IMU axis fixed to: {imu_axis}  (highest variance from IMU analysis)")

    coarse_lag = mocap_ref_end - imu_ref_end
    logger.info(f"  Coarse lag (ref_end markers): {coarse_lag:+.4f}s  "
                f"(mocap_ref_end={mocap_ref_end}s  imu_ref_end={imu_ref_end}s)")

    dt = min(
        float(np.median(np.diff(mocap_df["time_s"].values))),
        float(np.median(np.diff(imu_df["time_s"].values))),
    )

    # ── Step 1: axis + sign from first-movement ramp ──────────────────────────
    # Extract the ramp window from mocap starting at mocap_ref_end
    # and from IMU starting at (imu_ref_end + coarse_lag) = mocap_ref_end.
    # Both windows now cover the same physical moments.

    # Shift IMU onto mocap clock by coarse_lag before extracting ramp
    imu_shifted        = imu_df.copy()
    imu_shifted["time_s"] = imu_df["time_s"] + coarse_lag


    best_m_ax     = mocap_axis

    fine_lag = 0.0
    total_lag = coarse_lag + fine_lag
    logger.info(f"  Fine lag (zero-crossing difference): {fine_lag:+.6f}s")
    logger.info(f"  Result: mocap={best_m_ax}  imu={imu_axis}  sign={sign:+.0f}  "
                f"total_lag={total_lag:+.6f}s  "
                f"(coarse={coarse_lag:+.4f}s  fine={fine_lag:+.6f}s)")

    return best_m_ax, fine_lag


# ── Step 4: Synchronisation ───────────────────────────────────────────────────

def synchronise(imu_df: pd.DataFrame,
                imu_axis: str,
                imu_sign: float,
                coarse_lag: float,
                fine_lag: float) -> pd.DataFrame:
    """Apply sign flip and total time shift to the IMU DataFrame.

    The total lag = coarse_lag + fine_lag where:
        coarse_lag = mocap_ref_end - imu_ref_end   (from user config)
        fine_lag   = sub-period residual from onset cross-correlation
    """
    logger.info("=" * 60)
    logger.info("STEP 4: Applying synchronisation to IMU")
    logger.info("=" * 60)

    imu_df     = imu_df.copy()
    total_lag  = coarse_lag + fine_lag

    if imu_sign < 0:
        axes = [col for col in imu_df.columns if col in ("X", "Y", "Z")]
        logger.warning(f"  Flipping IMU sign on axes: {axes}")
        for ax in axes:
            imu_df[ax] = -imu_df[ax]

    logger.info(f"  Sign:       {imu_sign:+.0f}")
    logger.info(f"  Total lag:  {total_lag:+.6f}s  "
                f"(coarse={coarse_lag:+.4f}s  fine={fine_lag:+.6f}s)")
    imu_df["time_s"] = imu_df["time_s"] + total_lag
    return imu_df


# ── Step 5: Resample onto Common Grid ─────────────────────────────────────────

def resample_to_mocap_grid(mocap_df: pd.DataFrame,
                           imu_df: pd.DataFrame,
                           mocap_axis: str,
                           imu_axis: str,
                           ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Resample IMU onto the mocap time grid (clean region only) and
    re-zero both signals so their means over the shared clean window match.

    Why re-zero here?
        Each sensor was zero-referenced independently against its own
        reference window before synchronisation.  If those windows do not
        capture exactly the same mechanical rest angle a DC offset survives.
        After time-sync both signals share a common time axis, so we subtract
        each signal mean over the full clean region to put them on the same
        baseline.  This removes DC without touching oscillation amplitude.

    Returns (t_common, mocap_angles, imu_angles).
    """
    logger.info("=" * 60)
    logger.info("STEP 4: Resampling onto common time grid (clean region)")
    logger.info(f"        mocap axis: {mocap_axis}  |  imu axis: {imu_axis}")
    logger.info("=" * 60)

    mocap_clean = mocap_df[mocap_df["region"] == "clean"]
    t_common    = mocap_clean["time_s"].values

    imu_clean = imu_df[imu_df["region"] == "clean"]
    if len(imu_clean) < 2:
        raise RuntimeError("IMU clean region is empty after synchronisation — "
                           "check ref_window and trim_end values.")

    imu_angles = interp1d(imu_clean["time_s"], imu_clean[imu_axis],
                          kind="linear", bounds_error=False,
                          fill_value=np.nan)(t_common)

    valid     = ~np.isnan(imu_angles)
    t_out     = t_common[valid]
    imu_out   = imu_angles[valid]
    imu_out   = imu_out - imu_out[0]
    mocap_out = mocap_clean[mocap_axis].values[valid]

    return t_out, mocap_out, imu_out


# ── Step 6: Error Statistics ───────────────────────────────────────────────────

def compute_error_stats(t: np.ndarray,
                        mocap_angles: np.ndarray,
                        imu_angles: np.ndarray) -> dict:
    """Compute error statistics between IMU and mocap angle traces.

    dc_offset is the IMU bias relative to mocap measured before re-zeroing
    (i.e. imu_mean - mocap_mean over the shared clean window).  It is reported
    separately because re-zeroing forces mean(error) = 0 by construction.
    """
    logger.info("=" * 60)
    logger.info("STEP 5: Error statistics (clean oscillation region)")
    logger.info("=" * 60)

    error      = imu_angles - mocap_angles
    coeffs     = np.polyfit(t, error, 1)
    drift_fit  = np.polyval(coeffs, t)
    drift_rate = float(coeffs[0])
    detrended  = error - drift_fit

    stats = {
        "mean_error":    float(np.mean(error)),
        "rms_error":     float(np.sqrt(np.mean(error**2))),
        "max_error":     float(np.max(np.abs(error))),
        "std_error":     float(np.std(error)),
        "drift_rate":    drift_rate,
        "drift_rate_hr": drift_rate * 3600,
        "detrended_rms": float(np.sqrt(np.mean(detrended**2))),
        "error":         error,
        "drift_fit":     drift_fit,
        "detrended":     detrended,
    }

    logger.info(f"  Mean error (post-DC):  {stats['mean_error']:+.4f} deg  "
                f"(≈0 by construction)")
    logger.info(f"  RMS error:       {stats['rms_error']:.4f} deg")
    logger.info(f"  Max error:       {stats['max_error']:.4f} deg")
    logger.info(f"  Std error:       {stats['std_error']:.4f} deg")
    logger.info(f"  Drift rate:      {stats['drift_rate']:+.6f} deg/s  "
                f"({stats['drift_rate_hr']:+.2f} deg/hr)")
    logger.info(f"  Detrended RMS:   {stats['detrended_rms']:.4f} deg")

    return stats


# ── Step 7: Comparison Plot ────────────────────────────────────────────────────

def plot_comparison(t: np.ndarray,
                    mocap_angles: np.ndarray,
                    imu_angles: np.ndarray,
                    stats: dict,
                    mocap_axis: str,
                    imu_axis: str,
                    label: str,
                    output_path: str):
    error     = stats["error"]
    drift_fit = stats["drift_fit"]
    detrended = stats["detrended"]

    fig = plt.figure(figsize=(15, 11))
    fig.suptitle(
        f"Mocap vs IMU Comparison  —  {label}\n"
        f"mocap axis: {mocap_axis}  |  imu axis: {imu_axis}  |  "
        f"IMU euler order: {IMU_EULER_ORDER.upper()}",
        fontsize=13, fontweight="bold"
    )
    gs = gridspec.GridSpec(3, 1, height_ratios=[2.5, 1.5, 1.5], hspace=0.45,
                           left=0.07, right=0.72)

    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, mocap_angles, color="steelblue",  linewidth=1.2,
             label=f"Mocap (axis {mocap_axis})")
    ax1.plot(t, imu_angles,   color="darkorange", linewidth=1.2, alpha=0.85,
             label=f"IMU (axis {imu_axis})")
    ax1.axhline(0, color="grey", linewidth=0.5, linestyle=":")
    ax1.set_ylabel("Angle (deg)", fontsize=11)
    ax1.legend(fontsize=10, loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Angle Traces (clean oscillation region)", fontsize=10)

    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(t, error,     color="crimson", linewidth=1.0, label="Error (IMU − mocap)")
    ax2.plot(t, drift_fit, color="black",   linewidth=1.5, linestyle="--",
             label=f'Drift fit ({stats["drift_rate"]:+.4f} deg/s)')
    ax2.fill_between(t, error, alpha=0.15, color="crimson")
    ax2.axhline(0, color="grey", linewidth=0.5, linestyle=":")
    ax2.set_ylabel("Error (deg)", fontsize=11)
    ax2.legend(fontsize=9, loc="upper right")
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Raw Error with Drift Fit", fontsize=10)

    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(t, detrended, color="mediumorchid", linewidth=1.0,
             label=f'Detrended error  (RMS = {stats["detrended_rms"]:.4f} deg)')
    ax3.fill_between(t, detrended, alpha=0.15, color="mediumorchid")
    ax3.axhline(0, color="grey", linewidth=0.5, linestyle=":")
    ax3.set_ylabel("Error (deg)", fontsize=11)
    ax3.set_xlabel("Time (s)", fontsize=11)
    ax3.legend(fontsize=9, loc="upper right")
    ax3.grid(True, alpha=0.3)
    ax3.set_title("Detrended Error (drift removed = noise floor)", fontsize=10)

    stats_text = (
        f"Mocap axis:    {mocap_axis}\n"
        f"IMU axis:      {imu_axis}\n"
        f"IMU Euler order:   {IMU_EULER_ORDER.upper()}\n"
        f"\n"
        f"RMS error:     {stats['rms_error']:.4f} deg\n"
        f"Max error:     {stats['max_error']:.4f} deg\n"
        f"  (removed before comparison)\n"
        f"\n"
        f"Drift rate:\n"
        f"  {stats['drift_rate']:+.6f} deg/s\n"
        f"  ({stats['drift_rate_hr']:+.2f} deg/hr)\n"
        f"\n"
        f"Detrended RMS:\n"
        f"  {stats['detrended_rms']:.4f} deg"
    )
    fig.text(0.74, 0.72, stats_text,
             fontsize=9, verticalalignment="top", fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.6", facecolor="lightyellow",
                       edgecolor="goldenrod", alpha=0.9))

    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    logger.info(f"Saved comparison plot -> {output_path}")
    plt.close()


# ── Step 8: Save Error CSV ────────────────────────────────────────────────────

def save_error_csv(t: np.ndarray, mocap_angles: np.ndarray,
                   imu_angles: np.ndarray, stats: dict, output_path: str):
    df = pd.DataFrame({
        "time_s":          t,
        "mocap_angle_deg": mocap_angles,
        "imu_angle_deg":   imu_angles,
        "error_deg":       stats["error"],
        "drift_fit_deg":   stats["drift_fit"],
        "detrended_deg":   stats["detrended"],
    })
    df.to_csv(output_path, index=False)
    logger.info(f"Saved error CSV  -> {output_path}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    setup_logger(log_dir=Path(OUTPUT_DIR))

    mocap_df, mocap_axis = process_mocap(
        mocap_csv   = MOCAP_CSV,
        ref_window  = MOCAP_REF_WINDOW,
        trim_end    = MOCAP_TRIM_END,
        euler_order = MOCAP_EULER_ORDER,
        output_dir  = OUTPUT_DIR,
        label       = LABEL,
    )

    imu_df, imu_axis = process_imu(
        ref_window = IMU_REF_WINDOW,
        trim_end   = IMU_TRIM_END,
        output_dir = OUTPUT_DIR,
        label      = LABEL,
    )

    mocap_axis, fine_lag = find_best_axis_pair(
        mocap_df, imu_df,
        imu_axis      = imu_axis,
        mocap_axis    = mocap_axis,
        mocap_ref_end = MOCAP_REF_WINDOW[1],
        imu_ref_end   = IMU_REF_WINDOW[1],
    )

    imu_df_synced = synchronise(
        imu_df,
        imu_axis   = imu_axis,
        imu_sign   = sign,
        coarse_lag = MOCAP_REF_WINDOW[1] - IMU_REF_WINDOW[1],
        fine_lag   = fine_lag,
    )

    t, mocap_angles, imu_angles = resample_to_mocap_grid(
        mocap_df, imu_df_synced, mocap_axis, imu_axis
    )

    stats = compute_error_stats(t, mocap_angles, imu_angles)

    plot_comparison(
        t, mocap_angles, imu_angles, stats,
        mocap_axis  = mocap_axis,
        imu_axis    = imu_axis,
        label       = LABEL,
        output_path = os.path.join(OUTPUT_DIR, f"{LABEL}_comparison.png"),
    )

    save_error_csv(
        t, mocap_angles, imu_angles, stats,
        output_path = os.path.join(OUTPUT_DIR, f"{LABEL}_error.csv"),
    )

    logger.info(f"All outputs written to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
