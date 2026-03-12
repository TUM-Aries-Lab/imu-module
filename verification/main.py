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
from scipy.optimize import curve_fit
from scipy.signal import correlate, hilbert

from imu_python.utils import setup_logger

# ══════════════════════════════════════════════════════════════════════════════
# CONFIG — edit these values before running
# ══════════════════════════════════════════════════════════════════════════════

MOCAP_CSV  = "/home/aries-orin-1/lower_exosuit/imu/imu-module/data/mocap/test rig run 03.csv"
OUTPUT_DIR = "./results"
LABEL      = "trial_01"

# Mocap windows — seconds from the start of the MOCAP recording.
MOCAP_REF_WINDOW = (19.0, 26.0)
MOCAP_TRIM_END   = 90.0

# IMU windows — seconds from the start of the IMU recording.
IMU_CSV        = "/home/aries-orin-1/lower_exosuit/imu/imu-module/data/recordings/imu_data_BNO055_0_7_2026-03-10_15-03-45_test_run3.csv"
IMU_REF_WINDOW = (17.0, 24.0)
IMU_TRIM_END   = 88.0

# Both sensors are decomposed with the same Euler order so the round-trip
# through scipy Rotation gives a consistent, comparable representation.
EULER_ORDER = "xyz"


# ── Step 1: Process Mocap ─────────────────────────────────────────────────────

def process_mocap(mocap_csv: str, ref_window: tuple, trim_end: float,
                  euler_order: str, output_dir: str, label: str) -> pd.DataFrame:
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
    return pd.DataFrame({"time_s": time, **cols, "region": regions})


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

    from src.imu_python.data_handler.data_reader import load_imu_data
    imu_data    = load_imu_data(Path(IMU_CSV))
    timestamps  = imu_data.time.tolist()
    quaternions = imu_data.quats

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
        euler_order = EULER_ORDER,
    )

    eulers   = results["eulers_deg"]        # (N, 3)
    time     = results["time_s"]
    imu_axis = results["axis"]              # highest-variance axis from IMU analysis
    cols     = {ax.upper(): eulers[:, i] for i, ax in enumerate(EULER_ORDER)}
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


def _estimate_period(df: pd.DataFrame, axis: str, ref_end: float) -> float:
    """Estimate dominant oscillation period via autocorrelation of clean region.
    Returns period in seconds, clamped to [2, 30]. Falls back to 10s.
    """
    clean = df[df["region"] == "clean"]
    if len(clean) < 20:
        return 10.0
    dt  = float(np.median(np.diff(clean["time_s"].values)))
    sig = clean[axis].values
    sig = (sig - sig.mean()) / (sig.std() + 1e-9)
    ac  = correlate(sig, sig, mode="full")[len(sig) - 1:]
    ac /= ac[0]
    for i in range(1, len(ac) - 1):
        if ac[i] > ac[i - 1] and ac[i] > ac[i + 1] and ac[i] > 0.1:
            return float(np.clip(i * dt, 2.0, 30.0))
    logger.warning("  Could not estimate period — defaulting to 10.0s")
    return 10.0


def find_best_axis_pair(mocap_df: pd.DataFrame,
                        imu_df: pd.DataFrame,
                        imu_axis: str,
                        euler_order: str,
                        mocap_ref_end: float,
                        imu_ref_end: float,
                        ) -> tuple[str, float, float]:
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

    axes       = [ax.upper() for ax in euler_order]
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
    t_ramp  = np.arange(mocap_ref_end, mocap_ref_end + RAMP_WINDOW, dt)

    def _ramp(df, ax, t_start_physical):
        """Interpolate `ax` onto t_ramp, zero at t_start_physical."""
        sig = interp1d(df["time_s"], df[ax], kind="linear",
                       bounds_error=False, fill_value=0.0)(t_ramp)
        return sig - sig[0]   # remove initial offset but keep direction

    mocap_ramps = {ax: _ramp(mocap_df, ax, mocap_ref_end) for ax in axes}

    # Shift IMU onto mocap clock by coarse_lag before extracting ramp
    imu_shifted        = imu_df.copy()
    imu_shifted["time_s"] = imu_df["time_s"] + coarse_lag
    imu_ramp           = _ramp(imu_shifted, imu_axis, mocap_ref_end)
    imu_ramp_norm      = imu_ramp / (np.std(imu_ramp) + 1e-9)

    results_table = []
    best_score    = -np.inf
    best_m_ax     = axes[0]
    best_sign     = 1.0

    for m_ax in axes:
        mr   = mocap_ramps[m_ax]
        mr_n = mr / (np.std(mr) + 1e-9)
        dot  = float(np.dot(mr_n, imu_ramp_norm))
        sign = +1.0 if dot >= 0 else -1.0
        score = abs(dot)
        results_table.append((m_ax, score, sign, dot))
        if score > best_score:
            best_score, best_m_ax, best_sign = score, m_ax, sign

    logger.info(f"  Ramp window: {RAMP_WINDOW}s after ref_end")
    logger.info(f"  {'Mocap':>6}  {'IMU':>6}  {'|dot|':>8}  {'Sign':>5}  {'dot':>8}")
    for m_ax, score, sign, dot in sorted(results_table, key=lambda x: -x[1]):
        marker = "  ← SELECTED" if m_ax == best_m_ax else ""
        logger.info(f"  {m_ax:>6}  {imu_axis:>6}  {score:>8.4f}  {sign:>+5.0f}  {dot:>+8.4f}{marker}")

    # ── Step 2: fine lag via first upward zero-crossing after ref_end ─────────
    #
    # Zero-crossings are robust to waveform distortion (asymmetric peaks,
    # harmonics) because they depend only on where the signal crosses zero,
    # not on the shape of the peaks.  We find the first upward zero-crossing
    # after ref_end on each signal (after applying coarse lag to the IMU),
    # interpolate for sub-sample precision, and take the difference as the
    # fine lag.  This is unambiguous as long as both sensors have their first
    # crossing within the same cycle — which is guaranteed because the coarse
    # lag places them within ±(period/2) of each other.

    period = _estimate_period(mocap_df, best_m_ax, mocap_ref_end)
    logger.info(f"  Estimated period: {period:.3f}s")

    m_clean = mocap_df[mocap_df["region"] == "clean"]
    i_clean = imu_shifted[imu_shifted["region"] == "clean"]   # already coarse-shifted

    def _first_upward_zero_crossing(df, ax, sign, after_t, label):
        """Return time of first upward zero-crossing of `ax * sign` after `after_t`.
        Uses linear interpolation between the two bracketing samples for
        sub-sample precision.  Returns None if no crossing is found.
        """
        t   = df["time_s"].values
        sig = df[ax].values * sign
        sig = sig - sig.mean()   # remove DC so zero-crossing is well-defined

        # Only look within the first 2 periods after after_t to avoid
        # picking a crossing from a much later cycle
        mask = (t >= after_t) & (t <= after_t + 2 * period)
        t_w, s_w = t[mask], sig[mask]

        for i in range(len(s_w) - 1):
            if s_w[i] <= 0 < s_w[i + 1]:        # upward crossing between i and i+1
                # Linear interpolation: t_cross = t[i] - s[i]*(t[i+1]-t[i])/(s[i+1]-s[i])
                slope  = s_w[i + 1] - s_w[i]
                t_cross = t_w[i] - s_w[i] * (t_w[i + 1] - t_w[i]) / slope
                logger.info(f"  Zero-crossing [{label}]: t = {t_cross:.6f}s")
                return float(t_cross)

        logger.warning(f"  Zero-crossing [{label}]: not found after {after_t:.2f}s "
                       f"— falling back to fine_lag = 0")
        return None

    t_cross_m = _first_upward_zero_crossing(
        m_clean, best_m_ax, 1.0,       mocap_ref_end, f"mocap/{best_m_ax}")
    t_cross_i = _first_upward_zero_crossing(
        i_clean, imu_axis,  best_sign, mocap_ref_end, f"imu/{imu_axis}")

    if t_cross_m is not None and t_cross_i is not None:
        # fine_lag = how much extra shift the IMU needs on top of coarse_lag
        # so that its zero-crossing aligns with the mocap zero-crossing
        fine_lag = t_cross_m - t_cross_i
        # Sanity check: fine lag should be well within one period
        if abs(fine_lag) > period * 0.5:
            logger.warning(f"  Fine lag {fine_lag:+.4f}s exceeds half-period "
                           f"({period/2:.3f}s) — clamping to 0 and using coarse lag only")
            fine_lag = 0.0
    else:
        fine_lag = 0.0

    total_lag = coarse_lag + fine_lag
    logger.info(f"  Fine lag (zero-crossing difference): {fine_lag:+.6f}s")
    logger.info(f"  Result: mocap={best_m_ax}  imu={imu_axis}  sign={best_sign:+.0f}  "
                f"total_lag={total_lag:+.6f}s  "
                f"(coarse={coarse_lag:+.4f}s  fine={fine_lag:+.6f}s)")

    return best_m_ax, best_sign, fine_lag


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
    mocap_out = mocap_clean[mocap_axis].values[valid]

    # Re-zero: subtract each signal mean over the shared clean window.
    # Eliminates DC offset from imperfect per-sensor reference windows.
    # The DC offset itself (imu_dc - mocap_dc) is a meaningful measurement:
    # it is the bias between what the IMU and mocap report as "zero".
    mocap_dc  = float(np.mean(mocap_out))
    imu_dc    = float(np.mean(imu_out))
    dc_offset = imu_dc - mocap_dc   # IMU bias relative to mocap after sync
    mocap_out = mocap_out - mocap_dc
    imu_out   = imu_out   - imu_dc

    logger.info(f"Common grid: {len(t_out)} points, {t_out[0]:.2f}s - {t_out[-1]:.2f}s")
    logger.info(f"  DC removed — mocap: {mocap_dc:+.4f} deg  |  imu: {imu_dc:+.4f} deg  "
                f"|  IMU bias: {dc_offset:+.4f} deg")

    return t_out, mocap_out, imu_out, dc_offset


# ── Step 6: Error Statistics ───────────────────────────────────────────────────

def compute_error_stats(t: np.ndarray,
                        mocap_angles: np.ndarray,
                        imu_angles: np.ndarray,
                        dc_offset: float = 0.0) -> dict:
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
        "dc_offset":     dc_offset,           # IMU bias before re-zeroing (deg)
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

    logger.info(f"  IMU bias (DC offset):  {dc_offset:+.4f} deg  "
                f"(removed by re-zeroing)")
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
        f"euler order: {EULER_ORDER.upper()}",
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
        f"Euler order:   {EULER_ORDER.upper()}\n"
        f"\n"
        f"RMS error:     {stats['rms_error']:.4f} deg\n"
        f"Max error:     {stats['max_error']:.4f} deg\n"
        f"IMU DC bias:   {stats['dc_offset']:+.4f} deg\n"
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

    mocap_df = process_mocap(
        mocap_csv   = MOCAP_CSV,
        ref_window  = MOCAP_REF_WINDOW,
        trim_end    = MOCAP_TRIM_END,
        euler_order = EULER_ORDER,
        output_dir  = OUTPUT_DIR,
        label       = LABEL,
    )

    imu_df, imu_axis = process_imu(
        ref_window = IMU_REF_WINDOW,
        trim_end   = IMU_TRIM_END,
        output_dir = OUTPUT_DIR,
        label      = LABEL,
    )

    mocap_axis, imu_sign, fine_lag = find_best_axis_pair(
        mocap_df, imu_df,
        imu_axis      = imu_axis,
        euler_order   = EULER_ORDER,
        mocap_ref_end = MOCAP_REF_WINDOW[1],
        imu_ref_end   = IMU_REF_WINDOW[1],
    )

    imu_df_synced = synchronise(
        imu_df,
        imu_axis   = imu_axis,
        imu_sign   = imu_sign,
        coarse_lag = MOCAP_REF_WINDOW[1] - IMU_REF_WINDOW[1],
        fine_lag   = fine_lag,
    )

    t, mocap_angles, imu_angles, dc_offset = resample_to_mocap_grid(
        mocap_df, imu_df_synced, mocap_axis, imu_axis
    )

    stats = compute_error_stats(t, mocap_angles, imu_angles, dc_offset)

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
