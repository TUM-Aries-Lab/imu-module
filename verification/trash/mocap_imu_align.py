"""
mocap_imu_align.py
==================
Pipeline to align IMU data to Vicon mocap data for a single-axis
oscillation (leg-swing style) experiment.

Pipeline:
  1. Load & preprocess (mocap frames → timestamps, IMU quats → Rotations)
  2. Detect stationary windows independently for each sensor
  3. Frame alignment: R_offset + IMU drift rate from stationary window
  4. Apply drift correction + frame alignment to full IMU signal
  5. Temporal alignment: sync oscillation onset
  6. Clock drift correction (optional, Arduino-style): stretch/squeeze IMU time
  7. Resample IMU onto mocap timebase
  8. (Optional) refine drift from peak alignment
  9. Convert to Euler angles, plot, and report errors
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from scipy.signal import find_peaks
from scipy.spatial.transform import Rotation, Slerp

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
MOCAP_CSV = "/home/haoqing/Thesis Project/imu-module/data/mocap/test rig run 03 bno055jetson 01.csv"
IMU_CSV = "/home/haoqing/Thesis Project/imu-module/data/test_recordings/imu_data_BNO055_0_7_test01mag.csv"
OUTPUT = "./trials"
LABEL = "bno055_01"
jetson: bool = True

@dataclass
class AlignConfig:
    output_dir: str | Path = Path(f"{OUTPUT}")
    # --- Activity detection ---
    smoothing_window_sec:  float = 0.5   # smoothing kernel width — suppresses motor twitches
    sustained_exceed_sec:  float = 0.3   # must exceed threshold for this long to count as oscillation

    stationary_std_threshold: float = 1   # rad — signals below this are "still"
    stationary_window_sec: float = 3.0        # minimum seconds to qualify as stationary
    stationary_search_sec: float = 30.0       # only look in the first N seconds for the window

    # --- End trimming ---
    trim_end_sec: Optional[float] = None      # if set, cut both signals to this duration

    # --- Clock drift ---
    clock_drift_tolerance: float = 0.002      # frequency ratio tolerance before warping is applied

    # --- Oscillation detection ---
    min_peak_prominence: float = 0.05         # rad — minimum prominence for peak detection

    # --- Euler output convention ---
    euler_convention: str = "zyx"             # scipy convention string
    euler_degrees: bool = True

    # --- Pivot axis (IMU frame, unit vector) ---
    # For a leg swing mounted vertically, primary motion is around Z
    imu_pivot_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)

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

# ---------------------------------------------------------------------------
# Step 1 — Load & preprocess
# ---------------------------------------------------------------------------

def load_mocap(filepath: str | Path, trim_end_sec: Optional[float] = None
               ) -> tuple[np.ndarray, Rotation]:
    """
    Load Vicon CSV, convert frames → timestamps, helical angles → Rotations.
    Returns (timestamps_sec, Rotation array).
    """
    df, sample_rate = parse_mocap_csv(str(filepath))

    timestamps = df["Frame"].values / sample_rate  # frames → seconds

    # Vicon helical: magnitude of [RX,RY,RZ] is angle in degrees
    helical_deg = df[["RX", "RY", "RZ"]].values   # shape (N, 3)
    angles_deg  = np.linalg.norm(helical_deg, axis=1, keepdims=True)
    safe_angles = np.where(angles_deg < 1e-10, 1.0, angles_deg)
    axes        = helical_deg / safe_angles
    rotvecs_rad = axes * np.deg2rad(angles_deg)    # (N, 3)
    # zero-angle rows → zero rotvec
    rotvecs_rad[angles_deg[:, 0] < 1e-10] = 0.0

    R = Rotation.from_rotvec(rotvecs_rad)

    if trim_end_sec is not None:
        mask = timestamps <= trim_end_sec
        timestamps, R = timestamps[mask], R[mask]
        logger.info("Mocap trimmed to %.1f s (%d samples)", trim_end_sec, mask.sum())

    logger.info("Mocap loaded: %d samples, %.2f s @ %.1f Hz",
                len(timestamps), timestamps[-1], sample_rate)
    return timestamps, R


def load_imu(timestamps: np.ndarray, quaternions,
             trim_end_sec: Optional[float] = None) -> tuple[np.ndarray, Rotation]:
    """
    Accept pre-loaded timestamps + quaternion objects (w,x,y,z dataclass or array).
    Returns (timestamps_sec, Rotation array).
    """
    # Support both dataclass list and raw numpy array
    if hasattr(quaternions[0], "w"):
        q = np.array([[q.w, q.x, q.y, q.z] for q in quaternions])
    else:
        q = np.asarray(quaternions)

    # scipy expects [x, y, z, w]
    if q.shape[1] == 4:
        q_xyzw = q[:, [1, 2, 3, 0]]
    else:
        raise ValueError("Unexpected quaternion shape: %s" % str(q.shape))

    timestamps = np.asarray(timestamps, dtype=float)
    # Normalise to start at t=0
    timestamps = timestamps - timestamps[0]

    R = Rotation.from_quat(q_xyzw)

    if trim_end_sec is not None:
        mask = timestamps <= trim_end_sec
        timestamps, R = timestamps[mask], R[mask]
        logger.info("IMU trimmed to %.1f s (%d samples)", trim_end_sec, mask.sum())

    logger.info("IMU loaded: %d samples, %.2f s", len(timestamps), timestamps[-1])
    return timestamps, R


# ---------------------------------------------------------------------------
# Step 2 — Detect stationary window
# ---------------------------------------------------------------------------
def _angular_velocity(timestamps: np.ndarray, R: Rotation) -> np.ndarray:
    """
    Compute angular velocity as the magnitude of the relative rotation
    between consecutive frames. This is invariant to initial orientation.
    """
    dt = np.diff(timestamps)
    dt = np.where(dt < 1e-9, 1e-9, dt)

    # Relative rotation between consecutive frames
    R_rel     = R[:-1].inv() * R[1:]
    rotvec_rel = R_rel.as_rotvec()                          # (N-1, 3)
    angvel     = np.linalg.norm(rotvec_rel, axis=1) / dt   # rad/s

    return np.append(angvel, angvel[-1])                    # pad to length N

def detect_stationary_window(timestamps: np.ndarray, R: Rotation,
                              cfg: AlignConfig) -> tuple[int, int]:
    angvel    = _angular_velocity(timestamps, R)
    kernel    = max(1, int(_sample_rate(timestamps) * cfg.smoothing_window_sec))
    angvel_sm = np.convolve(angvel, np.ones(kernel) / kernel, mode="same")

    search_end = np.searchsorted(timestamps, timestamps[0] + cfg.stationary_search_sec)
    sustained  = max(1, int(_sample_rate(timestamps) * cfg.sustained_exceed_sec))
    threshold  = cfg.stationary_std_threshold

    end_idx = search_end

    for i in range(search_end - sustained):
        if np.all(angvel_sm[i:i + sustained] > threshold):
            end_idx = i
            break

    if end_idx <= 0:
        logger.warning(
            "Movement detected immediately at t=0 — stationary window is empty. "
            "Consider lowering threshold or increasing stationary_search_sec.")
    elif end_idx == search_end:
        logger.warning(
            "No movement detected within %.1f s — entire search window treated as stationary. "
            "Consider raising threshold or increasing stationary_search_sec.",
            cfg.stationary_search_sec)
    else:
        logger.info("Stationary window: t=%.2f s – %.2f s (%d samples)",
                    timestamps[0], timestamps[end_idx], end_idx)

    return 0, end_idx

def _sample_rate(timestamps: np.ndarray) -> float:
    return 1.0 / np.median(np.diff(timestamps))


# ---------------------------------------------------------------------------
# Step 3 — Frame alignment + drift rate estimation
# ---------------------------------------------------------------------------

def estimate_frame_alignment(
        t_mocap: np.ndarray, R_mocap: Rotation,
        t_imu:   np.ndarray, R_imu:   Rotation,
        mocap_static: tuple[int, int],
        imu_static:   tuple[int, int],
) -> tuple[Rotation, np.ndarray]:
    """
    Returns:
        R_offset  — fixed rotation: R_mocap_aligned = R_offset * R_imu
        drift_rate — (3,) rad/s vector, linear drift per axis in rotvec space
    """
    ms, me = mocap_static
    is_, ie = imu_static

    R_mocap_mean = R_mocap[ms:me].mean()
    R_imu_mean   = R_imu[is_:ie].mean()
    R_offset     = R_mocap_mean * R_imu_mean.inv()
    logger.info("R_offset (ZYX deg): %s",
                R_offset.as_euler("zyx", degrees=True).round(3))

    # Drift: linear fit to rotvec during IMU stationary window
    rotvecs_static = R_imu[is_:ie].as_rotvec()          # (M, 3)
    t_static       = t_imu[is_:ie] - t_imu[is_]        # relative time

    drift_rate = np.array([
        np.polyfit(t_static, rotvecs_static[:, i], 1)[0]
        for i in range(3)
    ])
    logger.info("IMU drift rate (rad/s): %s", drift_rate.round(6))
    return R_offset, drift_rate


# ---------------------------------------------------------------------------
# Step 4 — Apply drift correction + frame alignment
# ---------------------------------------------------------------------------

def apply_corrections(t_imu: np.ndarray, R_imu: Rotation,
                      R_offset: Rotation, drift_rate: np.ndarray,
                      t_ref: float) -> Rotation:
    """
    Remove linear drift (anchored at t_ref) then apply frame offset.
    t_ref should be the mean time of the IMU stationary window.
    """
    t_rel        = t_imu - t_ref
    drift_rotvec = np.outer(t_rel, drift_rate)          # (N, 3)
    R_drift      = Rotation.from_rotvec(drift_rotvec)
    R_corrected  = R_drift.inv() * R_imu
    R_aligned    = R_offset * R_corrected
    return R_aligned


# ---------------------------------------------------------------------------
# Step 5 — Temporal alignment (sync oscillation onset)
# ---------------------------------------------------------------------------

def detect_oscillation_onset(timestamps: np.ndarray, R: Rotation,
                              cfg: AlignConfig) -> int:
    """Return index where oscillation starts (activity exceeds threshold)."""
    angvel = _angular_velocity(timestamps, R)

    # Smooth slightly
    kernel    = int(max(1, _sample_rate(timestamps) * 0.1))
    angvel_sm = np.convolve(angvel, np.ones(kernel) / kernel, mode="same")

    onset = np.argmax(angvel_sm > cfg.stationary_std_threshold * 3)
    logger.info("Oscillation onset at idx %d (%.2f s)", onset, timestamps[onset])
    return int(onset)


def sync_onset(t_mocap: np.ndarray, R_mocap: Rotation,
               t_imu:   np.ndarray, R_imu:   Rotation,
               cfg: AlignConfig) -> tuple[np.ndarray, Rotation, np.ndarray, Rotation]:
    """
    Shift both signals so their oscillation onset is at t=0.
    Returns trimmed (t_mocap, R_mocap, t_imu, R_imu).
    """
    onset_m = detect_oscillation_onset(t_mocap, R_mocap, cfg)
    onset_i = detect_oscillation_onset(t_imu,   R_imu,   cfg)

    t_mocap  = t_mocap[onset_m:] - t_mocap[onset_m]
    R_mocap  = R_mocap[onset_m:]
    t_imu    = t_imu[onset_i:]   - t_imu[onset_i]
    R_imu    = R_imu[onset_i:]
    logger.info("Signals trimmed to oscillation onset; both start at t=0.")
    return t_mocap, R_mocap, t_imu, R_imu


# ---------------------------------------------------------------------------
# Step 6 — Clock drift correction (Arduino)
# ---------------------------------------------------------------------------

def _dominant_frequency(timestamps: np.ndarray, signal: np.ndarray) -> float:
    """Estimate dominant frequency via FFT."""
    n   = len(signal)
    dt  = np.median(np.diff(timestamps))
    fft = np.abs(np.fft.rfft(signal - signal.mean()))
    freqs = np.fft.rfftfreq(n, d=dt)
    return freqs[np.argmax(fft[1:]) + 1]   # skip DC


def correct_clock_drift(t_mocap: np.ndarray, R_mocap: Rotation,
                        t_imu:   np.ndarray, R_imu:   Rotation,
                        pivot_axis: np.ndarray,
                        cfg: AlignConfig) -> tuple[np.ndarray, Rotation]:
    """
    If IMU oscillation frequency differs from mocap by more than tolerance,
    warp IMU timestamps by the frequency ratio.
    Returns corrected (t_imu_warped, R_imu) — same Rotations, new timestamps.
    """
    angle_mocap = _project_to_axis(R_mocap, pivot_axis)
    angle_imu   = _project_to_axis(R_imu,   pivot_axis)

    f_mocap = _dominant_frequency(t_mocap, angle_mocap)
    f_imu   = _dominant_frequency(t_imu,   angle_imu)
    ratio   = f_mocap / f_imu

    logger.info("Mocap freq: %.4f Hz | IMU freq: %.4f Hz | ratio: %.5f",
                f_mocap, f_imu, ratio)

    if abs(ratio - 1.0) < cfg.clock_drift_tolerance:
        logger.info("Clock drift within tolerance — no warping applied.")
        return t_imu, R_imu

    logger.info("Applying clock warp (ratio %.5f) to IMU timestamps.", ratio)
    t_imu_warped = t_imu * ratio
    return t_imu_warped, R_imu


# ---------------------------------------------------------------------------
# Step 7 — Resample IMU onto mocap timebase
# ---------------------------------------------------------------------------

def resample_imu_to_mocap(t_mocap: np.ndarray, R_mocap: Rotation,
                           t_imu:   np.ndarray, R_imu:   Rotation
                           ):
    """
    Slerp-interpolate IMU rotations onto mocap timestamps.
    Only interpolates within the overlapping time range.
    """
    t_min = max(t_mocap[0],  t_imu[0])
    t_max = min(t_mocap[-1], t_imu[-1])

    mask    = (t_mocap >= t_min) & (t_mocap <= t_max)
    t_query = t_mocap[mask]

    slerp         = Slerp(t_imu, R_imu)
    R_imu_resampled = slerp(t_query)

    logger.info("Resampled IMU onto %d mocap timepoints (%.2f–%.2f s)",
                mask.sum(), t_min, t_max)
    return t_mocap[mask], R_mocap[mask], R_imu_resampled


# ---------------------------------------------------------------------------
# Step 8 — Optional: refine drift via peak alignment
# ---------------------------------------------------------------------------

def refine_drift_from_peaks(t_common: np.ndarray,
                             R_mocap:  Rotation,
                             R_imu:    Rotation,
                             pivot_axis: np.ndarray,
                             cfg: AlignConfig,
                             drift_rate: np.ndarray,
                             R_offset: Rotation,
                             t_imu_full: np.ndarray,
                             R_imu_full: Rotation,
                             t_ref: float,
                             max_iter: int = 5,
                             tol_ms: float = 5.0) -> np.ndarray:
    """
    Iteratively refine drift_rate so that IMU peaks align with mocap peaks.
    Returns refined drift_rate.
    """
    for iteration in range(max_iter):
        angle_m = _project_to_axis(R_mocap, pivot_axis)
        angle_i = _project_to_axis(R_imu,   pivot_axis)

        peaks_m, _ = find_peaks(angle_m, prominence=cfg.min_peak_prominence)
        peaks_i, _ = find_peaks(angle_i, prominence=cfg.min_peak_prominence)

        n = min(len(peaks_m), len(peaks_i))
        if n < 2:
            logger.warning("Too few peaks for drift refinement.")
            break

        dt_errors = t_common[peaks_m[:n]] - t_common[peaks_i[:n]]
        mean_err_ms = np.mean(np.abs(dt_errors)) * 1000

        logger.info("Drift refinement iter %d — mean peak error: %.2f ms",
                    iteration + 1, mean_err_ms)
        if mean_err_ms < tol_ms:
            logger.info("Converged.")
            break

        # Fit residual drift from peak timing errors
        residual_slope = np.polyfit(t_common[peaks_m[:n]], dt_errors, 1)[0]
        # Convert timing error slope to rotation rate correction
        # (small angle: residual angular drift ≈ slope * angular frequency)
        f_osc      = _dominant_frequency(t_common, angle_m)
        ang_freq   = 2 * np.pi * f_osc
        delta_drift = pivot_axis * residual_slope * ang_freq
        drift_rate  = drift_rate + delta_drift

        # Re-apply with refined drift
        R_imu_full_corr = apply_corrections(t_imu_full, R_imu_full,
                                             R_offset, drift_rate, t_ref)
        # Re-resample (caller must handle; here we just update angles for next iter)
        slerp = Slerp(t_imu_full, R_imu_full_corr)
        R_imu = slerp(t_common)

    return drift_rate


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _project_to_axis(R: Rotation, axis: np.ndarray) -> np.ndarray:
    """Project rotation vectors onto a unit axis → scalar angle timeseries."""
    axis = axis / np.linalg.norm(axis)
    return R.as_rotvec() @ axis


def rotations_to_euler(R: Rotation, cfg: AlignConfig) -> np.ndarray:
    return R.as_euler(cfg.euler_convention, degrees=cfg.euler_degrees)


# ---------------------------------------------------------------------------
# Step 9 — Plot & report
# ---------------------------------------------------------------------------

def rotations_to_comparable(R_mocap: Rotation, R_imu: Rotation,
                             pivot_axis: np.ndarray,
                             cfg: AlignConfig):
    """
    Returns signals for comparison that are robust to gimbal lock and unwrapping.
    Produces:
      - Angle around pivot axis (scalar, unwrapped, unbounded)
      - Full Euler angles with np.unwrap applied per axis
    """
    axis = pivot_axis / np.linalg.norm(pivot_axis)

    # 1. Scalar pivot angle — no gimbal lock, no wrapping issues
    angle_m = R_mocap.as_rotvec() @ axis
    angle_i = R_imu.as_rotvec()   @ axis

    # 2. Euler angles with unwrapping
    euler_m_raw = R_mocap.as_euler(cfg.euler_convention, degrees=False)
    euler_i_raw = R_imu.as_euler(  cfg.euler_convention, degrees=False)

    # np.unwrap handles the ±π wrap-around discontinuities
    euler_m = np.unwrap(euler_m_raw, axis=0)
    euler_i = np.unwrap(euler_i_raw, axis=0)

    if cfg.euler_degrees:
        euler_m = np.degrees(euler_m)
        euler_i = np.degrees(euler_i)
        angle_m = np.degrees(angle_m)
        angle_i = np.degrees(angle_i)

    return (angle_m, angle_i), (euler_m, euler_i)

def plot_and_report(t: np.ndarray,
                    euler_mocap: np.ndarray,
                    euler_imu:   np.ndarray,
                    cfg: AlignConfig,
                    title: str = "Mocap vs IMU") -> dict:
    """Plot aligned Euler angles and return error statistics."""

    labels = [f"{cfg.euler_convention[i].upper()}" for i in range(3)]
    unit   = "°" if cfg.euler_degrees else "rad"
    error  = euler_mocap - euler_imu

    # Drift-removed error: subtract linear trend from error
    error_detrended = np.zeros_like(error)
    for i in range(3):
        trend = np.polyval(np.polyfit(t, error[:, i], 1), t)
        error_detrended[:, i] = error[:, i] - trend

    stats = {}
    for i, lbl in enumerate(labels):
        stats[lbl] = {
            "RMSE":           float(np.sqrt(np.mean(error[:, i] ** 2))),
            "MAE":            float(np.mean(np.abs(error[:, i]))),
            "Max":            float(np.max(np.abs(error[:, i]))),
            "RMSE_detrended": float(np.sqrt(np.mean(error_detrended[:, i] ** 2))),
        }

    # --- Plot ---
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(title, fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    colors = ["#e74c3c", "#2ecc71", "#3498db"]

    for i, lbl in enumerate(labels):
        # Left: overlay
        ax_l = fig.add_subplot(gs[i, 0])
        ax_l.plot(t, euler_mocap[:, i], color=colors[i], lw=1.5, label="Mocap")
        ax_l.plot(t, euler_imu[:, i],   color=colors[i], lw=1.0,
                  linestyle="--", alpha=0.8, label="IMU")
        ax_l.set_ylabel(f"{lbl} ({unit})")
        ax_l.set_title(f"{lbl} — Overlay")
        ax_l.legend(fontsize=8)
        ax_l.grid(True, alpha=0.3)

        # Right: error
        ax_r = fig.add_subplot(gs[i, 1])
        ax_r.plot(t, error[:, i],            color="gray",    lw=1.0,
                  label=f"Error  RMSE={stats[lbl]['RMSE']:.2f}{unit}")
        ax_r.plot(t, error_detrended[:, i],  color=colors[i], lw=1.0,
                  label=f"Detrended RMSE={stats[lbl]['RMSE_detrended']:.2f}{unit}")
        ax_r.axhline(0, color="black", lw=0.5, linestyle=":")
        ax_r.set_ylabel(f"Δ{lbl} ({unit})")
        ax_r.set_title(f"{lbl} — Error")
        ax_r.legend(fontsize=8)
        ax_r.grid(True, alpha=0.3)

    for ax in fig.axes:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    out = Path(cfg.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    plt.savefig(f"{LABEL}_mocap_imu_comparison.png", dpi=150)
    plt.show()

    # Print summary
    print(f"\n{'='*55}")
    print(f"  {title}")
    print(f"{'='*55}")
    print(f"  {'Axis':<6} {'RMSE':>8} {'MAE':>8} {'Max':>8} {'RMSE(det)':>10}")
    print(f"  {'-'*46}")
    for lbl, s in stats.items():
        print(f"  {lbl:<6} {s['RMSE']:>7.3f}{unit} {s['MAE']:>7.3f}{unit} "
              f"{s['Max']:>7.3f}{unit} {s['RMSE_detrended']:>9.3f}{unit}")
    print(f"{'='*55}\n")

    return stats

def plot_and_report_pivot(t: np.ndarray,
                          angle_mocap: np.ndarray,
                          angle_imu:   np.ndarray,
                          cfg:         AlignConfig,
                          title:       str = "Mocap vs IMU — Pivot Axis") -> dict:
    """
    Plot and report errors for the scalar pivot-axis angle comparison.
    This is the primary comparison method — gimbal lock free, unwrap safe.
    """
    unit  = "°" if cfg.euler_degrees else "rad"
    error = angle_mocap - angle_imu

    # Drift-removed error: subtract linear trend
    trend           = np.polyval(np.polyfit(t, error, 1), t)
    error_detrended = error - trend

    stats = {
        "RMSE":           float(np.sqrt(np.mean(error ** 2))),
        "MAE":            float(np.mean(np.abs(error))),
        "Max":            float(np.max(np.abs(error))),
        "RMSE_detrended": float(np.sqrt(np.mean(error_detrended ** 2))),
        "drift_rate":     float(np.polyfit(t, error, 1)[0]),  # unit/sec
    }

    # --- Plot ---
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    fig.suptitle(title, fontsize=13, fontweight="bold")

    # Top: overlay
    axes[0].plot(t, angle_mocap, color="#e74c3c", lw=1.5, label="Mocap")
    axes[0].plot(t, angle_imu,   color="#3498db", lw=1.0,
                 linestyle="--", alpha=0.85, label="IMU")
    axes[0].set_ylabel(f"Angle ({unit})")
    axes[0].set_title("Pivot axis angle — overlay")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Middle: raw error + trend line
    axes[1].plot(t, error, color="gray", lw=1.0,
                 label=f"Error  RMSE={stats['RMSE']:.3f}{unit}")
    axes[1].plot(t, trend, color="#e67e22", lw=1.5,
                 linestyle="--", label=f"Drift trend ({stats['drift_rate']:.4f} {unit}/s)")
    axes[1].axhline(0, color="black", lw=0.5, linestyle=":")
    axes[1].set_ylabel(f"Error ({unit})")
    axes[1].set_title("Raw error + drift trend")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # Bottom: detrended error
    axes[2].plot(t, error_detrended, color="#2ecc71", lw=1.0,
                 label=f"Detrended RMSE={stats['RMSE_detrended']:.3f}{unit}")
    axes[2].axhline(0, color="black", lw=0.5, linestyle=":")
    axes[2].set_ylabel(f"Error ({unit})")
    axes[2].set_title("Drift-removed error")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    for ax in axes:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    out = Path(cfg.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    plt.savefig(f"{LABEL}_mocap_imu_pivot_comparison.png", dpi=150)
    plt.show()

    # Print summary
    print(f"\n{'='*45}")
    print(f"  {title}")
    print(f"{'='*45}")
    print(f"  RMSE:            {stats['RMSE']:.3f} {unit}")
    print(f"  MAE:             {stats['MAE']:.3f} {unit}")
    print(f"  Max error:       {stats['Max']:.3f} {unit}")
    print(f"  RMSE (detrended):{stats['RMSE_detrended']:.3f} {unit}")
    print(f"  Drift rate:      {stats['drift_rate']:.4f} {unit}/s")
    print(f"{'='*45}\n")

    return stats

# ---------------------------------------------------------------------------
# Master pipeline
# ---------------------------------------------------------------------------

def run_pipeline(mocap_csv:      str | Path,
                 imu_timestamps: list[float],
                 imu_quaternions,
                 cfg:            AlignConfig = AlignConfig(),
                 title:          str = "Mocap vs IMU") -> tuple[dict,dict]:
    """
    Full alignment pipeline. Returns error statistics dict.

    Parameters
    ----------
    mocap_csv       : path to Vicon CSV
    imu_timestamps  : 1-D array of IMU timestamps (seconds or ms — will be
                      normalised to seconds starting at 0)
    imu_quaternions : list of Quaternion(w,x,y,z) dataclass objects OR
                      numpy array of shape (N, 4) in [w,x,y,z] order
    cfg             : AlignConfig instance
    title           : plot title
    """

    # 1. Load ----------------------------------------------------------------
    t_m, R_m = load_mocap(mocap_csv, trim_end_sec=cfg.trim_end_sec)
    t_i, R_i = load_imu(np.array(imu_timestamps), imu_quaternions, trim_end_sec=cfg.trim_end_sec)

    angvel    = _angular_velocity(t_m, R_m)
    kernel    = max(1, int(_sample_rate(t_m) * cfg.smoothing_window_sec))
    angvel_sm = np.convolve(angvel, np.ones(kernel) / kernel, mode="same")
    search_end = np.searchsorted(t_m, t_m[0] + cfg.stationary_search_sec)
    sustained  = max(1, int(_sample_rate(t_m) * cfg.sustained_exceed_sec))

    print(f"search_end idx: {search_end}, t={t_m[search_end]:.2f}s")
    print(f"sustained samples: {sustained}")
    print(f"threshold: {cfg.stationary_std_threshold}")
    print(f"angvel max in search window: {angvel_sm[:search_end].max():.6f}")
    print(f"angvel mean in search window: {angvel_sm[:search_end].mean():.6f}")
    print(f"angvel max full signal: {angvel_sm.max():.6f}")
    print(f"first idx exceeding threshold: {np.argmax(angvel_sm > cfg.stationary_std_threshold)}")
    # 2. Detect stationary windows ------------------------------------------
    stat_m = detect_stationary_window(t_m, R_m, cfg)
    stat_i = detect_stationary_window(t_i, R_i, cfg)

    # 3. Frame alignment + drift rate ---------------------------------------
    R_offset, drift_rate = estimate_frame_alignment(
        t_m, R_m, t_i, R_i, stat_m, stat_i)
    t_ref = float(t_i[stat_i[0]:stat_i[1]].mean())

    # 4. Apply corrections --------------------------------------------------
    R_i_aligned = apply_corrections(t_i, R_i, R_offset, drift_rate, t_ref)

    # 5. Sync oscillation onset --------------------------------------------
    t_m, R_m, t_i, R_i_aligned = sync_onset(t_m, R_m, t_i, R_i_aligned, cfg)

    # 6. Clock drift correction --------------------------------------------
    pivot = np.array(cfg.imu_pivot_axis, dtype=float)
    t_i, R_i_aligned = correct_clock_drift(
        t_m, R_m, t_i, R_i_aligned, pivot, cfg)

    # 7. Resample onto mocap timebase --------------------------------------
    t_common, R_m_common, R_i_common = resample_imu_to_mocap(
        t_m, R_m, t_i, R_i_aligned)

    # 8. (Optional) refine drift from peaks --------------------------------
    # Uncomment to enable iterative refinement:
    # drift_rate = refine_drift_from_peaks(
    #     t_common, R_m_common, R_i_common, pivot, cfg,
    #     drift_rate, R_offset, t_i, R_i_aligned, t_ref)
    # Then re-resample after refinement.

    # 9. Convert to Euler + plot + report ----------------------------------
    # In run_pipeline, replace the Euler conversion + plot section:
    (angle_m, angle_i), (euler_m, euler_i) = rotations_to_comparable(
        R_m_common, R_i_common, pivot, cfg)

    # Primary comparison: pivot axis angle (most robust)
    stats_angle = plot_and_report_pivot(t_common, angle_m, angle_i, cfg, title=title)

    euler_m = rotations_to_euler(R_m_common, cfg)
    euler_i = rotations_to_euler(R_i_common, cfg)

    stats_euler = plot_and_report(t_common, euler_m, euler_i, cfg, title=title)
    return stats_angle, stats_euler


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # --- Paste your data loading here ---

    if jetson:
        # Option A: imu_python
        from imu_python.data_handler.data_reader import load_imu_data
        imu_data    = load_imu_data(Path(IMU_CSV))
        timestamps  = imu_data.time.tolist()
        quaternions = imu_data.quats
    else:
        # Option B: Arduino
        from read_matlab import load_quaternions_from_csv
        timestamps, quaternions = load_quaternions_from_csv(IMU_CSV)


    cfg = AlignConfig(
        stationary_std_threshold = 0.3,
        stationary_window_sec    = 3.0,
        stationary_search_sec    = 30.0,
        trim_end_sec             = None,       # set to e.g. 60.0 to cut at 60 s
        clock_drift_tolerance    = 0.002,      # ~0.2% — catches Arduino drift
        imu_pivot_axis           = (0, 0, 1),  # leg swing ≈ Z axis
        euler_convention         = "zyx",
        euler_degrees            = True,
    )

    stats_angle, stats_euler = run_pipeline(
        mocap_csv      = MOCAP_CSV,
        imu_timestamps = timestamps,
        imu_quaternions= quaternions,
        cfg            = cfg,
        title          = "Leg Swing — Mocap vs IMU",
    )
