"""Calibration for magnetometer."""

import argparse
import json
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from loguru import logger
from numpy.typing import NDArray
from scipy import linalg

from imu_python.base_classes import IMUSensorTypes, VectorXYZ
from imu_python.data_handler.data_reader import load_imu_data
from imu_python.definitions import (
    CALI_DIR,
    CALIBRATION_FILENAME_KEY,
    DEFAULT_LOG_LEVEL,
    ENCODING,
    IMU_FILENAME_KEY,
    I2CBusID,
    LogLevel,
)
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from imu_python.utils import setup_logger


class FittingAlgorithm(Enum):
    """Enum for ellipsoid fitting algorithms."""

    LS = "LS"
    MLE = "MLE"


@dataclass
class MagCalibration:
    """Magnetometer calibration using ellipsoid fitting.

    Attributes:
        sensor_name (str): Name of the sensor being calibrated.
        b (NDArray): Hard-iron offset vector.
        A_1 (NDArray): Soft-iron inverse transformation matrix.
        algorithm (str): The ellipsoid fitting algorithm used for calibration.

    """

    sensor_name: str
    b: NDArray
    A_1: NDArray
    algorithm: str

    def __init__(
        self, filepath: Path, algorithm: str = FittingAlgorithm.LS.value
    ) -> None:
        """Initialize the MagCalibration instance.

        :param filepath: Path to the IMU data file.
        :param data: Numpy array of shape (N, 3) with raw magnetometer data
        :param algorithm: The ellipsoid fitting algorithm to use.
        """
        try:
            self.sensor_name = filepath.name.replace("imu_data_", "")
        except AttributeError:
            self.sensor_name = "unknown_sensor"

        self.algorithm = algorithm
        try:
            logger.info(f"Calculating calibration for {filepath}")
            self.calibrate(filepath=filepath, algorithm=algorithm)
        except Exception:
            logger.exception(f"Failed to calculate calibration for {filepath}")

    def calibrate(self, filepath: Path, algorithm: str) -> None:
        """Perform magnetometer calibration from IMU data file.

        :param filepath: Path to the IMU data file.
        :param algorithm: The ellipsoid fitting algorithm to use.
        """
        data_file = load_imu_data(filepath)
        mag_data = data_file.mags
        mag_raw = self._convert_from_list(mag_data)

        if algorithm == FittingAlgorithm.LS.value:
            self.li_griffiths(mag_raw)
        elif algorithm == FittingAlgorithm.MLE.value:
            self.vasconcelos(mag_raw)
        else:
            logger.warning(f"Algorithm {algorithm} not recognized. Using LS.")
            self.li_griffiths(mag_raw)

        logger.info(f"Estimated hard-iron offset: {self.b}")
        logger.info(f"Estimated inverse matrix:\n {self.A_1}")

        mag_cal = self.apply_calibration(mag_raw)
        logger.info("Calibration metrics (on trimmed data):")
        metrics = CalibrationMetrics.evaluate(mag_cal)
        logger.info(metrics)
        if metrics.should_reject():
            logger.warning("Calibration parameters not stored due to poor fitting.")
        else:
            self.store_calibration()
        self.plot_data(raw_data=mag_raw, calibrated_data=mag_cal)

    def _convert_from_list(self, mag_data: list[VectorXYZ]) -> NDArray:
        """Convert magnetometer readings to an (N, 3) array.

        :param mag_data: List of VectorXYZ magnetometer readings.
        :return: Numpy array of shape (N, 3) with magnetometer data
        """
        if not mag_data:
            raise ValueError("mag_data must be a non-empty list of VectorXYZ")

        # Convert list[VectorXYZ] -> NDArray shape (N, 3)
        mag_arr = np.vstack([v.as_array() for v in mag_data])  # shape (N, 3)

        return mag_arr

    def vasconcelos(
        self,
        data: NDArray,
        max_iter: int = 50,
        tol: float = 1e-10,
        damping: float = 1e-3,
    ) -> None:
        """Estimate the ellipsoid parameters with Maximum Likelihood Estimator approach.

        :param data: numpy array of shape (N, 3) with magnetometer readings
        :param max_iter: maximum number of iterations for the optimization
        :param tol: tolerance for convergence
        :param damping: initial damping factor for the Gauss-Newton optimization

        Note: adapted from "Geometric Approach to Strapdown Magnetometer Calibration in Sensor Frame" by
        J. F. Vasconcelos, G. Elkaim, C. Silvestre, P. Oliveira and B. Cardeira
        """
        n = data.shape[0]
        b = data.mean(axis=0)
        # Cov-based inverse sqrt (rough ellipsoid -> sphere mapping)
        C = np.cov((data - b).T)
        w, V = np.linalg.eigh(C)
        w = np.maximum(w, 1e-12)
        inv_sqrt_C = V @ np.diag(1.0 / np.sqrt(w)) @ V.T
        # For uniform points on unit sphere, Cov ~= (1/3) I, so scale by 1/sqrt(3)
        T = (1.0 / np.sqrt(3.0)) * inv_sqrt_C

        last_cost = None

        for _it in range(max_iter):
            U = data - b  # (n,3)
            Vv = (T @ U.T).T  # (n,3) = T u_i
            norms = np.maximum(np.linalg.norm(Vv, axis=1), 1e-12)
            r = norms - 1.0  # residuals

            cost = float(np.dot(r, r))

            # Build Jacobian J: (n,12) for [vec(T); b]
            # dr_i/dT = vec( a_i u_i^T ) where a_i = (T u_i)/||T u_i||
            # dr_i/db = -T^T a_i
            J = np.zeros((n, 12), dtype=float)
            for i in range(n):
                a = Vv[i] / norms[i]  # (3,)
                dT = np.outer(a, U[i])  # (3,3)
                J[i, :9] = dT.reshape(-1, order="F")  # vec in Fortran order
                J[i, 9:] = -(T.T @ a)
            # Damped Gauss-Newton step: (J^T J + lam I) dx = -J^T r
            JTJ = J.T @ J
            g = J.T @ r
            A = JTJ + damping * np.eye(12)
            try:
                dx = np.linalg.solve(A, -g)
            except np.linalg.LinAlgError:
                # Increase damping if near-singular
                damping *= 10.0
                continue

            # Trial update
            dT = dx[:9].reshape(3, 3, order="F")
            db = dx[9:]
            T_new = T + dT
            b_new = b + db

            # Evaluate trial cost (simple acceptance with damping adjustment)
            U_new = data - b_new
            V_new = (T_new @ U_new.T).T
            norms_new = np.linalg.norm(V_new, axis=1)
            norms_new = np.maximum(norms_new, 1e-12)
            r_new = norms_new - 1.0
            cost_new = float(np.dot(r_new, r_new))

            if last_cost is None:
                last_cost = cost

            if cost_new < cost:
                # Accept and reduce damping
                T, b = T_new, b_new
                damping = max(damping / 3.0, 1e-12)
                last_cost = cost_new
                if np.linalg.norm(dx) < tol:
                    break
            else:
                # Reject and increase damping
                damping *= 10.0

        A_1 = self._calculate_a_1(T)

        self.b = b
        self.A_1 = A_1

    def _calculate_a_1(self, T: NDArray) -> NDArray:
        """Calculate the soft-iron correction matrix A_1 from the transformation T.

        :param T: The 3x3 transformation matrix from the MLE fitting.
        :return: The soft-iron correction matrix A_1 to be applied to raw data
        """
        # --- Proposition 2: SVD(T*) -> (R_L, S_L, b) and then A_1 = S_L^{-1} R_L^T ---
        # T* = U S V^T, with S diagonal (positive)
        U_svd, svals, Vt_svd = np.linalg.svd(T)
        V_svd = Vt_svd.T

        # Enforce V in SO(3) (det +1) as in the paper's convention
        if np.linalg.det(V_svd) < 0:
            V_svd[:, -1] *= -1.0
            U_svd[:, -1] *= -1.0
            # svals remain positive

        # From Proposition 2: R_L = V, S_L = S^{-1}, b = b_T
        # Therefore A_1 = S_L^{-1} R_L^T = S * V^T
        return np.diag(svals) @ V_svd.T

    def li_griffiths(self, data: NDArray, F: float = 1.0) -> None:
        """Estimate ellipsoid parameters using the least squares method.

        :param data: numpy array of shape (N, 3) with magnetometer readings
        :param F: expected magnetic field strength (default 1 for normalized data)

        Note: copied from https://github.com/nliaudat/magnetometer_calibration/
        """
        s = data.T

        D = np.array(
            [
                s[0] ** 2.0,
                s[1] ** 2.0,
                s[2] ** 2.0,
                2.0 * s[1] * s[2],
                2.0 * s[0] * s[2],
                2.0 * s[0] * s[1],
                2.0 * s[0],
                2.0 * s[1],
                2.0 * s[2],
                np.ones_like(s[0]),
            ]
        )

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6, :6]
        S_12 = S[:6, 6:]
        S_21 = S[6:, :6]
        S_22 = S[6:, 6:]

        # C (Eq. 8, k=4)
        C = np.array(
            [
                [-1, 1, 1, 0, 0, 0],
                [1, -1, 1, 0, 0, 0],
                [1, 1, -1, 0, 0, 0],
                [0, 0, 0, -4, 0, 0],
                [0, 0, 0, 0, -4, 0],
                [0, 0, 0, 0, 0, -4],
            ]
        )

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C), S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)
        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0:
            v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-linalg.inv(S_22), S_21), v_1)

        # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
        M = np.array(
            [
                [v_1[0], v_1[5], v_1[4]],
                [v_1[5], v_1[1], v_1[3]],
                [v_1[4], v_1[3], v_1[2]],
            ]
        )
        n = np.array([[v_2[0]], [v_2[1]], [v_2[2]]])
        d = v_2[3]

        M_1 = linalg.inv(M)

        self.b = -np.dot(M_1, n)
        self.b = self.b.reshape(
            3,
        )
        self.A_1 = np.real(
            F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M)
        )

    def store_calibration(self, filename: str = CALIBRATION_FILENAME_KEY) -> None:
        """Store the calibration configuration in a JSON file.

        :param filename: Base name for the calibration file (without extension). Default is CALIBRATION_FILENAME_KEY.

        Creates `calibration.json` in `CALI_DIR` if it doesn't exist, overwrites
        the sensor entry if present, or adds a new entry otherwise.
        """
        cal_file = CALI_DIR / f"{filename}.json"
        CALI_DIR.mkdir(parents=True, exist_ok=True)

        # Load existing data if available
        data: dict = {}
        if cal_file.exists():
            try:
                with cal_file.open("r", encoding=ENCODING) as fh:
                    data = json.load(fh)
            except Exception:
                logger.warning(
                    f"Could not read existing calibration file {cal_file}, overwriting"
                )
                data = {}

        # Store arrays as plain lists
        data[self.sensor_name] = {
            "b": list(np.array(self.b).flatten()),
            "A_1": np.array(self.A_1).tolist(),
        }

        # Write back
        with cal_file.open("w", encoding=ENCODING) as fh:
            json.dump(data, fh, indent=2)

        logger.info(f"Calibration for {self.sensor_name} stored in {cal_file}")

    def plot_data(self, raw_data, calibrated_data, max_points=100):
        """Make an interactive 3D comparison plot using Plotly.

        :param raw_data: Numpy array of shape (N, 3) with raw magnetometer data
        :param calibrated_data: Numpy array of shape (N, 3) with calibrated magnetometer data
        :param max_points: Maximum number of points to plot for interactivity (default 100)
        """
        # Sample data
        if len(raw_data) > max_points:
            idx = np.random.choice(len(raw_data), max_points, replace=False)
            raw_sample = raw_data[idx]
            cal_sample = calibrated_data[idx]
        else:
            raw_sample = raw_data
            cal_sample = calibrated_data

        # Create figure with dropdown
        fig = go.Figure()

        # Add raw data trace
        fig.add_trace(
            go.Scatter3d(
                x=raw_sample[:, 0],
                y=raw_sample[:, 1],
                z=raw_sample[:, 2],
                mode="markers",
                marker=dict(
                    size=3, color=raw_sample[:, 2], colorscale="Viridis", opacity=0.7
                ),
                name="Raw Data",
                visible=True,
            )
        )

        # Add calibrated data trace
        norms = np.linalg.norm(cal_sample, axis=1)
        custom_colorscale = ["red", "green", "red"]
        fig.add_trace(
            go.Scatter3d(
                x=cal_sample[:, 0],
                y=cal_sample[:, 1],
                z=cal_sample[:, 2],
                mode="markers",
                marker=dict(
                    size=3,
                    color=norms,
                    colorscale=custom_colorscale,
                    cmin=0.5,
                    cmax=1.5,
                    opacity=0.7,
                ),
                name="Calibrated Data",
                visible=False,
            )
        )

        # Add unit sphere
        u = np.linspace(0, 2 * np.pi, 25)
        v = np.linspace(0, np.pi, 25)
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))

        fig.add_trace(
            go.Surface(
                x=x,
                y=y,
                z=z,
                opacity=0.2,
                colorscale=[[0, "gray"], [1, "gray"]],
                showscale=False,
                name="Unit Sphere",
                visible=False,
            )
        )

        # Create buttons for interactivity
        fig.update_layout(
            updatemenus=[
                dict(
                    buttons=list(
                        [
                            dict(
                                args=[
                                    {"visible": [True, False, False]},
                                    {"title": "Raw Data Only"},
                                ],
                                label="Raw Data",
                                method="update",
                            ),
                            dict(
                                args=[
                                    {"visible": [False, True, True]},
                                    {"title": "Calibrated Data with Unit Sphere"},
                                ],
                                label="Calibrated",
                                method="update",
                            ),
                            dict(
                                args=[
                                    {"visible": [True, True, False]},
                                    {"title": "Both Datasets"},
                                ],
                                label="Both",
                                method="update",
                            ),
                        ]
                    ),
                    direction="down",
                    pad={"r": 10, "t": 10},
                    showactive=True,
                    x=0.1,
                    xanchor="left",
                    y=1.1,
                    yanchor="top",
                )
            ],
            title=f"Magnetometer Calibration for {self.sensor_name} using {self.algorithm} algorithm",
            scene=dict(
                xaxis_title="X",
                yaxis_title="Y",
                zaxis_title="Z",
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5)),
            ),
            width=800,
            height=600,
        )
        fig.show()

        return fig

    def apply_calibration(self, data) -> NDArray:
        """Apply calibration to magnetometer data.

        :param data: Numpy array of shape (N, 3) with raw magnetometer data
        :return: Numpy array of calibrated data
        """
        # Subtract hard iron bias and apply soft iron correction
        data_corrected = (data - self.b.T) @ self.A_1.T

        return data_corrected


@dataclass
class CalibrationMetrics:
    """Metrics to evaluate calibration quality."""

    r_hat: float
    rel_rms: float
    rel_radius_std: float
    coverage_ratio: float
    condition_number: float

    def __repr__(self) -> str:
        """Return a string representation of calibration metrics."""
        return (
            f"CalibrationMetrics(r_hat={self.r_hat:.3f}, "
            f"rel_rms={self.rel_rms:.3f}, "
            f"rel_radius_std={self.rel_radius_std:.3f}, "
            f"coverage_ratio={self.coverage_ratio:.3f}, "
            f"condition_number={self.condition_number:.1f})"
        )

    @classmethod
    def evaluate(cls, mag_cal: NDArray) -> "CalibrationMetrics":
        """Evaluate the calibration quality based on residuals from expected magnetic field strength.

        :param mag_cal: a set of calibrated magnetometer readings as a numpy array of shape (N, 3)
        :return: a dictionary with calibration metrics
        """
        # Norms of calibrated data
        norms = np.linalg.norm(mag_cal, axis=1, keepdims=True)

        # Estimated Earth field magnitude
        r_hat = np.mean(norms)

        # Residuals (likelihood error)
        residuals = norms - r_hat

        unit = mag_cal / norms

        # Covariance
        C = np.cov(unit.T)

        # Eigenvalues
        eigvals = np.linalg.eigvalsh(C)
        eigvals = np.sort(eigvals)

        lambda_min = eigvals[0]
        lambda_max = eigvals[-1]
        coverage_ratio = lambda_min / lambda_max

        condition = lambda_max / lambda_min

        # Metrics
        rms = np.sqrt(np.mean(residuals**2))
        rel_rms = rms / r_hat
        std_radius = np.std(norms) / r_hat

        return CalibrationMetrics(
            r_hat=r_hat,
            rel_rms=rel_rms,
            rel_radius_std=std_radius,
            coverage_ratio=coverage_ratio,
            condition_number=condition,
        )

    def should_reject(
        self,
        rel_rms_threshold: float = 0.05,
        cov_ratio_threshold: float = 0.6,
        condition_threshold: float = 1000,
    ) -> bool:
        """Determine if calibration should be rejected based on thresholds."""
        if self.rel_rms > rel_rms_threshold:
            logger.warning(
                f"Relative RMS error {self.rel_rms:.3f} exceeds threshold {rel_rms_threshold}"
            )
            return True
        if self.coverage_ratio < cov_ratio_threshold:
            logger.warning(
                f"Coverage ratio {self.coverage_ratio:.3f} is too low, indicating poor spatial coverage"
            )
            return True
        if self.condition_number > condition_threshold:
            logger.warning(
                f"Condition number {self.condition_number:.1f} is too high, indicating potential numerical instability"
            )
            return True
        return False


def has_magnetometer(manager: IMUManager) -> bool:
    """Check if the IMUManager has a magnetometer sensor.

    :param manager: IMUManager instance
    :return: True if magnetometer is present, False otherwise
    """
    return manager.imu_wrapper._read_plans[IMUSensorTypes.mag] is not None


def collect_calibration_data() -> None:
    """Collect magnetometer calibration data of all connected IMUs and save to files."""
    sensor_managers_l = IMUFactory.detect_and_create(
        i2c_id=I2CBusID.bus_1, log_data=True
    )
    sensor_managers_r = IMUFactory.detect_and_create(
        i2c_id=I2CBusID.bus_7, log_data=True
    )
    for manager in sensor_managers_l:
        if has_magnetometer(manager):
            manager.file_writer.calibration_mode = True
            manager.start()
    for manager in sensor_managers_r:
        if has_magnetometer(manager):
            manager.file_writer.calibration_mode = True
            manager.start()

    logger.info("Started magnetometer calibration. Press ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        for manager in sensor_managers_l:
            manager.stop()
        for manager in sensor_managers_r:
            manager.stop()

        time.sleep(1.0)  # wait for file write


def main(
    log_level: str,
    stderr_level: str,
    filepath: Path | None = None,
    algorithm: str = FittingAlgorithm.LS.value,
):  # pragma: no cover
    """Run magnetometer calibration.

    :param log_level: Log level for logger.
    :param stderr_level: Std err level for logger.
    :param filepath: Optional path to IMU data file for calibration.
    :param algorithm: The ellipsoid fitting algorithm to use.
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    if filepath:
        MagCalibration(filepath=filepath, algorithm=algorithm)
    else:
        collect_calibration_data()

        # Iterate through files in the calibration directory and calculate calibration
        for cal_file in CALI_DIR.iterdir():
            if cal_file.is_file() and cal_file.name.startswith(IMU_FILENAME_KEY):
                MagCalibration(filepath=cal_file, algorithm=algorithm)


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Run the pipeline.")
    parser.add_argument(
        "--log-level",
        "-l",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the log level.",
        type=str,
    )
    parser.add_argument(
        "--stderr-level",
        "-s",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the std err level.",
        type=str,
    )
    parser.add_argument(
        "--filepath",
        "-f",
        type=Path,
        help="Path to the IMU data file for calibration.",
    )
    parser.add_argument(
        "--algorithm",
        "-a",
        type=str,
        default=FittingAlgorithm.LS.value,
        choices=list(a.value for a in FittingAlgorithm),
        help="The ellipsoid fitting algorithm to use",
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        filepath=args.filepath,
        algorithm=args.algorithm,
    )
