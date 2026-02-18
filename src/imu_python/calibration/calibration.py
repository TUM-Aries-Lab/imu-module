"""Calibration for magnetometer."""

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from loguru import logger
from numpy.typing import NDArray

from imu_python.base_classes import IMUSensorTypes, VectorXYZ
from imu_python.calibration.ellipsoid_fitting import (
    FittingAlgorithmNames,
    LsFitting,
    MleFitting,
)
from imu_python.data_handler.data_reader import load_imu_data
from imu_python.definitions import (
    CAL_DIR,
    CAL_SAMPLE_POINTS_REQUIREMENT,
    DEFAULT_LOG_LEVEL,
    ENCODING,
    IMU_FILENAME_KEY,
    MAG_CAL_FILENAME_KEY,
    UNKNOWN_SENSOR_NAME,
    CalibrationMetricThresholds,
    CalibrationParamNames,
    I2CBusID,
    LogLevel,
)
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from imu_python.utils import setup_logger


@dataclass
class MagCalibration:
    """Magnetometer calibration using ellipsoid fitting.

    Attributes:
        sensor_name (str): Name of the sensor being calibrated.
        hard_iron (NDArray): Hard-iron offset vector.
        inv_soft_iron (NDArray): Soft-iron inverse transformation matrix.
        algorithm (str): The ellipsoid fitting algorithm used for calibration.
        cal_filepath(Path): Path to the calibration file where parameters are stored.

    """

    sensor_name: str
    hard_iron: NDArray
    inv_soft_iron: NDArray
    algorithm: str
    cal_filepath: Path

    def __init__(
        self,
        algorithm: str = FittingAlgorithmNames.LS,
        filepath: Path | None = None,
        data: NDArray | None = None,
        sensor_name: str | None = None,
        cal_folder: Path = CAL_DIR,
    ) -> None:
        """Initialize the MagCalibration instance.

        :param algorithm: The ellipsoid fitting algorithm to use.
        :param filepath: Path to the IMU data file.
        :param data: Numpy array of shape (N, 3) with raw magnetometer data.
        :param sensor_name: Name of the sensor being calibrated.
        :param cal_folder: Folder path for storing calibration files. Default is CAL_DIR.
        """
        if sensor_name is None:
            if filepath is not None:
                self.sensor_name = filepath.name.replace("imu_data_", "").replace(
                    ".csv", ""
                )
            else:
                self.sensor_name = UNKNOWN_SENSOR_NAME
        else:
            self.sensor_name = sensor_name

        self.algorithm = algorithm
        self.cal_filepath = cal_folder / f"{MAG_CAL_FILENAME_KEY}.json"

        try:
            logger.info(f"Calculating calibration for {filepath}")
            self.calibrate(algorithm=algorithm, filepath=filepath, data=data)
        except Exception:
            logger.exception(f"Failed to calculate calibration for {filepath}")

    def calibrate(
        self, algorithm: str, filepath: Path | None = None, data: NDArray | None = None
    ) -> None:
        """Perform magnetometer calibration from IMU data file.

        :param filepath: Path to the IMU data file.
        :param algorithm: The ellipsoid fitting algorithm to use.
        :param data: Numpy array of shape (N, 3) with raw magnetometer data
        """
        if filepath is None:
            if data is None:
                raise ValueError("Calibration requires either a filepath or raw data")
            else:
                mag_raw = data
        else:
            data_file = load_imu_data(filepath)
            mag_data = data_file.mags
            mag_raw = self._convert_from_list(mag_data)

        if mag_raw.shape[1] != 3 or mag_raw.shape[0] < CAL_SAMPLE_POINTS_REQUIREMENT:
            raise ValueError(
                f"Input data must be an (N, 3) array with at least {CAL_SAMPLE_POINTS_REQUIREMENT} samples"
            )

        if algorithm == FittingAlgorithmNames.LS:
            ls = LsFitting(data=mag_raw)
            self.hard_iron = ls.hard_iron
            self.inv_soft_iron = ls.inv_soft_iron
        elif algorithm == FittingAlgorithmNames.MLE:
            mle = MleFitting(data=mag_raw)
            self.hard_iron = mle.hard_iron
            self.inv_soft_iron = mle.inv_soft_iron
        else:
            logger.warning(f"Algorithm {algorithm} not recognized. Using LS.")
            ls = LsFitting(data=mag_raw)
            self.hard_iron = ls.hard_iron
            self.inv_soft_iron = ls.inv_soft_iron

        logger.info(f"Estimated hard-iron offset: {self.hard_iron}")
        logger.info(f"Estimated inverse matrix:\n {self.inv_soft_iron}")

        mag_cal = self.apply_calibration(mag_raw)
        logger.info("Calibration metrics (on trimmed data):")
        metrics = CalibrationMetrics.evaluate(mag_cal)
        logger.info(metrics)
        if self.sensor_name != UNKNOWN_SENSOR_NAME:
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

    def store_calibration(self) -> None:
        """Store the calibration configuration in a JSON file.

        :param filename: Base name for the calibration file (without extension). Default is MAG_CAL_FILENAME_KEY.

        Creates `MAG_CAL_FILENAME_KEY` in `CAL_DIR` if it doesn't exist, overwrites
        the sensor entry if present, or adds a new entry otherwise.
        """
        self.cal_filepath.parent.mkdir(parents=True, exist_ok=True)

        # Load existing data if available
        data: dict = {}
        if self.cal_filepath.exists():
            try:
                with self.cal_filepath.open("r", encoding=ENCODING) as fh:
                    data = json.load(fh)
            except Exception:
                logger.warning(
                    f"Could not read existing calibration file {self.cal_filepath}, overwriting"
                )
                data = {}

        # Store arrays as plain lists
        data[self.sensor_name] = {
            CalibrationParamNames.HARD_IRON: list(np.array(self.hard_iron).flatten()),
            CalibrationParamNames.INV_SOFT_IRON: np.array(self.inv_soft_iron).tolist(),
        }

        # Write back
        with self.cal_filepath.open("w", encoding=ENCODING) as fh:
            json.dump(data, fh, indent=2)

        logger.info(f"Calibration for {self.sensor_name} stored in {self.cal_filepath}")

    def plot_data(
        self, raw_data, calibrated_data, max_points=100
    ) -> None:  # pragma: no cover
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

    def apply_calibration(self, data) -> NDArray:
        """Apply calibration to magnetometer data.

        :param data: Numpy array of shape (N, 3) with raw magnetometer data
        :return: Numpy array of calibrated data
        """
        # Subtract hard iron bias and apply soft iron correction
        data_corrected = (data - self.hard_iron) @ self.inv_soft_iron.T

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
    ) -> bool:
        """Determine if calibration should be rejected based on thresholds."""
        thresholds = CalibrationMetricThresholds()
        if self.rel_rms > thresholds.rel_rms_threshold:
            logger.warning(
                f"Relative RMS error {self.rel_rms:.3f} exceeds threshold {thresholds.rel_rms_threshold}, indicating poor fit"
            )
            return True
        if self.coverage_ratio < thresholds.cov_ratio_threshold:
            logger.warning(
                f"Coverage ratio {self.coverage_ratio:.3f} lower than threshold {thresholds.cov_ratio_threshold}, indicating poor spatial coverage"
            )
            return True
        if self.condition_number > thresholds.condition_threshold:
            logger.warning(
                f"Condition number {self.condition_number:.1f} exceeds threshold {thresholds.condition_threshold}, indicating potential numerical instability"
            )
            return True
        return False


def has_magnetometer(manager: IMUManager) -> bool:  # pragma: no cover
    """Check if the IMUManager has a magnetometer sensor.

    :param manager: IMUManager instance
    :return: True if magnetometer is present, False otherwise
    """
    return manager.imu_wrapper._read_plans[IMUSensorTypes.mag] is not None


def collect_calibration_data() -> None:  # pragma: no cover
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


def load_calibration(
    sensor_name: str, folder: Path = CAL_DIR, filename: str = MAG_CAL_FILENAME_KEY
) -> tuple[NDArray, NDArray] | None:
    """Load calibration parameters for a given sensor name from the calibration file.

    :param sensor_name: Name of the sensor to load calibration for.
    :param folder: Folder path where calibration files are stored. Default is CAL_DIR.
    :param filename: Base name of the calibration file (without extension). Default is MAG_CAL_FILENAME_KEY.
    :return: Tuple of (hard_iron, inv_soft_iron) if successful, None if file or sensor entry is missing or malformed.
    """
    cal_file = folder / f"{filename}.json"
    if not cal_file.exists():
        logger.warning(f"Calibration file {filename} does not exist.")
        return None

    try:
        with cal_file.open("r", encoding=ENCODING) as fh:
            data = json.load(fh)
    except Exception:
        logger.warning(f"Could not read calibration file {cal_file}.")
        return None

    if sensor_name not in data:
        logger.warning(f"No calibration found for sensor {sensor_name} in {cal_file}.")
        return None

    try:
        hard_iron = np.array(data[sensor_name][CalibrationParamNames.HARD_IRON])
        inv_soft_iron = np.array(data[sensor_name][CalibrationParamNames.INV_SOFT_IRON])
        return hard_iron, inv_soft_iron
    except Exception:
        logger.warning(
            f"Calibration parameters for sensor {sensor_name} are malformed in {cal_file}."
        )
        return None


def main(
    log_level: str,
    stderr_level: str,
    algorithm: str,
    filepath: Path | None = None,
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
        for cal_file in CAL_DIR.iterdir():
            if cal_file.is_file() and cal_file.name.startswith(IMU_FILENAME_KEY):
                MagCalibration(filepath=cal_file, algorithm=algorithm)


if __name__ == "__main__":  # pragma: no cover
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
        default=FittingAlgorithmNames.LS,
        choices=list(a for a in FittingAlgorithmNames),
        help="The ellipsoid fitting algorithm to use",
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        filepath=args.filepath,
        algorithm=args.algorithm,
    )
