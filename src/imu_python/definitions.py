"""Common definitions for this module."""

import errno
from dataclasses import asdict, dataclass
from enum import Enum, IntEnum, StrEnum
from pathlib import Path
from typing import Final

import numpy as np

EREMOTEIO: Final[int] = getattr(errno, "EREMOTEIO", 121)  # EREMOTEIO only on Linux

np.set_printoptions(precision=3, floatmode="fixed", suppress=True)


class IMUUnits(StrEnum):
    """Configuration for the IMU."""

    ACCEL = "m/s^2"
    GYRO = "rad/s"
    MAG = "uT"


# --- Directories ---
ROOT_DIR: Path = Path(__file__).resolve().parents[2]
DATA_DIR: Path = ROOT_DIR / "data"
RECORDINGS_DIR: Path = DATA_DIR / "recordings"
LOG_DIR: Path = DATA_DIR / "logs"
CAL_DIR: Path = DATA_DIR / "calibration"

# data files
IMU_FILENAME_KEY = "imu_data"
MAG_CAL_FILENAME_KEY = "calibration"


class IMUDataFileColumns(StrEnum):
    """Configuration for the IMU data files."""

    TIMESTAMP = "timestamp (sec)"
    ACCEL_X = f"accel_x ({IMUUnits.ACCEL})"
    ACCEL_Y = f"accel_y ({IMUUnits.ACCEL})"
    ACCEL_Z = f"accel_z ({IMUUnits.ACCEL})"
    GYRO_X = f"gyro_x ({IMUUnits.GYRO})"
    GYRO_Y = f"gyro_y ({IMUUnits.GYRO})"
    GYRO_Z = f"gyro_z ({IMUUnits.GYRO})"
    MAG_X = f"mag_x ({IMUUnits.MAG})"
    MAG_Y = f"mag_y ({IMUUnits.MAG})"
    MAG_Z = f"mag_z ({IMUUnits.MAG})"
    POSE_W = "pose_w"
    POSE_X = "pose_x"
    POSE_Y = "pose_y"
    POSE_Z = "pose_z"


# Default encoding
ENCODING: str = "utf-8"

DATE_FORMAT = "%Y-%m-%d_%H-%M-%S"


@dataclass
class LogLevel:
    """Log level."""

    trace: str = "TRACE"
    debug: str = "DEBUG"
    info: str = "INFO"
    success: str = "SUCCESS"
    warning: str = "WARNING"
    error: str = "ERROR"
    critical: str = "CRITICAL"

    def __iter__(self):
        """Iterate over log levels."""
        return iter(asdict(self).values())


DEFAULT_LOG_LEVEL = LogLevel.info
DEFAULT_LOG_FILENAME = "log_file"

I2C_ERROR = EREMOTEIO


@dataclass
class IMUUpdateTime:
    """IMU Frequency."""

    freq_hz: float = 100.0
    period_sec: float = 1.0 / freq_hz


@dataclass
class Delay:
    """Delay."""

    i2c_error_retry = 0.5
    i2c_error_initialize = 6.0
    data_retry = 0.001
    initialization_retry = 0.5


THREAD_JOIN_TIMEOUT = 2.0


@dataclass
class FilterConfig:
    """Orientation filter configuration."""

    gain: float = 0.05
    freq_hz: float = IMUUpdateTime.freq_hz


class I2CBusID(IntEnum):
    """ID number of I2C Buses."""

    bus_1 = 1  # pin 27 (SDA) & 28 (SCL)
    bus_7 = 7  # pin 3 (SDA) & 5 (SCL)


ACCEL_GRAVITY_MSEC2 = 9.80665

ANGULAR_VELOCITY_DPS_TO_RADS = np.deg2rad(1.0)

CLIPPED_GAIN = 0.1
CLIP_MARGIN = 0.95

DEFAULT_QUAT_POSE = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
DEFAULT_ROTATION_MATRIX = np.eye(3, dtype=np.float64)


# Default plot settings
@dataclass
class FigureSettings:
    """Figure settings for matplotlib plots."""

    size: tuple[float, float] = (15, 8.5)  # inches
    alpha: float = 0.8
    legend_loc: str = "upper right"


class PreConfigStepType(Enum):
    """Types of pre-configuration steps for IMU sensors."""

    CALL = 1
    SET = 2


class IMUDeviceID(Enum):
    """Enumeration of IMU device IDs."""

    IMU0 = 0
    IMU1 = 1


@dataclass
class CalibrationMetricThresholds:
    """Metrics for evaluating calibration quality."""

    rel_rms_threshold: float = 0.05
    cov_ratio_threshold: float = 0.6
    condition_threshold: float = 20.0


CAL_SAMPLE_POINTS_REQUIREMENT = 100

MAGNETIC_FIELD_STRENGTH = 1.0  # for normalization


class CalibrationParamNames(StrEnum):
    """Names for calibration parameters in the output JSON."""

    HARD_IRON = "hard_iron"
    INV_SOFT_IRON = "inv_soft_iron"


class IMUNameFormat:
    """Format for IMU names used in file writer and calibration."""

    NAME_FORMAT = "{imu_name}_{imu_index}_{bus_id}"

    def __init__(self, imu_name: str, imu_index: int, bus_id: I2CBusID | None) -> None:
        """Initialize the IMU name format.

        :param imu_name: Name of the IMU sensor model (e.g., "BNO055")
        :param imu_index: Index of the IMU sensor (e.g., 0 for the first sensor)
        :param bus_id: I2C bus ID the sensor is connected to (e.g., 1 or 7)
        """
        self._imu_name: str = imu_name
        self._imu_index: int = imu_index
        self._bus_id: I2CBusID | None = bus_id

    def get_name(self) -> str:
        """Generate name for this IMU instance."""
        return self.NAME_FORMAT.format(
            imu_name=self._imu_name, imu_index=self._imu_index, bus_id=self._bus_id
        )
