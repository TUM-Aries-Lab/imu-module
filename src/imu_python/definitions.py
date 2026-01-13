"""Common definitions for this module."""

from dataclasses import asdict, dataclass
from enum import Enum, IntEnum
from pathlib import Path

import numpy as np

np.set_printoptions(precision=3, floatmode="fixed", suppress=True)


class IMUUnits(Enum):
    """Configuration for the IMU."""

    ACCEL = "m/s^2"
    GYRO = "rad/s"
    MAG = "uT"


# --- Directories ---
ROOT_DIR: Path = Path("src").parent
DATA_DIR: Path = ROOT_DIR / "data"
RECORDINGS_DIR: Path = DATA_DIR / "recordings"
LOG_DIR: Path = DATA_DIR / "logs"

# data files
IMU_FILENAME_KEY = "imu_data"


class IMUDataFileColumns(Enum):
    """Configuration for the IMU data files."""

    TIMESTAMP = "timestamp (sec)"
    ACCEL_X = f"accel_x ({IMUUnits.ACCEL.value})"
    ACCEL_Y = f"accel_y ({IMUUnits.ACCEL.value})"
    ACCEL_Z = f"accel_z ({IMUUnits.ACCEL.value})"
    GYRO_X = f"gyro_x ({IMUUnits.GYRO.value})"
    GYRO_Y = f"gyro_y ({IMUUnits.GYRO.value})"
    GYRO_Z = f"gyro_z ({IMUUnits.GYRO.value})"
    MAG_X = f"mag_x ({IMUUnits.MAG.value})"
    MAG_Y = f"mag_y ({IMUUnits.MAG.value})"
    MAG_Z = f"mag_z ({IMUUnits.MAG.value})"
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

I2C_ERROR = 121


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

    gain = 0.05
    freq_hz = IMUUpdateTime.freq_hz


class I2CBusID(IntEnum):
    """ID number of I2C Buses."""

    bus_1 = 1  # pin 27 (SDA) & 28 (SCL)
    bus_7 = 7  # pin 3 (SDA) & 5 (SCL)


ACCEL_GRAVITY_MSEC2 = 9.80665

ANGULAR_VELOCITY_DPS_TO_RADS = np.deg2rad(1.0)

CLIPPED_GAIN = 0.1
CLIP_MARGIN = 0.95

DEFAULT_QUAT_POSE = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


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
