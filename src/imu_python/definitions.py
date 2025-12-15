"""Common definitions for this module."""

from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path

import numpy as np

np.set_printoptions(precision=3, floatmode="fixed", suppress=True)


# --- Directories ---
ROOT_DIR: Path = Path("src").parent
DATA_DIR: Path = ROOT_DIR / "data"
RECORDINGS_DIR: Path = DATA_DIR / "recordings"
LOG_DIR: Path = DATA_DIR / "logs"

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

    left = 1  # pin 27 (SDA) & 28 (SCL)
    right = 7  # pin 3 (SDA) & 5 (SCL)


ACCEL_GRAVITY_MSEC2 = 9.80665

DEFAULT_QUAT_POSE = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
