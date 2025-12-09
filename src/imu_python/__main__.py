"""Sample doc string."""

import argparse
import time

from imu_python.definitions import DEFAULT_LOG_LEVEL, IMUUpdateTime, LogLevel
from imu_python.factory import IMUFactory
from imu_python.utils import setup_logger


def main(log_level: str, stderr_level: str, freq: float) -> None:  # pragma: no cover
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :param freq: The frequency to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    sensor_managers = IMUFactory.detect_and_create()
    for manager in sensor_managers:
        manager.start()

    try:
        while True:
            for manager in sensor_managers:
                manager.get_data()
            time.sleep(1 / freq)
    except KeyboardInterrupt:
        for manager in sensor_managers:
            manager.stop()


if __name__ == "__main__":  # pragma: no cover
    parser = argparse.ArgumentParser("Run the pipeline.")
    parser.add_argument(
        "--log-level",
        "-l",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the log level.",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--stderr-level",
        "-s",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the std err level.",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--freq",
        "-f",
        type=float,
        help="Frequency to use.",
        default=IMUUpdateTime.freq_hz,
    )
    args = parser.parse_args()

    main(log_level=args.log_level, stderr_level=args.stderr_level, freq=args.freq)
