"""Minimal wrapper around Madgwick filter to estimate orientation."""

from __future__ import annotations

import numpy as np
from ahrs.filters import Madgwick
from loguru import logger
from numpy.typing import NDArray

from imu_python.base_classes import Quaternion
from imu_python.definitions import CLIPPED_GAIN, DEFAULT_QUAT_POSE, IMUUpdateTime


class OrientationFilter:
    """Minimal wrapper around Madgwick filter to estimate orientation."""

    def __init__(self, gain: float, frequency: float):
        """Initialize the filter.

        :param gain: float
        :param frequency: float
        """
        self.prev_timestamp: float | None = None
        self.gain = gain
        self.filter = Madgwick(gain=gain, frequency=frequency)
        self.quat: NDArray[np.float64] = DEFAULT_QUAT_POSE

    def update(
        self,
        timestamp: float,
        accel: NDArray[np.float64],
        gyro: NDArray[np.float64],
        clipped: bool = False,
    ) -> Quaternion:
        """Update orientation quaternion using accelerometer + gyroscope (no magnetometer).

        See ahrs madgwick documentation here:
        https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-angular-rate

        :param timestamp: float
        :param accel: array_like shape (3, ) in m/s^2
        :param gyro: array_like shape (3, ) in rad/s
        :param clipped: bool indicating if sensor readings are clipped
        :return: Updated orientation quaternion [w, x, y, z]
        """
        if clipped:
            self.filter.gain = CLIPPED_GAIN
        else:
            self.filter.gain = self.gain
        if self.prev_timestamp is None:
            dt = IMUUpdateTime.period_sec
            self.prev_timestamp = timestamp
        else:
            dt = timestamp - self.prev_timestamp
            self.prev_timestamp = timestamp

        self.quat = self.filter.updateIMU(q=self.quat, gyr=gyro, acc=accel, dt=dt)
        logger.trace(
            f"Updating filter - "
            f"dt: {dt:.5f}, "
            f"acc: {accel}, "
            f"gyro: {gyro}, "
            f"quat: {self.quat}"
        )

        w, x, y, z = self.quat
        return Quaternion(w=w, x=x, y=y, z=z)
