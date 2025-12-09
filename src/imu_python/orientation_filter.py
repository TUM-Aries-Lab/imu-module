"""Minimal wrapper around Madgwick filter to estimate orientation."""

from __future__ import annotations

import time

import numpy as np
from ahrs.filters import Madgwick
from numpy.typing import NDArray

from imu_python.base_classes import Quaternion
from imu_python.definitions import IMUUpdateTime


class OrientationFilter:
    """Minimal wrapper around Madgwick filter to estimate orientation."""

    def __init__(self, gain: float, frequency: float):
        """Initialize the filter.

        :param gain: float
        :param frequency: float
        """
        self.prev_timestamp: float | None = None
        self.filter = Madgwick(gain=gain, frequency=frequency)
        self.pose: NDArray[np.float64] = np.array(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float64
        )

    def update(
        self, accel: NDArray[np.float64], gyro: NDArray[np.float64]
    ) -> Quaternion:
        """Update orientation quaternion using accelerometer + gyroscope (no magnetometer).

        See ahrs madgwick documentation here:
        https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-angular-rate

        :param accel: array_like shape (3, ) in m/s^2
        :param gyro: array_like shape (3, ) in rad/s
        :return: Updated orientation quaternion [w, x, y, z]
        """
        if self.prev_timestamp is None:
            dt = IMUUpdateTime.period_sec
            self.prev_timestamp = time.monotonic()
        else:
            now = time.monotonic()
            dt = now - self.prev_timestamp
            self.prev_timestamp = now
        self.pose = self.filter.updateIMU(q=self.pose, gyr=gyro, acc=accel, dt=dt)
        quat = Quaternion(
            w=self.pose[0], x=self.pose[1], y=self.pose[2], z=self.pose[3]
        )
        return quat
