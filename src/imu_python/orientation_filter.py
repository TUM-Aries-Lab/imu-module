"""Minimal wrapper around Madgwick filter to estimate orientation."""

from __future__ import annotations

import time

import numpy as np
from ahrs.filters import Madgwick
from numpy.typing import NDArray

from imu_python.definitions import IMUUpdateTime


class OrientationFilter:
    """Minimal wrapper around Madgwick filter to estimate orientation."""

    def __init__(self, gain: float = 0.1, frequency: float = IMUUpdateTime.freq_hz):
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
    ) -> NDArray[np.float64]:
        """Update orientation quaternion using accelerometer + gyroscope (no magnetometer).

        Parameters
        ----------
        accel : array_like shape (3,)
            Acceleration vector [ax, ay, az] in m/s^2

        gyro : array_like shape (3,)
            Gyroscope vector [gx, gy, gz] in rad/s

        Returns
        -------
        Quaternion : NDArray[np.float64]
            Updated orientation quaternion [w, x, y, z]

        """
        if self.prev_timestamp is None:
            dt = IMUUpdateTime.period_sec
            self.prev_timestamp = time.monotonic()
        else:
            now = time.monotonic()
            dt = now - self.prev_timestamp
            self.prev_timestamp = now
        self.pose = self.filter.updateIMU(q=self.pose, gyr=gyro, acc=accel, dt=dt)
        return self.pose
