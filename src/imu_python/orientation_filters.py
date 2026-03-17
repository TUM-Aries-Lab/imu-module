"""Minimal wrapper around Madgwick filter to estimate orientation."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from ahrs.filters import Madgwick as MadgwickAHRS
from loguru import logger
from numpy.typing import NDArray
from py_imu.fusion.madgwick import Madgwick as MadgwickPyIMU
from py_imu.fusion.quaternion import Vector3D

from imu_python.base_classes import Quaternion
from imu_python.definitions import CLIPPED_GAIN, DEFAULT_QUAT_POSE, IMUUpdateTime


class BaseIMUFilter(ABC):
    """Abstract base class for IMU filters."""

    def __init__(self, gain: float, frequency: float):
        """Initialize the base IMU filter.

        :param gain: filter gain.
        :param frequency: filter frequency.
        """
        self.prev_timestamp: float | None = None
        self.gain = gain
        self.frequency = frequency
        self.quat: NDArray[np.float64] = DEFAULT_QUAT_POSE

    @abstractmethod
    def update(
        self,
        timestamp: float,
        accel: NDArray[np.float64],
        gyro: NDArray[np.float64],
        mag: NDArray[np.float64] | None = None,
        clipped: bool = False,
    ) -> Quaternion:
        """Update IMU filter.

        :param timestamp: current timestamp.
        :param accel: acceleration in m/s^2.
        :param gyro: gyro in m/s^2.
        :param mag: normalized magnetic field values.
        :param clipped: if the IMU signals are saturated.
        """
        pass


class MadgwickFilterAHRS(BaseIMUFilter):
    """Minimal wrapper around Madgwick filter to estimate orientation."""

    def __init__(self, gain: float, frequency: float):
        """Initialize the filter.

        :param gain: float
        :param frequency: float
        """
        super().__init__(gain, frequency)
        self.filter = MadgwickAHRS(gain=gain, frequency=frequency)

    def update(
        self,
        timestamp: float,
        accel: NDArray[np.float64],
        gyro: NDArray[np.float64],
        mag: NDArray[np.float64] | None = None,
        clipped: bool = False,
    ) -> Quaternion:
        """Update orientation quaternion using accelerometer + gyroscope (no magnetometer).

        See ahrs madgwick documentation here:
        https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-angular-rate

        :param timestamp: float
        :param accel: array_like shape (3, ) in m/s^2
        :param gyro: array_like shape (3, ) in rad/s
        :param mag: array_like shape (3, ) in uT
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

        self.quat = (
            self.filter.updateIMU(q=self.quat, gyr=gyro, acc=accel, dt=dt)
            if mag is None
            else self.filter.updateMARG(
                q=self.quat, gyr=gyro, acc=accel, mag=mag, dt=dt
            )
        )
        logger.trace(
            f"Updating filter - "
            f"dt: {dt:.5f}, "
            f"acc: {accel}, "
            f"gyro: {gyro}, "
            f"quat: {self.quat}"
        )

        w, x, y, z = self.quat
        return Quaternion(w=w, x=x, y=y, z=z)


class MadgwickFilterPyImu(BaseIMUFilter):
    """Minimal wrapper around Madgwick filter to estimate orientation."""

    def __init__(self, gain: float, frequency: float):
        """Initialize the filter.

        :param gain: float
        :param frequency: float
        """
        super().__init__(gain, frequency)
        self.filter = MadgwickPyIMU(gain=gain, frequency=frequency)

    def update(
        self,
        timestamp: float,
        accel: NDArray[np.float64],
        gyro: NDArray[np.float64],
        mag: NDArray[np.float64] | None = None,
        clipped: bool = False,
    ) -> Quaternion:
        """Update orientation quaternion using accelerometer + gyroscope (no magnetometer).

        See ahrs madgwick documentation here:
        https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-angular-rate

        :param timestamp: float
        :param accel: array_like shape (3, ) in m/s^2
        :param gyro: array_like shape (3, ) in rad/s
        :param mag: array_like shape (3, ) in uT
        :param clipped: bool indicating if sensor readings are clipped
        :return: Updated orientation quaternion [w, x, y, z]
        """
        if self.prev_timestamp is None:
            dt = 1 / self.frequency
            logger.debug(f"No previous timestamp; using default dt={dt:.4f}s.")
        else:
            dt = timestamp - self.prev_timestamp

            if abs(dt) > 10 / self.frequency:
                logger.warning(
                    f"Large timestamp: "
                    f"dt={dt:.4f}s. "
                    f"Clipping to default dt={1 / self.frequency:.4f}s."
                )
                dt = np.clip(dt, 0, 1 / self.frequency)
            dt = max(0.0, dt)

        self.prev_timestamp = timestamp
        if mag is None:
            self.filter.update(
                gyr=Vector3D(x=gyro[0], y=gyro[1], z=gyro[2]),
                acc=Vector3D(x=accel[0], y=accel[1], z=accel[2]),
                dt=dt,
            )
        else:
            self.filter.update(
                gyr=Vector3D(x=gyro[0], y=gyro[1], z=gyro[2]),
                acc=Vector3D(x=accel[0], y=accel[1], z=accel[2]),
                mag=Vector3D(x=mag[0], y=mag[1], z=mag[2]),
                dt=dt,
            )
        quat = self.filter.q  # x, y, z, w

        if quat is not None:
            logger.debug(
                f"IMU: dt={dt:.4f}s, "
                f"quat(xyzw)={quat.x:.3f}, "
                f"{quat.y:.3f}, "
                f"{quat.z:.3f}, "
                f"{quat.w:.3f}"
            )
            return Quaternion(w=quat.w, x=quat.x, y=quat.y, z=quat.z)

        else:
            logger.debug(f"IMU: dt={dt:.4f}s, quat(xyzw)=None")
            raise ValueError("IMU orientation is None.")
