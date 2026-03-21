"""Manager for a sensor object."""

import os
import threading
import time
from pathlib import Path
from threading import Lock

from loguru import logger
from numpy.typing import NDArray

from imu_python.base_classes import IMUData, IMUDeviceData
from imu_python.data_handler.data_writer import IMUFileWriter
from imu_python.definitions import (
    ACCEL_GRAVITY_MSEC2,
    ANGULAR_VELOCITY_DPS_TO_RADS,
    CORE_COUNT,
    I2C_ERROR,
    THREAD_JOIN_TIMEOUT,
    Delay,
    I2CBusID,
)
from imu_python.wrapper import IMUWrapper


class IMUManager:
    """Thread-safe IMU data manager."""

    def __init__(
        self,
        imu_wrapper: IMUWrapper,
        i2c_lock: Lock,
        log_data: bool = False,
        calibration_mode: bool = False,
    ) -> None:
        """Initialize the IMU manager.

        :param imu_wrapper: IMUWrapper instance to manage
        :param i2c_lock: Shared I2C lock for all managers on the bus
        :param log_data: Flag to record the IMU data
        :param calibration_mode: Flag to log data to the calibration data folder
        """
        self.imu_wrapper: IMUWrapper = imu_wrapper
        self.log_data: bool = log_data
        self.i2c_id: I2CBusID | None = imu_wrapper.i2c_bus_id
        self.accel_range_m_s2: float = (
            imu_wrapper.config.accel_range_g * ACCEL_GRAVITY_MSEC2
        )
        self.gyro_range_rad_s: float = (
            imu_wrapper.config.gyro_range_dps * ANGULAR_VELOCITY_DPS_TO_RADS
        )
        self.imu_descriptor = imu_wrapper.imu_descriptor
        self.running: bool = False
        self.data_lock: Lock = threading.Lock()
        self.i2c_lock: Lock = i2c_lock
        self.core_id: int | None = None
        self.latest_data: IMUData | None = None
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        if log_data:
            self.file_writer: IMUFileWriter = IMUFileWriter(
                bus_id=self.i2c_id, imu_descriptor=self.imu_descriptor
            )
            self.file_writer.calibration_mode = calibration_mode
            self.IMUData_log: list[IMUData] = []

    def __repr__(self) -> str:
        """Return string representation of the sensor manager."""
        return (
            f"IMUManager(name:{self.imu_descriptor.name}, index:{self.imu_descriptor.index} "
            f"bus:{self.i2c_id}, "
            f"addr:{[hex(a) for d in self.imu_wrapper.config.devices.values() for a in d.addresses]}"
        )

    def start(self):
        """Start the sensor manager."""
        self._initialize_sensor()
        self.running = True
        self.thread.start()

    def _loop(self) -> None:
        """Read data from the IMU wrapper and update the latest data."""
        if (
            self.core_id is not None and self.core_id < CORE_COUNT
        ):  # core ID starts from 0
            os.sched_setaffinity(0, {self.core_id})
            logger.info(
                f"{self.imu_descriptor} running on core {os.sched_getaffinity(0)}"
            )
        while self.running:
            try:
                # Attempt to read all sensor data
                with self.i2c_lock:
                    data = self.imu_wrapper.get_imu_data()
                # Ensure new data
                if self._acc_gyro_are_fresh(data):
                    logger.debug(
                        f"reading from: {self.imu_descriptor.name} {self.imu_descriptor.index} new data:{data}"
                    )
                    with self.data_lock:
                        timestamp = time.monotonic()
                        pose_quat = self.imu_wrapper.filter.update(
                            timestamp=timestamp,
                            accel=data.accel.as_array(),
                            gyro=data.gyro.as_array(),
                            mag=data.mag.as_array()
                            if data.mag is not None and self._mag_is_fresh(data)
                            else None,
                            clipped=(
                                data.accel.is_clipped(
                                    sensor_range=self.accel_range_m_s2,
                                    sensor_type="Accel",
                                )
                                or data.gyro.is_clipped(
                                    sensor_range=self.gyro_range_rad_s,
                                    sensor_type="Gyro",
                                )
                            ),
                        )
                        self.latest_data = IMUData(
                            timestamp=timestamp,
                            quat=pose_quat,
                            device_data=data,
                        )
                        if self.log_data:
                            self.IMUData_log.append(self.latest_data)

            except OSError as err:
                # Catch I2C remote I/O errors
                self.imu_wrapper.started = False
                self.latest_data = None
                if err.errno == I2C_ERROR:
                    logger.error("I2C error detected. Reinitializing sensor...")
                    time.sleep(Delay.i2c_error_retry)  # short delay before retry
                    self._initialize_sensor()
                else:
                    # Reraise unexpected OS errors
                    logger.warning(f"Unexpected OS error: {err}")
                    raise
            except Exception as err:
                logger.error(f"Error reading IMU data: {err}")
                self.latest_data = None

    def get_data(self) -> IMUData | None:
        """Return sensor data as a IMUData object or None if the manager is not running, IMU hardware is disconnected, or IMU configuration is incorrect.

        :return: IMUData object or None
        """
        data = self.latest_data

        with self.data_lock:
            logger.debug(f"I2C Bus: {self}, data: {data}")
            return data

    def set_core_affinity(self, core_id: int) -> None:
        """Set CPU core affinity of the manager loop thread.

        :param core_id: CPU core ID
        """
        if self.running:
            logger.warning("Cannot set core affinity while manager is running.")
            return
        if core_id >= CORE_COUNT:
            logger.warning(
                "Core ID {core} exceeds detected CPU core count ({CORE_COUNT})"
            )
        self.core_id = core_id

    def set_rotation_matrix(self, rotation_matrix: NDArray) -> None:
        """Set the rotation matrix for remapping IMU axes.

        :param rotation_matrix: A 3x3 rotation matrix to apply to the IMU data.
        :return: None
        """
        if self.running:
            logger.warning("Cannot remap axes while sensor manager is running.")
            return
        if rotation_matrix.shape != (3, 3):
            raise ValueError("Rotation matrix must be of shape (3, 3).")
        with self.data_lock:
            self.imu_wrapper.rotation_matrix = rotation_matrix
            logger.info(f"Remapped IMU axes with rotation matrix:\n{rotation_matrix}")

    def stop(self) -> Path | None:
        """Stop the background loop and wait for the thread to finish.

        return: Path to the saved IMU data file if logging is enabled, otherwise None.
        """
        logger.info(f"Stopping {self}...")
        self.running = False
        self.imu_wrapper.started = False

        datafile: Path | None = None
        if self.log_data:
            self.file_writer.append_imu_data(self.IMUData_log)
            datafile = self.file_writer.save_dataframe()

        # Wait for thread to exit cleanly
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        logger.success(f"Stopped '{self}'.")
        return datafile

    def _initialize_sensor(self) -> None:
        """Initialize the IMU sensor, retrying on errors."""
        logger.info("Initializing sensor...")
        while not self.imu_wrapper.started:
            try:
                with self.i2c_lock:
                    self.imu_wrapper.reload()
            except OSError as init_error:
                logger.error(
                    f"Failed to initialize sensor due to I/O error: {init_error}, sleeping for {Delay.i2c_error_initialize} seconds..."
                )
                time.sleep(Delay.i2c_error_initialize)
            except Exception as init_error:
                logger.error(f"Failed to initialize sensor: {init_error}")
                time.sleep(Delay.initialization_retry)
        logger.success("Sensor initialized.")

    def _acc_gyro_are_fresh(self, new_data: IMUDeviceData) -> bool:
        """Check if accelerometer and gyroscope data are fresh compared to the latest data."""
        if new_data.accel is None:
            return False
        if new_data.gyro is None:
            return False
        if self.latest_data is None:
            return True
        if new_data.accel == self.latest_data.device_data.accel:
            return False
        if new_data.gyro == self.latest_data.device_data.gyro:
            return False
        return True

    def _mag_is_fresh(self, new_data: IMUDeviceData) -> bool:
        """Check if magnetometer data is fresh compared to the latest data."""
        if new_data.mag is None:
            return False
        if self.latest_data is None:
            return True
        if new_data.mag == self.latest_data.device_data.mag:
            return False
        return True
