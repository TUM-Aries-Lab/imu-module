"""Manager for a sensor object."""

import threading
import time

from loguru import logger

from imu_python.base_classes import IMUData
from imu_python.data_handler.data_writer import IMUFileWriter
from imu_python.definitions import (
    ACCEL_GRAVITY_MSEC2,
    ANGULAR_VELOCITY_DPS_TO_RADS,
    I2C_ERROR,
    THREAD_JOIN_TIMEOUT,
    Delay,
    I2CBusID,
)
from imu_python.wrapper import IMUWrapper


class IMUManager:
    """Thread-safe IMU data manager."""

    def __init__(
        self, imu_wrapper: IMUWrapper, i2c_id: I2CBusID | None, log_data: bool = False
    ) -> None:
        """Initialize the sensor manager.

        :param imu_wrapper: IMUWrapper instance to manage
        :param i2c_id: I2C bus identifier
        :param log_data: Flag to record the IMU data
        """
        self.imu_wrapper: IMUWrapper = imu_wrapper
        self.log_data: bool = log_data
        self.i2c_id: I2CBusID | None = i2c_id
        self.accel_range_m_s2: float = (
            imu_wrapper.config.accel_range_g * ACCEL_GRAVITY_MSEC2
        )
        self.gyro_range_rad_s: float = (
            imu_wrapper.config.gyro_range_dps * ANGULAR_VELOCITY_DPS_TO_RADS
        )
        self.running: bool = False
        self.lock = threading.Lock()
        self.latest_data: IMUData | None = None
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        if log_data:
            self.file_writer: IMUFileWriter = IMUFileWriter()
            self.IMUData_log: list[IMUData] = []

    def __repr__(self) -> str:
        """Return string representation of the sensor manager."""
        return (
            f"IMUManager(name:{self.imu_wrapper.config.name}, "
            f"bus:{self.i2c_id}, "
            f"addr:{self.imu_wrapper.config.addresses})"
        )

    def start(self):
        """Start the sensor manager."""
        self._initialize_sensor()
        self.running = True
        self.thread.start()

    def _loop(self) -> None:
        """Read data from the IMU wrapper and update the latest data."""
        while self.running:
            try:
                # Attempt to read all sensor data
                data = self.imu_wrapper.get_raw_data()
                # Ensure new data
                if self.latest_data is None or data != self.latest_data.raw_data:
                    logger.debug(
                        f"reading from:{self.imu_wrapper.config.name} new data:{data}"
                    )
                    with self.lock:
                        timestamp = time.monotonic()
                        pose_quat = self.imu_wrapper.filter.update(
                            timestamp=timestamp,
                            accel=data.accel.as_array(),
                            gyro=data.gyro.as_array(),
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
                            raw_data=data,
                        )
                        if self.log_data:
                            self.IMUData_log.append(self.latest_data)
                time.sleep(Delay.data_retry)

            except OSError as err:
                # Catch I2C remote I/O errors
                self.imu_wrapper.started = False
                self.latest_data = None
                if err.errno == I2C_ERROR:
                    logger.error("I2C error detected. Reinitializing sensor...")
                    time.sleep(Delay.i2c_error_retry)  # short delay before retry
                    self._initialize_sensor()
                else:
                    # Reraise unexpected errors
                    logger.warning(f"Unexpected error: {err}")
                    raise

    def get_data(self) -> IMUData:
        """Return sensor data as a IMUData object."""
        data = self.latest_data
        while data is None:
            time.sleep(Delay.data_retry)
            data = self.latest_data
        with self.lock:
            logger.debug(f"I2C Bus: {self}, data: {data}")
            return data

    def stop(self) -> None:
        """Stop the background loop and wait for the thread to finish."""
        logger.info(f"Stopping {self}...")
        self.running = False
        self.imu_wrapper.started = False

        if self.log_data:
            self.file_writer.append_imu_data(self.IMUData_log)
            self.file_writer.save_dataframe()

        # Wait for thread to exit cleanly
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        logger.success(f"Stopped '{self.imu_wrapper.config}'.")

    def _initialize_sensor(self) -> None:
        logger.info("Initializing sensor...")
        while not self.imu_wrapper.started:
            try:
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
