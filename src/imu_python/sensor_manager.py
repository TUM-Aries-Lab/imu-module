"""Manager for a sensor object."""

import threading
import time

from loguru import logger

from imu_python.base_classes import IMUData
from imu_python.definitions import I2C_ERROR, THREAD_JOIN_TIMEOUT, Delay, IMUUpdateTime
from imu_python.wrapper import IMUWrapper


class IMUManager:
    """Thread-safe IMU data manager."""

    def __init__(self, imu_wrapper: IMUWrapper) -> None:
        """Initialize the sensor manager.

        :param imu_wrapper: IMUWrapper instance to manage
        """
        self.imu_wrapper: IMUWrapper = imu_wrapper
        self.running: bool = False
        self.lock = threading.Lock()
        self.latest_data: IMUData | None = None
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        self.period: float = IMUUpdateTime.period_sec

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
                data = self.imu_wrapper.get_data()
                with self.lock:
                    self.latest_data = data

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
            logger.debug(
                f"IMU: {self.imu_wrapper.config.name}, "
                f"addr: {self.imu_wrapper.config.addresses}, "
                f"acc={data.accel}, gyro={data.gyro}, "
                f"pose={data.pose}"
            )
            return data

    def stop(self) -> None:
        """Stop the background loop and wait for the thread to finish."""
        logger.info(f"Stopping {self.imu_wrapper.config.name}...")
        self.running = False
        self.imu_wrapper.started = False

        # Wait for thread to exit cleanly
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        logger.success(f"Stopped '{self.imu_wrapper.config.name}'.")

    def _initialize_sensor(self) -> None:
        logger.info("Initializing sensor...")
        while not self.imu_wrapper.started:
            try:
                self.imu_wrapper.reload()
                logger.success("Sensor initialized.")
            except Exception as init_error:
                logger.error(f"Failed to initialize sensor: {init_error}")
                time.sleep(Delay.initialization_retry)
