"""Manager for a sensor object."""

import threading
import time

from loguru import logger

from imu_python.base_classes import IMUData
from imu_python.definitions import Delay, IMUFrequency, i2c_error, thread_join_timeout
from imu_python.wrapper import IMUWrapper


class SensorManager:
    """Thread-safe IMU data manager.

    :param imu_wrapper: IMUWrapper instance to manage
    """

    def __init__(self, imu_wrapper: IMUWrapper) -> None:
        self.imu_wrapper: IMUWrapper = imu_wrapper
        self.running: bool = False
        self.lock = threading.Lock()
        self.latest_data: IMUData | None = None
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        """Start the sensor manager."""
        self._initialize_sensor()
        self.running = True
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                # Attempt to read all sensor data
                data = self.imu_wrapper.all()
                with self.lock:
                    self.latest_data = data

            except OSError as err:
                # Catch I2C remote I/O errors
                self.imu_wrapper.started = False
                self.latest_data = None
                if err.errno == i2c_error:
                    logger.error("I2C error detected. Reinitializing sensor...")
                    time.sleep(Delay.i2c_error_retry)  # short delay before retry
                    self._initialize_sensor()
                else:
                    # Reraise unexpected errors
                    raise
            # Sleep to control streaming rate
            time.sleep(IMUFrequency.imu_read_frequency)

    def get_data(self) -> IMUData:
        """Return sensor data as a IMUData object."""
        while self.latest_data is None:
            time.sleep(Delay.data_retry)
        with self.lock:
            data = self.latest_data
            logger.debug(
                f"Information from {self.imu_wrapper.config.name}: "
                f"IMU: acc={data.accel}, gyro={data.gyro}"
            )
            return self.latest_data

    def stop(self) -> None:
        """Stop the background loop and wait for the thread to finish."""
        self.running = False
        self.imu_wrapper.started = False
        # Wait for thread to exit cleanly
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=thread_join_timeout)

    def _initialize_sensor(self) -> None:
        while not self.imu_wrapper.started:
            try:
                self.imu_wrapper.initialize()
                logger.success("Sensor initialized.")
            except Exception as init_error:
                logger.error(f"Failed to initialize sensor: {init_error}")
                time.sleep(Delay.initialization_retry)
