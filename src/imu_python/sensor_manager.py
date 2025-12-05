"""Manager for a sensor object."""

import threading
import time

from loguru import logger

from .base_classes import IMUData
from .imu_wrapper import IMUWrapper


class SensorManager:
    """Thread-safe IMU data manager."""

    def __init__(self, imu_wrapper):
        self.imu_wrapper: IMUWrapper = imu_wrapper
        self.running: bool = False
        self.lock = threading.Lock()
        self.latest_data: IMUData | None = None
        self.thread = None

    def start(self):
        """Start the sensor manager."""
        self._initialize_sensor()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                # Attempt to read all sensor data
                data = self.imu_wrapper.all()
                with self.lock:
                    self.latest_data = data

            except OSError as e:
                # Catch I2C remote I/O errors
                self.imu_wrapper.started = False
                self.latest_data = None
                if e.errno == 121:
                    logger.error("I2C error detected. Reinitializing sensor...")
                    time.sleep(1)  # short delay before retry
                    self._initialize_sensor()
                else:
                    # Reraise unexpected errors
                    raise
            # Sleep to control streaming rate
            time.sleep(1)

    def get_data(self):
        """Return sensor data as a IMUData object."""
        while self.latest_data is None:
            time.sleep(0.5)
        with self.lock:
            return self.latest_data

    def log_data(self):
        """Log sensor data as a IMUData object using loguru."""
        while self.latest_data is None:
            time.sleep(0.5)
        with self.lock:
            logger.info(
                f"Information from {self.imu_wrapper.config.name}: "
                f"IMU: acc={self.latest_data.accel.as_array()}, "
                f"gyro={self.latest_data.gyro.as_array()}, "
            )

    def stop(self):
        """Stop the background loop and wait for the thread to finish."""
        self.running = False
        self.imu_wrapper.started = False
        # Wait for thread to exit cleanly
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)

    def _initialize_sensor(self):
        while self.imu_wrapper.started is False:
            try:
                self.imu_wrapper.initialize()
                logger.success("Sensor initialized.")
            except Exception as init_error:
                logger.error(f"Failed to initialize sensor: {init_error}")
                time.sleep(1)
