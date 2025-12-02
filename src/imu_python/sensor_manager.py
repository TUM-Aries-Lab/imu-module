"""Manager for a sensor object."""

import threading
import time


class SensorManager:
    """Thread-safe IMU data manager."""

    def __init__(self, imu):
        self.imu = imu
        self.running = False
        self.lock = threading.Lock()
        self.latest_data = None
        self.thread = None

    def start(self):
        """Start the sensor manager."""
        self.imu.initialize()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            data = self.imu.all()
            with self.lock:
                self.latest_data = data
            time.sleep(1)

    def get_data(self):
        """Return sensor data as a IMUData object."""
        with self.lock:
            return self.latest_data

    def stop(self):
        """Stop the background loop and wait for the thread to finish."""
        self.running = False

        # Wait for thread to exit cleanly
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        # Let IMU drivers clean up if needed
        if hasattr(self.imu, "shutdown"):
            self.imu.shutdown()
