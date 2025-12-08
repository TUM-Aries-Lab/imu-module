"""Factory that creates IMU object from given IMU type."""

from loguru import logger

try:
    import board
except ModuleNotFoundError as err:
    logger.error(f"Failed to import '{err}'. Are you running without the Jetson?")
    board = None  # allow desktop environments to import this file

from imu_python.devices import IMUDevices
from imu_python.sensor_manager import SensorManager
from imu_python.wrapper import IMUWrapper


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def detect_and_create(i2c_bus=None) -> list[SensorManager]:
        """Automatically detect addresses and create sensor managers.

        :param i2c_bus: I2C bus instance. If None, attempt to use board.I2C().
        :return: list of SensorManager instances.
        """
        # If caller did not provide an I2C bus, try to use board.I2C.
        if i2c_bus is None:
            if board is None:
                logger.warning(
                    "board module not available — skipping hardware IMU detection."
                )
                cfg = IMUDevices.MOCK.config
                wrapper = IMUWrapper(cfg, i2c_bus=None)
                return [SensorManager(imu_wrapper=wrapper)]

        detected_addresses = IMUFactory.scan_i2c_bus(i2c_bus)
        imu_managers: list[SensorManager] = []

        for enum_item in IMUDevices:
            cfg = enum_item.config
            if address := IMUFactory.compare_addresses(
                cfg.addresses, detected_addresses
            ):
                logger.info(f"Detected {cfg.name} at I2C address {address}")
                imu_wrapper = IMUWrapper(config=cfg, i2c_bus=i2c_bus)
                sensor_manager = SensorManager(imu_wrapper=imu_wrapper)
                imu_managers.append(sensor_manager)

        return imu_managers

    @staticmethod
    def scan_i2c_bus(i2c) -> list[int]:
        """Scan the I2C bus for sensor addresses."""
        try:
            while not i2c.try_lock():
                pass
            try:
                return i2c.scan()
            finally:
                i2c.unlock()
        except Exception as err:
            logger.error(f"I2C scan failed: {err}")
            return []

    @staticmethod
    def compare_addresses(
        imu_address: list[int], detected_addresses: list[int]
    ) -> int | None:
        """Compare IMU address candidates to detected addresses."""
        matches = set(detected_addresses) & set(imu_address)

        if len(matches) == 1:
            return next(iter(matches))
        if len(matches) > 1:
            logger.warning("Multiple IMU addresses detected — skipping.")
        return None
