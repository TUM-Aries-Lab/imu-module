"""Factory that creates IMU object from given IMU type."""

from threading import Lock
from typing import Any

from loguru import logger

from imu_python.builtin_devices import MOCK
from imu_python.definitions import CORE_COUNT, GIL_ENABLED, I2CBusID
from imu_python.devices import get_config
from imu_python.i2c_bus import I2CBusDescriptor, JetsonBus
from imu_python.sensor_manager import IMUManager
from imu_python.wrapper import IMUWrapper


class IMUFactory:
    """Factory that creates IMU object from given IMU type."""

    @staticmethod
    def detect_and_create(
        free_threading: bool = True,
        log_data: bool = False,
        calibration_mode: bool = False,
    ) -> list[IMUManager]:
        """Automatically detect addresses on all buses defined in I2CBUSID and create sensor managers.

        :param free_threading: Flag to enable free threading.
        :param log_data: Flag to record the IMU data.
        :param calibration_mode: Flag to use calibration mode.
        :return: list of IMUManager instances.
        """
        # GIL enabled or core_count == 0 means no free threading
        free_threading = free_threading and not GIL_ENABLED and CORE_COUNT != 0
        core_id: int = 0
        if free_threading:
            logger.info("Free threading enabled.")
        else:
            logger.warning("Free threading disabled or conditions not satisfied.")
        managers: list[IMUManager] = []
        for bus in I2CBusID:
            i2c_lock = Lock()
            managers += IMUFactory._detect_and_create_per_bus(
                i2c_lock=i2c_lock,
                i2c_id=bus,
                log_data=log_data,
                calibration_mode=calibration_mode,
            )
        if free_threading:
            if len(managers) > CORE_COUNT:
                logger.warning(
                    f"Number of detected IMUs ({len(managers)}) is larger than number of detected CPU cores({CORE_COUNT})."
                )
            for manager in managers:
                manager.set_core_affinity(core_id)
                core_id += 1
                core_id %= CORE_COUNT

        return managers

    @staticmethod
    def _detect_and_create_per_bus(
        i2c_lock: Lock,
        i2c_id: I2CBusID | None = None,
        log_data: bool = False,
        calibration_mode: bool = False,
    ) -> list[IMUManager]:
        """Automatically detect addresses on the given bus and create sensor managers.

        :param i2c_lock: a shared i2c lock among IMU managers on this bus.
        :param i2c_id: I2C bus identifier. If None, attempt to use board.I2C().
        :param log_data: Flag to record the IMU data.
        :param calibration_mode: Flag to use calibration mode.
        :return: list of IMUManager instances.
        """
        imu_managers: list[IMUManager] = []

        i2c_bus = JetsonBus.get(bus_id=i2c_id)

        addresses = IMUFactory.scan_i2c_bus(i2c=i2c_bus)

        detected_configs = get_config(addresses=addresses)

        for imu_descriptor, cfg in detected_configs.items():
            imu_wrapper = IMUWrapper(
                config=cfg,
                imu_descriptor=imu_descriptor,
                i2c_bus_descriptor=I2CBusDescriptor(
                    bus_instance=i2c_bus, bus_id=i2c_id
                ),
                calibration_mode=calibration_mode,
            )

            imu_managers.append(
                IMUManager(
                    imu_wrapper=imu_wrapper,
                    log_data=log_data,
                    calibration_mode=calibration_mode,
                    i2c_lock=i2c_lock,
                )
            )

            logger.info(
                f"Detected {imu_descriptor} with roles {list(cfg.roles.keys())} "
                f"on address(es) {[hex(a) for d in cfg.devices.values() for a in d.addresses]}"
            )

        return imu_managers

    @staticmethod
    def scan_i2c_bus(i2c: Any) -> list[int]:
        """Scan the I2C bus for sensor addresses."""
        try:
            while not i2c.try_lock():
                pass
            addresses = i2c.scan()
            i2c.unlock()
            return addresses
        except Exception as err:
            logger.warning(f"I2C scan failed: {err}. Returning {MOCK} addresses.")
            return [a for d in MOCK.devices.values() for a in d.addresses]
