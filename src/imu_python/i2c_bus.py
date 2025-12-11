"""Unified Jetson I2C bus handler."""

from __future__ import annotations

from adafruit_extended_bus import ExtendedI2C
from loguru import logger

from imu_python.definitions import I2CBusID


class JetsonBus:
    """Manage left/right Jetson I2C buses with graceful desktop fallback."""

    _left_bus: ExtendedI2C | None = None
    _right_bus: ExtendedI2C | None = None
    _initialized: bool = False

    @classmethod
    def _initialize(cls) -> None:
        """Attempt to initialize Jetson I2C buses."""
        if cls._initialized:
            return

        try:
            cls._left_bus = ExtendedI2C(I2CBusID.left)
            cls._right_bus = ExtendedI2C(I2CBusID.right)
            logger.info("Jetson I2C buses initialized.")

        except ValueError as err:
            # Happens when running on non-Jetson hardware
            logger.warning(f"{err}. Using None for Jetson I2C buses.")
            cls._left_bus = None
            cls._right_bus = None

        cls._initialized = True

    @classmethod
    def get(cls, bus_id: int | None) -> ExtendedI2C | None:
        """Return the Jetson I2C bus for a given ID.

        :param bus_id: One of I2CBusID.left, I2CBusID.right, or None.
        :return: I2C bus instance or None.
        """
        cls._initialize()

        if bus_id is I2CBusID.left:
            return cls._left_bus
        elif bus_id is I2CBusID.right:
            return cls._right_bus
        else:
            logger.error(f"Invalid Jetson I2C bus ID: {bus_id}.")
            return None
