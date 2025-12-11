"""Unified Jetson I2C bus handler."""

from __future__ import annotations

from typing import ClassVar

from adafruit_extended_bus import ExtendedI2C
from loguru import logger

from imu_python.definitions import I2CBusID


class JetsonBus:
    """Manage left/right Jetson I2C buses with graceful desktop fallback."""

    _initialized: bool = False
    _bus_map: ClassVar[dict[I2CBusID, ExtendedI2C | None]] = {}

    @classmethod
    def _initialize(cls) -> None:
        """Attempt to initialize Jetson I2C buses."""
        if cls._initialized:
            return
        for bus_id in I2CBusID:
            try:
                cls._bus_map[bus_id] = ExtendedI2C(bus_id)
            except ValueError as bus_err:
                logger.warning(f"{bus_err}. Using None for Jetson I2C buses.")

        logger.info("Jetson I2C buses initialized.")
        cls._initialized = True

    @classmethod
    def get(cls, bus_id: I2CBusID | None) -> ExtendedI2C | None:
        """Return the Jetson I2C bus for a given ID.

        :param bus_id: One of I2CBusID.left, I2CBusID.right, or None.
        :return: I2C bus instance or None.
        """
        if bus_id is None:
            return None
        cls._initialize()
        if bus_id not in cls._bus_map:
            logger.error(f"Invalid Jetson I2C bus ID: {bus_id}.")
            return None
        return cls._bus_map[bus_id]
