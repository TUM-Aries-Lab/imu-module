"""I2C Bus handler."""

from typing import Any

from loguru import logger

from imu_python.definitions import I2CBusID

try:
    from adafruit_extended_bus import ExtendedI2C

    I2C_BUS_L = ExtendedI2C(I2CBusID.left)  # pin 27 (SDA) & 28 (SCL)
    I2C_BUS_R = ExtendedI2C(I2CBusID.right)  # pin 3 (SDA) & 5 (SCL)
except ValueError as device_err:
    logger.warning(f"{device_err}. Are you running without the Jetson?")
    I2C_BUS_L = None  # allow desktop environments to import this file
    I2C_BUS_R = None


class I2CBus:
    """I2C Bus handler."""

    @staticmethod
    def get_i2c_bus(i2c_id: int | None = None) -> Any:
        """Get the I2C bus with the given identifier.

        :param i2c_id: I2C bus identifier.
        :return: I2C bus instance or None if identifier is None.
        """
        if i2c_id is None:
            return None
        else:
            bus_map = {
                I2CBusID.left: I2C_BUS_L,
                I2CBusID.right: I2C_BUS_R,
            }
            i2c_bus = bus_map.get(i2c_id)
            if i2c_bus is None:
                logger.error(f"Bus ID {i2c_id} not supported.")
                return None
            return i2c_bus
