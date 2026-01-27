"""IMU data recording."""

from pathlib import Path

import pandas as pd
from loguru import logger

from imu_python.base_classes import IMUConfig, IMUData
from imu_python.definitions import (
    IMU_FILENAME_KEY,
    RECORDINGS_DIR,
    I2CBusID,
    IMUDataFileColumns,
)
from imu_python.utils import create_timestamped_filepath


class IMUFileWriter:
    """IMU data recording with Pandas."""

    def __init__(self, imu_config: IMUConfig, bus_id: I2CBusID | None):
        self.data_frame: pd.DataFrame = self._init_dataframe()
        self._imu_config = imu_config
        self._bus_id = bus_id

    @staticmethod
    def _init_dataframe() -> pd.DataFrame:
        """Initialize an empty IMU DataFrame with proper columns."""
        logger.debug("Initializing IMU DataFrame.")
        init_df = {col.value: pd.Series(dtype="float64") for col in IMUDataFileColumns}
        return pd.DataFrame(init_df)

    def append_imu_data(self, data: list[IMUData] | None) -> None:
        """Append an IMUData entry to the DataFrame."""
        if data is None:
            logger.warning("No IMU data to append.")
            return
        rows: list[dict[str, float | None]] = []
        for imu in data:
            rows.append(
                {
                    IMUDataFileColumns.TIMESTAMP.value: imu.timestamp,
                    IMUDataFileColumns.ACCEL_X.value: imu.device_data.accel.x,
                    IMUDataFileColumns.ACCEL_Y.value: imu.device_data.accel.y,
                    IMUDataFileColumns.ACCEL_Z.value: imu.device_data.accel.z,
                    IMUDataFileColumns.GYRO_X.value: imu.device_data.gyro.x,
                    IMUDataFileColumns.GYRO_Y.value: imu.device_data.gyro.y,
                    IMUDataFileColumns.GYRO_Z.value: imu.device_data.gyro.z,
                    IMUDataFileColumns.MAG_X.value: imu.device_data.mag.x
                    if imu.device_data.mag
                    else None,
                    IMUDataFileColumns.MAG_Y.value: imu.device_data.mag.y
                    if imu.device_data.mag
                    else None,
                    IMUDataFileColumns.MAG_Z.value: imu.device_data.mag.z
                    if imu.device_data.mag
                    else None,
                    IMUDataFileColumns.POSE_W.value: imu.quat.w,
                    IMUDataFileColumns.POSE_X.value: imu.quat.x,
                    IMUDataFileColumns.POSE_Y.value: imu.quat.y,
                    IMUDataFileColumns.POSE_Z.value: imu.quat.z,
                }
            )

        self.data_frame = pd.concat(
            [self.data_frame, pd.DataFrame(rows)], ignore_index=True
        )
        return

    def save_dataframe(
        self,
        output_dir: Path = RECORDINGS_DIR,
    ) -> Path:
        """Save IMU DataFrame to a CSV file.

        :param output_dir: Directory to save the IMU DataFrame into.
        :return: Path to the CSV file.
        """
        prefix = IMU_FILENAME_KEY + self._add_prefix()

        filepath = create_timestamped_filepath(
            output_dir=output_dir, prefix=prefix, suffix="csv"
        )
        logger.info(f"Saving IMU DataFrame to '{filepath}'.")
        filepath.parent.mkdir(parents=True, exist_ok=True)
        self.data_frame.to_csv(filepath, index=False)
        return filepath

    def _add_prefix(self) -> str:
        """Add IMU address and bus ID to the output file prefix."""
        return (
            "_"
            + self._imu_config.name
            + "_"
            + str(self._bus_id)
            + "_"
            + str(self._imu_config.addresses[0])
        )
