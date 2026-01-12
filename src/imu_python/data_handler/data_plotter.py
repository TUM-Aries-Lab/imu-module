"""IMU data plotter."""

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from numpy.typing import NDArray

from imu_python.base_classes import IMUDataFile, Quaternion, VectorXYZ
from imu_python.data_handler.data_reader import load_imu_data
from imu_python.definitions import FigureSettings, IMUUnits


class IMUPlotter:  # pragma: no cover
    """Plot IMU data."""

    def __init__(
        self,
        imu_data_file: IMUDataFile,
    ) -> None:
        """Initialize the plotter and make a plot of the IMU data file.

        :param imu_data_frame: An IMUDataFile that contains IMUData readings.
        """
        self.data = imu_data_file
        self.fig, self.axes = self._create_figs()
        self._plot_data()
        self._show()

    @staticmethod
    def _create_figs():
        num_rows = 4
        fig, axes = plt.subplots(
            nrows=num_rows, ncols=1, figsize=FigureSettings.size, sharex=True
        )
        return fig, axes

    @staticmethod
    def _show():
        try:
            plt.show()
        except KeyboardInterrupt:
            plt.close()

    def _plot_data(self) -> None:
        plot_vectors(
            vectors=vectors_to_array(self.data.accels),
            ax=self.axes[0],
            time=self.data.time,
            y_label=f"Acceleration ({IMUUnits.ACCEL.value})",
        )
        plot_vectors(
            vectors=vectors_to_array(self.data.gyros),
            ax=self.axes[1],
            time=self.data.time,
            y_label=f"Angular Rate ({IMUUnits.GYRO.value})",
        )
        plot_vectors(
            vectors=vectors_to_array(self.data.mags),
            ax=self.axes[2],
            time=self.data.time,
            y_label=f"Magnetic Field ({IMUUnits.MAG.value})",
        )

        plot_quaternions(
            quaternions=self.data.quats,
            ax=self.axes[3],
            time=self.data.time,
        )
        self.axes[-1].set_xlabel("Time (s)")
        plt.suptitle("IMU Data")
        plt.tight_layout()

    @staticmethod
    def _extract_units_from_column_name(column_name: str) -> str:
        """Extract the first substring inside parentheses.

        :param column_name: Column name string.
        :return: Extracted units string.
        """
        match = re.search(r"\(([^)]*)\)", column_name)
        return match.group(1) if match else ""


def vectors_to_array(vectors: list[VectorXYZ]) -> NDArray:  # pragma: no cover
    """Convert a list of VectorXYZ to an NP array."""
    return np.array([(v.x, v.y, v.z) for v in vectors], dtype=np.float32)


def plot_imu_data(imu_data_file: IMUDataFile) -> None:  # pragma: no cover
    """Plot IMU data."""
    IMUPlotter(imu_data_file=imu_data_file)


def plot_vectors(
    vectors: NDArray, ax: Axes, time: NDArray, y_label: str
) -> Axes:  # pragma: no cover
    """Plot 3D vector."""
    alpha = FigureSettings.alpha
    ax.plot(time, vectors[:, 0], label="x", color="r", alpha=alpha)
    ax.plot(time, vectors[:, 1], label="y", color="g", alpha=alpha)
    ax.plot(time, vectors[:, 2], label="z", color="b", alpha=alpha)
    ax.set_ylabel(y_label)
    ax.legend(loc=FigureSettings.legend_loc)
    ax.grid(True)
    return ax


def plot_quaternions(
    quaternions: list[Quaternion], ax: Axes, time: NDArray
) -> None:  # pragma: no cover
    """Plot quaternions."""
    if quaternions is None:
        return
    ax.scatter(time, [q.x for q in quaternions], label="x", color="r", s=1)
    ax.scatter(time, [q.y for q in quaternions], label="y", color="g", s=1)
    ax.scatter(time, [q.z for q in quaternions], label="z", color="b", s=1)
    ax.scatter(time, [q.w for q in quaternions], label="w", color="m", s=1)

    ax.set_ylabel("Quaternion")
    ax.legend(loc=FigureSettings.legend_loc)
    ax.grid(True)


if __name__ == "__main__":  # pragma: no cover
    """Test the IMU device."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", "-f", type=Path, required=True)
    args = parser.parse_args()

    imu_data_file = load_imu_data(filepath=args.filepath)
    plot_imu_data(imu_data_file=imu_data_file)
