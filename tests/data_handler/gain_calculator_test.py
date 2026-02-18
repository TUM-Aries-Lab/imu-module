"""Test the gain calculator."""

import math
from pathlib import Path

from imu_python.calibration.gain_calculator import calculate_gain
from imu_python.definitions import IMU_FILENAME_KEY


def test_gain_calculator() -> None:
    """Test the gain calculator."""
    filepath = Path("tests", "data", f"{IMU_FILENAME_KEY}_test.csv")

    gain = calculate_gain(filepath=filepath)
    hand_calculated_gain = 0.01194646478  # Pre-calculated gain for this test data

    # Compare with a precalculated gain
    assert math.isclose(gain, hand_calculated_gain, abs_tol=0.00001)
