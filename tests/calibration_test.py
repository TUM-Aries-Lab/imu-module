"""Test magnetometer calibration."""

from unittest.mock import patch

import numpy as np
import pytest
from numpy.typing import NDArray

from imu_python.data_handler.calibration import (
    CalibrationMetrics,
    FittingAlgorithm,
    MagCalibration,
)

default_hard_iron = np.array([10.0, -5.0, 3.0])
default_soft_iron = np.array([[1.3, 0.15, -0.08], [0.0, 0.95, 0.12], [0.1, 0.0, 1.1]])


def generate_test_data(
    n_points: int = 1000,
    hard_iron: NDArray = default_hard_iron,
    soft_iron: NDArray = default_soft_iron,
    noise_std: float = 0.01,
    quality: str = "good",
) -> NDArray:
    """Generate magnetometer test data with controllable quality.

    Parameters
    ----------
    n_points : int
        Number of data points to generate
    hard_iron : np.ndarray (3,)
        Hard-iron offset vector
    soft_iron : np.ndarray (3,3)
        Soft-iron transformation matrix
    noise_std : float
        Standard deviation of Gaussian noise
    quality : str
        "good" - uniform sphere coverage, guaranteed good calibration
        "bad_planar" - nearly planar, guaranteed bad calibration
        "bad_ring" - only two-axis rotation, poor coverage
        "bad_clustered" - clustered points, non-uniform
        "bad_cap" - small spherical cap, incomplete coverage

    Returns
    -------
    data : np.ndarray (n_points, 3)
        Raw magnetometer readings

    """
    # === Generate base points on unit sphere ===
    if quality == "good":
        # Fibonacci sphere - perfectly uniform
        phi = np.pi * (np.sqrt(5) - 1)
        indices = np.arange(n_points)
        y = 1 - (indices / (n_points - 1)) * 2
        radius = np.sqrt(1 - y * y)
        theta = phi * indices
        x = radius * np.cos(theta)
        z = radius * np.sin(theta)
        sphere_points = np.column_stack([x, y, z])

    elif quality == "bad_planar":
        # Almost planar - tiny Z variation
        theta = np.linspace(0, 2 * np.pi, n_points)
        x = np.cos(theta)
        y = np.sin(theta)
        z = np.random.uniform(-0.05, 0.05, n_points)  # Tiny Z
        sphere_points = np.column_stack([x, y, z])

    elif quality == "bad_ring":
        # Only two-axis rotation (ring around equator)
        theta = np.linspace(0, 2 * np.pi, n_points)
        x = np.cos(theta)
        y = np.sin(theta)
        z = np.zeros(n_points)  # All on equator
        sphere_points = np.column_stack([x, y, z])

    elif quality == "bad_clustered":
        # Four tight clusters
        n_clusters = 4
        points_per_cluster = n_points // n_clusters
        sphere_points = []
        centers = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.7, 0.7, 0]]
        for c in centers:
            center = np.array(c) / np.linalg.norm(c)
            for _ in range(points_per_cluster):
                point = center + np.random.normal(0, 0.1, 3)
                point = point / np.linalg.norm(point)
                sphere_points.append(point)
        sphere_points = np.array(sphere_points[:n_points])

    elif quality == "bad_cap":
        # Small spherical cap (30° from pole)
        phi = np.random.uniform(0, np.pi / 6, n_points)
        theta = np.random.uniform(0, 2 * np.pi, n_points)
        x = np.sin(phi) * np.cos(theta)
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)
        sphere_points = np.column_stack([x, y, z])

    else:
        raise ValueError(f"Unknown quality: {quality}")

    # === Common transformation for all data types ===
    A1_inv = np.linalg.inv(soft_iron)
    ellipsoid_points = sphere_points @ A1_inv.T
    data = ellipsoid_points + hard_iron

    # === Add noise (smaller for good data, larger for bad) ===
    if quality == "good":
        noise_std = min(noise_std, 0.02)
    else:
        noise_std = max(noise_std, 0.05)  # More noise for bad data

    data += np.random.normal(0, noise_std, data.shape)

    return data


@pytest.mark.parametrize("algorithm", [a.value for a in FittingAlgorithm])
def test_calibration_good_data(algorithm):
    """Test calibration with high-quality data."""
    # Arrange
    raw_data = generate_test_data(quality="good")
    with patch.object(MagCalibration, "plot_data", return_value=None):
        # Act
        calib = MagCalibration(data=raw_data, algorithm=algorithm)
        calib_data = calib.apply_calibration(raw_data)
        matrics = CalibrationMetrics.evaluate(calib_data)
        # Assert
        assert not matrics.should_reject(), (
            "Calibration should be accepted for good data"
        )

        assert np.allclose(calib.b, default_hard_iron, rtol=1e-2, atol=1e-5)
        # Polar decomposition of A-1 to extract scaling + shear
        _vecU_a, vals_a, vecV_a = np.linalg.svd(default_soft_iron)
        P_a = vecV_a.T @ np.diag(vals_a) @ vecV_a
        _vecU_e, vals_e, vecV_e = np.linalg.svd(calib.A_1)
        P_e = vecV_e.T @ np.diag(vals_e) @ vecV_e
        assert np.allclose(P_a, P_e, rtol=1e-2, atol=1e-2)


@pytest.mark.parametrize("algorithm", [a.value for a in FittingAlgorithm])
def test_calibration_bad_data(algorithm):
    """Test calibration with various bad-quality data."""
    for quality in ["bad_planar", "bad_ring", "bad_clustered", "bad_cap"]:
        # Arrange
        raw_data = generate_test_data(quality=quality)
        with patch.object(MagCalibration, "plot_data", return_value=None):
            # Act
            calib = MagCalibration(data=raw_data, algorithm=algorithm)
            calib_data = calib.apply_calibration(raw_data)
            matrics = CalibrationMetrics.evaluate(calib_data)
            # Assert
            assert matrics.should_reject(), (
                f"Calibration should be rejected for {quality} data"
            )
