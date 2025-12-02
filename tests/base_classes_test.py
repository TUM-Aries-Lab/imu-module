"""Test the base classes module."""

import numpy as np
import pytest

from imu_python.base_classes import IMUData, VectorXYZ


@pytest.mark.parametrize("t, x, y, z", [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 2.0, 3.0)])
def test_imu_data(t: float, x: float, y: float, z: float) -> None:
    """Test imu_data."""
    # Arrange
    vec_xyz = VectorXYZ(x=x, y=y, z=z)

    # Act
    imu_data = IMUData(timestamp=t, accel=vec_xyz, gyro=vec_xyz, mag=vec_xyz)

    # Assert
    assert imu_data.timestamp == t
    assert imu_data.accel == vec_xyz
    assert imu_data.gyro == vec_xyz
    assert imu_data.mag == vec_xyz


@pytest.mark.parametrize("x, y, z", [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)])
def test_vector_xyz(x: float, y: float, z: float) -> None:
    """Test vector xyz."""
    # Arrange

    # Act
    vector = VectorXYZ(x=x, y=y, z=z)

    # Assert
    assert vector.x == x
    assert vector.y == y
    assert vector.z == z


@pytest.mark.parametrize("x, y, z", [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)])
def test_vector_xyz_from_tuple(x: float, y: float, z: float) -> None:
    """Test vector xyz from tuple."""
    # Act
    vec = VectorXYZ.from_tuple((x, y, z))

    # Assert
    assert vec.x == x
    assert vec.y == y
    assert vec.z == z


def test_vector_xyz_from_tuple_bad() -> None:
    """Test vector xyz from tuple."""
    with pytest.raises(ValueError):
        VectorXYZ.from_tuple((0.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("x, y, z", [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)])
def test_vector_xyz_as_array(x: float, y: float, z: float) -> None:
    """Test vector xyz from tuple."""
    vector = VectorXYZ(x=x, y=y, z=z)

    vec = vector.as_array()

    assert vec.shape == (3,)


@pytest.mark.parametrize("num_elements", [1, 2, 3])
def test_vector_xyz_from_array(num_elements: int) -> None:
    """Test vector xyz from tuple."""
    x, y, z = np.arange(num_elements), np.arange(num_elements), np.arange(num_elements)
    vector = VectorXYZ(x=x, y=y, z=z)

    assert vector.x.shape == (num_elements,)
    assert vector.y.shape == (num_elements,)
    assert vector.z.shape == (num_elements,)
