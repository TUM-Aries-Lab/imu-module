"""Test the base_classes module."""

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


@pytest.mark.parametrize("x, y, z", [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)])
def test_vector_xyz_as_array(x: float, y: float, z: float) -> None:
    """Test vector xyz from tuple."""
    vector = VectorXYZ(x=x, y=y, z=z)

    vec = vector.as_array()

    assert vec.shape == (3,)


@pytest.mark.parametrize("num_elements", [1, 2, 3, 4, 5])
def test_vector_xyz_from_array(num_elements: int) -> None:
    """Test vector xyz from tuple."""
    x, y, z = np.arange(num_elements), np.arange(num_elements), np.arange(num_elements)
    vector = VectorXYZ(x=x, y=y, z=z)

    assert vector.as_array().shape == (3, num_elements)


@pytest.mark.parametrize("num_elements", [1, 2, 3])
def test_vector_xyz_rotate(num_elements: int) -> None:
    """Test vector xyz rotation."""
    # Arrange
    x, y, z = (
        1 * np.ones(num_elements),
        -1 * np.ones(num_elements),
        np.ones(num_elements),
    )
    vector = VectorXYZ(x=x, y=y, z=z)
    rotation_matrix = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

    # Act
    vector.rotate(rotation_matrix)

    # Assert
    if isinstance(vector.x, np.ndarray):
        assert vector.x[0] == y[0]
    else:
        assert vector.x == x

    if isinstance(vector.y, np.ndarray):
        assert vector.y[0] == x[0]
    else:
        assert vector.y == y

    if isinstance(vector.z, np.ndarray):
        assert vector.z[0] == -z[0]
    else:
        assert vector.z == -z


def test_vector_xyz_rotate_error() -> None:
    """Test vector xyz rotation error."""
    # Arrange
    vector = VectorXYZ(x=1.0, y=1.0, z=1.0)

    # Act and Assert
    with pytest.raises(ValueError):
        matrix = np.eye(4)
        vector.rotate(matrix)
