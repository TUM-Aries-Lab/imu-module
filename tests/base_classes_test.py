"""Test the base_classes module."""

import numpy as np
import pytest

from imu_python.base_classes import IMUData, IMURawData, Quaternion, VectorXYZ


@pytest.mark.parametrize("t, x, y, z", [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 2.0, 3.0)])
def test_imu_data(t: float, x: float, y: float, z: float) -> None:
    """Test imu_data."""
    # Arrange
    vec_xyz = VectorXYZ(x=x, y=y, z=z)
    quat = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

    # Act
    imu_data = IMUData(
        timestamp=t,
        raw_data=IMURawData(
            accel=vec_xyz,
            gyro=vec_xyz,
            mag=vec_xyz,
        ),
        quat=quat,
    )

    # Assert
    assert imu_data.timestamp == t
    assert imu_data.raw_data.accel == vec_xyz
    assert imu_data.raw_data.gyro == vec_xyz
    assert imu_data.raw_data.mag == vec_xyz
    assert imu_data.quat == quat


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


def test_vector_xyz_rotate() -> None:
    """Test vector xyz rotation."""
    # Arrange
    x, y, z = (1.0, -1.0, 2.0)
    vector = VectorXYZ(x=x, y=y, z=z)
    rotation_matrix = np.array(
        [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
    )

    # Act
    vector.rotate(rotation_matrix)

    # Assert
    assert vector.x == y

    assert vector.y == x

    assert vector.z == -z


def test_vector_xyz_rotate_error() -> None:
    """Test vector xyz rotation error."""
    # Arrange
    vector = VectorXYZ(x=1.0, y=1.0, z=1.0)

    # Act and Assert
    with pytest.raises(ValueError):
        matrix = np.eye(4)
        vector.rotate(rotation_matrix=matrix)


@pytest.mark.parametrize("rot_x_rad", [0, np.pi / 2, -np.pi / 2])
def test_quaternion_to_euler(rot_x_rad: float) -> None:
    """Test quaternion to euler method."""
    # Arrange
    quat = Quaternion(w=np.cos(rot_x_rad / 2), x=np.sin(rot_x_rad / 2), y=0.0, z=0.0)

    # Act
    euler = quat.to_euler(seq="xyz")

    # Assert
    np.testing.assert_almost_equal(euler.x, rot_x_rad)
    np.testing.assert_almost_equal(euler.y, 0)
    np.testing.assert_almost_equal(euler.z, 0)
