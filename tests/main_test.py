"""Test the main program."""

from imu_utilities.__main__ import main


def test_main():
    """Test the main function."""
    assert main() is None
