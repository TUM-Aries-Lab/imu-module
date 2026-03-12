import csv
from imu_python.base_classes import Quaternion

def load_quaternions_from_csv(filepath):
    """
    Load timestamps and quaternions from a CSV file.

    Args:
        filepath: Path to the CSV file with columns: time, w, x, y, z

    Returns:
        tuple: (timestamps list, quaternions list)
    """
    timestamps = []
    quaternions = []

    with open(filepath, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            timestamps.append(float(row['time']))
            quaternion = Quaternion(
                w=float(row['w']),
                x=float(row['x']),
                y=float(row['y']),
                z=float(row['z'])
            )
            quaternions.append(quaternion)

    return timestamps, quaternions
