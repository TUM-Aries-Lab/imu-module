# imu-python

IMU sensor management package for the ARIES Lab (IBRS, TUM). Handles hardware communication, auto-detection, orientation estimation, magnetometer calibration, and thread-safe data delivery for BNO055, LSM6DSOX+LIS3MDL, and BNO08x IMU sensors on a Jetson Orin Nano.

**PyPI:** imu-python | **GitHub:** TUM-Aries-Lab/imu-module | **Current version:** 0.1.0 | **Python:** 3.11+

## Project Structure

```
src/imu_python/
├── __init__.py
├── __main__.py                  # Standalone testing (detects IMUs, prints Euler angles)
├── definitions.py               # ALL constants, configs, enums
├── base_classes.py              # Core data classes: VectorXYZ, Quaternion, IMUData, IMUConfig, etc.
├── factory.py                   # IMUFactory — auto-detects sensors on I2C bus
├── sensor_manager.py            # IMUManager — thread-safe background reading
├── wrapper.py                   # IMUWrapper — hardware abstraction, axis remapping, mag calibration
├── devices.py                   # IMUDevices enum — config registry for all supported sensors
├── i2c_bus.py                   # JetsonBus — I2C bus singleton for Jetson Orin Nano
├── orientation_filters.py       # Madgwick filter wrappers (PyIMU default, AHRS alternative)
├── calibration/
│   ├── calibration.py           # Top-level calibration workflow
│   ├── mag_calibration.py       # MagCalibration — ellipsoid fitting + quality metrics
│   ├── ellipsoid_fitting.py     # LS and MLE fitting algorithms
│   └── gain_calculator.py       # Madgwick filter gain from gyro noise
├── data_handler/
│   ├── data_writer.py           # IMUFileWriter — CSV recording
│   ├── data_reader.py           # load_imu_data() — CSV loading
│   └── data_plotter.py          # Matplotlib IMU data visualization
└── utils.py                     # Logging setup, timestamped files
```

Uses src-layout. Package metadata and dependencies in `pyproject.toml`.
Tests in `tests/` at root level.

## Data Flow

```
Hardware (BNO055 / LSM6DSOX+LIS3MDL / BNO08x)
    │
    ├─► IMUWrapper.get_imu_data()
    │       ├─► read_sensor(accel) → VectorXYZ (m/s²)
    │       ├─► read_sensor(gyro) → VectorXYZ (rad/s)
    │       ├─► read_sensor(mag) → VectorXYZ | None
    │       ├─► apply 3×3 rotation matrix (axis remapping)
    │       ├─► apply mag calibration (hard-iron + soft-iron)
    │       └─► return IMUDeviceData (frozen dataclass)
    │
    ├─► IMUManager._loop() [daemon thread]
    │       ├─► check data freshness (reject stale register reads)
    │       ├─► MadgwickFilter.update(accel, gyro, mag) → Quaternion
    │       ├─► create IMUData(timestamp, quat, device_data)
    │       ├─► store as latest_data (thread-safe via Lock)
    │       └─► on I2C error: log, reinitialize sensor, retry
    │
    └─► IMUManager.get_data() → IMUData
            └─► Consumer (exosuit-python control loop)
                    ├─► data.quat.to_euler("xyz").z  → hip angle (rad)
                    └─► data.device_data.gyro.z      → angular velocity (rad/s)
```

## Supported Sensors

- **BNO055** — 9-DOF (Bosch). AMG mode (no onboard fusion), software Madgwick. Addresses: 0x28/0x29. Accel ±4g, Gyro ±2000 dps, filter gain 0.00225.
- **LSM6DSOX + LIS3MDL** — Split IMU (two chips). LSM6DSOX (accel+gyro) 0x6A/0x6B + LIS3MDL (mag) 0x1C/0x1E. Accel ±4g, Gyro ±500 dps, 104 Hz. Filter gain 0.000573.
- **BNO08x** — 9-DOF (Hillcrest/CEVA). Feature reports. Addresses: 0x4A/0x4B. 20 Hz default. Filter gain 0.001538.
- **MOCK** — Simulated IMU for testing without hardware. Random noise around gravity.

## Key Design Patterns

- **Factory pattern** for auto-detection: `IMUFactory.detect_and_create()` scans I2C bus, matches addresses to known configs, creates managers. Swap hardware without code changes.
- **Thread-per-sensor**: Each `IMUManager` runs a daemon thread for continuous reading. Main thread retrieves latest data via `get_data()` with Lock.
- **Driver-agnostic wrapper**: `IMUWrapper` dynamically imports driver modules from `SensorConfig.library` at runtime. Adding a new sensor = adding an `IMUDevices` enum entry only.
- **Software orientation fusion**: Even BNO055 uses AMG mode (raw readings) + software Madgwick, for consistent behavior across sensor types.
- **Offline mag calibration**: Ellipsoid fitting (LS/MLE) → JSON → applied in real-time during reads. No mag cal = mag disabled automatically.
- **Frozen dataclasses** for immutable sensor readings: `IMUData`, `IMUDeviceData`

## Hardware Context

- **Platform:** Jetson Orin Nano
- **I2C Bus 1** (pins 27/28): Hip IMU
- **I2C Bus 7** (pins 3/5): Left and right leg IMUs
- `JetsonBus` is a singleton — initialized lazily, cached for reuse

## Important Implementation Details

- **Large dt protection** in `MadgwickFilterPyIMU`: If dt > 10/frequency, dt is clipped to 1/frequency to prevent filter divergence after reconnection. `MadgwickFilterAHRS` does NOT have this protection.
- **Clipped parameter** is accepted in `MadgwickFilterPyIMU.update()` but not used — clipping detection has no effect with the default filter (only `MadgwickFilterAHRS` adjusts gain on clipping).
- **Data freshness check**: `_acc_gyro_are_fresh()` and `_mag_is_fresh()` reject duplicate hardware register reads. Mag checked independently since it may update at a different rate.
- **IMUDataFile iteration quirk**: `__iter__` yields `list[IMUData]` where each list contains exactly one element — known design quirk.
- **BNO055 gyro range**: The range setting (±2000 dps) doesn't actually work on BNO055 hardware — it's a known hardware limitation.

## How to Run

```bash
# Install from source
pip install -e .

# Standalone test (detect + print Euler angles)
python -m imu_python

# Run magnetometer calibration
python -m imu_python.calibration

# Run tests
pytest

# Run linter
ruff check .
```

## This Package in Context

`imu-python` is consumed by `exosuit-python`, which calls `IMUFactory.detect_and_create()` at startup. The hip angle and velocity from `IMUData` are passed to `hip-controller` for gait phase estimation and motor command generation. Sibling packages:
- `hip-controller` — Gait phase estimation and motor command generation
- `motor-python` — CubeMars motor communication via CAN/serial
- `exosuit-python` — Top-level integration, threading, CSV recording

## Conventions

See `.claude/skills/code-review.md` for the full coding conventions checklist. Key highlights:
- Physical quantities must include units in variable names: `accel_range_g`, `gyro_range_dps`
- Type hints on all function signatures, modern syntax (`float | None`, `NDArray`)
- Sphinx/reST docstrings on all public classes and functions
- Use `loguru` for logging, not Python's built-in `logging`
- Use `pathlib.Path`, not `os.path`
- Frozen dataclasses for immutable data objects
- No bare `except:` — always catch specific exceptions (especially `OSError` for I2C)

## Restrictions

- **Never modify I2C bus pin assignments** without hardware verification
- **Never change sensor address mappings** in `IMUDevices` without checking physical wiring
- **Never modify magnetometer calibration parameters** — they are computed from physical measurements
- **Never remove the large-dt protection** in the Madgwick filter — it prevents filter divergence
- **Never hardcode hardware-specific paths** (I2C bus numbers, device paths)
- **Never push directly to main** — always use pull requests
- Always run `ruff check .` and `pytest` before considering a task complete
