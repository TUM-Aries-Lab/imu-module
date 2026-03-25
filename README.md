# IMU Module for Python
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/imu-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/imu-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/imu-module/actions/workflows/ci.yml/badge.svg)

This is the repository for imu sensor codes for the lower-limb exosuit.

To maximize data rates of multiple connected IMUs, **Python free-threading build is required for free threading**. Free threading is always disabled when running on versions prior to 3.13, and non free-threading builds.

## Install
To install the library, run:

```bash
uv pip install imu-python
```
OR
```bash
uv pip install git+https://github.com/TUM-Aries-Lab/imu-module.git@<specific-tag>
```


## Development
1. Install [uv](https://docs.astral.sh/uv/getting-started/installation/)
2. ```git clone git@github.com:TUM-Aries-Lab/imu-module.git```
3. `make init` to create the virtual environment and install dependencies
4. `make format` to format the code and check for errors
5. `make test` to run the test suite
6. `make clean` to delete the temporary files and directories

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package, run:
1. Update the version number in pyproject.toml and imu_module/__init__.py
2. Commit your changes and add a git tag "<new.version.number>"
3. Push the tag `git push --tag`

The package can then be found at: https://pypi.org/project/imu-python

## Module Usage

```python
"""Basic module usage example."""
import time

from loguru import logger

from imu_python.definitions import IMUUpdateTime
from imu_python.factory import IMUFactory


def main() -> None:
    """Run a simple demonstration."""
    sensor_managers = IMUFactory.detect_and_create()
    for manager in sensor_managers:
        manager.start()

    try:
        while True:
            for manager in sensor_managers:
                logger.info(manager.get_data())
            time.sleep(IMUUpdateTime.freq_hz)
            #Note: this read frequency is independent of IMUs actual hardware frequency
    except KeyboardInterrupt:
        for manager in sensor_managers:
            manager.stop()


if __name__ == "__main__":
    main()
```

## Program Usage
To run the main pipeline for all connected sensors with optional flag `-r` to record data:
```bash
uv run python -m imu_python
```
To plot a recorded data file, replace `filepath` with a path to data file:
```bash
uv run python src/imu_python/data_handler/data_plotter.py -f "filepath"
```
To calibrate all connected sensors:
```bash
make calibrate
```

## Structure
<!-- TREE-START -->
```
├── src
│   └── imu_python
│       ├── calibration
│       │   ├── calibration.py
│       │   ├── ellipsoid_fitting.py
│       │   ├── gain_calculator.py
│       │   └── mag_calibration.py
│       ├── data_handler
│       │   ├── data_plotter.py
│       │   ├── data_reader.py
│       │   └── data_writer.py
│       ├── __init__.py
│       ├── __main__.py
│       ├── base_classes.py
│       ├── definitions.py
│       ├── devices.py
│       ├── factory.py
│       ├── i2c_bus.py
│       ├── orientation_filters.py
│       ├── sensor_manager.py
│       ├── utils.py
│       └── wrapper.py
├── tests
│   ├── calibration
│   │   ├── calibration_test.py
│   │   └── gain_calculator_test.py
│   ├── data_handler
│   │   ├── reader_test.py
│   │   └── writer_test.py
│   ├── __init__.py
│   ├── base_classes_test.py
│   ├── conftest.py
│   ├── devices_test.py
│   ├── factory_test.py
│   ├── i2c_bus_test.py
│   ├── orientation_filter_test.py
│   ├── sensor_manager_test.py
│   ├── utils_test.py
│   └── wrapper_test.py
├── .dockerignore
├── .gitignore
├── .pre-commit-config.yaml
├── .python-version
├── CONTRIBUTING.md
├── Dockerfile
├── LICENSE
├── Makefile
├── README.md
├── pyproject.toml
├── repo_tree.py
└── uv.lock
```
<!-- TREE-END -->
