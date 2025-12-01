# IMU Module for Python
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/imu-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/imu-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/imu-module/actions/workflows/ci.yml/badge.svg)

This is the repository for imu sensor codes for the lower-limb exosuit.


## Install
To install the library run:

```bash
uv install imu_python
```

OR

```bash
uv install git+https://github.com/TUM-Aries-Lab/imu_python.git@<specific-tag>
```

## Development
0. Install [uv](https://docs.astral.sh/uv/getting-started/installation/)
1. ```git clone git@github.com:TUM-Aries-Lab/imu-module.git```
2. `make init` to create the virtual environment and install dependencies
3. `make format` to format the code and check for errors
4. `make test` to run the test suite
5. `make clean` to delete the temporary files and directories

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:

```bash
uv build
uv publish  # make sure your version in pyproject.toml is updated
```
The package can then be found at: https://pypi.org/project/imu_python

## Module Usage
```python
"""Basic docstring for my module."""

from loguru import logger

from imu_python.config import definitions

def main() -> None:
    """Run a simple demonstration."""
    logger.info("Hello World!")

if __name__ == "__main__":
    main()
```

## Program Usage
```bash
uv run python -m imu_python
```

## Structure
<!-- TREE-START -->
```
├── src
│   └── imu_python
│       ├── __init__.py
│       ├── __main__.py
│       ├── definitions.py
│       ├── imu_sensor.py
│       └── utils.py
├── tests
│   ├── __init__.py
│   ├── conftest.py
│   └── utils_test.py
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
