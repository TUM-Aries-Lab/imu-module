# template-python
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/template-python/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/template-python?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/template-python/actions/workflows/ci.yml/badge.svg)

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
1. Install [pyenv](https://github.com/pyenv/pyenv?tab=readme-ov-file#installation)
2. ```pyenv install 3.12  # install the required python version 3.12```
3. ```pyenv global 3.12  # set the required python version 3.12```
4. ```git clone git@github.com:TUM-Aries-Lab/imu-module.git```
5. `make init` to create the virtual environment and install dependencies
6. `make format` to format the code and check for errors
7. `make test` to run the test suite
8. `make clean` to delete the temporary files and directories

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
