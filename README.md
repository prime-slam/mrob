[![PyPi version](https://img.shields.io/pypi/v/mrob.svg)](https://pypi.org/project/mrob/)
[![PyPi downloads](https://img.shields.io/pypi/dm/mrob.svg)](https://pypi.org/project/mrob/)
[![Documentation Status](https://readthedocs.org/projects/mrob/badge/?version=latest)](https://mrob.readthedocs.io/en/latest/?badge=latest)

<p align="center">
  <img src="https://sites.skoltech.ru/app/data/uploads/sites/50/2018/02/mr_animate1.gif" width="450">
</p>

# MROB: Mobile Robotics library
The Mobile Robotics library (mrob) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities such as geometric transformations (SE3), factor graphs for general state estimation, optimization, 3D point cloud registration and more to come.

The general structure for the algorithms implemented:
* [common](https://github.com/prime-slam/mrob/tree/master/src/common): common matrix definitions and typedefs.
* [geometry](https://github.com/prime-slam/mrob/tree/master/src/geometry): Geometric transformations, mostly Rotations and Rigid Body Transformations in 3D.
* [Fgraph](https://github.com/prime-slam/mrob/tree/master/src/FGraph): Factor Graphs for state estimation
* [PCReg](https://github.com/prime-slam/mrob/tree/master/src/PCRegistration): Point Cloud Registration.
* [pybind](https://github.com/prime-slam/mrob/tree/master/src/pybind) Python bindings (using pybind11) for the above methods.


## Python package
The library is mainly designed to run in Python, that is, algorithms are written in C++ and bound with Python for general purpose use.

MROB has a [Python package](https://pypi.org/project/mrob/) for a wide range of versions of Windows, macOS and Linux.
### Installation
```bash
python -m pip install mrob
```
### Install on local machine 
```
python -m pip install cibuildwheel
cibuildwheel --platform [linux|windows|macos] mrobpy/
python -m pip install mrob --no-index --find-links wheelhouse/
```
For more options visit https://cibuildwheel.readthedocs.io/en/stable/setup/

### Documentation
For comprehensive description and usage examples in Python please refer to [mrobpy](https://github.com/prime-slam/mrob/tree/master/mrobpy#readme).

## Build from source
You can also use MROB as a pure C++ library.
### Dependencies
The present library is meant to be a self-contained library. However, there are few dependencies:
* C++'14
* CMake
* Python >= 3.8, with dev headers
* [Eigen](https://gitlab.com/libeigen/eigen) (fetched automatically by CMake at configure time)
* [pybind11](https://github.com/pybind/pybind11) (fetched automatically by CMake at configure time)
* [Catch2 v2.x branch](https://github.com/catchorg/Catch2/tree/v2.x) (fetched automatically by CMake at configure time, only when `-DBUILD_TESTING=ON`)

These dependencies are downloaded via CMake `FetchContent` when you configure the project, so the first configure needs network access; there are no git submodules to initialize.

This is the list of required packages to install:

`sudo apt install build-essential cmake python3-dev`

### Repository 
Standard github cloning:

`git clone https://github.com/prime-slam/mrob.git`

### Build with CMake
```
cmake -B build [-DBUILD_TESTING=ON]
cmake --build build --config Release
```

### Build Python wheel
You can also manually build a Python wheel, using [cibuildwheel](https://cibuildwheel.readthedocs.io/) as described above, or by building the bindings and then invoking Python's own build frontend:
```
cmake -B build -DPYTHON_EXECUTABLE=$(which python3)
cmake --build build --target python-package
python -m build --wheel mrobpy/
```
To build correct redistributable wheels we recommend building them on manylinux2014 for Linux, and on whatever macOS/Windows versions you need to support — see the actual CI matrix in [.github/workflows/wheels.yml](https://github.com/prime-slam/mrob/tree/master/.github/workflows/wheels.yml) for the versions this project currently builds and tests against.

You may also find useful pipeline presented in [tools](https://github.com/prime-slam/mrob/tree/master/tools).

**Note:** If your OS is Windows and you don't have Microsoft Visual C++ Redistributable package installed, 
then you need to [install it](https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-160#visual-studio-2015-2017-2019-and-2022) additionally.
If you are using a 32-bit Python, then install the package for the X86 architecture. 
If you are using 64-bit Python, then install the package for the X64 architecture. 
Don't be afraid to install both packages.


## License
Apache-2.0 License
