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
### Documentation
For comprehensive description and usage examples in Python please refer to [mrobpy](https://github.com/prime-slam/mrob/tree/master/mrobpy#readme).

## Build from source
You can also use MROB as a pure C++ library.
### Dependencies
The present library is meant to be a self-contained library. However, there are few dependencies:
* C++'14
* CMake
* [Eigen](https://gitlab.com/libeigen/eigen) (included as a submodule)
* [pybind11](https://github.com/pybind/pybind11) (included as a submodule)
  - python3-distutils
  - python3-dev
* [Catch2 v2.x branch](https://github.com/catchorg/Catch2/tree/v2.x) (included as a submodule)

This is the list of required packages to install:

`sudo apt install build-essential cmake python3-distutils python3-dev`

### Repository 
Standard github cloning, adding the recursive term for submodules.

`git clone --recursive https://github.com/prime-slam/mrob.git`

If there was ever a submodule update (not frequently) the command to use:

`git submodule update --recursive`

### Build with CMake
```
cmake -B build [-DBUILD_TESTING=ON]
cmake --build build --config Release
```

### Build Python wheel
You can also manually build a Python wheel
```
cmake -B build 
cmake --build build --target build-wheel
```
To build a correct redistributable wheel we recommend building them on manylinux-2014, macOS 10.15 and Windows 2019 for a variety of version of Linux, macOS and Windows respectively.

You may also find useful pipeline presented in [tools](https://github.com/prime-slam/mrob/tree/master/tools).

**Note:** If your OS is Windows and you don't have Microsoft Visual C++ Redistributable package installed, 
then you need to [install it](https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-160#visual-studio-2015-2017-2019-and-2022) additionally.
If you are using a 32-bit Python, then install the package for the X86 architecture. 
If you are using 64-bit Python, then install the package for the X64 architecture. 
Don't be afraid to install both packages.


## License
Apache-2.0 License
