#!/bin/bash
set -euo pipefail
export LC_ALL=C

export CIBW_ARCHS_MACOS="arm64"


# Get Python MAJOR.MINOR version to specify Python path for pybind
PYTHON_VERSION=$(python3 --version | grep -o 3.[0-9]*)

cmake -B cppbuild -DCMAKE_OSX_ARCHITECTURES=arm64 -DPYTHON_EXECUTABLE=$(which python${PYTHON_VERSION})
cmake --build cppbuild --target python-package 
