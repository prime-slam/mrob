#!/bin/bash
#  Copyright (c) 2023, Gonzalo Ferrer
# 
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

set -euo pipefail
export LC_ALL=C

# Build only the Python bindings against the core prebuilt by tools/unix-core.sh
# (cibuildwheel: before-build, once per Python version). The build dir is recreated
# from scratch each time: FindPython caches its detection per build dir, so a fresh
# dir per interpreter is what makes multi-version builds correct by construction.

# Get Python MAJOR.MINOR version to specify Python path for pybind
PYTHON_VERSION=$(python3 --version | grep -o 3.[0-9]*)

rm -rf pybuild
cmake -B pybuild -S src/pybind -DCMAKE_OSX_ARCHITECTURES="x86_64;arm64" \
      -DPYTHON_EXECUTABLE=$(which python${PYTHON_VERSION}) \
      -DMROB_CORE_BUILD_DIR=$PWD/cppbuild
cmake --build pybuild --target python-package --parallel "$(getconf _NPROCESSORS_ONLN)"
