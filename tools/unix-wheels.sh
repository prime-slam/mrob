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

# Get Python MAJOR.MINOR version to specify Python path for pybind
PYTHON_VERSION=$(python3 --version | grep -o 3.[0-9]*)

cmake -B cppbuild -DCMAKE_OSX_ARCHITECTURES="x86_64;arm64" -DPYTHON_EXECUTABLE=$(which python${PYTHON_VERSION})
cmake --build cppbuild --target python-package 
