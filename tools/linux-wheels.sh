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

#Early check for build tools
cmake --version

# Python 3.6-3.10
PYTHON_VERSION=(36 37 38 39 310)

NUMPROC=$(nproc)
echo "Running $NUMPROC parallel jobs"

cmake -B build

for v in "${PYTHON_VERSION[@]}"; do
  PYTHON_PATH=(/opt/python/cp"${v}"*/bin/python)
  cmake -B build -DPYTHON_EXECUTABLE="${PYTHON_PATH[0]}"
  cmake --build build --target build-wheel
done

auditwheel repair ./build/wheels/*.whl

echo "Installing wheel and running example"

## TODO: Run examples
# for v in "${PYTHON_VERSION[@]}"; do
#   PYTHON_PATH=(/opt/python/cp"${v}"*/bin/python)
#   "${PYTHON_PATH[0]}" -m pip install mrob --find-links=./build/wheels
#   "${PYTHON_PATH[0]}" <path_to_example>
# done
