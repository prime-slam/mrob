#!/bin/bash
# Copyright (c) 2022, Arthur Saliou, Anastasiia Kornilova
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail
export LC_ALL=C
export MACOSX_DEPLOYMENT_TARGET=10.15

cmake -B build 

# Python 3.6-3.10
PYTHON_VERSION=("3.6" "3.7" "3.8" "3.9" "3.10")

for v in "${PYTHON_VERSION[@]}"; do
  PYTHON_PATH=(/Users/runner/hostedtoolcache/Python/"${v}"/x64/bin/python*?[0-9])
  "${PYBIN}" -m pip install build
  cmake -B build -DPYTHON_EXECUTABLE="${PYBIN}"
  cmake --build build --target build-wheel
done

python3 -m pip install delocate==0.10.4

delocate-wheel -w wheelhouse -v ./build/wheels/*.whl
