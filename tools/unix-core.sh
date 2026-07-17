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

# Build the C++ core once per platform (cibuildwheel: before-all). It does not depend
# on Python, so every per-version wheel build reuses these libraries via the build-tree
# export cppbuild/mrobTargets.cmake instead of recompiling them.
cmake -B cppbuild -DCMAKE_OSX_ARCHITECTURES="x86_64;arm64" -DBUILD_PYTHON_BINDINGS=OFF
cmake --build cppbuild --parallel "$(getconf _NPROCESSORS_ONLN)"
