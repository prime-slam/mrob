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

# Build only the Python bindings against the core prebuilt by tools/windows-core.py
# (cibuildwheel: before-build, once per Python version). The build dir is recreated
# from scratch each time: FindPython caches its detection per build dir, so a fresh
# dir per interpreter is what makes multi-version builds correct by construction.
import os
import shutil
import subprocess
import sys

if __name__ == "__main__":
    shutil.rmtree("pybuild", ignore_errors=True)
    subprocess.run(["cmake", "-B", "pybuild", "-S", os.path.join("src", "pybind"),
                    f"-DPYTHON_EXECUTABLE={sys.executable}",
                    f"-DMROB_CORE_BUILD_DIR={os.path.abspath('build')}"], check=True)
    subprocess.run(["cmake", "--build", "pybuild", "--target", "python-package",
                    "--config", "Release", "--parallel"], check=True)
