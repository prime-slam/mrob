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
import sys 
import subprocess


if __name__ == "__main__":
    python_path = sys.executable
    build_dir = "build"
    target_name = "python-package"
    config_mode = "Release"

    cmake_configure = subprocess.run(["cmake", "-B", build_dir, f"-DPYTHON_EXECUTABLE={python_path}"])
    cmake_build = subprocess.run(["cmake", "--build", build_dir, "--target", target_name, "--config", config_mode])
