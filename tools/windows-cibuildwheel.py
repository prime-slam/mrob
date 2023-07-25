import sys 
import subprocess


if __name__ == "__main__":
    python_path = sys.executable
    build_dir = "build"
    target_name = "python-package"
    config_mode = "Release"

    cmake_configure = subprocess.run(["cmake", "-B", build_dir, f"-DPYTHON_EXECUTABLE={python_path}"])
    cmake_build = subprocess.run(["cmake", "--build", build_dir, "--target", target_name, "--config", config_mode])
