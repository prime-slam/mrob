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

from setuptools import find_packages, setup
from setuptools.command.install import install as _install
import ctypes
import logging

import shutil
import pathlib


cmdclass = dict()
package_name = "mrob"

# Clear setuptools build lib directory (https://github.com/pypa/setuptools/issues/1871)
build_lib_path = pathlib.Path(f"build/lib/{package_name}")
if build_lib_path.exists():
    shutil.rmtree(build_lib_path)

try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel

    class bdist_wheel(_bdist_wheel):
        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            self.root_is_pure = False

        def get_tag(self):
            python, abi, plat = _bdist_wheel.get_tag(self)
            if plat[:5] == 'linux':
                libc = ctypes.CDLL('libc.so.6')
                libc.gnu_get_libc_version.restype = ctypes.c_char_p
                GLIBC_VER = libc.gnu_get_libc_version().decode('utf8').split('.')
                plat = f'manylinux_{GLIBC_VER[0]}_{GLIBC_VER[1]}{plat[5:]}'
            return python, abi, plat


    cmdclass['bdist_wheel'] = bdist_wheel

except ImportError:
    logging.warn("Wheel package missing!")


class install(_install):
    def finalize_options(self):
        _install.finalize_options(self)
        self.install_libbase = self.install_platlib
        self.install_lib = self.install_platlib


cmdclass['install'] = install

print("pcks", find_packages())
setup_args = dict(
    name=package_name,
    setuptools_git_versioning={
        "enabled": True,
        "sort_by": "creatordate",
    },
    setup_requires=["setuptools-git-versioning<2"],
    zip_safe=False,
    packages=find_packages(),
    include_package_data=True,
    cmdclass=cmdclass
)

setup(**setup_args)
