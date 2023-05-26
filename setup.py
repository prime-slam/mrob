#  Copyright (c) 2022, Gonzalo Ferrer
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
# 
# 
#  setup.py
# 
#  Created on: Jan 22, 2020
#       Author: Lyubov Miloserdova
#               miloslubov@gmail.com
#


from setuptools import find_packages, setup
from setuptools.command.install import install as _install
import ctypes

cmdclass = dict()

try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
    import platform

    class bdist_wheel(_bdist_wheel):
        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            self.root_is_pure = False

        def get_tag(self):
            python, abi, plat = _bdist_wheel.get_tag(self)
            print(f"BDIST PY,ABI,PLAT: {python}, {abi}, {plat}") 
            if platform.system() == "Windows":
                if ctypes.sizeof(ctypes.c_voidp) * 8 > 32:
                    plat = "win_" + platform.machine().lower()
                else:
                    plat = "win32"
            return python, abi, plat
        
    
    cmdclass['bdist_wheel'] = bdist_wheel

except ImportError:
    bdist_wheel = None


class install(_install):
    def finalize_options(self):
        _install.finalize_options(self)
        self.install_libbase = self.install_platlib
        self.install_lib = self.install_platlib


cmdclass['install'] = install


setuptools.setup(
    setuptools_git_versioning={
        "enabled": True,
        "sort_by": "creatordate",
    },
    setup_requires=['setuptools-git-versioning<2'],
    cmdclass=cmdclass
)
