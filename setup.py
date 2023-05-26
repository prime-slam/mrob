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


import setuptools


try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
    import platform, os, ctypes

    class bdist_wheel(_bdist_wheel):

        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            if platform.system() == "Darwin":
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

except ImportError:
    bdist_wheel = None

setuptools.setup(
    setuptools_git_versioning={
        "enabled": True,
        "sort_by": "creatordate",
    },
    setup_requires=['setuptools-git-versioning<2'],
    cmdclass={'bdist_wheel': bdist_wheel}
)
