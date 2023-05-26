from setuptools import find_packages, setup
from setuptools.command.install import install as _install
import ctypes
import logging
import os


cmdclass = dict()

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
    name="deplex",
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
