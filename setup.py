# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

import os
import platform
import shutil
import sysconfig
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

#######
# This forces wheels to be platform specific
from setuptools.dist import Distribution
from wheel.bdist_wheel import bdist_wheel as _bdist_wheel

class bdist_wheel(_bdist_wheel):
    def finalize_options(self):
        _bdist_wheel.finalize_options(self)
        self.root_is_pure = False

class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name"""
    def has_ext_modules(foo):
        return True
#######

# For the CMake Extensions
this_directory = os.path.abspath(os.path.dirname(__file__))
build_dir = os.path.join(this_directory, "build")

class CMakeExtension(Extension):

    def __init__(self, name, sourcedir='', **kwa):
        Extension.__init__(self, name, sources=[], **kwa)
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuildExt(build_ext):
    
    def copy_extensions_to_source(self):
        newext = []
        for ext in self.extensions:
            if isinstance(ext, CMakeExtension): continue
            newext.append( ext )
        self.extensions = newext
        super().copy_extensions_to_source()
    
    def build_extension(self, ext):
        if not isinstance(ext, CMakeExtension):
            super().build_extension(ext)

        else:
            # Ensure that CMake is present and working
            try:
                self.spawn(['cmake', '--version'])
            except OSError:
                raise RuntimeError('Cannot find CMake executable')
            
            # Refresh build directory
            #os.makedirs(localdir, exist_ok=True)

            cmake_args = ['-DBUILD_SHARED_LIBS=OFF']
            cmake_args += ['-DCMAKE_Fortran_FLAGS=-ffree-line-length-0']
            cmake_args += ['-DCMAKE_INSTALL_PREFIX={}'.format(this_directory)]

            # Help Cmake find libraries in python locations
            python_root = os.path.dirname( os.path.dirname( sysconfig.get_path('stdlib') ) )
            user_root = sysconfig.get_config_var("userbase")
            cmake_args += [f'-DCMAKE_PREFIX_PATH={python_root}']

            if platform.system() == 'Windows':
                if not "FC" in os.environ:
                    os.environ["FC"] = "gfortran"
                    
                if "gfortran" in os.environ["FC"].lower():
                    cmake_args += ['-G', 'MinGW Makefiles']
                elif self.compiler.compiler_type == 'msvc':
                    cmake_args += ['-DCMAKE_GENERATOR_PLATFORM=x64']
                else:
                    raise ValueError("Unable to find the system's Fortran compiler.")

            self.build_temp = build_dir

            # Need fresh build directory for CMake
            os.makedirs(self.build_temp, exist_ok=True)

            self.spawn(['cmake', '-B', self.build_temp, '-S', ext.sourcedir] + cmake_args)
            self.spawn(['cmake', '--build', self.build_temp, '--target', 'install', '--config', 'Release'])

            
if __name__ == "__main__":
    # Start with clean build directory
    shutil.rmtree(build_dir, ignore_errors=True)
    
    setup(cmdclass={'bdist_wheel': bdist_wheel, 'build_ext': CMakeBuildExt},
          distclass=BinaryDistribution,
          ext_modules=[ CMakeExtension('rosco',os.path.join('rosco','controller')) ],
          )
    
