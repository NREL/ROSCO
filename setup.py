# Copyright 2019 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

"""The setup script."""

# This setup file was made to mimic the setup.py script for https://github.com/NREL/floris/
# Accessed on January 9, 2020

# Note: To use the 'upload' functionality of this file, you must:
#   $ pip install twine

import io
import os
import sys
from shutil import rmtree, copy
import glob 
import platform
from setuptools import find_packages, setup, Command, Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.install import install as _install

from io import open

# Package meta-data.
NAME = 'rosco'
DESCRIPTION = 'A reference open source controller toolset for wind turbine applications.'
URL = 'https://github.com/NREL/ROSCO'
EMAIL = 'nikhar.abbas@nrel.gov'
AUTHOR = 'NREL, National Wind Technology Center'
REQUIRES_PYTHON = '>=3.4'
VERSION = '2.5.0'

# These packages are required for all of the code to be executed. 
# - Maybe you can get away with older versions...
REQUIRED = [
    'matplotlib',
    'numpy',
    'pytest',
    'scipy',
    'pyYAML',
    'future',
    'pandas'
]


# For the CMake Extensions
this_directory = os.path.abspath(os.path.dirname(__file__))
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
            localdir = os.path.join(this_directory, 'ROSCO','install')
            os.makedirs(localdir, exist_ok=True)

            cmake_args = ['-DBUILD_SHARED_LIBS=OFF']
            cmake_args += ['-DCMAKE_Fortran_FLAGS=-ffree-line-length-0']
            cmake_args += ['-DCMAKE_INSTALL_PREFIX={}'.format(localdir)]

            if platform.system() == 'Windows':
                if "gfortran" in os.environ["FC"].lower():
                    cmake_args += ['-G', 'MinGW Makefiles']
                elif self.compiler.compiler_type == 'msvc':
                    cmake_args += ['-DCMAKE_GENERATOR_PLATFORM=x64']
                else:
                    raise ValueError("Unable to find the system's Fortran compiler.")

            self.build_temp = os.path.join( os.path.dirname( os.path.realpath(__file__) ), 'ROSCO', 'build')
            os.makedirs(localdir, exist_ok=True)
            # Need fresh build directory for CMake
            os.makedirs(self.build_temp, exist_ok=True)

            self.spawn(['cmake', '-S', ext.sourcedir, '-B', self.build_temp] + cmake_args)
            self.spawn(['cmake', '--build', self.build_temp, '--target', 'install', '--config', 'Release'])



# All of the extensions
roscoExt   = CMakeExtension('rosco','ROSCO')

# The rest you shouldn't have to touch too much :)
# ------------------------------------------------
# Except, perhaps the License and Trove Classifiers!
# If you do change the License, remember to change the Trove Classifier for that!

here = os.path.abspath(os.path.dirname(__file__))

# Import the README and use it as the long-description.
try:
    with io.open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
        long_description = '\n' + f.read()
except FileNotFoundError:
    long_description = DESCRIPTION

# Load the package's __version__.py module as a dictionary.
about = {}
if not VERSION:
    project_slug = NAME.lower().replace("-", "_").replace(" ", "_")
    with open(os.path.join(here, project_slug, '__version__.py')) as f:
        exec(f.read(), about)
else:
    about['__version__'] = VERSION


class UploadCommand(Command):
    """Support setup.py upload."""

    description = 'Build and publish the package.'
    user_options = []

    @staticmethod
    def status(s):
        """Prints things in bold."""
        print('\033[1m{0}\033[0m'.format(s))

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        try:
            self.status('Removing previous builds...')
            rmtree(os.path.join(here, 'dist'))
        except OSError:
            pass

        self.status('Building Source and Wheel (universal) distribution...')
        os.system('{0} setup.py sdist bdist_wheel --universal'.format(sys.executable))

        self.status('Uploading the package to PyPI via Twine...')
        os.system('twine upload dist/*')

        self.status('Pushing git tags...')
        os.system('git tag v{0}'.format(about['__version__']))
        os.system('git push --tags')
        
        sys.exit()



metadata = dict(
    name                          = NAME,
    version                       = about['__version__'],
    description                   = DESCRIPTION,
    long_description              = long_description,
    long_description_content_type = 'text/markdown',
    author                        = AUTHOR,
    author_email                  = EMAIL,
    url                           = URL,
    install_requires              = REQUIRED,
    python_requires               = REQUIRES_PYTHON,
    packages                      = find_packages(exclude=["tests", "*.tests", "*.tests.*", "tests.*"]),
    package_data                  = {'': ['*.yaml']},
    license                       = 'Apache License, Version 2.0',
    cmdclass                      = {'build_ext': CMakeBuildExt, 'upload': UploadCommand},
    zip_safe                      = False,
)
if "--compile-rosco" in sys.argv:
    metadata['ext_modules'] = [roscoExt]
    sys.argv.remove("--compile-rosco")

setup(**metadata)
