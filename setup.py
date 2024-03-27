import os
import sys
from pathlib import Path
import platform

import cmake_build_extension
import setuptools

# Extra options passed to the CI/CD pipeline that uses cibuildwheel
CIBW_CMAKE_OPTIONS = []
if "CIBUILDWHEEL" in os.environ and os.environ["CIBUILDWHEEL"] == "1":
    # The manylinux variant runs in Debian Stretch and it uses lib64 folder
    if sys.platform == "linux":
        CIBW_CMAKE_OPTIONS += ["-DCMAKE_INSTALL_LIBDIR=lib"]


# Set Cmake options
if "CMAKE_ARGS" in os.environ:
    cmake_args = [m for m in os.environ["CMAKE_ARGS"].split()]
    user_flag = True
else:
    user_flag = False
    cmake_args = []

# Always build shared libraries
cmake_args += [f"-DPython3_ROOT_DIR={Path(sys.prefix)}",
               "-DCALL_FROM_SETUP_PY:BOOL=ON",
               "-DBUILD_SHARED_LIBS=ON"]

# Append fortran flags
if not user_flag or "-DCMAKE_Fortran_FLAGS" not in os.environ["CMAKE_ARGS"]:
    cmake_args += ['-DCMAKE_Fortran_FLAGS=-ffree-line-length-0']
else:
    for im, m in enumerate(cmake_args):
        if m.find("-DCMAKE_Fortran_FLAGS") >= 0:
            cmake_args[im] += ' -ffree-line-length-0' 

# Set if unset
#if not user_flag or "-DCMAKE_INSTALL_PREFIX" not in os.environ["CMAKE_ARGS"]:
#    cmake_args += [f'-DCMAKE_INSTALL_PREFIX={this_directory}/rosco']

# Set if unset
#if not user_flag or "-DCMAKE_PREFIX_PATH" not in os.environ["CMAKE_ARGS"]:
#    python_root = os.path.dirname( os.path.dirname( sysconfig.get_path('stdlib') ) )
#    cmake_args += [f'-DCMAKE_PREFIX_PATH={python_root}']

if platform.system() == 'Windows':
    if "FC" not in os.environ:
        os.environ["FC"] = "gfortran"

    #if "gfortran" in os.environ["FC"].lower():
    #    cmake_args += ['-G', 'MinGW Makefiles']
    #else:
    #    cmake_args += ['-DCMAKE_GENERATOR_PLATFORM=x64']
        
setuptools.setup(
    ext_modules=[
        cmake_build_extension.CMakeExtension(
            # This could be anything you like, it is used to create build folders
            name="rosco",
            install_prefix="rosco",
            # Selects the folder where the main CMakeLists.txt is stored
            source_dir=os.path.join('rosco','controller'),
            cmake_configure_options=cmake_args + CIBW_CMAKE_OPTIONS,
        ),
    ],
    cmdclass={'build_ext': cmake_build_extension.BuildExtension},
)
