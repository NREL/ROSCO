.. toctree::

.. _install:

Installing the ROSCO tools
===========================
Depending on what is needed, a user can choose to use just the ROSCO controller or to use both the ROSCO controller and the toolbox. Both the controller and the toolbox should be installed if one wishes to leverage the full ROSCO toolchain. 

It is recommended to install the ROSCO toolset in full following the instruction in :ref:`full_rosco`

For users planning to only download and/or compile the ROSCO controller, please follow the instructions on :ref:`rosco_controller`. 

.. _full_rosco:

Full ROSCO 
----------

We recommend using the full ROSCO toolchain. This also eases the installation process.

Installing
..............
Installation of the complete ROSCO toolset is made easy through `Anaconda <https://www.anaconda.com/>`_. If you do not already have Anaconda installed on your machine, please install it. Additionally, we primarily support the use of CMake_ to control the software compilation process. If you plan to compile the ROSCO controller's source code, we request that you download CMake as well.

Then please follow the following steps:

1.  Create a conda environment for ROSCO

    ::

        conda config --add channels conda-forge
        conda create -y --name rosco-env python=3.8
        conda activate rosco-env

2.  Install WISDEM

    ::

        conda install -y wisdem

You should then do step 3a *or* 3b. 
If you do not want to compile the ROSCO controller while also installing the ROSCO toolbox, please follow the instructions for :ref:`compiling_rosco`.

3. Clone and Install the ROSCO toolbox with ROSCO

    :: 

        git clone https://github.com/NREL/ROSCO_toolbox.git
        cd ROSCO_toolbox
        conda install compilers # (Mac/Linux only)
        conda install m2w64-toolchain libpython # (Windows only)
        python setup.py install --compile-rosco 

4.	Clone and Install the ROSCO toolbox without ROSCO

    ::
    
        git clone https://github.com/NREL/ROSCO_toolbox.git
        cd ROSCO_toolbox
        python setup.py install


**Alternatively...**

If you wish to write your own scripts to leverage the ROSCO tools, but do not necessarily need the source code or to run any of the examples, the ROSCO toolbox is available via Conda-Forge:

    ::

        conda install -y ROSCO

Note that if you do choose to install ROSCO this way, you will not have the source code. This will install the python-based ROSCO toolbox and download a compiled ROSCO controller binary file into the default filepath. The ROSCO binary file can be copied to your desired folder using:
::

    cp $CONDA_PREFIX/lib/libdiscon.* <desired_folder>

on linux or:
::

    copy %CONDA_PREFIX%/lib/libdiscon.dll <desired_folder>

on Windows.


.. _rosco_controller:

ROSCO controller
----------------
The standard ROSCO controller is based in Fortran and must be compiled; this code can be found at: https://github.com/NREL/ROSCO/ROSCO. 
We primarily support the use of CMake_ for setting up the necessary build files to compile ROSCO. The most recent tagged version releases of the controller are `available for download <https://github.com/NREL/ROSCO/tags>`_. 

If one wishes to download the code via the command line, we provide two supported options in the subsections below. 
For non-developers (those not interested in modifying the source code), the a 64-bit version of the compiled controller can be downloaded via Anaconda. 
For users needing a 32-bit version on Windows and/or developers, CMake can be used to properly compile the Fortran code. 


.. _compiling_rosco:

Anaconda download for non-developers
.....................................

For users familiar with Anaconda_, a 64-bit version of ROSCO is available through the conda-forge channel. 
In order to download the most recently compiled version release, from an anaconda powershell (Windows) or terminal (Mac/Linux) window, create a new anaconda virtual environment: 
::

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.8
    conda activate rosco-env

navigate to your desired folder to save the compiled binary using:
::

    cd <my_desired_folder>
    
and download the controller:
::

    conda install -y ROSCO

This will download a compiled ROSCO binary file into the default filepath for any dynamic libraries downloaded via anaconda while in the ROSCO-env. 
The ROSCO binary file can be copied to your desired folder using:
::

    cp $CONDA_PREFIX/lib/libdiscon.* .

on linux or:
::

    copy %CONDA_PREFIX%/lib/libdiscon.* .


on Windows.


CMake for developers (Mac/linux)
.................................
CMake_ provides a straightforward option for many users, particularly those on a Mac or Linux. We recommend that users use CMake if at all possible, as it is more difficult for us to support the use of other tools to aid with compiling ROSCO

On Mac/Linux, ROSCO can be compiled by first cloning the source code from git using:
::

    git clone https://github.com/NREL/ROSCO.git

And then compiling using CMake:
::

    cd ROSCO/ROSCO
    mkdir build
    cd build
    cmake ..
    make install

This will generate a file called :code:`libdiscon.so` (Linux) or :code:`libdiscon.dylib` (Mac) in the :code:`/ROSCO/install/lib` directory. 



CMake for developers/32-bit (Windows)
......................................

To compile ROSCO on Windows, you first need a Fortran compiler. If you need a 32-bit DLL, then we recommend `installing MinGW <http://capsis.cirad.fr/capsis/documentation/mingw-installation>`_ (Section 2).
If you require a 64-bit version, you can install the MSYS2 toolchain through conda::

    conda install m2w64-toolchain libpython

Note that if you have the 64-bit toolchain installed in your environment, you might have conflicts with the 32-bit compiler. We recommend therefore keeping separate environments if you want to compile 32- or 64-bit.

Once you have your Fortran compiler successfully installed and configured, the build process is similar to on Mac and linux:
::

    cd ROSCO/ROSCO
    mkdir build
    cd build
    cmake .. -G "MinGW Makefiles"
    mingw32-make

Note that the :code:`mingw32-make` command is (confusingly) valid for both 64-bit and 32-bit MinGW.

This will generate a file called :code:`libdiscon.dll` in the :code:`/ROSCO/install/lib` directory. 


Getting Started
...................
Please see :ref:`standard_use` for several example scripts using ROSCO and the ROSCO_toolbox.


.. _Anaconda: https://www.anaconda.com/
.. _CMake: https://cmake.org/
.. _MinGW: https://mingw-w64.org/

