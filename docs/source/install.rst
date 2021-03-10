.. toctree::

.. _install:

Installing the ROSCO tools
===========================
Both the ROSCO controller and ROSCO toolbox should be installed if one wishes to leverage the full ROSCO toolchain. 
Of course, some users may only wish to download and install only the controller or the toolbox. 

For users wishing to use just the ROSCO toolbox or the ROSCO toolbox and controller, please skip to the section on section :ref:`installing_rt`. 
Here we will provide the necessary steps to install the ROSCO toolbox and the ROSCO controller simultaneously. 
For users planning to only download and compile the ROSCO controller, please follow the instructions on :ref:`compiling_rosco`. 
For information on best practices to update to the most recent version of the ROSCO toolbox, see :ref:`updating_rt`.


.. _compiling_rosco:

Compiling ROSCO
----------------
The standard ROSCO controller is based in Fortran and must be compiled; this code can be found at: https://github.com/NREL/ROSCO. 
Of course, the advanced user can compile the downloaded code using their own desired methods (e.g. Visual Studio). 
Otherwise, a few of the more common compiling methods are detailed on this page. 
Additionally, the most recent tagged version releases are `available for download <https://github.com/NREL/ROSCO/tags>`_. 

If one wishes to download the code via the command line, we provide two supported options. 
For non-developers (those not interested in modifying the source code), the controller can be downloaded via Anaconda. 
For developers, CMake can be used to compile the Fortran code. 


Anaconda for non-developers:
.............................

For users familiar with Anaconda_, ROSCO is available through the conda-forge channel. 
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


CMake for developers:
.....................
CMake_ provides a straightforward option for many users, particularly those on a Mac or Linux. 
For windows users, we recommend using MinGW_ to compile similarly. 
ROSCO can be compiled by first cloning the source code from git using:
::

    git clone https://github.com/NREL/ROSCO.git

And then compiling using CMake:
::

    cd ROSCO
    mkdir build
    cd build
    cmake ..                        # Mac/Linus
    cmake .. -G "MinGW Makefiles"   # Windows
    make install

This will generate a file called :code:`libdiscon.*` in the :code:`/ROSCO/install/lib` directory. 

Note: when compiling with MinGW on Windows, modifying the cmake line (4th command) in the above code block to :code:`cmake .. -G "MinGWMakefiles" -DCMake_Fortran_COMPILER=gfortran` may solve compiler errors that may arise, depending on your local configuration.

.. _installing_rt:

Installing the ROSCO toolbox
----------------------------
Installation of the complete ROSCO toolbox is made easy through `Anaconda <https://www.anaconda.com/>`_. 
If you do not already have Anaconda installed on your machine, please install it. 

Then please follow the following steps:

1.  Create a conda environment for ROSCO
    ::

        conda config --add channels conda-forge
        conda create -y --name rosco-env python=3.8
        conda activate rosco-env
    ::

2.  Install WISDEM
    ::

        conda install -y wisdem


You should then do step three *or* four. 
If you do not want to compile the ROSCO controller within the installation of the ROSCO toolbox, please follow the instructions for :ref:`compiling_rosco`.

3.  Clone and Install the ROSCO toolbox with ROSCO
    :: 

        git clone https://github.com/NREL/ROSCO_toolbox.git
        cd ROSCO_toolbox
        git submodule init
        git submodule update
        conda install compilers 					# (Mac/Linux only)
        conda install m2w64-toolchain libpython     # (Windows only)
        python setup.py install --compile-rosco

4.	Clone and Install the ROSCO toolbox *without* ROSCO
    ::
    
        git clone https://github.com/NREL/ROSCO_toolbox.git
        cd ROSCO_toolbox
        python setup.py install 


**Alternatively...**

If you wish to write your own scripts to leverage the ROSCO toolbox tools, but do not necessarily need the source code or to run any of the examples, the ROSCO toolbox is available via PyPi:

    ::

        pip install rosco_toolbox

Note that if you do choose to install the ROSCO Toolbox this way, you will not have the source code. Additionally, you will need to download WISDEM and the ROSCO controller separately if you wish to use any of the ROSCO toolbox functionalities that need those software packages. 

.. _updating_rt:

Updating the ROSCO Toolbox
--------------------------
Simple git commands should update the toolbox and controller as development continues:
```
git pull
git submodule update 
```
and then recompile and reinstall as necessary...

.. _Anaconda: https://www.anaconda.com/
.. _CMake: https://cmake.org/
.. _MinGW: https://mingw-w64.org/


Getting Started with the ROSCO Toolbox
--------------------------------------
Please see a the :ref:`standard_use` for several example scripts using ROSCO and the ROSCO_toolbox.
