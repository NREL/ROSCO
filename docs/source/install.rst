.. toctree::

.. _install:

Installing the ROSCO tools
===========================
As a reminder, the ROSCO toolbox is a python-based tool used to write the DISCON.IN file, which is read by the ROSCO controller (a compiled binary file). If you only wish to run the controller, you *do not* need to install the ROSCO toolbox. 

Depending on what is needed, a user can choose to use just the ROSCO controller or to use both the ROSCO controller and the toolbox. Both the controller and the toolbox should be installed if one wishes to leverage the full ROSCO tool-chain. :numref:`rosco_table` provides an overview of the primary methods available for :ref:`rosco_controller`. Additionally, :numref:`roscotoolbox_table` provides an overview of the primary methods available to acquire the ROSCO toolbox. Finally, if you wish to install and use both the controller and toolbox, the section about :ref:`full_rosco` provides the best methods of doing so. 

.. _rosco_table:
.. list-table:: Methods for Installing the ROSCO Controller
   :widths: 30 70
   :header-rows: 1

   * - Method
     - Use Case
   * - :ref:`rosco_direct_download`
     - Best for users who simply want to use a released version of the controller without working through the compilation procedures. 
   * - :ref:`rosco_anaconda_download`
     - Best for users who just want to use the controller but prefer to download using the Anaconda package man age Full ROSCO Installation.
   * - :ref:`full_rosco`
     - Best for users who wish to both use the controller and leverage the tools in the ROSCO toolbox
   * - :ref:`cmake_compile`
     - Best for users who need to re-compile the source code often, plan to use non-released versions of ROSCO (including modified source code), or who simply want to compile the controller themselves so they have the full code available locally.

.. _roscotoolbox_table:
.. list-table:: Methods for Installing the ROSCO Toolbox
   :widths: 30 70
   :header-rows: 1

   * - Method
     - Use Case
   * - :ref:`roscotoolbox_anaconda_download`
     - Best for users who simply want to use the primary ROSCO toolbox functions
   * - :ref:`full_rosco`
     - (Recommended) Best for users who wish to both use the primary ROSCO toolbox functions, as well run and use the many example and testing scripts available. This process can be done with or without compiling ROSCO.

For many of the methods used to install both ROSCO and the ROSCO toolbox, both Anaconda_ and CMake_ are necessary. Anaconda is a popular package manager used to distribute software packages of various types. Anaconda is used to download requisite packages and distribute pre-compiled versions of the ROSCO tools. CMake is a build configuration system that creates files as input to a build tool like GNU Make, Visual Studio, or Ninja. CMake does not compile code or run compilers directly, but rather creates the environment needed for another tool to run compilers and create binaries. CMake is used to ease the processes of compiling the ROSCO controller locally. For more information on CMake, please see `understanding CMake <https://openfast.readthedocs.io/en/main/source/install/index.html#understanding-cmake>`_ in the OpenFAST documentation.


.. _rosco_controller:

Installing the ROSCO controller
--------------------------------
The standard ROSCO controller is based in Fortran and must be compiled; the source code can be found at: https://github.com/NREL/ROSCO/ROSCO. 

.. _rosco_direct_download:

Direct Download
................
The most recent tagged version releases of the controller are `available for download <https://github.com/NREL/ROSCO/tags>`_. One can simply download these compiled binary files for their system and point to them in their simulation tools (e.g. through :code:`DLL_FileName` in the ServoDyn input file of OpenFAST).

.. _rosco_anaconda_download:

Anaconda Download - ROSCO
..........................
Using the popular package manager, Anaconda_, the tagged 64-bit versions of ROSCO are available through the conda-forge channel. 
In order to download the most recently compiled version release, from an anaconda powershell (Windows) or terminal (Mac/Linux) window, create a new anaconda virtual environment: 

.. code-block:: bash

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.8
    conda activate rosco-env

navigate to your desired folder to save the compiled binary using:

.. code-block:: bash

    cd <desired_folder>
    
and download the controller:

.. code-block:: bash

    conda install -y ROSCO

This will download a compiled ROSCO binary file into the default filepath for any dynamic libraries downloaded via anaconda while in the ROSCO-env. 
The ROSCO binary file can be copied to your desired folder using:

.. code-block:: bash

    cp $CONDA_PREFIX/lib/libdiscon.* <desired_folder>

on linux or:

.. code-block:: bash

    copy %CONDA_PREFIX%/lib/libdiscon.* <desired_folder>


on Windows.


.. _cmake_compile:

Compile using CMake
.....................
CMake_ eases the compiling process significantly. We recommend that users use CMake if at all possible, as we cannot guarantee support for the use of other tools to aid with compiling ROSCO.

On Mac/Linux, standard compilers are generally available without any additional downloads. On 32-bit windows, we recommend that you `install MinGW <http://capsis.cirad.fr/capsis/documentation/mingw-installation>`_ (Section 2). On 64-bit Windows, you can simply install the MSYS2 toolchain through Anaconda:

.. code-block:: bash

    conda install m2w64-toolchain libpython

Once the CMake and the required compilers are downloaded, the following code can be used to compile ROSCO.

.. code-block:: bash

    # Clone ROSCO
    git clone https://github.com/NREL/ROSCO.git

    # Compile ROSCO
    cd ROSCO/ROSCO
    mkdir build
    cd build
    cmake ..                        # Mac/linux only
    cmake .. -G "MinGW Makefiles"   # Windows only 
    make install

This will generate a file called :code:`libdiscon.so` (Linux), :code:`libdiscon.dylib` (Mac), or :code:`libdisscon.dll` (Windows) in the :code:`/ROSCO/install/lib` directory. 

.. _rosco_toolbox_install:

Installing the ROSCO toolbox
----------------------------
The ROSCO toolbox is based in python and contains all relevant ROSCO tools; the source code can be found at: https://github.com/NREL/ROSCO/. In addition to tuning procedures, the ROSCO toolbox also contains example scripts, a Simulink Model of ROSCO, OpenFAST pre-and post-processing functions, linearized systems analysis tools, and a testing suite. 

.. _roscotoolbox_anaconda_download:

Anaconda Download - ROSCO Toolbox
..................................
If one wishes to simply use the modules provided in the ROSCO toolbox through scripts of their own, the ROSCO toolbox can be installed via the conda-forge channel of Anaconda. They can then be accessed using the standard methods of loading modules in python, e.g:

.. code-block:: python

    from ROSCO_toolbox import controller as ROSCO_controller
    from ROSCO_toolbox import turbine as ROSCO_turbine

Note that the install procedures for the ROSCO toolbox are the same as in :ref:`rosco_anaconda_download`, but do not involve moving the controller binary file. 
In order to download the most recently compiled version release, from an anaconda powershell (Windows) or terminal (Mac/Linux) window, create a new anaconda virtual environment: 

.. code-block:: bash

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.8
    conda activate rosco-env

navigate to your desired folder to save the compiled binary using:

.. code-block:: bash

    cd <desired_folder>
    
and download the controller:

.. code-block:: bash

    conda install -y ROSCO



.. _full_rosco:

Full ROSCO Installation
-----------------------

We recommend using the full ROSCO tool-chain. This allows for full use of the provided functions along with the developed python packages and controller code, 

Please follow the following steps to install the ROSCO tool-chain. You should do step 3 *or* 4. If you simply want to install the ROSCO toolbox without the controller, do step 3. If you would like to install the ROSCO toolbox and compile the controller simultaneously, do step 4. 

1. Create a conda environment for ROSCO

.. code-block:: bash

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.8
    conda activate rosco-env

2. Clone and Install the ROSCO toolbox with ROSCO
    
.. code-block:: bash

    git clone https://github.com/NREL/ROSCO.git
    cd ROSCO
    conda install compilers # (Mac/Linux only)
    conda install m2w64-toolchain libpython # (Windows only)
    conda install -y wisdem
    python setup.py install --compile-rosco 

3. Clone and Install the ROSCO toolbox without ROSCO
    
.. code-block:: bash

    git clone https://github.com/NREL/ROSCO.git
    cd ROSCO
    python setup.py install


Getting Started
----------------
Please see :ref:`standard_use` for several example scripts using ROSCO and the ROSCO_toolbox.


.. _Anaconda: https://www.anaconda.com/
.. _CMake: https://cmake.org/
.. _MinGW: https://mingw-w64.org/
