.. toctree::

.. _standard_use:

Standard ROSCO Workflow
=======================


.. _fig-RT:
.. figure:: /source/figures/ROSCOFramework.svg
   :align: center
   :width: 90%

   ROSCO toolchain general workflow

:numref:`fig-RT` shows the general workflow for the ROSCO tool-chain with OpenFAST.
For the standard use case in OpenFAST (or similar), ROSCO controller needs to be compiled.
The controller is a fortran based module that follows the bladed-style control interface.
Compiling the controller ouputs a dynamic-link library (or equivalent) called :code:`libdiscon.dll` for windows, :code:`libdiscon.so` for linux and, :code:`libdiscon.dylib` for mac-os.
Instructions for the compilation are provided in :ref:`install`.
Once the controller is compiled the turbine simulation tool must point to the compiled library.
In OpenFAST, this is ensured by changing the :code:`DLL_FileName` parameter in the ServoDyn input file. 
This step enables communication between the ROSCO controller and OpenFAST.

The compiled ROSCO controller library requires an input file (generally called :code:`DISCON.IN`).
It stores several flags and parameters needed by the controller and is read by the compiled dynamic-link library.
Several different :code:`DISCON.IN` files, for varous turbines and controller tunings, can use the same dynamic-link library.
In OpenFAST, the :code:`DLL_InFile` parameter in the ServoDyn input file determines the desired input file.

The ROSCO toolbox is used to tune the ROSCO controller and generate a :code:`DISCON.IN` input file.
To tune the controller, ROSCO toolbox needs the OpenFAST model of the turbine and some user inputs in the form of a :code:`tuning.yaml` file.
The functionality of ROSCO toolset can be best understood by following the set of included example scripts in :ref:`examplepage`.
ROSCO toolset can be installed using the instructions provided in :ref:`install`.
