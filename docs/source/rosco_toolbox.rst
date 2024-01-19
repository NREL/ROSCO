.. toctree::

.. _rosco_toolbox_main:

ROSCO Structure: Toolbox 
========================
Here, we give an overview of the structure of the ROSCO toolbox and how the code is implemented. 

-----

ROSCO Toolbox File Structure
----------------------------
The primary tools of the ROSCO toolbox are separated into several folders. They include the following:

ROSCO_toolbox
.............
The source code for the ROSCO toolbox generic tuning implementations lives here. 

* :code:`turbine.py` loads a wind turbine model from OpenFAST_ input files.
* :code:`controller.py` contains the generic controller tuning scripts
* :code:`utilities.py` has most of the input/output file management scripts
* :code:`control_interface.py` enables a python interface to the ROSCO controller
* :code:`sim.py` is a simple 1-DOF model simulator
* **ofTools** is a folder containing a large set of tools to handle OpenFAST_ input files - this is primarily used to run large simulation sets and to handle reading and processing of OpenFAST input and output files. 

Examples
.........
A number of examples are included to showcase the numerous capabilities of the ROSCO toolbox; they are described in the :ref:`examplepage`.

ROSCO_testing
.............
Testing scripts for the ROSCO toolbox are held here and showcased with :code:`run_testing.py`. These can be used to compare different controller tunings or different controllers all together.

Examples/Test_Cases
...................
Example OpenFAST models consistent with the latest release of OpenFAST are provided here for simple testing and simulation cases.

Examples/Tune_Cases
...................
Some example tuning scripts and tuning input files are provided here. The code found in :code:`tune_ROSCO.py` can be modified by the user to easily enable tuning of their own wind turbine model.

The ROSCO Toolbox Tuning File
------------------------------
A yaml_ formatted input file is used for the standard ROSCO toolbox tuning process. This file contains the necessary inputs for the ROSCO toolbox to load an OpenFAST input file deck and tune the ROSCO controller. It can be found here: :ref:`rt_tuning_yaml`.

Matlab_Toolbox
...............
A simulink implementation of the ROSCO controller is included in the Matlab Toolbox. Some requisite MATLAB utility scripts are also included.
These scripts are not maintained by NREL as of ROSCO v2.5.0.
The Simulink controller there should not be used and referenenced as "ROSCO" because it has never been validated against the official ROSCO dynamic library and has drifted away from the official implementation.
We will leave the implementation in place to be used only as a learning tool.     

.. _OpenFAST: https://github.com/openfast/openfast
.. _yaml: https://yaml.org/
