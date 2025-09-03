.. toctree::

.. _rosco:

ROSCO Structure: Controller
===========================
Here, we give an overview of the structure of the ROSCO controller and how the code is implemented. 

-----

ROSCO File Structure
---------------------
The primary functions of the ROSCO toolbox are separated into several files. They include the following:

* :code:`DISCON.f90` is the primary driver function. 
* :code:`ReadSetParameters.f90` primarily handles file I/O and the Bladed Interface.
* :code:`ROSCO_Types.f90` allocates variables in memory; it is procedurally generated from :code:`rosco_registry`
* :code:`Constants.f90` establishes some global constants.
* :code:`Controllers.f90` contains the primary controller algorithms (e.g. blade pitch control)
* :code:`ControllerBlocks.f90` contains additional control features that are not necessarily primary controllers (e.g. wind speed estimator)
* :code:`Filters.f90` contains the various filter implementations.
* :code:`Functions.f90` contains various functions used in the controller.
* :code:`ExtControl.f90` contains subroutines for calling external dynamic libraries
* :code:`ROSCO_Helpers.f90` contains subroutines for file I/O and other helpful routines, borrowed heavily from NWTC.IO in OpenFAST
* :code:`ROSCO_IO.f90` is procedurally generated using the :code:`rosco_registry` for writing debug and checkpoint files

.. _discon_in: 

The DISCON.IN file
------------------------------
A standard file structure is used as an input to the ROSCO controller. 
This is, generically, dubbed the DISCON.IN file, though it can be renamed (In OpenFAST_, this file is pointed to by :code:`DLL_InFile` in the ServoDyn file. 
Examples of the DISCON.IN file are found in each of the Test Cases in the ROSCO toolbox, and in the :code:`parameter_files` folder of ROSCO. 

Detailed and up-to-date documentation of the DISCON inputs can be found in the :ref:`rt_tuning_yaml` page in controller_params -> DISCON. 

.. _OpenFAST: https://github.com/openfast/openfast
.. _yaml: https://yaml.org/
