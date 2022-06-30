.. toctree::

.. _standard_use:

Standard ROSCO Workflow
=======================
This page outlines methods for reading turbine models, generating the control parameters of a :code:`DISCON.IN`: file, and running aeroelastic simulations to test controllers.
A set of `example scripts <https://github.com/NREL/ROSCO/tree/main/Examples>`_ demonstrate the functionality of the ROSCO toolbox and controller.


Reading Turbine Models
----------------------
Control parameters depend on the turbine model.
The ROSCO_toolbox uses OpenFAST inputs and an additional :code:`.yaml` formatted file to set up a :code:`turbine` object in python.
Several OpenFAST inputs are located in `Test_Cases/ <https://github.com/NREL/ROSCO/tree/main/Test_Cases>`_.
The controller tuning :code:`.yaml` are located in `Tune_Cases/ <https://github.com/NREL/ROSCO/tree/main/Tune_Cases>`_.
A detailed description of the ROSCO control inputs and tuning :code:`.yaml` are provided in :ref:`discon_in` and :ref:`rt_tuning_yaml`, respectively.

* :code:`example_01.py` loads an OpenFAST turbine model and displays a summary of its information
* :code:`example_02.py` plots the :math:`C_p` surface of a turbine

ROSCO requires the power and thrust coefficients for tuning control inputs and running the extended Kalman filter wind speed estimator.

* :code:`example_03.py` runs cc-blade, a blade element momentum solver from WISDEM, to generate a :math:`C_p` surface.
  
The :code:`Cp_Cq_Ct.txt` (or similar) file contains the rotor performance tables that are necessary to run the ROSCO controller. 
This file can be located wherever you desire, just be sure to point to it properly with the :code:`PerfFileName` parameter in :code:`DISCON.IN`.


Tuning Controllers and Generating DISCON.IN
-------------------------------------------
The ROSCO :code:`turbine` object, which contains turbine information required for controller tuning, along with control parameters in the tuning yaml and the :math:`C_p` surface are used to generate control parameters and :code:`DISCON.IN` files.
To tune the PI gains of the torque control, set :code:`omega_vs` and :code:`zeta_vs` in the yaml.
Similarly, set :code:`omega_pc` and :code:`zeta_pc` to tune the PI pitch controller; gain scheduling is automatically handled using turbine information.
Generally :code:`omega_*` increases the responsiveness of the controller, reducing generator speed variations, but an also increases loading on the turbine.
:code:`zeta_*` changes the damping of the controller and is generally less important of a tuning parameter, but could also help with loading.
The default parameters in `Tune_Cases/ <https://github.com/NREL/ROSCO/tree/main/Tune_Cases>`_ are known to work well with the turbines in this repository.

* :code:`example_04.py` loads a turbine and tunes the PI control gains
* :code:`example_05.py` tunes a controller and runs a simple simualtion (not using OpenFAST)
* :code:`example_06.py` loads a turbine, tunes a controller, and runs an OpenFAST simulation

Each of these examples generates a :code:`DISCON.IN` file, which is an input to libdiscon.*.
When running the controller in OpenFAST, :code:`DISCON.IN` must be appropriately named using the :code:`DLL_FileName` parameter in ServoDyn. 

OpenFAST can be installed from `source <https://github.com/OpenFAST/openfast>`_ or in a conda environment using:

.. code-block:: bash

  conda install -c conda-forge openfast

ROSCO can implement peak shaving (or thrust clipping) by changing the minimum pitch angle based on the estimated wind speed:

* :code:`example_07.py` loads a turbine and tunes a controller with peak shaving.

By setting the :code:`ps_percent` value in the tuning yaml, the minimum pitch versus wind speed table changes and is updated in the :code:`DISCON.IN` file.

ROSCO also contains a method for distributed aerodynamic control (e.g., via trailing edge flaps):

* :code:`example_10.py` tunes a controller for distributed aerodynamic control

The ROSCO toolbox also contains methods for working with OpenFAST linear models
* :code:`example_11.py` exports a file of the parameters used for the simplified linear models used to tune ROSCO
* :code:`example_12.py` shows how linear models generated using OpenFAST can be used to tune controllers with robust stability properties. 
* :code:`example_13.py` shows the tuning procedure for IPC

Running OpenFAST Simulations
----------------------------

To run an aeroelastic simulation with ROSCO, the ROSCO input (:code:`DISCON.IN`) must point to a properly formatted :code:`Cp_Cq_Ct.txt` file using the :code:`PerfFileName` parameter.
If called from OpenFAST, the main OpenFAST input points to the ServoDyn input, which points to the :code:`DISCON.IN` file and the :code:`libdiscon.*` dynamic library.

For example in `Test_Cases/NREL-5MW`:

* :code:`NREL-5MW.fst` has :code:`"NRELOffshrBsline5MW_Onshore_ServoDyn.dat"`  as the :code:`ServoFile` input
* :code:`NRELOffshrBsline5MW_Onshore_ServoDyn.dat` has :code:`"../../ROSCO/build/libdiscon.dylib"` as the :code:`DLL_FileName` input and :code:`"DISCON.IN"` as the :code:`DLL_InFile` input.
  Note that these file paths are relative to the path of the main fast input (:code:`NREL-5MW.fst`)
* :code:`DISCON.IN` has :code:`"Cp_Ct_Cq.NREL5MW.txt"` as the :code:`PerfFileName` input

The ROSCO_toolbox has methods for running OpenFAST (and other) binary executables using system calls, as well as post-processing tools in `ofTools/ <https://github.com/NREL/ROSCO/tree/main/ROSCO_toolbox/ofTools>`_.

Several example scripts are set up to quickly simulate ROSCO with OpenFAST:

* :code:`example_06.py` loads a turbine, tunes a controller, and runs an OpenFAST simulation
* :code:`example_08.py` loads the OpenFAST output files and plots the results
* :code:`example_09.py` runs TurbSim, for generating turbulent wind inputs
* :code:`example_14.py` runs an OpenFAST simulation with ROSCO providing open loop control inputs


Testing ROSCO
-------------

The ROSCO_toolbox also contains tools for testing ROSCO in IEC design load cases (DLCs), located in `ROSCO_testing/ <https://github.com/NREL/ROSCO/tree/main/ROSCO_testing>`_.
The script :code:`run_Testing.py` allows the user to set up their own set of tests.
By setting :code:`testtype`, the user can run a variety of tests:

* :code:`lite`, which runs DLC 1.1 simulations at 5 wind speed from cut-in to cut-out, in 330 second simulations
* :code:`heavy`, which runs DLC 1.3 from cut-in to cut-out in 2 m/s steps and 2 seeds for each, in 630 seconds, as well as DLC 1.4 simulations
* :code:`binary-comp`, where the user can compare :code:`libdiscon.*` dynamic libraries (compiled ROSCO source code), with either a lite or heavy set of simulations
* :code:`discon-comp`, where the user can compare :code:`DISCON.IN` controller tunings (and the complied ROSCO source is constant)

Setting the :code:`turbine2test` allows the user to test either the IEA-15MW with the UMaine floating semisubmersible or the NREL-5MW reference onshore turbine.

