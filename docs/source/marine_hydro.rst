.. marine_hydro:

ROSCO Control of Marine Hydrokinetic Turbines (MHKs)
====================================================


Here is some text

.. _dummy_example:
.. figure:: /images/29_AWC.png
   :align: center
   :width: 90%


Introduction
---------------


Over/Underspeed Reference Setpoints
-----------------------------------


Over/Underspeed Dynamics
------------------------


Fixed-Blade-Pitch (FBP) Control
--------------------------------


Alternate Region 3 Operating Schedules
---------------------------------------

Toolbox Implementation
-----------------------

The following inputs to the ROSCO tuning yaml will generate DISCON inputs to ROSCO.

.. list-table::
   :header-rows: 1
   :widths: auto

   * -  Parameter
     -  Description
   * -  VS_FBP
     -  FBP Control Mode (0 = variable pitch, 1 = constant power overspeed (nonlinear), 2 = WSE-lookup reference tracking, 3 = torque-lookup reference tracking)
   * -  FBP_speed_mode
     -  Over/underspeed mode (0 = underspeed, 1 = overspeed)
   * -  FBP_power_mode
     -  Normalized or exact power curve values (0 = relative to rated, 1 = exact) 
   * -  FBP_U
     -  Flow speed setpoints for power curve lookup table
   * -  FBP_P
     -  Power curve lookup table


Note that the ROSCO input schema (:ref:`rt_tuning_yaml`) contains the latest input definitions.


ROSCO Implementation
-----------------------

The following DISCON parameters are generated using the ROSCO toolbox, or can be determined directly in the DISCON.IN file.

.. list-table::
   :header-rows: 1
   :widths: auto

   * -  Parameter
     -  Description
   * -  VS_FBP
     -  FBP Control Mode (0 = variable pitch, 1 = constant power overspeed (nonlinear), 2 = WSE-lookup reference tracking, 3 = torque-lookup reference tracking)
   * -  VS_FBP_n
     -  Number of values in operating schedule lookup table
   * -  VS_FBP_U
     -  Flow speed operating points in lookup table
   * -  VS_FBP_Omega
     -  Generator speed operating points in lookup table
   * -  VS_FBP_Tau
     -  Generator torque operating points in lookup table

Note that the ROSCO input schema (:ref:`rt_tuning_yaml`) contains the latest input definitions (under :code:`controller_params`, :code:`DISCON`).


Simulation Verification
-----------------------

Recommendations
-----------------------

