.. marine_hydro:

ROSCO Control of Marine Hydrokinetic Turbines (MHKs)
====================================================


Here is some text


Introduction
---------------

.. .. _cp_surface:
.. .. figure:: /mhk_images/01_cp_surface.png
..    :align: center
..    :width: 90%

.. _cp_surface_annotated:
.. figure:: /mhk_images/02_cp_surface_annotated.png
   :align: center
   :width: 90%


Over/Underspeed Reference Setpoints
-----------------------------------

.. _cp_wg_sched:
.. figure:: /mhk_images/03_cp_wg_sched.png
   :align: center
   :width: 90%

.. _cp_tg_sched:
.. figure:: /mhk_images/04_cp_tg_sched.png
   :align: center
   :width: 90%

.. _cp_wg_tg_sched:
.. figure:: /mhk_images/05_cp_wg_tg_sched.png
   :align: center
   :width: 90%


Over/Underspeed Dynamics
------------------------

.. .. _cp_Agen_sched:
.. .. figure:: /mhk_images/06_cp_Agen_sched.png
..    :align: center
..    :width: 90%

.. _cp_Agen_sched_annotated:
.. figure:: /mhk_images/07_cp_Agen_sched_annotated.png
   :align: center
   :width: 90%

.. .. _cp_wg_Ta_contour:
.. .. figure:: /mhk_images/08_cp_wg_Ta_contour.png
..    :align: center
..    :width: 90%

.. _cp_wg_Ta_contour_annotated:
.. figure:: /mhk_images/09_cp_wg_Ta_contour_annotated.png
   :align: center
   :width: 90%


Fixed-Blade-Pitch (FBP) Control
--------------------------------

.. _cp_kp_ki_sched:
.. figure:: /mhk_images/10_cp_kp_ki_sched.png
   :align: center
   :width: 90%


Alternate Region 3 Operating Schedules
---------------------------------------

.. _ext_P:
.. figure:: /mhk_images/11_ext_P.png
   :align: center
   :width: 90%

.. _ext_wg_tg_sched:
.. figure:: /mhk_images/12_ext_wg_tg_sched.png
   :align: center
   :width: 90%

.. _ext_wg_tg_P_contour:
.. figure:: /mhk_images/13_ext_wg_tg_P_contour.png
   :align: center
   :width: 90%

.. .. _ext_wg_sched:
.. .. figure:: /mhk_images/15_ext_wg_sched.png
..    :align: center
..    :width: 90%

.. _ext_wg_sched_annotated:
.. figure:: /mhk_images/16_ext_wg_sched_annotated.png
   :align: center
   :width: 90%

.. _ext_tg_sched:
.. figure:: /mhk_images/17_ext_tg_sched.png
   :align: center
   :width: 90%

.. _ext_Agen_sched:
.. figure:: /mhk_images/18_ext_Agen_sched.png
   :align: center
   :width: 90%

.. _ext_wg_thrust_contour:
.. figure:: /mhk_images/19_ext_wg_thrust_contour.png
   :align: center
   :width: 90%

.. _ext_wg_thrust_sched:
.. figure:: /mhk_images/20_ext_wg_thrust_sched.png
   :align: center
   :width: 90%


Toolbox Implementation
-----------------------

.. _fbp_flow_chart:
.. figure:: /mhk_images/14_fbp_flow_chart.png
   :align: center
   :width: 90%

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

.. _cases_P_wg_tg_sched:
.. figure:: /mhk_images/21_cases_P_wg_tg_sched.png
   :align: center
   :width: 90%

.. _case1_P_wg_tg_ss:
.. figure:: /mhk_images/22_case1_P_wg_tg_ss.png
   :align: center
   :width: 90%

.. _case2_P_wg_tg_ss:
.. figure:: /mhk_images/23_case2_P_wg_tg_ss.png
   :align: center
   :width: 90%

.. _case3_P_wg_tg_ss:
.. figure:: /mhk_images/24_case3_P_wg_tg_ss.png
   :align: center
   :width: 90%

.. _turb_intensity:
.. figure:: /mhk_images/25_turb_intensity.png
   :align: center
   :width: 90%

.. _case1_P_wg_tg_turb:
.. figure:: /mhk_images/26_case1_P_wg_tg_turb.png
   :align: center
   :width: 90%

.. _case2_P_wg_tg_turb:
.. figure:: /mhk_images/27_case2_P_wg_tg_turb.png
   :align: center
   :width: 90%

.. _case3_P_wg_tg_turb:
.. figure:: /mhk_images/28_case3_P_wg_tg_turb.png
   :align: center
   :width: 90%


Recommendations
-----------------------


