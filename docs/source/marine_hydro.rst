.. _marine_hydro:

ROSCO Control of Marine Hydrokinetic Turbines (MHKs)
====================================================

Introduction
---------------

Here, we detail the control of MHK turbines in above rated flow speeds.
In below rated flow speeds, torque control is used to operate the turbine at it's maximum tip speed ratio (TSR) and pitch angle.

For MHK turbines equipped with pitch actuators, those turbine can can use control schemes similar to those used by wind turbines, which reduce the Cp surface by increasing the pitch angle (black).
For MHK turbines without pitch actuation, we provide a few control methods for controlling the power of the turbine using only torque control.
Overspeed control increases the TSR along the fixed pitch line on the Cp surface below (blue circle) by decreasing the generator torque.
Underspeed control decreases the TSR (red circle) by increasing the torque.

.. _cp_surface_annotated:
.. figure:: /images/mhk/02_cp_surface_annotated.png
   :align: center
   :width: 90%


Over/Underspeed Reference Setpoints
-----------------------------------

The steady state generator-speed setpoints are determined by the Cp contour.
Overspeed achieves up to 3x rated speed, which has additional consequences for blade loads (e.g., cavitation)

.. _cp_wg_sched:
.. figure:: /images/mhk/03_cp_wg_sched.png
   :align: center
   :width: 90%

Torque setpoints (:math:`\bar{\tau}`) determined by constant-power relationship :math:`\bar{\tau} = \frac{P_{rated}}{{\bar{\omega}}}`, where :math:`P_{rated}` is the rated power and :math:`\bar{\omega}` is the steady state generator speed.

.. _cp_tg_sched:
.. figure:: /images/mhk/04_cp_tg_sched.png
   :align: center
   :width: 90%

In Region 3, the relationship between torque and speed are nonmonotic.
Thus, more careful reference control design is required for managing the transition region.
There are examples in the literature for saturation/smoothing during the transition region.

.. _cp_wg_tg_sched:
.. figure:: /images/mhk/05_cp_wg_tg_sched.png
   :align: center
   :width: 90%


Over/Underspeed Dynamics
------------------------

.. .. _cp_Agen_sched:
.. .. figure:: /images/mhk/06_cp_Agen_sched.png
..    :align: center
..    :width: 90%

At each operating point, the sensitivity is computed using the gradients of the Cp surface.
The first-order system decay rate is represented by a single pole on the real axis: more negative means more rapidly stable (positive means unstable).
Underspeed set points are open-loop unstable at high flow speeds.

.. _cp_Agen_sched_annotated:
.. figure:: /images/mhk/07_cp_Agen_sched_annotated.png
   :align: center
   :width: 90%

.. .. _cp_wg_Ta_contour:
.. .. figure:: /images/mhk/08_cp_wg_Ta_contour.png
..    :align: center
..    :width: 90%

@DS: what is shown here?  

.. _cp_wg_Ta_contour_annotated:
.. figure:: /images/mhk/09_cp_wg_Ta_contour_annotated.png
   :align: center
   :width: 90%


Fixed-Blade-Pitch (FBP) Control
--------------------------------

At each setpoint, the torque controller gains are determined using the process as the normal torque control in ROSCO.
High magnitude gains are required to compensate for the open-loop instability of the underspeed system (red).

.. _cp_kp_ki_sched:
.. figure:: /images/mhk/10_cp_kp_ki_sched.png
   :align: center
   :width: 90%


Alternate Region 3 Operating Schedules
---------------------------------------

Using the ROSCO toolbox, we enable the user to determine their own operational power curve, besides a constant rated power.

.. _ext_P:
.. figure:: /images/mhk/11_ext_P.png
   :align: center
   :width: 90%

The alternative power curves result in different speed and torque set points (dashed lines represent underspeed, solid overspeed).

.. _ext_wg_sched_annotated:
.. figure:: /images/mhk/16_ext_wg_sched_annotated.png
   :align: center
   :width: 90%

.. @DS: might not need this one, I'll leave the final revision decision to you.  I re-ordered, too
.. .. _ext_wg_tg_sched:
.. .. figure:: /images/mhk/12_ext_wg_tg_sched.png
..    :align: center
..    :width: 90%

.. _ext_tg_sched:
.. figure:: /images/mhk/17_ext_tg_sched.png
   :align: center
   :width: 90%

.. _ext_wg_tg_P_contour:
.. figure:: /images/mhk/13_ext_wg_tg_P_contour.png
   :align: center
   :width: 90%

.. .. _ext_wg_sched:
.. .. figure:: /images/mhk/15_ext_wg_sched.png
..    :align: center
..    :width: 90%

The stability can be represented by the sensitivity :math:`\frac{d\tau}{d\Omega}`.
Values less than 0 are open-loop stable.  

.. _ext_Agen_sched:
.. figure:: /images/mhk/18_ext_Agen_sched.png
   :align: center
   :width: 90%

The power curve selection also impacts the rotor thrust (F).
Underspeed control and lower power generally results in lower thrust.

.. _ext_wg_thrust_contour:
.. figure:: /images/mhk/19_ext_wg_thrust_contour.png
   :align: center
   :width: 90%

.. .. _ext_wg_thrust_sched:
.. .. figure:: /images/mhk/20_ext_wg_thrust_sched.png
..    :align: center
..    :width: 90%


Toolbox Implementation
-----------------------

The ROSCO toolbox works by determining the speed and torque set points required to operate at a TSR and Cp for the desired power.

.. _fbp_flow_chart:
.. figure:: /images/mhk/14_fbp_flow_chart.png
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

A handful of example controller case studies have been developed using the RM1
marine turbine to showcase the implemented features of FBP control. These
configurations are

* Example 1: Overspeed, constant power
* Example 2: Underspeed, torque-based reference tracking
* Example 3: Underspeed, WSE-based reference tracking

For each example, a power curve is defined and input to the ROSCO toolbox to
generate operating schedules and auto-tune the gains used by the torque
controller.
The operating schedules for generator power, speed, and torque for each example
test case are shown in the following figure. 

.. _cases_P_wg_tg_sched:
.. figure:: /images/mhk/21_cases_P_wg_tg_sched.png
   :align: center
   :width: 90%

Each example controller is then simulated with the RM1 marine turbine model
using OpenFAST in both steady and turbulent inflow. The steady-state performance
of each controller is compared to the operating schedules generated by the ROSCO
toolbox. The turbulent inflow uses the intensity shown in the following figure. 

.. _turb_intensity:
.. figure:: /images/mhk/25_turb_intensity.png
   :align: center
   :width: 90%


Example 1
^^^^^^^^^

The first example test case uses the naturally stable nonlinear feedback control
law. This controller is confined to operating in the constant power, overspeed
configuration. The explicit (non-reference-tracking) control law is analogous to
the `k\Omega^2` control law sometimes used in Region 2 for wind and marine
turbines. 

.. _case1_P_wg_tg_ss:
.. figure:: /images/mhk/22_case1_P_wg_tg_ss.png
   :align: center
   :width: 90%

This controller has the best power tracking in Region 3, but it only allows constant power. 
The power-focused feedback approach accommodates offsets in equilibrium speed and torque made by inaccuracies in the simplified tuning model. 

.. _case1_P_wg_tg_turb:
.. figure:: /images/mhk/26_case1_P_wg_tg_turb.png
   :align: center
   :width: 90%


Example 2
^^^^^^^^^

The second example test case 

* Region-2 mode: 		TSR-tracking with torque-based reference
* Region-3 FBP mode: 	reference tracking with torque lookup
* The power curve may be arbitrarily specified, but should be a nondecreasing function so that the torque schedule is monotonically increasing

.. _case2_P_wg_tg_ss:
.. figure:: /images/mhk/23_case2_P_wg_tg_ss.png
   :align: center
   :width: 90%

* Linearly increasing power in Region 3, up to 2x rated
* Power curve must be set so that torque schedule is monotonic
* Decent power tracking
* May have some misalignment with flow speed setpoint
* Accommodates some offsets in torque and speed
* Should be combined with reference-tracking controller in Region 2

.. _case2_P_wg_tg_turb:
.. figure:: /images/mhk/27_case2_P_wg_tg_turb.png
   :align: center
   :width: 90%


Example 3
^^^^^^^^^

The third example test case

* Region-2 mode: 		TSR-tracking with WSE-based reference
* Region-3 FBP mode: 	reference tracking with WSE lookup
* The power curve can be completely arbitrarily specified


.. _case3_P_wg_tg_ss:
.. figure:: /images/mhk/24_case3_P_wg_tg_ss.png
   :align: center
   :width: 90%

* Smoothly increasing power curve in Region 3
* Would allow arbitrarily increasing or decreasing
* Best gen speed tracking
* May offset equilibrium torque leading to power curve error
* Should be combined with reference-tracking controller in Region 2

.. _case3_P_wg_tg_turb:
.. figure:: /images/mhk/28_case3_P_wg_tg_turb.png
   :align: center
   :width: 90%


Recommendations
-----------------------

FBP control is well suited to marine turbines without blade pitch actuators. 
In certain applications, the ability to follow a generic power curve with a
limit actuation capability is more advantageous than using variable-blade-pitch
(VBP) control. 
VBP control allows constant-power operation in Region 3 matched with constant
speed and torque for a flat operating schedule. Pitch-actuated turbines also
experience smaller blade loads in Region 3. The FBP approach satisfies
applications in which the cost and complexity of the actuators themselves are
prohibitive. 

Generic user input allows flexibility for variety of applications. 

FBP controller implementation in ROSCO with auto-tuning and automatic generation of operating schedule to follow power curve.

Because the Region-2 and Region-3 controllers utilize the same actuator, the
transition region is markedly different than what is required for a VBP
Region-3 controller. 


