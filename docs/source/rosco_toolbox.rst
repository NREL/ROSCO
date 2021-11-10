.. toctree::

.. _rosco_toolbox_main:

ROSCO Toolbox Structure
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
A number of examples are included to showcase the numerous capabilities of the ROSCO toolbox; they are described in the :ref:`standard_use`.

Matlab_Toolbox
...............
A simulink implementation of the ROSCO controller is included in the Matlab Toolbox. Some requisite MATLAB utility scripts are also included.

ROSCO_testing
.............
Testing scripts for the ROSCO toolbox are held here and showcased with :code:`run_testing.py`. These can be used to compare different controller tunings or different controllers all together.

Test_Cases
..........
Example OpenFAST models consistent with the latest release of OpenFAST are provided here for simple testing and simulation cases.

Tune_Cases
..........
Some example tuning scripts and tuning input files are provided here. The code found in :code:`tune_ROSCO.py` can be modified by the user to easily enable tuning of their own wind turbine model.

.. _rt_tuning_yaml: 

The ROSCO Toolbox Tuning File
------------------------------
A yaml_ formatted input file is used for the standard ROSCO toolbox tuning process. This file contains the necessary inputs for the ROSCO toolbox to load an OpenFAST input file deck and tune the ROSCO controller. It contains the following inputs:

.. list-table:: ROSCO toolbox input yaml
  :header-rows: 1
  :widths: 15 15 10 10 50
  :stub-columns: 1

  * - Primary Section
    - Variable
    - Required
    - Type
    - Description
  * - :code:`path_params`
    - :code:`FAST_InputFile`
    - Yes
    - String
    - Name of the primary (\*.fst) OpenFAST input file
  * - 
    - :code:`FAST_directory`
    - Yes
    - String
    - Main OpenFAST model directory, where the \*.fst lives
  * - 
    - :code:`rotor_performance_filename`
    - No
    - String
    - Filename for rotor performance text file. If this is not specified, and an existing rotor performance file cannot be found, cc-blade will be run
  * - :code:`turbine_params` 
    - :code:`rotor_interia`
    - Yes
    - Float
    - Rotor inertia [kg m^2], (Available in Elastodyn .sum file)
  * - 
    - :code:`rated_rotor_speed`
    - Yes
    - Float
    - Rated rotor speed of the turbine [rad/s]
  * - 
    - :code:`v_min`
    - Yes
    - Float
    - Cut-in wind speed [m/s] 
  * - 
    - :code:`v_max`
    - Yes
    - Float
    - Cut-out wind speed [m/s] 
  * - 
    - :code:`max_pitch_rate`
    - Yes
    - Float
    - Maximum blade pitch rate [rad/s]
  * - 
    - :code:`max_torque_rate`
    - Yes
    - Float
    -  Maximum generator torque rate [Nm/s]
  * - 
    - :code:`rated_power`
    - Yes
    - Float
    - Rated Power [W].
  * -
    - :code:`bld_edgewise_freq`
    - Yes
    - Float
    - Blade edgewise first natural frequency [rad/s].  Set this even if you are using stiff blades. It becomes the generator speed LPF bandwidth.
  * - 
    - :code:`TSR_operational`
    - No
    - Float
    - Desired below-rated operation tip speed ratio [-]. If this is not specified, the Cp-maximizing TSR from the Cp surface is used. 
  * - 
    - :code:`twr_freq`
    - No
    - Float
    - Tower first fore-aft natural frequency [rad/s]. Required for floating wind turbine control.
  * - 
    - :code:`ptfm_freq`
    - No
    - Float
    - Platform first fore-aft natural frequency [rad/s]. Required for floating wind turbine control.
  * - :code:`controller_params`
    - :code:`LoggingLevel`
    - Yes
    - Int
    - 0: write no debug files, 1: write standard output .dbg-file, 2: write standard output .dbg-file and complete avrSWAP-array .dbg2-file
  * - 
    - :code:`F_LPFType`
    - Yes
    - Int
    - Type of Low pass filter for the generator speed feedback signal [rad/s]. 1: first-order low-pass filter, 2: second-order low-pass filter.
  * - 
    - :code:`F_NotchType`
    - Yes
    - Int
    - Notch filter on generator speed and/or tower fore-aft motion, used for floating wind turbine control. 0: disable, 1: generator speed, 2: tower-top fore-aft motion, 3: generator speed and tower-top fore-aft motion
  * - 
    - :code:`IPC_ControlMode`
    - Yes
    - Int
    - Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution). 0: off, 1: 1P reductions, 2: 1P+2P reductions.
  * - 
    - :code:`VS_ControlMode` 
    - Yes
    - Int 
    - Generator torque control mode. 0: :math:`k\omega^2` below rated, constant torque above rated, 1: :math:`k\omega^2` below rated, constant power above rated, 2: TSR tracking PI control below rated, constant torque above rated, 3: TSR tracking PI control below rated, constant power above rated.
  * - 
    - :code:`PC_ControlMode`
    - Yes
    - Int 
    - Blade pitch control mode. 0: No pitch control, fix to fine pitch, 1: active PI blade pitch control
  * - 
    - :code:`Y_ControlMode`
    - Yes
    - Int
    - Yaw control mode. 0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC
  * - 
    - :code:`SS_Mode`
    - Yes
    - Int
    - Setpoint Smoother mode. 0: no set point smoothing, 1: set point smoothing
  * - 
    - :code:`WE_Mode`
    - Yes
    - Int 
    - Wind speed estimator mode. 0: One-second low pass filtered hub height wind speed, 1: Immersion and Invariance Estimator (Ortega et al.), 2: Extended Kalman filter
  * - 
    - :code:`PS_Mode`
    - Yes
    - Int 
    - Pitch saturation mode. 0: no pitch saturation, 1: peak shaving, 2: Cp-maximizing pitch saturation, 3: peak shaving and Cp-maximizing pitch saturation
  * - 
    - :code:`SD_Mode`
    - Yes
    - Int 
    - Shutdown mode. 0: no shutdown procedure, 1: pitch to max pitch at shutdown.
  * -
    - :code:`Fl_Mode`
    - Yes
    - Int 
    - Floating feedback mode. 0: no nacelle rotational velocity feedback, 1: nacelle rotational velocity feedback
  * - 
    - :code:`Flp_Mode`
    - Yes
    - Int
    - Flap control mode. 0: no flap control, 1: steady state flap angle, 2: Proportional flap control
  * - 
    - :code:`zeta_pc`
    - Yes
    - Float
    - Pitch controller desired damping ratio [-]
  * - 
    - :code:`omega_pc`
    - Yes
    - Float
    - Pitch controller desired natural frequency [rad/s]
  * - 
    - :code:`zeta_vs`
    - Yes
    - Float
    - Torque controller desired damping ratio [-]
  * - 
    - :code:`omega_vs`
    - Yes
    - Float
    - Torque controller desired natural frequency [rad/s]
  * - 
    - :code:`zeta_flp`
    - No
    - Float
    - Flap controller desired damping ratio [-]. Required if :code:`Flp_Mode`>0
  * - 
    - :code:`omega_flp`
    - No
    - Float
    - Flap controller desired natural frequency [rad/s]. Required if :code:`Flp_Mode`>0
  * - 
    - :code:`max_pitch`
    - No
    - Float
    - Maximum blade pitch angle [rad]. Default is 1.57 rad (90 degrees).
  * - 
    - :code:`min_pitch`
    - No
    - Float
    - Minimum blade pitch angle [rad]. Default is 0 degrees.
  * - 
    - :code:`vs_minspd`
    - No
    - Float
    - Minimum rotor speed [rad/s]. Default is 0 rad/s.
  * - 
    - :code:`ss_cornerfreq`
    - No
    - Float
    - First order low-pass filter cornering frequency for setpoint smoother [rad/s]. Default is .6283 rad/s.
  * - 
    - :code:`ss_vsgain`
    - No
    - Float
    - Torque controller set point smoother gain bias percentage [:math:`\leq` 1]. Default is 1.
  * - 
    - :code:`ss_pcgain`
    - No
    - Float
    - Pitch controller set point smoother gain bias percentage [:math:`\leq` 1]. Default is 0.001.
  * - 
    - :code:`ps_percent`
    - No
    - Float
    - Percent peak shaving  [:math:`\leq` 1]. Default is 0.8.
  * - 
    - :code:`sd_maxpit`
    - No
    - Float
    - Maximum blade pitch angle to initiate shutdown [rad]. Default is the blade pitch angle at :code:`v_max`.
  * - 
    - :code:`sd_cornerfreq`
    - No
    - Float
    - Cutoff Frequency for first order low-pass filter for blade pitch angle [rad/s]. Default is 0.41888 rad/s.
  * - 
    - :code:`flp_maxpit`
    - No
    - Float
    - Maximum (and minimum) flap pitch angle [rad]. Default is 0.1745 rad (10 degrees).
     

.. _OpenFAST: https://github.com/openfast/openfast
.. _yaml: https://yaml.org/
