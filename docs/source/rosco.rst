.. toctree::

.. _rosco:

ROSCO Controller Structure
==========================
Here, we give an overview of the structure of the ROSCO controller and how the code is implemented. 

-----

ROSCO File Structure
---------------------
The primary functions of the ROSCO toolbox are separated into several files. They include the following:

* :code:`DISCON.f90` is the primary driver function. 
* :code:`ReadSetParameters.f90` primarily handles file I/O and the Bladed Interface.
* :code:`ROSCO_Types.f90` allocates variables in memory.
* :code:`Constants.f90` establishes some global constants.
* :code:`Controllers.f90` contains the primary controller algorithms (e.g. blade pitch control)
* :code:`ControllerBlocks.f90` contains additional control features that are not necessarily primary controllers (e.g. wind speed estimator)
* :code:`Filters.f90` contains the various filter implementations.
* :code:`Functions.f90` contains various functions used in the controller.

.. _discon_in: 

The DISCON.IN file
------------------------------
A standard file structure is used as an input to the ROSCO controller. 
This is, generically, dubbed the DISCON.IN file, though it can be renamed (In OpenFAST_, this file is pointed to by :code:`DLL_InFile` in the ServoDyn file. 
Examples of the DISCON.IN file are found in each of the Test Cases in the ROSCO toolbox, and in the :code:`parameter_files` folder of ROSCO. 

.. list-table:: DISCON.IN 
  :header-rows: 1
  :widths: 10 10 10 70
  :stub-columns: 1

  * - Primary Section
    - Variable
    - Type
    - Description
  * - DEBUG
    - :code:`LoggingLevel`
    - Int
    - 0: write no debug files, 1: write standard output .dbg-file, 2: write standard output .dbg-file and complete avrSWAP-array .dbg2-file
  * - CONTROLLER FLAGS
    - :code:`F_LPFType`
    - Int
    - Filter type for generator speed feedback signal. 1: first-order low-pass filter, 2: second-order low-pass filter.
  * - 
    - :code:`F_NotchType`
    - Int
    - Notch filter on the measured generator speed and/or tower fore-aft motion (used for floating). 0: disable, 1: generator speed, 2: tower-top fore-aft motion, 3: generator speed and tower-top fore-aft motion. 
  * - 
    - :code:`IPC_ControlMode`
    - Int
    - Individual Pitch Control (IPC) type for fatigue load reductions (pitch contribution). 0: off, 1: 1P reductions, 2: 1P+2P reductions. 
  * - 
    - :code:`VS_ControlMode`
    - Int
    - Generator torque control mode type. 0: :math:`k\omega^2` below rated, constant torque above rated, 1: :math:`k\omega^2` below rated, constant power above rated, 2: TSR tracking PI control below rated, constant torque above rated, 3: TSR tracking PI control below rated, constant torque above rated
  * - 
    - :code:`PC_ControlMode`
    - Int
    - Blade pitch control mode. 0: No pitch, fix to fine pitch, 1: active PI blade pitch control.
  * - 
    - :code:`Y_ControlMode`
    - Int
    - Yaw control mode. 0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC.
  * - 
    - :code:`SS_Mode`
    - Int
    - Setpoint Smoother mode. 0: no set point smoothing, 1: use set point smoothing.
  * - 
    - :code:`WE_Mode`
    - Int
    - Wind speed estimator mode. 0: One-second low pass filtered hub height wind speed, 1: Immersion and Invariance Estimator, 2: Extended Kalman Filter. 
  * -
    - :code:`PS_Mode`
    - Int
    - Pitch saturation mode. 0: no pitch saturation, 1: implement pitch saturation
  * -
    - :code:`SD_Mode`
    - Int 
    - Shutdown mode. 0: no shutdown procedure, 1: shutdown triggered by max blade pitch.
  * -
    - :code:`Fl_Mode`
    - Int 
    - Floating feedback mode. 0: no nacelle velocity feedback, 1: nacelle velocity feedback (parallel compensation).
  * -
    - :code:`Flp_Mode`
    - Int 
    - Flap control mode. 0: no flap control, 1: steady state flap angle, 2: PI flap control. 
  * - FILTERS
    - :code:`F_LPFCornerFreq`	
    - Float
    - Corner frequency (-3dB point) in the generator speed low-pass filter, [rad/s]
  * -
    - :code:`F_LPFDamping`
    - Float 
    - Damping coefficient in the generator speed low-pass filter, [-]. Only used only when F_FilterType = 2
  * -
    - :code:`F_NotchCornerFreq`	
    - Float 
    - Natural frequency of the notch filter, [rad/s]
  * -
    - :code:`F_NotchBetaNumDen`
    - Float Float
    - Notch damping values of numerator and denominator - determines the width and depth of the notch, [-]
  * -
    - :code:`F_SSCornerFreq`
    - Float
    - Corner frequency (-3dB point) in the first order low pass ..filter for the set point smoother, [rad/s].
  * -
    - :code:`F_FlCornerFreq`
    - Float Float
    - Corner frequency and damping ratio for the second order low pass filter of the tower-top fore-aft motion for floating feedback control [rad/s, -].
  * -
    - :code:`F_WECornerFreq`
    - Float
    - Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s].
  * -
    - :code:`F_FlHighPassFreq`
    - Float
    -  Natural frequency of first-order high-pass filter for nacelle fore-aft motion [rad/s]..
  * -
    - :code:`F_FlpCornerFreq`
    - Float Float
    - Corner frequency and damping ratio in the second order low pass filter of the blade root bending moment for flap control [rad/s, -].
  * - BLADE PITCH CONTROL
    - :code:`PC_GS_n`
    - Int 
    - Number of gain-scheduling table entries
  * - 
    - :code:`PC_GS_angles`
    - Float array, length = :code:`PC_GS_n`
    - Gain-schedule table: pitch angles [rad].
  * - 
    - :code:`PC_GS_KP`
    - Float array, length = :code:`PC_GS_n`
    - Gain-schedule table: pitch controller proportional gains [s].
  * - 
    - :code:`PC_GS_KI`
    - Float array, length = :code:`PC_GS_n`
    - Gain-schedule table: pitch controller integral gains [-]. 
  * - 
    - :code:`PC_GS_KD`
    - Float array, length = :code:`PC_GS_n`
    - Gain-schedule table: pitch controller derivative gains [:math:`s^2`]. Currently unused!
  * - 
    - :code:`PC_GS_TF`
    - Float array, length = :code:`PC_GS_n`
    - Gain-schedule table: transfer function gains [:math:`s^2`]. Currently unused!
  * - 
    - :code:`PC_MaxPit` 
    - Float 
    - Maximum physical pitch limit, [rad].
  * - 
    - :code:`PC_MinPit`
    - Float
    - Minimum physical pitch limit, [rad].
  * - 
    - :code:`PC_MaxRat`
    - Float 
    - Maximum pitch rate (in absolute value) of pitch controller, [rad/s].
  * - 
    - :code:`PC_MinRat`
    - Float
    - Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
  * - 
    - :code:`PC_RefSpd`
    - Float 
    - Desired (reference) HSS speed for pitch controller, [rad/s].
  * - 
    - :code:`PC_FinePit`
    - Float
    - Below-rated pitch angle set-point, [rad]
  * - 
    - :code:`PC_Switch`
    - Float
    - Angle above lowest :code:`PC_MinPit` to switch to above rated torque control, [rad]. Used for :code:`VS_ControlMode` = 0,1.
  * - INDIVIDUAL PITCH CONTROL
    - :code:`IPC_IntSat`
    - Float
    - Integrator saturation point (maximum signal amplitude contribution to pitch from IPC), [rad]
  * - 
    - :code:`IPC_KI`
    - Float Float
    - Integral gain for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [-, -].
  * - 
    - :code:`IPC_aziOffset`
    - Float Float 
    - Phase offset added to the azimuth angle for the individual pitch controller: first parameter for 1P reductions, second for 2P reductions, [rad]. 
  * - 
    - :code:`IPC_CornerFreqAct` 
    - Float 
    - Corner frequency of the first-order actuators model, used to induce a phase lag in the IPC signal [rad/s]. 0: Disable.
  * - VS TORQUE CONTROL
    - :code:`VS_GenEff`
    - Float
    - Generator efficiency from mechanical power -> electrical power, [should match the efficiency defined in the generator properties!], [%]
  * - 
    - :code:`VS_ArSatTq`	
    - Float
    - Above rated generator torque PI control saturation limit, [Nm].
  * - 
    - :code:`VS_MaxRat`
    - Float 
    - Maximum generator torque rate (in absolute value) [Nm/s].
  * - 
    - :code:`VS_MaxTq`
    - Float 
    - Maximum generator torque (HSS), [Nm].
  * - 
    - :code:`VS_MinTq` 
    - Float 
    - Minimum generator torque (HSS) [Nm].
  * - 
    - :code:`VS_MinOMSpd`
    - Float 
    - Cut-in speed towards optimal mode gain path, [rad/s]. Used if :code:`VS_ControlMode` = 0,1.
  * - 
    - :code:`VS_Rgn2K`
    - Float 
    - Generator torque constant in Region 2 (HSS side), [N-m/(rad/s)^2]. Used if :code:`VS_ControlMode` = 0,1.
  * - 
    - :code:`VS_RtPwr`
    - Float 
    - Rated power [W]
  * - 
    - :code:`VS_RtTq`	
    - Float
    - Rated torque, [Nm].
  * - 
    - :code:`VS_RefSpd`
    - Float
    - Rated generator speed used by torque controller [rad/s].
  * - 
    - :code:`VS_n`
    - Int
    - Number of generator PI torque controller gains. Only 1 is currently supported.
  * - 
    - :code:`VS_KP`
    - Float
    - Proportional gain for generator PI torque controller [1/(rad/s) Nm]. (Used in the transition 2.5 region if :code:`VS_ControlMode` = 0,1. Always used if :code:`VS_ControlMode` = 2,3)
  * - 
    - :code:`VS_KI`
    - Float
    - Integral gain for generator PI torque controller [1/rad Nm]. (Only used in the transition 2.5 region if :code:`VS_ControlMode` = 0,1. Always used if :code:`VS_ControlMode` = 2,3)
  * - 
    - :code:`VS_TSRopt`
    - Float
    - Region 2 tip-speed-ratio [rad]. Generally, the power maximizing TSR. Can use non-optimal TSR for low axial induction rotors.
  * - SETPOINT SMOOTHER
    - :code:`SS_VSGain`
    - Float
    - Variable speed torque controller setpoint smoother gain, [-].
  * - 
    - :code:`SS_PCGain`
    - Float
    - Collective pitch controller setpoint smoother gain, [-].
  * - WIND SPEED ESTIMATOR
    - :code:`WE_BladeRadius`
    - Float
    - Blade length (distance from hub center to blade tip), [m]
  * - 
    - :code:`WE_CP_n`
    - Int	
    - Number of parameters in the Cp array
  * - 
    - :code:`WE_CP`
    - Float Float Float Float
    - Parameters that define the parameterized CP(lambda) function
  * - 
    - :code:`WE_Gamma`
    - Float
    - Adaption gain for the I&I wind speed estimator algorithm [m/rad]
  * - 
    - :code:`WE_GearboxRatio`
    - Float
    - Gearbox ratio [>=1],  [-]
  * - 
    - :code:`WE_Jtot`
    - Float
    - Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, [kg m^2]
  * - 
    - :code:`WE_RhoAir`
    - Float
    - Air density, [kg m^-3]
  * - 
    - :code:`PerfFileName`
    - String
    - File containing rotor performance tables (Cp,Ct,Cq)
  * - 
    - :code:`PerfTableSize`
    - Int Int 
    - Size of rotor performance tables in :code:`PerfFileName`, first number refers to number of blade pitch angles (num columns), second number refers to number of tip-speed ratios (num rows)
  * - 
    - :code:`WE_FOPoles_N`
    - Int 
    - Number of first-order system poles used in the Extended Kalman Filter
  * - 
    - :code:`WE_FOPoles_v`
    - Float array, length = :code:`WE_FOPoles_N`
    - Wind speeds for first-order system poles lookup table [m/s]
  * - 
    - :code:`WE_FOPoles`
    - Float array, length = :code:`WE_FOPoles_N`
    - First order system poles [1/s]
  * - YAW CONTROL
    - :code:`Y_ErrThresh`
    - Float
    - Yaw error threshold. Turbine begins to yaw when it passes this. [rad^2 s]
  * - 
    - :code:`Y_IPC_IntSat`
    - Float 
    - Integrator saturation (maximum signal amplitude contribution to pitch from yaw-by-IPC), [rad]
  * - 
    - :code:`Y_IPC_n`
    - Int
    - Number of controller gains for yaw-by-IPC
  * - 
    - :code:`Y_IPC_KP`
    - Float array, length = :code:`Y_IPC_n`
    - Yaw-by-IPC proportional controller gains Kp [s]
  * - 
    - :code:`Y_IPC_KI`
    - Float array, length = :code:`Y_IPC_n`
    - Yaw-by-IPC integral controller gain Ki [-]
  * - 
    - :code:`Y_IPC_omegaLP`
    - Float
    - Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].
  * - 
    - :code:`Y_IPC_zetaLP`
    - Float 
    - Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].
  * - 
    - :code:`Y_MErrSet`
    - Float 
    - Yaw alignment error set point, [rad].
  * - 
    - :code:`Y_omegaLPFast`
    - Float
    - Corner frequency fast low pass filter, [rad/s].
  * - 
    - :code:`Y_omegaLPSlow`
    - Float
    - Corner frequency slow low pass filter, [rad/s].
  * - 
    - :code:`Y_Rate`
    - Float
    - Yaw rate, [rad/s]. 
  * - TOWER FORE-AFT DAMPING
    - :code:`FA_KI`
    - Float
    - Integral gain for the fore-aft tower damper controller [rad*s/m]. -1 = off
  * - 
    - :code:`FA_HPF_CornerFreq`
    - Float	
    - Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]
  * - 
    - :code:`FA_IntSat`
    - Float 
    - Integrator saturation (maximum signal amplitude contribution to pitch from FA damper), [rad] 
  * - MINIMUM PITCH SATURATION
    - :code:`PS_BldPitchMin_N`  
    - Int
    - Number of values in minimum blade pitch lookup table.
  * - 
    - :code:`PS_WindSpeeds`
    - Float array, length = :code:`PS_BldPitchMin_n` 
    - Wind speeds corresponding to minimum blade pitch angles [m/s]
  * - 
    - :code:`PS_BldPitchMin` 
    - Float array, length = :code:`PS_BldPitchMin_n`
    - Minimum blade pitch angles [rad]
  * - SHUTDOWN
    - :code:`SD_MaxPit`
    - Float
    - Maximum blade pitch angle to initiate shutdown, [rad]
  * - 
    - :code:`SD_CornerFreq`
    - Float 
    - Cutoff Frequency for first order low-pass filter for blade pitch angle, [rad/s]
  * - FLOATING
    - :code:`Fl_Kp`
    - Float
    - Nacelle velocity proportional feedback gain [s]
  * - FLAP ACTUATION
    - :code:`Flp_Angle`
    - Float 
    - Initial or steady state flap angle [rad]
  * - 
    - :code:`Flp_Kp` 
    - Float 
    - Trailing edge flap control proportional gain [s]
  * - 
    - :code:`Flp_Ki` 
    - Float 
    - Trailing edge flap control integral gain [s]
  * - 
    - :code:`Flp_MaxPit` 
    - Float 
    - Maximum (and minimum) flap angle [rad]


.. _OpenFAST: https://github.com/openfast/openfast
.. _yaml: https://yaml.org/
