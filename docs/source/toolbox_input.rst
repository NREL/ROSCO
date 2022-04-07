
.. toctree::

.. _rt_tuning_yaml: 

**************************
ROSCO_Toolbox tuning .yaml
**************************
Definition of inputs for ROSCO tuning procedure


/Users/dzalkind/Tools/ROSCO/ROSCO_toolbox/inputs/toolbox_schema.



path_params
****************************************


:code:`FAST_InputFile` : String
    Name of `*.fst` file

:code:`FAST_directory` : String
    Main OpenFAST model directory, where the `*.fst` lives, relative
    to ROSCO dir (if applicable)

:code:`rotor_performance_filename` : String
    Filename for rotor performance text file (if it has been generated
    by ccblade already)



turbine_params
****************************************


:code:`rotor_inertia` : Float, kg m^2
    Rotor inertia [kg m^2], {Available in Elastodyn .sum file}

:code:`rated_rotor_speed` : Float, rad/s
    Rated rotor speed [rad/s]

    *Minimum* = 0

:code:`v_min` : Float, m/s
    Cut-in wind speed of the wind turbine.

    *Minimum* = 0

:code:`v_max` : Float, m/s
    Cut-out wind speed of the wind turbine.

    *Minimum* = 0

:code:`max_pitch_rate` : Float, rad/s
    Maximum blade pitch rate [rad/s]

    *Minimum* = 0

:code:`max_torque_rate` : Float, Nm/s
    Maximum torque rate [Nm/s], {~1/4 VS_RtTq/s}

    *Minimum* = 0

:code:`rated_power` : Float, W
    Rated Power [W]

    *Minimum* = 0

:code:`bld_edgewise_freq` : Float, rad/s
    Blade edgewise first natural frequency [rad/s]

    *Default* = 4.0

    *Minimum* = 0

:code:`bld_flapwise_freq` : Float, rad/s
    Blade flapwise first natural frequency [rad/s]

    *Default* = 0

    *Minimum* = 0

:code:`TSR_operational` : Float
    Optimal tip speed ratio, if 0 the optimal TSR will be determined
    by the Cp surface

    *Default* = 0

    *Minimum* = 0



controller_params
****************************************


:code:`LoggingLevel` : Float
    0- write no debug files, 1- write standard output .dbg-file, 2-
    write standard output .dbg-file and complete avrSWAP-array
    .dbg2-file

    *Default* = 1

    *Minimum* = 0    *Maximum* = 2


:code:`F_LPFType` : Float
    1- first-order low-pass filter, 2- second-order low-pass filter,
    [rad/s] (currently filters generator speed and pitch control
    signals)

    *Default* = 1

    *Minimum* = 1    *Maximum* = 2


:code:`F_NotchType` : Float
    Notch on the measured generator speed and/or tower fore-aft motion
    (for floating) {0- disable, 1- generator speed, 2- tower-top fore-
    aft motion, 3- generator speed and tower-top fore-aft motion}

    *Default* = 0

    *Minimum* = 0    *Maximum* = 3


:code:`IPC_ControlMode` : Float
    Turn Individual Pitch Control (IPC) for fatigue load reductions
    (pitch contribution) (0- off, 1- 1P reductions, 2- 1P+2P
    reduction)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`VS_ControlMode` : Float
    Generator torque control mode in above rated conditions (0-
    constant torque, 1- constant power, 2- TSR tracking PI control
    with constant torque, 3- TSR tracking with constant power)

    *Default* = 2

    *Minimum* = 0    *Maximum* = 3


:code:`PC_ControlMode` : Float
    Blade pitch control mode (0- No pitch, fix to fine pitch, 1-
    active PI blade pitch control)

    *Default* = 1

    *Minimum* = 0    *Maximum* = 1


:code:`Y_ControlMode` : Float
    Yaw control mode (0- no yaw control, 1- yaw rate control, 2- yaw-
    by-IPC)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`SS_Mode` : Float
    Setpoint Smoother mode (0- no setpoint smoothing, 1- introduce
    setpoint smoothing)

    *Default* = 1

    *Minimum* = 0    *Maximum* = 2


:code:`WE_Mode` : Float
    Wind speed estimator mode (0- One-second low pass filtered hub
    height wind speed, 1- Immersion and Invariance Estimator (Ortega
    et al.)

    *Default* = 2

    *Minimum* = 0    *Maximum* = 2


:code:`PS_Mode` : Float
    Pitch saturation mode (0- no pitch saturation, 1- peak shaving, 2-
    Cp-maximizing pitch saturation, 3- peak shaving and Cp-maximizing
    pitch saturation)

    *Default* = 3

    *Minimum* = 0    *Maximum* = 3


:code:`SD_Mode` : Float
    Shutdown mode (0- no shutdown procedure, 1- pitch to max pitch at
    shutdown)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 1


:code:`Fl_Mode` : Float
    Floating specific feedback mode (0- no nacelle velocity feedback,
    1 - nacelle velocity feedback, 2 - nacelle pitching acceleration
    feedback)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`Flp_Mode` : Float
    Flap control mode (0- no flap control, 1- steady state flap angle,
    2- Proportional flap control)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`PwC_Mode` : Float
    Active Power Control Mode (0- no active power control 1- constant
    active power control, 2- open loop power vs time, 3- open loop
    power vs. wind speed)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`U_pc` : Array of Floats
    List of wind speeds to schedule pitch control zeta and omega

    *Default* = [12]

    *Minimum* = 0

:code:`zeta_pc` : Array of Floats or Float
    List of pitch controller desired damping ratio at U_pc [-]

    *Default* = [1.0]

:code:`omega_pc` : Array of Floats or Float, rad/s
    List of pitch controller desired natural frequency at U_pc [rad/s]

    *Default* = [0.2]

:code:`interp_type` : String from, ['sigma', 'linear', 'quadratic', 'cubic']
    Type of interpolation between above rated tuning values (only used
    for multiple pitch controller tuning values)

    *Default* = sigma

:code:`zeta_vs` : Float
    Torque controller desired damping ratio [-]

    *Default* = 1.0

    *Minimum* = 0

:code:`omega_vs` : Float, rad/s
    Torque controller desired natural frequency [rad/s]

    *Default* = 0.2

    *Minimum* = 0

:code:`max_pitch` : Float, rad
    Maximum pitch angle [rad], {default = 90 degrees}

    *Default* = 1.57

:code:`min_pitch` : Float, rad
    Minimum pitch angle [rad], {default = 0 degrees}

    *Default* = 0

:code:`vs_minspd` : Float, rad/s
    Minimum rotor speed [rad/s], {default = 0 rad/s}

    *Default* = 0

:code:`ss_vsgain` : Float
    Torque controller setpoint smoother gain bias percentage [%, <= 1
    ], {default = 100%}

    *Default* = 1.0

:code:`ss_pcgain` : Float, rad
    Pitch controller setpoint smoother gain bias percentage  [%, <= 1
    ], {default = 0.1%}

    *Default* = 0.001

:code:`ps_percent` : Float, rad
    Percent peak shaving  [%, <= 1 ], {default = 80%}

    *Default* = 0.8    *Maximum* = 1


:code:`sd_maxpit` : Float, rad
    Maximum blade pitch angle to initiate shutdown [rad], {default =
    40 deg.}

    *Default* = 0.6981

:code:`flp_maxpit` : Float, rad
    Maximum (and minimum) flap pitch angle [rad]

    *Default* = 0.1745

:code:`twr_freq` : Float, rad/s
    Tower natural frequency, for floating only

    *Minimum* = 0

:code:`ptfm_freq` : Float, rad/s
    Platform natural frequency, for floating only

    *Minimum* = 0

:code:`WS_GS_n` : Float
    Number of wind speed breakpoints

    *Default* = 60

    *Minimum* = 0

:code:`PC_GS_n` : Float
    Number of pitch angle gain scheduling breakpoints

    *Default* = 30

    *Minimum* = 0

:code:`Kp_float` : Float, s
    Gain of floating feedback control

:code:`tune_Fl` : Boolean
    Whether to automatically tune Kp_float

    *Default* = True

:code:`zeta_flp` : Float
    Flap controller desired damping ratio [-]

    *Minimum* = 0

:code:`omega_flp` : Float, rad/s
    Flap controller desired natural frequency [rad/s]

    *Minimum* = 0

:code:`flp_kp_norm` : Float
    Flap controller normalization term for DC gain (kappa)

    *Minimum* = 0

:code:`flp_tau` : Float, s
    Flap controller time constant for integral gain

    *Minimum* = 0

:code:`IPC_Kp1p` : Float, s
    Proportional gain for IPC, 1P [s]

    *Default* = 0.0

    *Minimum* = 0

:code:`IPC_Kp2p` : Float
    Proportional gain for IPC, 2P [-]

    *Default* = 0.0

    *Minimum* = 0

:code:`IPC_Ki1p` : Float, s
    Integral gain for IPC, 1P [s]

    *Default* = 0.0

    *Minimum* = 0

:code:`IPC_Ki2p` : Float
    integral gain for IPC, 2P [-]

    *Default* = 0.0

    *Minimum* = 0



filter_params
########################################


:code:`f_lpf_cornerfreq` : Float, rad/s
    Corner frequency (-3dB point) in the first order low pass filter
    of the generator speed [rad/s]

    *Minimum* = 0

:code:`f_lpf_damping` : Float, rad/s
    Damping ratio in the first order low pass filter of the generator
    speed [-]

    *Minimum* = 0

:code:`f_we_cornerfreq` : Float, rad/s
    Corner frequency (-3dB point) in the first order low pass filter
    for the wind speed estimate [rad/s]

    *Default* = 0.20944

    *Minimum* = 0

:code:`f_fl_highpassfreq` : Float, rad/s
    Natural frequency of first-order high-pass filter for nacelle
    fore-aft motion [rad/s]

    *Default* = 0.01042

    *Minimum* = 0

:code:`f_ss_cornerfreq` : Float, rad/s
    First order low-pass filter cornering frequency for setpoint
    smoother [rad/s]

    *Default* = 0.6283

    *Minimum* = 0

:code:`f_sd_cornerfreq` : Float, rad
    Cutoff Frequency for first order low-pass filter for blade pitch
    angle [rad/s], {default = 0.41888 ~ time constant of 15s}

    *Default* = 0.41888



open_loop
########################################


:code:`flag` : Boolean
    Flag to use open loop control

    *Default* = False

:code:`filename` : String
    Filename of open loop input that ROSCO reads

    *Default* = unused

:code:`OL_Ind_Breakpoint` : Float
    Index (column, 1-indexed) of breakpoint (time) in open loop index

    *Default* = 1

:code:`OL_Ind_BldPitch` : Float
    Index (column, 1-indexed) of breakpoint (time) in open loop index

    *Default* = 0

:code:`OL_Ind_GenTq` : Float
    Index (column, 1-indexed) of breakpoint (time) in open loop index

    *Default* = 0

:code:`OL_Ind_YawRate` : Float
    Index (column, 1-indexed) of breakpoint (time) in open loop index

    *Default* = 0



linmodel_tuning
****************************************


Inputs used for tuning ROSCO using linear (level 2) models

:code:`type` : String from, ['none', 'robust', 'simulation']
    Type of level 2 based tuning - robust gain scheduling (robust) or
    simulation based optimization (simulation)

    *Default* = none

:code:`linfile_path` : String
    Path to OpenFAST linearization (.lin) files, if they exist

    *Default* = none

:code:`lintune_outpath` : String
    Path for outputs from linear model based tuning

    *Default* = lintune_outfiles

:code:`load_parallel` : Boolean
    Load linearization files in parallel (True/False)

    *Default* = False

:code:`stability_margin` : Float or Array of Floats
    Desired maximum stability margin

    *Default* = 0.1

