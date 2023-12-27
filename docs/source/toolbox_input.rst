
.. toctree::

.. _rt_tuning_yaml: 

**************************
ROSCO_Toolbox tuning .yaml
**************************
Definition of inputs for ROSCO tuning procedure


toolbox_schema



path_params
****************************************


:code:`FAST_InputFile` : String
    Name of `*.fst` file

:code:`FAST_directory` : String
    Main OpenFAST model directory, where the `*.fst` lives, relative
    to directory of this yaml (if applicable)

:code:`rotor_performance_filename` : String
    Filename for rotor performance text file (if it has been generated
    by ccblade already), relative to directory of this yaml



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

:code:`reynolds_ref` : Float
    Reynolds number near rated speeds, used to interpolate airfoils,
    if provided

    *Default* = 0

    *Minimum* = 0



controller_params
****************************************


:code:`LoggingLevel` : Float
    0- write no debug files, 1- write standard output .dbg-file, 2-
    write standard output .dbg-file and complete avrSWAP-array
    .dbg2-file

    *Default* = 1

    *Minimum* = 0    *Maximum* = 3


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
    Generator torque control mode in above rated conditions (0- no
    torque control, 1- k*omega^2 with PI transitions, 2- WSE TSR
    Tracking, 3- Power-based TSR Tracking)

    *Default* = 2

    *Minimum* = 0    *Maximum* = 3


:code:`VS_ConstPower` : Float
    Do constant power torque control, where above rated torque varies,
    0 for constant torque

    *Default* = 0

    *Minimum* = 0    *Maximum* = 1


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


:code:`TD_Mode` : Float
    Tower damper mode (0- no tower damper, 1- feed back translational
    nacelle accelleration to pitch angle

    *Default* = 0

    *Minimum* = 0    *Maximum* = 1


:code:`TRA_Mode` : Float
    Tower resonance avoidance mode (0- no tower resonsnace avoidance,
    1- use torque control setpoints to avoid a specific frequency

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


:code:`ZMQ_Mode` : Float
    ZMQ Mode (0 - ZMQ Inteface, 1 - ZMQ for yaw control)

    *Default* = 0

    *Minimum* = 0    *Maximum* = 1


:code:`ZMQ_UpdatePeriod` : Float
    Call ZeroMQ every [x] seconds, [s]

    *Default* = 2

    *Minimum* = 0

:code:`PA_Mode` : Float
    Pitch actuator mode {0 - not used, 1 - first order filter, 2 -
    second order filter}

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`PF_Mode` : Float
    Pitch fault mode {0 - not used, 1 - constant offset on one or more
    blades}

    *Default* = 0

    *Minimum* = 0    *Maximum* = 1


:code:`OL_Mode` : Float
    Open loop control mode {0- no open loop control, 1- open loop
    control}

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`AWC_Mode` : Float
    Active wake control mode {0 - not used, 1 - SNL method, 2 - NREL
    method}

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`Ext_Mode` : Float
    External control mode [0 - not used, 1 - call external dynamic
    library]

    *Default* = 0

    *Minimum* = 0    *Maximum* = 1


:code:`CC_Mode` : Float
    Cable control mode [0- unused, 1- User defined, 2- Open loop
    control]

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`StC_Mode` : Float
    Structural control mode [0- unused, 1- User defined, 2- Open loop
    control]

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

:code:`Kp_float` : Float, s or Array of Floats
    Gain(s) of floating feedback control

:code:`tune_Fl` : Boolean
    Whether to automatically tune Kp_float

    *Default* = True

:code:`U_Fl` : Array of Floats or String or Float
    List of wind speeds for tuning floating feedback, or "all" for all
    above-rated wind speeds

    *Default* = []

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

:code:`max_torque_factor` : Float
    Maximum torque = rated torque * max_torque_factor

    *Default* = 1.1

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

:code:`IPC_Vramp` : Array of Floats
    wind speeds for IPC cut-in sigma function [m/s]

    *Default* = [0.0, 0.0]

    *Minimum* = 0.0

:code:`rgn2k_factor` : Float
    Factor on VS_Rgn2K to increase/decrease optimal torque control
    gain, default is 1.  Sometimes environmental conditions or
    differences in BEM solvers necessitate this change.

    *Default* = 1

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

:code:`f_yawerr` : Float, rad/s
    Low pass filter corner frequency for yaw controller [rad/

    *Default* = 0.17952

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

:code:`Ind_Breakpoint` : Float
    Index (column, 1-indexed) of breakpoint (time) in open loop index

    *Default* = 1

    *Minimum* = 0

:code:`Ind_BldPitch` : Array of Floats
    Indices (columns, 1-indexed) of pitch (1,2,3) inputs in open loop
    input

    *Default* = [0, 0, 0]

    *Minimum* = 0

:code:`Ind_GenTq` : Float
    Index (column, 1-indexed) of generator torque in open loop input

    *Default* = 0

    *Minimum* = 0

:code:`Ind_YawRate` : Float
    Index (column, 1-indexed) of nacelle yaw in open loop input

    *Default* = 0

    *Minimum* = 0

:code:`Ind_Azimuth` : Float
    The column in OL_Filename that contains the desired azimuth
    position in rad (used if OL_Mode = 2)

    *Default* = 0

:code:`Ind_CableControl` : Array of Floats
    The column in OL_Filename that contains the cable control inputs
    in m

:code:`Ind_StructControl` : Array of Floats
    The column in OL_Filename that contains the structural control
    inputs in various units

:code:`PA_CornerFreq` : Float, rad/s
    Pitch actuator natural frequency [rad/s]

    *Default* = 3.14

    *Minimum* = 0

:code:`PA_Damping` : Float
    Pitch actuator damping ratio [-]

    *Default* = 0.707

    *Minimum* = 0



DISCON
########################################


These are pass-through parameters for the DISCON.IN file.  Use with caution. Do not set defaults in schema.

:code:`LoggingLevel` : Float
    (0- write no debug files, 1- write standard output .dbg-file, 2-
    write standard output .dbg-file and complete avrSWAP-array
    .dbg2-file)

:code:`Echo` : Float
    0 - no Echo, 1 - Echo input data to <RootName>.echo

    *Default* = 0

:code:`DT_Out` : Float
    Time step to output .dbg* files, or 0 to match sampling period of
    OpenFAST

    *Default* = 0

:code:`F_LPFType` : Float
    1- first-order low-pass filter, 2- second-order low-pass filter
    (currently filters generator speed and pitch control signals

:code:`VS_ControlMode` : Float
    Generator torque control mode in above rated conditions (0- no
    torque control, 1- k*omega^2 with PI transitions, 2- WSE TSR
    Tracking, 3- Power-based TSR Tracking)

    *Minimum* = 0    *Maximum* = 3


:code:`VS_ConstPower` : Float
    Do constant power torque control, where above rated torque varies

    *Minimum* = 0    *Maximum* = 1


:code:`F_NotchType` : Float
    Notch on the measured generator speed and/or tower fore-aft motion
    (for floating) (0- disable, 1- generator speed, 2- tower-top fore-
    aft motion, 3- generator speed and tower-top fore-aft motion)

:code:`IPC_ControlMode` : Float
    Turn Individual Pitch Control (IPC) for fatigue load reductions
    (pitch contribution) (0- off, 1- 1P reductions, 2- 1P+2P
    reductions)

:code:`PC_ControlMode` : Float
    Blade pitch control mode (0- No pitch, fix to fine pitch, 1-
    active PI blade pitch control)

:code:`Y_ControlMode` : Float
    Yaw control mode (0- no yaw control, 1- yaw rate control, 2- yaw-
    by-IPC)

:code:`SS_Mode` : Float
    Setpoint Smoother mode (0- no setpoint smoothing, 1- introduce
    setpoint smoothing)

:code:`WE_Mode` : Float
    Wind speed estimator mode (0- One-second low pass filtered hub
    height wind speed, 1- Immersion and Invariance Estimator, 2-
    Extended Kalman Filter)

:code:`PS_Mode` : Float
    Pitch saturation mode (0- no pitch saturation, 1- implement pitch
    saturation)

:code:`SD_Mode` : Float
    Shutdown mode (0- no shutdown procedure, 1- pitch to max pitch at
    shutdown)

:code:`Fl_Mode` : Float
    Floating specific feedback mode (0- no nacelle velocity feedback,
    1- feed back translational velocity, 2- feed back rotational
    veloicty)

:code:`Flp_Mode` : Float
    Flap control mode (0- no flap control, 1- steady state flap angle,
    2- Proportional flap control)

:code:`OL_Mode` : Float
    Open loop control mode (0 - no open-loop control, 1 - direct open
    loop control, 2 - rotor position control)

:code:`F_LPFCornerFreq` : Float, rad/s
    Corner frequency (-3dB point) in the low-pass filters,

:code:`F_LPFDamping` : Float
    Damping coefficient (used only when F_FilterType = 2 [-]

:code:`F_NumNotchFilts` : Float
    Number of notch filters placed on sensors

:code:`F_NotchFreqs` : Array of Floats or Float, rad/s
    Natural frequency of the notch filters. Array with length
    F_NumNotchFilts

:code:`F_NotchBetaNum` : Array of Floats or Float
    Damping value of numerator (determines the width of notch). Array
    with length F_NumNotchFilts, [-]

:code:`F_NotchBetaDen` : Array of Floats or Float
    Damping value of denominator (determines the depth of notch).
    Array with length F_NumNotchFilts, [-]

:code:`F_GenSpdNotch_N` : Float
    Number of notch filters on generator speed

:code:`F_TwrTopNotch_N` : Float
    Number of notch filters on tower top acceleration signal

:code:`F_GenSpdNotch_Ind` : Array of Floats or Float
    Indices of notch filters on generator speed

:code:`F_TwrTopNotch_Ind` : Array of Floats or Float
    Indices of notch filters on tower top acceleration signal

:code:`F_SSCornerFreq` : Float, rad/s.
    Corner frequency (-3dB point) in the first order low pass filter
    for the setpoint smoother,

:code:`F_WECornerFreq` : Float, rad/s.
    Corner frequency (-3dB point) in the first order low pass filter
    for the wind speed estimate

:code:`F_FlCornerFreq` : Array of Floats
    Natural frequency and damping in the second order low pass filter
    of the tower-top fore-aft motion for floating feedback control

:code:`F_FlHighPassFreq` : Float, rad/s
    Natural frequency of first-order high-pass filter for nacelle
    fore-aft motion

:code:`F_FlpCornerFreq` : Array of Floats
    Corner frequency and damping in the second order low pass filter
    of the blade root bending moment for flap control

:code:`PC_GS_n` : Float
    Amount of gain-scheduling table entries

:code:`PC_GS_angles` : Array of Floats
    Gain-schedule table- pitch angles

:code:`PC_GS_KP` : Array of Floats
    Gain-schedule table- pitch controller kp gains

:code:`PC_GS_KI` : Array of Floats
    Gain-schedule table- pitch controller ki gains

:code:`PC_GS_KD` : Array of Floats
    Gain-schedule table- pitch controller kd gains

:code:`PC_GS_TF` : Array of Floats
    Gain-schedule table- pitch controller tf gains (derivative filter)

:code:`PC_MaxPit` : Float, rad
    Maximum physical pitch limit,

:code:`PC_MinPit` : Float, rad
    Minimum physical pitch limit,

:code:`PC_MaxRat` : Float, rad/s.
    Maximum pitch rate (in absolute value) in pitch controller

:code:`PC_MinRat` : Float, rad/s.
    Minimum pitch rate (in absolute value) in pitch controller

:code:`PC_RefSpd` : Float, rad/s.
    Desired (reference) HSS speed for pitch controller

:code:`PC_FinePit` : Float, rad
    Record 5- Below-rated pitch angle set-point

:code:`PC_Switch` : Float, rad
    Angle above lowest minimum pitch angle for switch

:code:`IPC_IntSat` : Float, rad
    Integrator saturation (maximum signal amplitude contribution to
    pitch from IPC)

:code:`IPC_SatMode` : Integer
    IPC Saturation method (0 - no saturation, 1 - saturate by
    PC_MinPit, 2 - saturate by PS_BldPitchMin)

:code:`IPC_KP` : Array of Floats
    Proportional gain for the individual pitch controller- first
    parameter for 1P reductions, second for 2P reductions, [-]

:code:`IPC_KI` : Array of Floats
    Integral gain for the individual pitch controller- first parameter
    for 1P reductions, second for 2P reductions, [-]

:code:`IPC_aziOffset` : Array of Floats
    Phase offset added to the azimuth angle for the individual pitch
    controller

:code:`IPC_CornerFreqAct` : Float, rad/s
    Corner frequency of the first-order actuators model, to induce a
    phase lag in the IPC signal (0- Disable)

:code:`VS_GenEff` : Float, percent
    Generator efficiency mechanical power -> electrical power, should
    match the efficiency defined in the generator properties

:code:`VS_ArSatTq` : Float, Nm
    Above rated generator torque PI control saturation

:code:`VS_MaxRat` : Float, Nm/s
    Maximum torque rate (in absolute value) in torque controller

:code:`VS_MaxTq` : Float, Nm
    Maximum generator torque in Region 3 (HSS side)

:code:`VS_MinTq` : Float, Nm
    Minimum generator torque (HSS side)

:code:`VS_MinOMSpd` : Float, rad/s
    Minimum generator speed

:code:`VS_Rgn2K` : Float, Nm/(rad/s)^2
    Generator torque constant in Region 2 (HSS side). Only used in
    VS_ControlMode = 1,3

:code:`VS_RtPwr` : Float, W
    Wind turbine rated power

:code:`VS_RtTq` : Float, Nm
    Rated torque

:code:`VS_RefSpd` : Float, rad/s
    Rated generator speed

:code:`VS_n` : Float
    Number of generator PI torque controller gains

:code:`VS_KP` : Float
    Proportional gain for generator PI torque controller. (Only used
    in the transitional 2.5 region if VS_ControlMode =/ 2)

:code:`VS_KI` : Float, s
    Integral gain for generator PI torque controller  (Only used in
    the transitional 2.5 region if VS_ControlMode =/ 2)

:code:`VS_TSRopt` : Float, rad
    Power-maximizing region 2 tip-speed-ratio. Only used in
    VS_ControlMode = 2.

:code:`VS_PwrFiltF` : Float, rad
    Low pass filter on power used to determine generator speed set
    point.  Only used in VS_ControlMode = 3.

    *Default* = 0.314

:code:`SS_VSGain` : Float
    Variable speed torque controller setpoint smoother gain

:code:`SS_PCGain` : Float
    Collective pitch controller setpoint smoother gain

:code:`PRC_Mode` : Float
    Power reference tracking mode, 0- use standard rotor speed set
    points, 1- use PRC rotor speed setpoints

:code:`PRC_WindSpeeds` : Array of Floats
    Array of wind speeds used in rotor speed vs. wind speed lookup
    table [m/s]

:code:`PRC_GenSpeeds` : Array of Floats
    Array of generator speeds corresponding to PRC_WindSpeeds [rad/s]

:code:`PRC_LPF_Freq` : Float
    Frequency of the low pass filter on the wind speed estimate used
    to set PRC_GenSpeeds [rad/s]

    *Default* = 0.078539

:code:`PRC_n` : Float
    Number of elements in PRC_WindSpeeds and PRC_GenSpeeds array

:code:`TRA_ExclSpeed` : Float
    Rotor speed for exclusion [LSS, rad/s]

    *Default* = 0.0

    *Minimum* = 0

:code:`TRA_ExclBand` : Float
    Size of the rotor frequency exclusion band [LSS, rad/s]. Torque
    controller reference will be TRA_ExclSpeed +/- TRA_ExlBand/2

    *Default* = 0.0

    *Minimum* = 0

:code:`TRA_RateLimit` : Float
    Rate limit of change in rotor speed reference [LSS, rad/s].
    Suggested to be VS_RefSpd/400.

    *Default* = 0.0

    *Minimum* = 0

:code:`WE_BladeRadius` : Float, m
    Blade length (distance from hub center to blade tip)

:code:`WE_CP_n` : Float
    Amount of parameters in the Cp array

:code:`WE_CP` : Array of Floats
    Parameters that define the parameterized CP(lambda) function

:code:`WE_Gamma` : Float, m/rad
    Adaption gain of the wind speed estimator algorithm

:code:`WE_GearboxRatio` : Float
    Gearbox ratio, >=1

:code:`WE_Jtot` : Float, kg m^2
    Total drivetrain inertia, including blades, hub and casted
    generator inertia to LSS

:code:`WE_RhoAir` : Float, kg m^-3
    Air density

:code:`PerfFileName` : String
    File containing rotor performance tables (Cp,Ct,Cq) (absolute path
    or relative to this file)

:code:`PerfTableSize` : Float
    Size of rotor performance tables, first number refers to number of
    blade pitch angles, second number referse to number of tip-speed
    ratios

:code:`WE_FOPoles_N` : Float
    Number of first-order system poles used in EKF

:code:`WE_FOPoles_v` : Array of Floats
    Wind speeds corresponding to first-order system poles

:code:`WE_FOPoles` : Array of Floats
    First order system poles

:code:`Y_ErrThresh` : Float, rad^2 s
    Yaw error threshold. Turbine begins to yaw when it passes this

:code:`Y_IPC_IntSat` : Float, rad
    Integrator saturation (maximum signal amplitude contribution to
    pitch from yaw-by-IPC)

:code:`Y_IPC_n` : Float
    Number of controller gains (yaw-by-IPC)

:code:`Y_IPC_KP` : Float
    Yaw-by-IPC proportional controller gain Kp

:code:`Y_IPC_KI` : Float
    Yaw-by-IPC integral controller gain Ki

:code:`Y_IPC_omegaLP` : Float, rad/s.
    Low-pass filter corner frequency for the Yaw-by-IPC controller to
    filtering the yaw alignment error

:code:`Y_IPC_zetaLP` : Float
    Low-pass filter damping factor for the Yaw-by-IPC controller to
    filtering the yaw alignment error.

:code:`Y_MErrSet` : Float, rad
    Yaw alignment error, set point

:code:`Y_omegaLPFast` : Float, rad/s
    Corner frequency fast low pass filter, 1.0

:code:`Y_omegaLPSlow` : Float, rad/s
    Corner frequency slow low pass filter, 1/60

:code:`Y_Rate` : Float, rad/s
    Yaw rate

:code:`FA_KI` : Float, rad s/m
    Integral gain for the fore-aft tower damper controller, -1 = off /
    >0 = on

:code:`FA_HPFCornerFreq` : Float, rad/s
    Corner frequency (-3dB point) in the high-pass filter on the fore-
    aft acceleration signal

:code:`FA_IntSat` : Float, rad
    Integrator saturation (maximum signal amplitude contribution to
    pitch from FA damper)

:code:`PS_BldPitchMin_N` : Float
    Number of values in minimum blade pitch lookup table (should equal
    number of values in PS_WindSpeeds and PS_BldPitchMin)

:code:`PS_WindSpeeds` : Array of Floats
    Wind speeds corresponding to minimum blade pitch angles

:code:`PS_BldPitchMin` : Array of Floats
    Minimum blade pitch angles

:code:`SD_MaxPit` : Float, rad
    Maximum blade pitch angle to initiate shutdown

:code:`SD_CornerFreq` : Float, rad/s
    Cutoff Frequency for first order low-pass filter for blade pitch
    angle

:code:`Fl_n` : Float, s
    Number of Fl_Kp gains in gain scheduling, optional with default of
    1

:code:`Fl_Kp` : Array of Floats
    Nacelle velocity proportional feedback gain

:code:`Fl_U` : Array of Floats
    Wind speeds for scheduling Fl_Kp, optional if Fl_Kp is single
    value [m/s]

:code:`Flp_Angle` : Float, rad
    Initial or steady state flap angle

:code:`Flp_Kp` : Float, s
    Blade root bending moment proportional gain for flap control

:code:`Flp_Ki` : Float
    Flap displacement integral gain for flap control

:code:`Flp_MaxPit` : Float, rad
    Maximum (and minimum) flap pitch angle

:code:`OL_Filename` : String
    Input file with open loop timeseries (absolute path or relative to
    this file)

:code:`Ind_Breakpoint` : Float
    The column in OL_Filename that contains the breakpoint (time if
    OL_Mode > 0)

:code:`Ind_BldPitch` : Float
    The column in OL_Filename that contains the blade pitch input in
    rad

:code:`Ind_GenTq` : Float
    The column in OL_Filename that contains the generator torque in Nm

:code:`Ind_YawRate` : Float
    The column in OL_Filename that contains the generator torque in Nm

:code:`Ind_Azimuth` : Float
    The column in OL_Filename that contains the desired azimuth
    position in rad (used if OL_Mode = 2)

:code:`RP_Gains` : Array of Floats
    PID gains and Tf of derivative for rotor position control (used if
    OL_Mode = 2)

    *Default* = [0, 0, 0, 0]

:code:`Ind_CableControl` : Array of Floats
    The column in OL_Filename that contains the cable control inputs
    in m

:code:`Ind_StructControl` : Array of Floats
    The column in OL_Filename that contains the structural control
    inputs in various units

:code:`DLL_FileName` : String
    Name/location of the dynamic library {.dll [Windows] or .so
    [Linux]} in the Bladed-DLL format

    *Default* = unused

:code:`DLL_InFile` : String
    Name of input file sent to the DLL

    *Default* = unused

:code:`DLL_ProcName` : String
    Name of procedure in DLL to be called

    *Default* = DISCON

:code:`PF_Offsets` : Array of Floats
    Pitch angle offsets for each blade (array with length of 3)

    *Default* = [0, 0, 0]

:code:`CC_Group_N` : Float
    Number of cable control groups

    *Default* = 0

:code:`CC_GroupIndex` : Array of Floats
    First index for cable control group, should correspond to deltaL

    *Default* = [0]

:code:`CC_ActTau` : Float
    Time constant for line actuator [s]

    *Default* = 20

:code:`StC_Group_N` : Float
    Number of cable control groups

    *Default* = 0

:code:`StC_GroupIndex` : Array of Floats
    First index for structural control group, options specified in
    ServoDyn summary output

    *Default* = [0]

:code:`AWC_Mode` : Float
    Active wake control mode {0 - not used, 1 - complex number method,
    2 - Coleman transformation method}

    *Default* = 0

    *Minimum* = 0    *Maximum* = 2


:code:`AWC_NumModes` : Float, rad
    Number of AWC modes

    *Default* = 1

:code:`AWC_n` : Array of Floats
    AWC azimuthal number (only used in complex number method)

    *Default* = [1]

:code:`AWC_harmonic` : Array of Integers
    AWC Coleman transform harmonic (only used in Coleman transform
    method)

    *Default* = [1]

:code:`AWC_freq` : Array of Floats
    AWC frequency [Hz]

    *Default* = [0.05]

:code:`AWC_amp` : Array of Floats
    AWC amplitude [deg]

    *Default* = [1.0]

:code:`AWC_clockangle` : Array of Floats
    AWC clock angle [deg]

    *Default* = [0]

:code:`ZMQ_CommAddress` : String
    Communication address for ZMQ server, (e.g.
    "tcp://localhost:5555")

    *Default* = tcp://localhost:5555

:code:`ZMQ_UpdatePeriod` : Float
    Update period at zmq interface to send measurements and wait for
    setpoint [sec.]

    *Default* = 1.0

:code:`ZMQ_ID` : Float
    Integer identifier of turbine

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

