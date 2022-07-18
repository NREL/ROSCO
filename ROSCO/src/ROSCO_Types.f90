! ROSCO Registry
! This file is automatically generated by write_registry.py using ROSCO v2.5.0
! For any modification to the registry, please edit the rosco_types.yaml accordingly
 
MODULE ROSCO_Types
USE, INTRINSIC :: ISO_C_Binding
USE Constants
IMPLICIT NONE

TYPE, PUBLIC :: ControlParameters
    INTEGER(IntKi)                :: LoggingLevel                ! 0 - write no debug files, 1 - write standard output .dbg-file, 2 - write standard output .dbg-file and complete avrSWAP-array .dbg2-file
    INTEGER(IntKi)                :: F_LPFType                   ! Low pass filter on the rotor and generator speed {1 - first-order low-pass filter, 2 - second-order low-pass filter}, [rad/s]
    INTEGER(IntKi)                :: F_NotchType                 ! Notch on the measured generator speed {0 - disable, 1 - enable}
    REAL(DbKi)                    :: F_LPFCornerFreq             ! Corner frequency (-3dB point) in the first-order low-pass filter, [rad/s]
    REAL(DbKi)                    :: F_LPFDamping                ! Damping coefficient [used only when F_FilterType = 2]
    REAL(DbKi)                    :: F_NotchCornerFreq           ! Natural frequency of the notch filter, [rad/s]
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: F_NotchBetaNumDen           ! These two notch damping values (numerator and denominator) determines the width and depth of the notch
    REAL(DbKi)                    :: F_SSCornerFreq              ! Corner frequency (-3dB point) in the first order low pass filter for the setpoint smoother [rad/s]
    REAL(DbKi)                    :: F_WECornerFreq              ! Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s]
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: F_FlCornerFreq              ! Corner frequency (-3dB point) in the second order low pass filter of the tower-top fore-aft motion for floating feedback control [rad/s].
    REAL(DbKi)                    :: F_FlHighPassFreq            ! Natural frequency of first-roder high-pass filter for nacelle fore-aft motion [rad/s].
    REAL(DbKi)                    :: F_YawErr                    ! Corner low pass filter corner frequency for yaw controller [rad/s].
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: F_FlpCornerFreq             ! Corner frequency (-3dB point) in the second order low pass filter of the blade root bending moment for flap control [rad/s].
    INTEGER(IntKi)                :: TD_Mode                     ! Tower damper mode (0- no tower damper, 1- feed back translational nacelle accelleration to pitch angle
    REAL(DbKi)                    :: FA_HPFCornerFreq            ! Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]
    REAL(DbKi)                    :: FA_IntSat                   ! Integrator saturation (maximum signal amplitude contrbution to pitch from FA damper), [rad]
    REAL(DbKi)                    :: FA_KI                       ! Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m]
    INTEGER(IntKi)                :: IPC_ControlMode             ! Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {0 - off, 1 - 1P reductions, 2 - 1P+2P reductions}
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: IPC_Vramp                   ! Wind speeds for IPC cut-in sigma function [m/s]
    REAL(DbKi)                    :: IPC_IntSat                  ! Integrator saturation (maximum signal amplitude contrbution to pitch from IPC)
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: IPC_KP                      ! Integral gain for the individual pitch controller, [-].
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: IPC_KI                      ! Integral gain for the individual pitch controller, [-].
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: IPC_aziOffset               ! Phase offset added to the azimuth angle for the individual pitch controller, [rad].
    REAL(DbKi)                    :: IPC_CornerFreqAct           ! Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {0 - Disable}, [rad/s]
    INTEGER(IntKi)                :: PC_ControlMode              ! Blade pitch control mode {0 - No pitch, fix to fine pitch, 1 - active PI blade pitch control}
    INTEGER(IntKi)                :: PC_GS_n                     ! Amount of gain-scheduling table entries
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PC_GS_angles                ! Gain-schedule table - pitch angles
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PC_GS_KP                    ! Gain-schedule table - pitch controller kp gains
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PC_GS_KI                    ! Gain-schedule table - pitch controller ki gains
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PC_GS_KD                    ! Gain-schedule table - pitch controller kd gains
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PC_GS_TF                    ! Gain-schedule table - pitch controller tf gains (derivative filter)
    REAL(DbKi)                    :: PC_MaxPit                   ! Maximum physical pitch limit, [rad].
    REAL(DbKi)                    :: PC_MinPit                   ! Minimum physical pitch limit, [rad].
    REAL(DbKi)                    :: PC_MaxRat                   ! Maximum pitch rate (in absolute value) in pitch controller, [rad/s].
    REAL(DbKi)                    :: PC_MinRat                   ! Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
    REAL(DbKi)                    :: PC_RefSpd                   ! Desired (reference) HSS speed for pitch controller, [rad/s].
    REAL(DbKi)                    :: PC_FinePit                  ! Record 5 - Below-rated pitch angle set-point (deg) [used only with Bladed Interface]
    REAL(DbKi)                    :: PC_Switch                   ! Angle above lowest minimum pitch angle for switch [rad]
    INTEGER(IntKi)                :: VS_ControlMode              ! Generator torque control mode in above rated conditions {0 - constant torque, 1 - constant power, 2 - TSR Tracking, 3 - TSR Tracking w/ const power}
    REAL(DbKi)                    :: VS_GenEff                   ! Generator efficiency mechanical power -> electrical power [-]
    REAL(DbKi)                    :: VS_ArSatTq                  ! Above rated generator torque PI control saturation, [Nm] -- 212900
    REAL(DbKi)                    :: VS_MaxRat                   ! Maximum torque rate (in absolute value) in torque controller, [Nm/s].
    REAL(DbKi)                    :: VS_MaxTq                    ! Maximum generator torque in Region 3 (HSS side), [Nm]. -- chosen to be 10% above VS_RtTq
    REAL(DbKi)                    :: VS_MinTq                    ! Minimum generator (HSS side), [Nm].
    REAL(DbKi)                    :: VS_MinOMSpd                 ! Optimal mode minimum speed, [rad/s]
    REAL(DbKi)                    :: VS_Rgn2K                    ! Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2
    REAL(DbKi)                    :: VS_RtPwr                    ! Wind turbine rated power [W]
    REAL(DbKi)                    :: VS_RtTq                     ! Rated torque, [Nm].
    REAL(DbKi)                    :: VS_RefSpd                   ! Rated generator speed [rad/s]
    INTEGER(IntKi)                :: VS_n                        ! Number of controller gains
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: VS_KP                       ! Proportional gain for generator PI torque controller, used in the transitional 2.5 region
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: VS_KI                       ! Integral gain for generator PI torque controller, used in the transitional 2.5 region
    REAL(DbKi)                    :: VS_TSRopt                   ! Power-maximizing region 2 tip-speed ratio [rad]
    INTEGER(IntKi)                :: SS_Mode                     ! Setpoint Smoother mode {0 - no setpoint smoothing, 1 - introduce setpoint smoothing}
    REAL(DbKi)                    :: SS_VSGain                   ! Variable speed torque controller setpoint smoother gain, [-].
    REAL(DbKi)                    :: SS_PCGain                   ! Collective pitch controller setpoint smoother gain, [-].
    INTEGER(IntKi)                :: WE_Mode                     ! Wind speed estimator mode {0 - One-second low pass filtered hub height wind speed, 1 - Imersion and Invariance Estimator (Ortega et al.)
    REAL(DbKi)                    :: WE_BladeRadius              ! Blade length [m]
    INTEGER(IntKi)                :: WE_CP_n                     ! Amount of parameters in the Cp array
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: WE_CP                       ! Parameters that define the parameterized CP(\lambda) function
    REAL(DbKi)                    :: WE_Gamma                    ! Adaption gain of the wind speed estimator algorithm [m/rad]
    REAL(DbKi)                    :: WE_GearboxRatio             ! Gearbox ratio, >=1  [-]
    REAL(DbKi)                    :: WE_Jtot                     ! Total drivetrain inertia, including blades, hub and casted generator inertia to LSS [kg m^2]
    REAL(DbKi)                    :: WE_RhoAir                   ! Air density [kg m^-3]
    CHARACTER(1024)               :: PerfFileName                ! File containing rotor performance tables (Cp,Ct,Cq)
    INTEGER(IntKi), DIMENSION(:), ALLOCATABLE     :: PerfTableSize               ! Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios
    INTEGER(IntKi)                :: WE_FOPoles_N                ! Number of first-order system poles used in EKF
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: WE_FOPoles_v                ! Wind speeds corresponding to first-order system poles [m/s]
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: WE_FOPoles                  ! First order system poles
    INTEGER(IntKi)                :: Y_ControlMode               ! Yaw control mode {0 - no yaw control, 1 - yaw rate control}
    REAL(DbKi)                    :: Y_uSwitch                   ! Wind speed to switch between Y_ErrThresh. If zero, only the first value of Y_ErrThresh is used [m/s]
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: Y_ErrThresh                 ! Error threshold [rad]. Turbine begins to yaw when it passes this
    REAL(DbKi)                    :: Y_Rate                      ! Yaw rate [rad/s]
    REAL(DbKi)                    :: Y_MErrSet                   ! Yaw alignment error, setpoint (for wake steering) [rad]
    REAL(DbKi)                    :: Y_IPC_IntSat                ! Integrator saturation (maximum signal amplitude contrbution to pitch from yaw-by-IPC)
    REAL(DbKi)                    :: Y_IPC_KP                    ! Yaw-by-IPC proportional controller gain Kp
    REAL(DbKi)                    :: Y_IPC_KI                    ! Yaw-by-IPC integral controller gain Ki
    INTEGER(IntKi)                :: PS_Mode                     ! Pitch saturation mode {0 - no peak shaving, 1 -  implement pitch saturation}
    INTEGER(IntKi)                :: PS_BldPitchMin_N            ! Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PS_WindSpeeds               ! Wind speeds corresponding to minimum blade pitch angles [m/s]
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: PS_BldPitchMin              ! Minimum blade pitch angles [rad]
    INTEGER(IntKi)                :: SD_Mode                     ! Shutdown mode {0 - no shutdown procedure, 1 - pitch to max pitch at shutdown}
    REAL(DbKi)                    :: SD_MaxPit                   ! Maximum blade pitch angle to initiate shutdown, [rad]
    REAL(DbKi)                    :: SD_CornerFreq               ! Cutoff Frequency for first order low-pass filter for blade pitch angle, [rad/s]
    INTEGER(IntKi)                :: Fl_Mode                     ! Floating specific feedback mode {0 - no nacelle velocity feedback, 1 - nacelle velocity feedback}
    REAL(DbKi)                    :: Fl_Kp                       ! Nacelle velocity proportional feedback gain [s]
    INTEGER(IntKi)                :: Flp_Mode                    ! Flap actuator mode {0 - off, 1 - fixed flap position, 2 - PI flap control}
    REAL(DbKi)                    :: Flp_Angle                   ! Fixed flap angle (degrees)
    REAL(DbKi)                    :: Flp_Kp                      ! PI flap control proportional gain
    REAL(DbKi)                    :: Flp_Ki                      ! PI flap control integral gain
    REAL(DbKi)                    :: Flp_MaxPit                  ! Maximum (and minimum) flap pitch angle [rad]
    CHARACTER(1024)               :: OL_Filename                 ! Input file with open loop timeseries
    INTEGER(IntKi)                :: OL_Mode                     ! Open loop control mode {0 - no open loop control, 1 - open loop control vs. time, 2 - open loop control vs. wind speed}
    INTEGER(IntKi)                :: Ind_Breakpoint              ! The column in OL_Filename that contains the breakpoint (time if OL_Mode = 1)
    INTEGER(IntKi)                :: Ind_BldPitch                ! The column in OL_Filename that contains the blade pitch input in rad
    INTEGER(IntKi)                :: Ind_GenTq                   ! The column in OL_Filename that contains the generator torque in Nm
    INTEGER(IntKi)                :: Ind_YawRate                 ! The column in OL_Filename that contains the generator torque in Nm
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: OL_Breakpoints              ! Open loop breakpoints in timeseries
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: OL_BldPitch                 ! Open blade pitch timeseries
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: OL_GenTq                    ! Open generator torque timeseries
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: OL_YawRate                  ! Open yaw rate timeseries
    REAL(DbKi), DIMENSION(:,:), ALLOCATABLE     :: OL_Channels                 ! Open loop channels in timeseries
    INTEGER(IntKi)                :: PA_Mode                     ! Pitch actuator mode {0 - not used, 1 - first order filter, 2 - second order filter}
    REAL(DbKi)                    :: PA_CornerFreq               ! Pitch actuator bandwidth/cut-off frequency [rad/s]
    REAL(DbKi)                    :: PA_Damping                  ! Pitch actuator damping ratio [-, unused if PA_Mode = 1]
    INTEGER(IntKi)                :: Ext_Mode                    ! External control mode (0 - not used, 1 - call external control library)
    CHARACTER(1024)               :: DLL_FileName                ! File name of external dynamic library
    CHARACTER(1024)               :: DLL_InFile                  ! Name of input file called by dynamic library (DISCON.IN, e.g.)
    CHARACTER(1024)               :: DLL_ProcName                ! Process name of subprocess called in DLL_Filename (Usually DISCON)
    INTEGER(IntKi)                :: ZMQ_Mode                    ! Flag for ZeroMQ (0-off, 1-yaw}
    CHARACTER(256)                :: ZMQ_CommAddress             ! Comm Address to zeroMQ client
    REAL(DbKi)                    :: ZMQ_UpdatePeriod            ! Integer for zeromq update frequency
    REAL(DbKi)                    :: PC_RtTq99                   ! 99% of the rated torque value, using for switching between pitch and torque control, [Nm].
    REAL(DbKi)                    :: VS_MaxOMTq                  ! Maximum torque at the end of the below-rated region 2, [Nm]
    REAL(DbKi)                    :: VS_MinOMTq                  ! Minimum torque at the beginning of the below-rated region 2, [Nm]
END TYPE ControlParameters

TYPE, PUBLIC :: WE
    REAL(DbKi)                    :: om_r                        ! Estimated rotor speed [rad/s]
    REAL(DbKi)                    :: v_t                         ! Estimated wind speed, turbulent component [m/s]
    REAL(DbKi)                    :: v_m                         ! Estimated wind speed, 10-minute averaged [m/s]
    REAL(DbKi)                    :: v_h                         ! Combined estimated wind speed [m/s]
    REAL(DbKi), DIMENSION(3,3)     :: P                           ! Covariance estiamte
    REAL(DbKi), DIMENSION(3,1)     :: xh                          ! Estimated state matrix
    REAL(DbKi), DIMENSION(3,1)     :: K                           ! Kalman gain matrix
END TYPE WE

TYPE, PUBLIC :: FilterParameters
    REAL(DbKi), DIMENSION(99)     :: lpf1_a1                     ! First order filter - Denominator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: lpf1_a0                     ! First order filter - Denominator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: lpf1_b1                     ! First order filter - Numerator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: lpf1_b0                     ! First order filter - Numerator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: lpf1_InputSignalLast        ! First order filter - Previous input
    REAL(DbKi), DIMENSION(99)     :: lpf1_OutputSignalLast       ! First order filter - Previous output
    REAL(DbKi), DIMENSION(99)     :: lpf2_a2                     ! Second order filter - Denominator coefficient 2
    REAL(DbKi), DIMENSION(99)     :: lpf2_a1                     ! Second order filter - Denominator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: lpf2_a0                     ! Second order filter - Denominator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: lpf2_b2                     ! Second order filter - Numerator coefficient 2
    REAL(DbKi), DIMENSION(99)     :: lpf2_b1                     ! Second order filter - Numerator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: lpf2_b0                     ! Second order filter - Numerator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: lpf2_InputSignalLast2       ! Second order filter - Previous input 2
    REAL(DbKi), DIMENSION(99)     :: lpf2_OutputSignalLast2      ! Second order filter - Previous output 2
    REAL(DbKi), DIMENSION(99)     :: lpf2_InputSignalLast1       ! Second order filter - Previous input 1
    REAL(DbKi), DIMENSION(99)     :: lpf2_OutputSignalLast1      ! Second order filter - Previous output 1
    REAL(DbKi), DIMENSION(99)     :: hpf_InputSignalLast         ! High pass filter - Previous output 1
    REAL(DbKi), DIMENSION(99)     :: hpf_OutputSignalLast        ! High pass filter - Previous output 1
    REAL(DbKi), DIMENSION(99)     :: nfs_OutputSignalLast1       ! Notch filter slopes previous output 1
    REAL(DbKi), DIMENSION(99)     :: nfs_OutputSignalLast2       ! Notch filter slopes previous output 2
    REAL(DbKi), DIMENSION(99)     :: nfs_InputSignalLast1        ! Notch filter slopes previous input 1
    REAL(DbKi), DIMENSION(99)     :: nfs_InputSignalLast2        ! Notch filter slopes previous input 1
    REAL(DbKi), DIMENSION(99)     :: nfs_b2                      ! Notch filter slopes numerator coefficient 2
    REAL(DbKi), DIMENSION(99)     :: nfs_b0                      ! Notch filter slopes numerator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: nfs_a2                      ! Notch filter slopes denominator coefficient 2
    REAL(DbKi), DIMENSION(99)     :: nfs_a1                      ! Notch filter slopes denominator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: nfs_a0                      ! Notch filter slopes denominator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: nf_OutputSignalLast1        ! Notch filter previous output 1
    REAL(DbKi), DIMENSION(99)     :: nf_OutputSignalLast2        ! Notch filter previous output 2
    REAL(DbKi), DIMENSION(99)     :: nf_InputSignalLast1         ! Notch filter previous input 1
    REAL(DbKi), DIMENSION(99)     :: nf_InputSignalLast2         ! Notch filter previous input 2
    REAL(DbKi), DIMENSION(99)     :: nf_b2                       ! Notch filter numerator coefficient 2
    REAL(DbKi), DIMENSION(99)     :: nf_b1                       ! Notch filter numerator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: nf_b0                       ! Notch filter numerator coefficient 0
    REAL(DbKi), DIMENSION(99)     :: nf_a1                       ! Notch filter denominator coefficient 1
    REAL(DbKi), DIMENSION(99)     :: nf_a0                       ! Notch filter denominator coefficient 0
END TYPE FilterParameters

TYPE, PUBLIC :: piParams
    REAL(DbKi), DIMENSION(99)     :: ITerm                       ! Integrator term
    REAL(DbKi), DIMENSION(99)     :: ITermLast                   ! Previous integrator term
    REAL(DbKi), DIMENSION(99)     :: ITerm2                      ! Integrator term - second integrator
    REAL(DbKi), DIMENSION(99)     :: ITermLast2                  ! Previous integrator term - second integrator
END TYPE piParams

TYPE, PUBLIC :: LocalVariables
    INTEGER(IntKi)                :: iStatus                     ! Initialization status
    REAL(DbKi)                    :: Time                        ! Time [s]
    REAL(DbKi)                    :: DT                          ! Time step [s]
    REAL(DbKi)                    :: VS_GenPwr                   ! Generator power [W]
    REAL(DbKi)                    :: GenSpeed                    ! Generator speed (HSS) [rad/s]
    REAL(DbKi)                    :: RotSpeed                    ! Rotor speed (LSS) [rad/s]
    REAL(DbKi)                    :: NacHeading                  ! Nacelle heading of the turbine w.r.t. north [deg]
    REAL(DbKi)                    :: NacVane                     ! Nacelle vane angle [deg]
    REAL(DbKi)                    :: HorWindV                    ! Hub height wind speed m/s
    REAL(DbKi)                    :: rootMOOP(3)                 ! Blade root bending moment [Nm]
    REAL(DbKi)                    :: rootMOOPF(3)                ! Filtered Blade root bending moment [Nm]
    REAL(DbKi)                    :: BlPitch(3)                  ! Blade pitch [rad]
    REAL(DbKi)                    :: Azimuth                     ! Rotor aziumuth angle [rad]
    INTEGER(IntKi)                :: NumBl                       ! Number of blades [-]
    REAL(DbKi)                    :: FA_Acc                      ! Tower fore-aft acceleration [m/s^2]
    REAL(DbKi)                    :: NacIMU_FA_Acc               ! Tower fore-aft acceleration [rad/s^2]
    REAL(DbKi)                    :: FA_AccHPF                   ! High-pass filtered fore-aft acceleration [m/s^2]
    REAL(DbKi)                    :: FA_AccHPFI                  ! Tower velocity, high-pass filtered and integrated fore-aft acceleration [m/s]
    REAL(DbKi)                    :: FA_PitCom(3)                ! Tower fore-aft vibration damping pitch contribution [rad]
    REAL(DbKi)                    :: RotSpeedF                   ! Filtered LSS (generator) speed [rad/s].
    REAL(DbKi)                    :: GenSpeedF                   ! Filtered HSS (generator) speed [rad/s].
    REAL(DbKi)                    :: GenTq                       ! Electrical generator torque, [Nm].
    REAL(DbKi)                    :: GenTqMeas                   ! Measured generator torque [Nm]
    REAL(DbKi)                    :: GenArTq                     ! Electrical generator torque, for above-rated PI-control [Nm].
    REAL(DbKi)                    :: GenBrTq                     ! Electrical generator torque, for below-rated PI-control [Nm].
    REAL(DbKi)                    :: IPC_PitComF(3)              ! Commanded pitch of each blade as calculated by the individual pitch controller, F stands for low-pass filtered [rad].
    REAL(DbKi)                    :: PC_KP                       ! Proportional gain for pitch controller at rated pitch (zero) [s].
    REAL(DbKi)                    :: PC_KI                       ! Integral gain for pitch controller at rated pitch (zero) [-].
    REAL(DbKi)                    :: PC_KD                       ! Differential gain for pitch controller at rated pitch (zero) [-].
    REAL(DbKi)                    :: PC_TF                       ! First-order filter parameter for derivative action
    REAL(DbKi)                    :: PC_MaxPit                   ! Maximum pitch setting in pitch controller (variable) [rad].
    REAL(DbKi)                    :: PC_MinPit                   ! Minimum pitch setting in pitch controller (variable) [rad].
    REAL(DbKi)                    :: PC_PitComT                  ! Total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(DbKi)                    :: PC_PitComT_Last             ! Last total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(DbKi)                    :: PC_PitComTF                 ! Filtered Total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(DbKi)                    :: PC_PitComT_IPC(3)           ! Total command pitch based on the sum of the proportional and integral terms, including IPC term [rad].
    REAL(DbKi)                    :: PC_PwrErr                   ! Power error with respect to rated power [W]
    REAL(DbKi)                    :: PC_SpdErr                   ! Current speed error (pitch control) [rad/s].
    REAL(DbKi)                    :: IPC_AxisTilt_1P             ! Integral of the direct axis, 1P
    REAL(DbKi)                    :: IPC_AxisYaw_1P              ! Integral of quadrature, 1P
    REAL(DbKi)                    :: IPC_AxisTilt_2P             ! Integral of the direct axis, 2P
    REAL(DbKi)                    :: IPC_AxisYaw_2P              ! Integral of quadrature, 2P
    REAL(DbKi)                    :: IPC_KI(2)                   ! Integral gain for IPC, after ramp [-]
    REAL(DbKi)                    :: IPC_KP(2)                   ! Proportional gain for IPC, after ramp [-]
    INTEGER(IntKi)                :: PC_State                    ! State of the pitch control system
    REAL(DbKi)                    :: PitCom(3)                   ! Commanded pitch of each blade the last time the controller was called [rad].
    REAL(DbKi)                    :: PitComAct(3)                ! Actuated pitch of each blade the last time the controller was called [rad].
    REAL(DbKi)                    :: SS_DelOmegaF                ! Filtered setpoint shifting term defined in setpoint smoother [rad/s].
    REAL(DbKi)                    :: TestType                    ! Test variable, no use
    REAL(DbKi)                    :: VS_MaxTq                    ! Maximum allowable generator torque [Nm].
    REAL(DbKi)                    :: VS_LastGenTrq               ! Commanded electrical generator torque the last time the controller was called [Nm].
    REAL(DbKi)                    :: VS_LastGenPwr               ! Commanded electrical generator torque the last time the controller was called [Nm].
    REAL(DbKi)                    :: VS_MechGenPwr               ! Mechanical power on the generator axis [W]
    REAL(DbKi)                    :: VS_SpdErrAr                 ! Current speed error for region 2.5 PI controller (generator torque control) [rad/s].
    REAL(DbKi)                    :: VS_SpdErrBr                 ! Current speed error for region 1.5 PI controller (generator torque control) [rad/s].
    REAL(DbKi)                    :: VS_SpdErr                   ! Current speed error for tip-speed-ratio tracking controller (generator torque control) [rad/s].
    INTEGER(IntKi)                :: VS_State                    ! State of the torque control system
    REAL(DbKi)                    :: VS_Rgn3Pitch                ! Pitch angle at which the state machine switches to region 3, [rad].
    REAL(DbKi)                    :: WE_Vw                       ! Estimated wind speed [m/s]
    REAL(DbKi)                    :: WE_Vw_F                     ! Filtered estimated wind speed [m/s]
    REAL(DbKi)                    :: WE_VwI                      ! Integrated wind speed quantity for estimation [m/s]
    REAL(DbKi)                    :: WE_VwIdot                   ! Differentiated integrated wind speed quantity for estimation [m/s]
    REAL(DbKi)                    :: VS_LastGenTrqF              ! Differentiated integrated wind speed quantity for estimation [m/s]
    LOGICAL                       :: SD                          ! Shutdown, .FALSE. if inactive, .TRUE. if active
    REAL(DbKi)                    :: Fl_PitCom                   ! Shutdown, .FALSE. if inactive, .TRUE. if active
    REAL(DbKi)                    :: NACIMU_FA_AccF              ! None
    REAL(DbKi)                    :: FA_AccF                     ! None
    REAL(DbKi)                    :: Flp_Angle(3)                ! Flap Angle (rad)
    REAL(DbKi)                    :: RootMyb_Last(3)             ! Last blade root bending moment (Nm)
    INTEGER(IntKi)                :: ACC_INFILE_SIZE             ! Length of parameter input filename
    CHARACTER, DIMENSION(:), ALLOCATABLE     :: ACC_INFILE                  ! Parameter input filename
    LOGICAL                       :: restart                     ! Restart flag
    TYPE(WE)                      :: WE                          ! Wind speed estimator parameters derived type
    TYPE(FilterParameters)        :: FP                          ! Filter parameters derived type
    TYPE(piParams)                :: piP                         ! PI parameters derived type
END TYPE LocalVariables

TYPE, PUBLIC :: ObjectInstances
    INTEGER(IntKi)                :: instLPF                     ! Low-pass filter instance
    INTEGER(IntKi)                :: instSecLPF                  ! Second order low-pass filter instance
    INTEGER(IntKi)                :: instHPF                     ! High-pass filter instance
    INTEGER(IntKi)                :: instNotchSlopes             ! Notch filter slopes instance
    INTEGER(IntKi)                :: instNotch                   ! Notch filter instance
    INTEGER(IntKi)                :: instPI                      ! PI controller instance
END TYPE ObjectInstances

TYPE, PUBLIC :: PerformanceData
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: TSR_vec                     ! TSR vector for performance surfaces
    REAL(DbKi), DIMENSION(:), ALLOCATABLE     :: Beta_vec                    ! Blade pitch vector for performance surfaces [deg]
    REAL(DbKi), DIMENSION(:,:), ALLOCATABLE     :: Cp_mat                      ! Power coefficient surface
    REAL(DbKi), DIMENSION(:,:), ALLOCATABLE     :: Ct_mat                      ! Thrust coefficient surface
    REAL(DbKi), DIMENSION(:,:), ALLOCATABLE     :: Cq_mat                      ! Torque coefficient surface
END TYPE PerformanceData

TYPE, PUBLIC :: DebugVariables
    REAL(DbKi)                    :: WE_Cp                       ! Cp that WSE uses to determine aerodynamic torque [-]
    REAL(DbKi)                    :: WE_b                        ! Pitch that WSE uses to determine aerodynamic torque [-]
    REAL(DbKi)                    :: WE_w                        ! Rotor Speed that WSE uses to determine aerodynamic torque [-]
    REAL(DbKi)                    :: WE_t                        ! Torque that WSE uses [-]
    REAL(DbKi)                    :: WE_Vm                       ! Mean wind speed component in WSE [m/s]
    REAL(DbKi)                    :: WE_Vt                       ! Turbulent wind speed component in WSE [m/s]
    REAL(DbKi)                    :: WE_Vw                       ! Estimated wind speed in WSE [m/s]
    REAL(DbKi)                    :: WE_lambda                   ! TSR in WSE [rad]
    REAL(DbKi)                    :: PC_PICommand                ! Commanded collective pitch from pitch PI controller [rad]
    REAL(DbKi)                    :: GenSpeedF                   ! Filtered generator speed [rad/s]
    REAL(DbKi)                    :: RotSpeedF                   ! Filtered rotor speed [rad/s]
    REAL(DbKi)                    :: NacIMU_FA_AccF              ! Filtered NacIMU_FA_Acc [rad/s]
    REAL(DbKi)                    :: FA_AccF                     ! Filtered FA_Acc [m/s]
    REAL(DbKi)                    :: Fl_PitCom                   ! Floating contribution to the pitch command [rad]
    REAL(DbKi)                    :: PC_MinPit                   ! Minimum blade pitch angle [rad]
    REAL(DbKi)                    :: axisTilt_1P                 ! Tilt component of coleman transformation, 1P
    REAL(DbKi)                    :: axisYaw_1P                  ! Yaw component of coleman transformation, 1P
    REAL(DbKi)                    :: axisTilt_2P                 ! Tilt component of coleman transformation, 2P
    REAL(DbKi)                    :: axisYaw_2P                  ! Yaw component of coleman transformation, 2P
    REAL(DbKi)                    :: YawRateCom                  ! Commanded yaw rate [rad/s].
    REAL(DbKi)                    :: NacHeadingTarget            ! Target nacelle heading [rad].
    REAL(DbKi)                    :: NacVaneOffset               ! Nacelle vane angle with offset [rad].
    REAL(DbKi)                    :: Yaw_err                     ! Yaw error [rad].
    REAL(DbKi)                    :: YawState                    ! State of yaw controller
END TYPE DebugVariables

TYPE, PUBLIC :: ErrorVariables
    INTEGER(IntKi)                :: size_avcMSG                 ! None
    INTEGER(C_INT)                :: aviFAIL                     ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
    INTEGER(C_INT)                :: ErrStat                     ! An error status flag used by OpenFAST processes
    CHARACTER(:), ALLOCATABLE     :: ErrMsg                      ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
END TYPE ErrorVariables

TYPE, PUBLIC :: ExtDLL_Type
    INTEGER(C_INTPTR_T)           :: FileAddr                    ! The address of file FileName. (RETURN value from LoadLibrary ) [Windows]
    TYPE(C_PTR)                   :: FileAddrX = C_NULL_PTR      ! The address of file FileName. (RETURN value from dlopen ) [Linux]
    TYPE(C_FUNPTR)                :: ProcAddr(3) = C_NULL_FUNPTR   ! The address of procedure ProcName. (RETURN value from GetProcAddress or dlsym) [initialized to Null for pack/unpack]
    CHARACTER(1024)               :: FileName                    ! The name of the DLL file including the full path to the current working directory.
    CHARACTER(1024)               :: ProcName(3) = ""            ! The name of the procedure in the DLL that will be called.
END TYPE ExtDLL_Type

TYPE, PUBLIC :: ZMQ_Variables
    LOGICAL                       :: ZMQ_Flag                    ! Flag if we're using zeroMQ at all (0-False, 1-True)
    REAL(DbKi)                    :: Yaw_Offset                  ! Yaw offsety command, [rad]
END TYPE ZMQ_Variables

TYPE, PUBLIC :: ExtControlType
    REAL(C_FLOAT), DIMENSION(:), ALLOCATABLE     :: avrSWAP                     ! The swap array- used to pass data to and from the DLL controller [see Bladed DLL documentation]
END TYPE ExtControlType

END MODULE ROSCO_Types