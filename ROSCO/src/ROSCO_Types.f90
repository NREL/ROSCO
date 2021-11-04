! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Define variable types

! Types:
!       ControlParameters: Parameters read from DISCON.IN
!       LocalVariables: Varaibles shared by controller modules
!       ObjectInstances: Instances used for recursive functions (i.e. filters)
!       PerformanceData: Rotor performance surface data

MODULE ROSCO_Types
! Define Types
USE, INTRINSIC  :: ISO_C_Binding
IMPLICIT NONE


TYPE, PUBLIC :: ControlParameters
    INTEGER(4)                          :: LoggingLevel                 ! 0 = write no debug files, 1 = write standard output .dbg-file, 2 = write standard output .dbg-file and complete avrSWAP-array .dbg2-file
    
    INTEGER(4)                          :: F_LPFType                    ! {1: first-order low-pass filter, 2: second-order low-pass filter}, [rad/s] 
    INTEGER(4)                          :: F_NotchType                  ! Notch on the measured generator speed {0: disable, 1: enable} 
    REAL(8)                             :: F_LPFCornerFreq              ! Corner frequency (-3dB point) in the first-order low-pass filter, [rad/s]
    REAL(8)                             :: F_LPFDamping                 ! Damping coefficient [used only when F_FilterType = 2]
    REAL(8)                             :: F_NotchCornerFreq            ! Natural frequency of the notch filter, [rad/s]
    REAL(8), DIMENSION(:), ALLOCATABLE  :: F_NotchBetaNumDen            ! These two notch damping values (numerator and denominator) determines the width and depth of the notch
    REAL(8)                             :: F_SSCornerFreq               ! Setpoint Smoother mode {0: no setpoint smoothing, 1: introduce setpoint smoothing}
    REAL(8), DIMENSION(:), ALLOCATABLE  :: F_FlCornerFreq               ! Corner frequency (-3dB point) in the second order low pass filter of the tower-top fore-aft motion for floating feedback control [rad/s].
    REAL(8), DIMENSION(:), ALLOCATABLE  :: F_FlpCornerFreq              ! Corner frequency (-3dB point) in the second order low pass filter of the blade root bending moment for flap control [rad/s].

    REAL(8)                             :: FA_HPFCornerFreq             ! Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]
    REAL(8)                             :: FA_IntSat                    ! Integrator saturation (maximum signal amplitude contrbution to pitch from FA damper), [rad]
    REAL(8)                             :: FA_KI                        ! Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m]
    
    INTEGER(4)                          :: IPC_ControlMode              ! Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {0: off, 1: 1P reductions, 2: 1P+2P reductions}
    REAL(8)                             :: IPC_IntSat                   ! Integrator saturation (maximum signal amplitude contrbution to pitch from IPC)
    REAL(8), DIMENSION(:), ALLOCATABLE  :: IPC_KI                       ! Integral gain for the individual pitch controller, [-]. 8E-10
    REAL(8), DIMENSION(:), ALLOCATABLE  :: IPC_aziOffset                ! Phase offset added to the azimuth angle for the individual pitch controller, [rad].
    REAL(8)                             :: IPC_CornerFreqAct            ! Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {0: Disable}, [rad/s]
    
    INTEGER(4)                          :: PC_ControlMode               ! Blade pitch control mode {0: No pitch, fix to fine pitch, 1: active PI blade pitch control}
    INTEGER(4)                          :: PC_GS_n                      ! Amount of gain-scheduling table entries
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PC_GS_angles                 ! Gain-schedule table: pitch angles
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PC_GS_KP                     ! Gain-schedule table: pitch controller kp gains
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PC_GS_KI                     ! Gain-schedule table: pitch controller ki gains
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PC_GS_KD                     ! Gain-schedule table: pitch controller kd gains
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PC_GS_TF                     ! Gain-schedule table: pitch controller tf gains (derivative filter)
    REAL(8)                             :: PC_MaxPit                    ! Maximum physical pitch limit, [rad].
    REAL(8)                             :: PC_MinPit                    ! Minimum physical pitch limit, [rad].
    REAL(8)                             :: PC_MaxRat                    ! Maximum pitch rate (in absolute value) in pitch controller, [rad/s].
    REAL(8)                             :: PC_MinRat                    ! Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
    REAL(8)                             :: PC_RefSpd                    ! Desired (reference) HSS speed for pitch controller, [rad/s].
    REAL(8)                             :: PC_FinePit                   ! Record 5: Below-rated pitch angle set-point (deg) [used only with Bladed Interface]
    REAL(8)                             :: PC_Switch                    ! Angle above lowest minimum pitch angle for switch [rad]
    
    INTEGER(4)                          :: VS_ControlMode               ! Generator torque control mode in above rated conditions {0: constant torque, 1: constant power, 2: TSR Tracking, 3: TSR Tracking w/ const power}
    REAL(8)                             :: VS_GenEff                    ! Generator efficiency mechanical power -> electrical power [-]
    REAL(8)                             :: VS_ArSatTq                   ! Above rated generator torque PI control saturation, [Nm] -- 212900
    REAL(8)                             :: VS_MaxRat                    ! Maximum torque rate (in absolute value) in torque controller, [Nm/s].
    REAL(8)                             :: VS_MaxTq                     ! Maximum generator torque in Region 3 (HSS side), [Nm]. -- chosen to be 10% above VS_RtTq
    REAL(8)                             :: VS_MinTq                     ! Minimum generator (HSS side), [Nm].
    REAL(8)                             :: VS_MinOMSpd                  ! Optimal mode minimum speed, [rad/s]
    REAL(8)                             :: VS_Rgn2K                     ! Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2
    REAL(8)                             :: VS_RtPwr                     ! Wind turbine rated power [W]
    REAL(8)                             :: VS_RtTq                      ! Rated torque, [Nm].
    REAL(8)                             :: VS_RefSpd                    ! Rated generator speed [rad/s]
    INTEGER(4)                          :: VS_n                         ! Number of controller gains
    REAL(8), DIMENSION(:), ALLOCATABLE  :: VS_KP                        ! Proportional gain for generator PI torque controller, used in the transitional 2.5 region
    REAL(8), DIMENSION(:), ALLOCATABLE  :: VS_KI                        ! Integral gain for generator PI torque controller, used in the transitional 2.5 region
    REAL(8)                             :: VS_TSRopt                    ! Power-maximizing region 2 tip-speed ratio [rad]
    
    INTEGER(4)                          :: SS_Mode                      ! Setpoint Smoother mode {0: no setpoint smoothing, 1: introduce setpoint smoothing}
    REAL(8)                             :: SS_VSGain                    ! Variable speed torque controller setpoint smoother gain, [-].
    REAL(8)                             :: SS_PCGain                    ! Collective pitch controller setpoint smoother gain, [-].

    INTEGER(4)                          :: WE_Mode                      ! Wind speed estimator mode {0: One-second low pass filtered hub height wind speed, 1: Imersion and Invariance Estimator (Ortega et al.)
    REAL(8)                             :: WE_BladeRadius               ! Blade length [m]
    INTEGER(4)                          :: WE_CP_n                      ! Amount of parameters in the Cp array
    REAL(8), DIMENSION(:), ALLOCATABLE  :: WE_CP                        ! Parameters that define the parameterized CP(\lambda) function
    REAL(8)                             :: WE_Gamma                     ! Adaption gain of the wind speed estimator algorithm [m/rad]
    REAL(8)                             :: WE_GearboxRatio              ! Gearbox ratio, >=1  [-]
    REAL(8)                             :: WE_Jtot                      ! Total drivetrain inertia, including blades, hub and casted generator inertia to LSS [kg m^2]
    REAL(8)                             :: WE_RhoAir                    ! Air density [kg m^-3]
    CHARACTER(1024)                     :: PerfFileName                 ! File containing rotor performance tables (Cp,Ct,Cq)
    INTEGER(4), DIMENSION(:), ALLOCATABLE  :: PerfTableSize             ! Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios
    INTEGER(4)                          :: WE_FOPoles_N                 ! Number of first-order system poles used in EKF
    REAL(8), DIMENSION(:), ALLOCATABLE  :: WE_FOPoles_v                 ! Wind speeds corresponding to first-order system poles [m/s]
    REAL(8), DIMENSION(:), ALLOCATABLE  :: WE_FOPoles                   ! First order system poles

    INTEGER(4)                          :: Y_ControlMode                ! Yaw control mode {0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC}
    REAL(8)                             :: Y_ErrThresh                  ! Error threshold [rad]. Turbine begins to yaw when it passes this. (104.71975512) -- 1.745329252
    REAL(8)                             :: Y_IPC_IntSat                 ! Integrator saturation (maximum signal amplitude contrbution to pitch from yaw-by-IPC)
    INTEGER(4)                          :: Y_IPC_n                      ! Number of controller gains (yaw-by-IPC)
    REAL(8), DIMENSION(:), ALLOCATABLE  :: Y_IPC_KP                     ! Yaw-by-IPC proportional controller gain Kp
    REAL(8), DIMENSION(:), ALLOCATABLE  :: Y_IPC_KI                     ! Yaw-by-IPC integral controller gain Ki
    REAL(8)                             :: Y_IPC_omegaLP                ! Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].
    REAL(8)                             :: Y_IPC_zetaLP                 ! Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].
    REAL(8)                             :: Y_MErrSet                    ! Yaw alignment error, setpoint [rad]
    REAL(8)                             :: Y_omegaLPFast                ! Corner frequency fast low pass filter, 1.0 [Hz]
    REAL(8)                             :: Y_omegaLPSlow                ! Corner frequency slow low pass filter, 1/60 [Hz]
    REAL(8)                             :: Y_Rate                       ! Yaw rate [rad/s]
    
    INTEGER(4)                          :: PS_Mode                      ! Pitch saturation mode {0: no peak shaving, 1: implement pitch saturation}
    INTEGER(4)                          :: PS_BldPitchMin_N             ! Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PS_WindSpeeds                ! Wind speeds corresponding to minimum blade pitch angles [m/s]
    REAL(8), DIMENSION(:), ALLOCATABLE  :: PS_BldPitchMin               ! Minimum blade pitch angles [rad]

    INTEGER(4)                          :: SD_Mode                      ! Shutdown mode {0: no shutdown procedure, 1: pitch to max pitch at shutdown}
    REAL(8)                             :: SD_MaxPit                    ! Maximum blade pitch angle to initiate shutdown, [rad]
    REAL(8)                             :: SD_CornerFreq                ! Cutoff Frequency for first order low-pass filter for blade pitch angle, [rad/s]
    
    INTEGER(4)                          :: Fl_Mode                      ! Floating specific feedback mode {0: no nacelle velocity feedback, 1: nacelle velocity feedback}
    REAL(8)                             :: Fl_Kp                        ! Nacelle velocity proportional feedback gain [s]

    INTEGER(4)                          :: Flp_Mode                     ! Flap actuator mode {0: off, 1: fixed flap position, 2: PI flap control}
    REAL(8)                             :: Flp_Angle                    ! Fixed flap angle (degrees)
    REAL(8)                             :: Flp_Kp                       ! PI flap control proportional gain 
    REAL(8)                             :: Flp_Ki                       ! PI flap control integral gain 
    REAL(8)                             :: Flp_MaxPit                   ! Maximum (and minimum) flap pitch angle [rad]
    
    REAL(8)                             :: PC_RtTq99                    ! 99% of the rated torque value, using for switching between pitch and torque control, [Nm].
    REAL(8)                             :: VS_MaxOMTq                   ! Maximum torque at the end of the below-rated region 2, [Nm]
    REAL(8)                             :: VS_MinOMTq                   ! Minimum torque at the beginning of the below-rated region 2, [Nm]

END TYPE ControlParameters

type :: WE                                    
        REAL(8)                   :: om_r             ! Estimated rotor speed [rad/s]
        REAL(8)                   :: v_t              ! Estimated wind speed, turbulent component [m/s]
        REAL(8)                   :: v_m              ! Estimated wind speed, 10-minute averaged [m/s]
        REAL(8)                   :: v_h              ! Combined estimated wind speed [m/s]
        REAL(8), DIMENSION(3,3)   :: P                ! Covariance estiamte 
        REAL(8), DIMENSION(3,1)   :: xh               ! Estimated state matrix
        REAL(8), DIMENSION(3,1)   :: K                ! Kalman gain matrix

end type WE

type :: FilterParameters
    REAL(8), DIMENSION(99)        :: lpf1_a1                   ! First order filter - Denominator coefficient 1
    REAL(8), DIMENSION(99)        :: lpf1_a0                   ! First order filter - Denominator coefficient 0
    REAL(8), DIMENSION(99)        :: lpf1_b1                   ! First order filter - Numerator coefficient 1
    REAL(8), DIMENSION(99)        :: lpf1_b0                   ! First order filter - Numerator coefficient 0
    REAL(8), DIMENSION(99)        :: lpf1_InputSignalLast      ! First order filter - Previous input
    REAL(8), DIMENSION(99)        :: lpf1_OutputSignalLast     ! First order filter - Previous output
    REAL(8), DIMENSION(99)        :: lpf2_a2                   ! Second order filter - Denominator coefficient 2
    REAL(8), DIMENSION(99)        :: lpf2_a1                   ! Second order filter - Denominator coefficient 1
    REAL(8), DIMENSION(99)        :: lpf2_a0                   ! Second order filter - Denominator coefficient 0
    REAL(8), DIMENSION(99)        :: lpf2_b2                   ! Second order filter - Numerator coefficient 2
    REAL(8), DIMENSION(99)        :: lpf2_b1                   ! Second order filter - Numerator coefficient 1
    REAL(8), DIMENSION(99)        :: lpf2_b0                   ! Second order filter - Numerator coefficient 0
    REAL(8), DIMENSION(99)        :: lpf2_InputSignalLast2     ! Second order filter - Previous input 2
    REAL(8), DIMENSION(99)        :: lpf2_OutputSignalLast2    ! Second order filter - Previous output 2
    REAL(8), DIMENSION(99)        :: lpf2_InputSignalLast1     ! Second order filter - Previous input 1 
    REAL(8), DIMENSION(99)        :: lpf2_OutputSignalLast1    ! Second order filter - Previous output 1
    REAL(8), DIMENSION(99)        :: hpf_InputSignalLast       ! High pass filter - Previous output 1
    REAL(8), DIMENSION(99)        :: hpf_OutputSignalLast      ! High pass filter - Previous output 1
    REAL(8), DIMENSION(99)        :: nfs_OutputSignalLast1     ! Notch filter slopes previous output 1
    REAL(8), DIMENSION(99)        :: nfs_OutputSignalLast2     ! Notch filter slopes previous output 2
    REAL(8), DIMENSION(99)        :: nfs_InputSignalLast1      ! Notch filter slopes previous input 1
    REAL(8), DIMENSION(99)        :: nfs_InputSignalLast2      ! Notch filter slopes previous input 1
    REAL(8), DIMENSION(99)        :: nfs_b2                    ! Notch filter slopes numerator coefficient 2
    REAL(8), DIMENSION(99)        :: nfs_b0                    ! Notch filter slopes numerator coefficient 0
    REAL(8), DIMENSION(99)        :: nfs_a2                    ! Notch filter slopes denominator coefficient 2
    REAL(8), DIMENSION(99)        :: nfs_a1                    ! Notch filter slopes denominator coefficient 1
    REAL(8), DIMENSION(99)        :: nfs_a0                    ! Notch filter slopes denominator coefficient 0
    REAL(8), DIMENSION(99)        :: nf_OutputSignalLast1      ! Notch filter previous output 1
    REAL(8), DIMENSION(99)        :: nf_OutputSignalLast2      ! Notch filter previous output 2
    REAL(8), DIMENSION(99)        :: nf_InputSignalLast1       ! Notch filter previous input 1
    REAL(8), DIMENSION(99)        :: nf_InputSignalLast2       ! Notch filter previous input 2
    REAL(8), DIMENSION(99)        :: nf_b2                     ! Notch filter numerator coefficient 2
    REAL(8), DIMENSION(99)        :: nf_b1                     ! Notch filter numerator coefficient 1
    REAL(8), DIMENSION(99)        :: nf_b0                     ! Notch filter numerator coefficient 0
    REAL(8), DIMENSION(99)        :: nf_a1                     ! Notch filter denominator coefficient 1
    REAL(8), DIMENSION(99)        :: nf_a0                     ! Notch filter denominator coefficient 0
end type FilterParameters

type :: piParams
    REAL(8), DIMENSION(99)        :: ITerm                   ! Integrator term
    REAL(8), DIMENSION(99)        :: ITermLast               ! Previous integrator term
    REAL(8), DIMENSION(99)        :: ITerm2                  ! Integrator term - second integrator
    REAL(8), DIMENSION(99)        :: ITermLast2              ! Previous integrator term - second integrator
end type piParams

TYPE, PUBLIC :: LocalVariables
    ! ---------- From avrSWAP ----------
    INTEGER(4)                   :: iStatus
    REAL(8)                      :: Time
    REAL(8)                      :: DT
    REAL(8)                      :: VS_GenPwr
    REAL(8)                      :: GenSpeed
    REAL(8)                      :: RotSpeed
    REAL(8)                      :: Y_M
    REAL(8)                      :: HorWindV
    REAL(8)                      :: rootMOOP(3)
    REAL(8)                      :: BlPitch(3)
    REAL(8)                      :: Azimuth
    INTEGER(4)                   :: NumBl
    REAL(8)                      :: FA_Acc                       ! Tower fore-aft acceleration [m/s^2]
    REAL(8)                      :: NacIMU_FA_Acc                       ! Tower fore-aft acceleration [rad/s^2]

    ! ---------- -Internal controller variables ----------
    REAL(8)                               :: FA_AccHPF                    ! High-pass filtered fore-aft acceleration [m/s^2]
    REAL(8)                               :: FA_AccHPFI                   ! Tower velocity, high-pass filtered and integrated fore-aft acceleration [m/s]
    REAL(8)                               :: FA_PitCom(3)                 ! Tower fore-aft vibration damping pitch contribution [rad]
    REAL(8)                               :: RotSpeedF                    ! Filtered LSS (generator) speed [rad/s].
    REAL(8)                               :: GenSpeedF                    ! Filtered HSS (generator) speed [rad/s].
    REAL(8)                               :: GenTq                        ! Electrical generator torque, [Nm].
    REAL(8)                               :: GenTqMeas                    ! Measured generator torque [Nm]
    REAL(8)                               :: GenArTq                      ! Electrical generator torque, for above-rated PI-control [Nm].
    REAL(8)                               :: GenBrTq                      ! Electrical generator torque, for below-rated PI-control [Nm].
    REAL(8)                               :: IPC_PitComF(3)               ! Commanded pitch of each blade as calculated by the individual pitch controller, F stands for low-pass filtered [rad].
    REAL(8)                               :: IPC_IntAxisTilt_1P           ! Integral of the direct axis, 1P
    REAL(8)                               :: IPC_IntAxisYaw_1P            ! Integral of quadrature axis, 1P
    REAL(8)                               :: IPC_IntAxisTilt_2P           ! Integral of the direct axis, 2P
    REAL(8)                               :: IPC_IntAxisYaw_2P            ! Integral of the quadrature axis, 2P
    REAL(8)                               :: PC_KP                        ! Proportional gain for pitch controller at rated pitch (zero) [s].
    REAL(8)                               :: PC_KI                        ! Integral gain for pitch controller at rated pitch (zero) [-].
    REAL(8)                               :: PC_KD                        ! Differential gain for pitch controller at rated pitch (zero) [-].
    REAL(8)                               :: PC_TF                        ! First-order filter parameter for derivative action
    REAL(8)                               :: PC_MaxPit                    ! Maximum pitch setting in pitch controller (variable) [rad].
    REAL(8)                               :: PC_MinPit                    ! Minimum pitch setting in pitch controller (variable) [rad].
    REAL(8)                               :: PC_PitComT                   ! Total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(8)                               :: PC_PitComT_Last              ! Last total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(8)                               :: PC_PitComTF                  ! Filtered total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(8)                               :: PC_PitComT_IPC(3)            ! Total command pitch based on the sum of the proportional and integral terms, including IPC term [rad].
    REAL(8)                               :: PC_PwrErr                    ! Power error with respect to rated power [W]
    REAL(8)                               :: PC_SineExcitation            ! Sine contribution to pitch signal
    REAL(8)                               :: PC_SpdErr                    ! Current speed error (pitch control) [rad/s].
    INTEGER(4)                            :: PC_State                     ! State of the pitch control system
    REAL(8)                               :: PitCom(3)                    ! Commanded pitch of each blade the last time the controller was called [rad].
    REAL(8)                               :: SS_DelOmegaF                 ! Filtered setpoint shifting term defined in setpoint smoother [rad/s].
    REAL(8)                               :: TestType                     ! Test variable, no use
    REAL(8)                               :: VS_MaxTq                     ! Maximum allowable generator torque [Nm].
    REAL(8)                               :: VS_LastGenTrq                ! Commanded electrical generator torque the last time the controller was called [Nm].
    REAL(8)                               :: VS_LastGenPwr                ! Commanded electrical generator torque the last time the controller was called [Nm].
    REAL(8)                               :: VS_MechGenPwr                ! Mechanical power on the generator axis [W]
    REAL(8)                               :: VS_SpdErrAr                  ! Current speed error for region 2.5 PI controller (generator torque control) [rad/s].
    REAL(8)                               :: VS_SpdErrBr                  ! Current speed error for region 1.5 PI controller (generator torque control) [rad/s].
    REAL(8)                               :: VS_SpdErr                    ! Current speed error for tip-speed-ratio tracking controller (generator torque control) [rad/s].
    INTEGER(4)                            :: VS_State                     ! State of the torque control system
    REAL(8)                               :: VS_Rgn3Pitch                 ! Pitch angle at which the state machine switches to region 3, [rad].
    REAL(8)                               :: WE_Vw                        ! Estimated wind speed [m/s]
    REAL(8)                               :: WE_Vw_F                      ! Filtered estimated wind speed [m/s]
    REAL(8)                               :: WE_VwI                       ! Integrated wind speed quantity for estimation [m/s]
    REAL(8)                               :: WE_VwIdot                    ! Differentiated integrated wind speed quantity for estimation [m/s]
    REAL(8)                               :: VS_LastGenTrqF               ! Differentiated integrated wind speed quantity for estimation [m/s]
    REAL(8)                               :: Y_AccErr                     ! Accumulated yaw error [rad].
    REAL(8)                               :: Y_ErrLPFFast                 ! Filtered yaw error by fast low pass filter [rad].
    REAL(8)                               :: Y_ErrLPFSlow                 ! Filtered yaw error by slow low pass filter [rad].
    REAL(8)                               :: Y_MErr                       ! Measured yaw error, measured + setpoint [rad].
    REAL(8)                               :: Y_YawEndT                    ! Yaw end time [s]. Indicates the time up until which yaw is active with a fixed rate
    LOGICAL(1)                            :: SD                           ! Shutdown, .FALSE. if inactive, .TRUE. if active
    REAL(8)                               :: Fl_PitCom                           ! Shutdown, .FALSE. if inactive, .TRUE. if active
    REAL(8)                               :: NACIMU_FA_AccF
    REAL(8)                               :: FA_AccF
    REAL(8)                               :: Flp_Angle(3)                 ! Flap Angle (rad)
    REAL(8)                               :: RootMyb_Last(3)              ! Previous blade root bending moment
    INTEGER(4)                            :: ACC_INFILE_SIZE
    CHARACTER, DIMENSION(:), ALLOCATABLE  :: ACC_INFILE
    LOGICAL(4)                            :: restart

    type(WE)                            :: WE
    type(FilterParameters)              :: FP
    type(piParams)                      :: piP
    END TYPE LocalVariables

TYPE, PUBLIC :: ObjectInstances
    INTEGER(4)                          :: instLPF
    INTEGER(4)                          :: instSecLPF
    INTEGER(4)                          :: instHPF
    INTEGER(4)                          :: instNotchSlopes
    INTEGER(4)                          :: instNotch
    INTEGER(4)                          :: instPI
END TYPE ObjectInstances

TYPE, PUBLIC :: PerformanceData
    REAL(8), DIMENSION(:), ALLOCATABLE      :: TSR_vec
    REAL(8), DIMENSION(:), ALLOCATABLE      :: Beta_vec
    REAL(8), DIMENSION(:,:), ALLOCATABLE    :: Cp_mat
    REAL(8), DIMENSION(:,:), ALLOCATABLE    :: Ct_mat
    REAL(8), DIMENSION(:,:), ALLOCATABLE    :: Cq_mat
END TYPE PerformanceData

TYPE, PUBLIC :: DebugVariables                                          
! Variables used for debug purposes
    REAL(8)                             :: WE_Cp                        ! Cp that WSE uses to determine aerodynamic torque[-]
    REAL(8)                             :: WE_b                         ! Pitch that WSE uses to determine aerodynamic torque[-]
    REAL(8)                             :: WE_w                         ! Rotor Speed that WSE uses to determine aerodynamic torque[-]
    REAL(8)                             :: WE_t                         ! Torque that WSE uses[-]
    REAL(8)                             :: WE_Vm                        ! Mean wind speed component in WSE [m/s]
    REAL(8)                             :: WE_Vt                        ! Turbulent wind speed component in WSE [m/s]
    REAL(8)                             :: WE_lambda                    ! TSR in WSE [rad]
    !
    REAL(8)                             :: PC_PICommand                 

END TYPE DebugVariables

TYPE, PUBLIC :: ErrorVariables
    ! Error Catching
    INTEGER(4)                      :: size_avcMSG
    INTEGER(C_INT)                  :: aviFAIL             ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
    ! CHARACTER(:), ALLOCATABLE  :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
    CHARACTER(:), ALLOCATABLE       :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
END TYPE ErrorVariables

END MODULE ROSCO_Types
