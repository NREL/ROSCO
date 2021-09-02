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
    REAL(C_Double)                             :: F_LPFCornerFreq              ! Corner frequency (-3dB point) in the first-order low-pass filter, [rad/s]
    REAL(C_Double)                             :: F_LPFDamping                 ! Damping coefficient [used only when F_FilterType = 2]
    REAL(C_Double)                             :: F_NotchCornerFreq            ! Natural frequency of the notch filter, [rad/s]
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: F_NotchBetaNumDen            ! These two notch damping values (numerator and denominator) determines the width and depth of the notch
    REAL(C_Double)                             :: F_SSCornerFreq               ! Setpoint Smoother mode {0: no setpoint smoothing, 1: introduce setpoint smoothing}
    REAL(C_Double)                             :: F_WECornerFreq               ! Corner frequency (-3dB point) in the first order low pass filter for the wind speed estimate [rad/s]
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: F_FlCornerFreq               ! Corner frequency (-3dB point) in the second order low pass filter of the tower-top fore-aft motion for floating feedback control [rad/s].
    REAL(C_Double)                             :: F_FlHighPassFreq             ! Natural frequency of first-roder high-pass filter for nacelle fore-aft motion [rad/s]
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: F_FlpCornerFreq              ! Corner frequency (-3dB point) in the second order low pass filter of the blade root bending moment for flap control [rad/s].

    REAL(C_Double)                             :: FA_HPFCornerFreq             ! Corner frequency (-3dB point) in the high-pass filter on the fore-aft acceleration signal [rad/s]
    REAL(C_Double)                             :: FA_IntSat                    ! Integrator saturation (maximum signal amplitude contrbution to pitch from FA damper), [rad]
    REAL(C_Double)                             :: FA_KI                        ! Integral gain for the fore-aft tower damper controller, -1 = off / >0 = on [rad s/m]
    
    INTEGER(4)                          :: IPC_ControlMode              ! Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) {0: off, 1: 1P reductions, 2: 1P+2P reductions}
    REAL(C_Double)                             :: IPC_IntSat                   ! Integrator saturation (maximum signal amplitude contrbution to pitch from IPC)
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: IPC_KI                       ! Integral gain for the individual pitch controller, [-]. 8E-10
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: IPC_aziOffset                ! Phase offset added to the azimuth angle for the individual pitch controller, [rad].
    REAL(C_Double)                             :: IPC_CornerFreqAct            ! Corner frequency of the first-order actuators model, to induce a phase lag in the IPC signal {0: Disable}, [rad/s]
    
    INTEGER(4)                          :: PC_ControlMode               ! Blade pitch control mode {0: No pitch, fix to fine pitch, 1: active PI blade pitch control}
    INTEGER(4)                          :: PC_GS_n                      ! Amount of gain-scheduling table entries
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PC_GS_angles                 ! Gain-schedule table: pitch angles
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PC_GS_KP                     ! Gain-schedule table: pitch controller kp gains
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PC_GS_KI                     ! Gain-schedule table: pitch controller ki gains
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PC_GS_KD                     ! Gain-schedule table: pitch controller kd gains
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PC_GS_TF                     ! Gain-schedule table: pitch controller tf gains (derivative filter)
    REAL(C_Double)                             :: PC_MaxPit                    ! Maximum physical pitch limit, [rad].
    REAL(C_Double)                             :: PC_MinPit                    ! Minimum physical pitch limit, [rad].
    REAL(C_Double)                             :: PC_MaxRat                    ! Maximum pitch rate (in absolute value) in pitch controller, [rad/s].
    REAL(C_Double)                             :: PC_MinRat                    ! Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
    REAL(C_Double)                             :: PC_RefSpd                    ! Desired (reference) HSS speed for pitch controller, [rad/s].
    REAL(C_Double)                             :: PC_FinePit                   ! Record 5: Below-rated pitch angle set-point (deg) [used only with Bladed Interface]
    REAL(C_Double)                             :: PC_Switch                    ! Angle above lowest minimum pitch angle for switch [rad]
    
    INTEGER(4)                          :: VS_ControlMode               ! Generator torque control mode in above rated conditions {0: constant torque, 1: constant power, 2: TSR Tracking, 3: TSR Tracking w/ const power}
    REAL(C_Double)                             :: VS_GenEff                    ! Generator efficiency mechanical power -> electrical power [-]
    REAL(C_Double)                             :: VS_ArSatTq                   ! Above rated generator torque PI control saturation, [Nm] -- 212900
    REAL(C_Double)                             :: VS_MaxRat                    ! Maximum torque rate (in absolute value) in torque controller, [Nm/s].
    REAL(C_Double)                             :: VS_MaxTq                     ! Maximum generator torque in Region 3 (HSS side), [Nm]. -- chosen to be 10% above VS_RtTq
    REAL(C_Double)                             :: VS_MinTq                     ! Minimum generator (HSS side), [Nm].
    REAL(C_Double)                             :: VS_MinOMSpd                  ! Optimal mode minimum speed, [rad/s]
    REAL(C_Double)                             :: VS_Rgn2K                     ! Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2
    REAL(C_Double)                             :: VS_RtPwr                     ! Wind turbine rated power [W]
    REAL(C_Double)                             :: VS_RtTq                      ! Rated torque, [Nm].
    REAL(C_Double)                             :: VS_RefSpd                    ! Rated generator speed [rad/s]
    INTEGER(4)                          :: VS_n                         ! Number of controller gains
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: VS_KP                        ! Proportional gain for generator PI torque controller, used in the transitional 2.5 region
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: VS_KI                        ! Integral gain for generator PI torque controller, used in the transitional 2.5 region
    REAL(C_Double)                             :: VS_TSRopt                    ! Power-maximizing region 2 tip-speed ratio [rad]
    
    INTEGER(4)                          :: SS_Mode                      ! Setpoint Smoother mode {0: no setpoint smoothing, 1: introduce setpoint smoothing}
    REAL(C_Double)                             :: SS_VSGain                    ! Variable speed torque controller setpoint smoother gain, [-].
    REAL(C_Double)                             :: SS_PCGain                    ! Collective pitch controller setpoint smoother gain, [-].

    INTEGER(4)                          :: WE_Mode                      ! Wind speed estimator mode {0: One-second low pass filtered hub height wind speed, 1: Imersion and Invariance Estimator (Ortega et al.)
    REAL(C_Double)                             :: WE_BladeRadius               ! Blade length [m]
    INTEGER(4)                          :: WE_CP_n                      ! Amount of parameters in the Cp array
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: WE_CP                        ! Parameters that define the parameterized CP(\lambda) function
    REAL(C_Double)                             :: WE_Gamma                     ! Adaption gain of the wind speed estimator algorithm [m/rad]
    REAL(C_Double)                             :: WE_GearboxRatio              ! Gearbox ratio, >=1  [-]
    REAL(C_Double)                             :: WE_Jtot                      ! Total drivetrain inertia, including blades, hub and casted generator inertia to LSS [kg m^2]
    REAL(C_Double)                             :: WE_RhoAir                    ! Air density [kg m^-3]
    CHARACTER(1024)                     :: PerfFileName                 ! File containing rotor performance tables (Cp,Ct,Cq)
    INTEGER(4), DIMENSION(:), ALLOCATABLE  :: PerfTableSize             ! Size of rotor performance tables, first number refers to number of blade pitch angles, second number referse to number of tip-speed ratios
    INTEGER(4)                          :: WE_FOPoles_N                 ! Number of first-order system poles used in EKF
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: WE_FOPoles_v                 ! Wind speeds corresponding to first-order system poles [m/s]
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: WE_FOPoles                   ! First order system poles

    INTEGER(4)                          :: Y_ControlMode                ! Yaw control mode {0: no yaw control, 1: yaw rate control, 2: yaw-by-IPC}
    REAL(C_Double)                             :: Y_ErrThresh                  ! Error threshold [rad]. Turbine begins to yaw when it passes this. (104.71975512) -- 1.745329252
    REAL(C_Double)                             :: Y_IPC_IntSat                 ! Integrator saturation (maximum signal amplitude contrbution to pitch from yaw-by-IPC)
    INTEGER(4)                          :: Y_IPC_n                      ! Number of controller gains (yaw-by-IPC)
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: Y_IPC_KP                     ! Yaw-by-IPC proportional controller gain Kp
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: Y_IPC_KI                     ! Yaw-by-IPC integral controller gain Ki
    REAL(C_Double)                             :: Y_IPC_omegaLP                ! Low-pass filter corner frequency for the Yaw-by-IPC controller to filtering the yaw alignment error, [rad/s].
    REAL(C_Double)                             :: Y_IPC_zetaLP                 ! Low-pass filter damping factor for the Yaw-by-IPC controller to filtering the yaw alignment error, [-].
    REAL(C_Double)                             :: Y_MErrSet                    ! Yaw alignment error, setpoint [rad]
    REAL(C_Double)                             :: Y_omegaLPFast                ! Corner frequency fast low pass filter, 1.0 [Hz]
    REAL(C_Double)                             :: Y_omegaLPSlow                ! Corner frequency slow low pass filter, 1/60 [Hz]
    REAL(C_Double)                             :: Y_Rate                       ! Yaw rate [rad/s]
    
    INTEGER(4)                          :: PS_Mode                      ! Pitch saturation mode {0: no peak shaving, 1: implement pitch saturation}
    INTEGER(4)                          :: PS_BldPitchMin_N             ! Number of values in minimum blade pitch lookup table (should equal number of values in PS_WindSpeeds and PS_BldPitchMin)
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PS_WindSpeeds                ! Wind speeds corresponding to minimum blade pitch angles [m/s]
    REAL(C_Double), DIMENSION(:), ALLOCATABLE  :: PS_BldPitchMin               ! Minimum blade pitch angles [rad]

    INTEGER(4)                          :: SD_Mode                      ! Shutdown mode {0: no shutdown procedure, 1: pitch to max pitch at shutdown}
    REAL(C_Double)                             :: SD_MaxPit                    ! Maximum blade pitch angle to initiate shutdown, [rad]
    REAL(C_Double)                             :: SD_CornerFreq                ! Cutoff Frequency for first order low-pass filter for blade pitch angle, [rad/s]
    
    INTEGER(4)                          :: Fl_Mode                      ! Floating specific feedback mode {0: no nacelle velocity feedback, 1: nacelle velocity feedback}
    REAL(C_Double)                             :: Fl_Kp                        ! Nacelle velocity proportional feedback gain [s]

    INTEGER(4)                          :: Flp_Mode                     ! Flap actuator mode {0: off, 1: fixed flap position, 2: PI flap control}
    REAL(C_Double)                             :: Flp_Angle                    ! Fixed flap angle (degrees)
    REAL(C_Double)                             :: Flp_Kp                       ! PI flap control proportional gain 
    REAL(C_Double)                             :: Flp_Ki                       ! PI flap control integral gain 
    REAL(C_Double)                             :: Flp_MaxPit                   ! Maximum (and minimum) flap pitch angle [rad]
    
    REAL(C_Double)                             :: PC_RtTq99                    ! 99% of the rated torque value, using for switching between pitch and torque control, [Nm].
    REAL(C_Double)                             :: VS_MaxOMTq                   ! Maximum torque at the end of the below-rated region 2, [Nm]
    REAL(C_Double)                             :: VS_MinOMTq                   ! Minimum torque at the beginning of the below-rated region 2, [Nm]

END TYPE ControlParameters

TYPE, PUBLIC :: LocalVariables
    ! ---------- From avrSWAP ----------
    INTEGER(4)                      :: iStatus
    REAL(C_Double)                      :: Time
    REAL(C_Double)                      :: DT
    REAL(C_Double)                      :: VS_GenPwr
    REAL(C_Double)                      :: GenSpeed
    REAL(C_Double)                      :: RotSpeed
    REAL(C_Double)                      :: Y_M
    REAL(C_Double)                      :: HorWindV
    REAL(C_Double)                      :: rootMOOP(3)
    REAL(C_Double)                      :: BlPitch(3)
    REAL(C_Double)                      :: Azimuth
    INTEGER(4)                   :: NumBl
    REAL(C_Double)                      :: FA_Acc                       ! Tower fore-aft acceleration [m/s^2]
    REAL(C_Double)                      :: NacIMU_FA_Acc                       ! Tower fore-aft acceleration [rad/s^2]

    ! ---------- -Internal controller variables ----------
    REAL(C_Double)                             :: FA_AccHPF                    ! High-pass filtered fore-aft acceleration [m/s^2]
    REAL(C_Double)                             :: FA_AccHPFI                   ! Tower velocity, high-pass filtered and integrated fore-aft acceleration [m/s]
    REAL(C_Double)                             :: FA_PitCom(3)                 ! Tower fore-aft vibration damping pitch contribution [rad]
    REAL(C_Double)                             :: RotSpeedF                    ! Filtered LSS (generator) speed [rad/s].
    REAL(C_Double)                             :: GenSpeedF                    ! Filtered HSS (generator) speed [rad/s].
    REAL(C_Double)                             :: GenTq                        ! Electrical generator torque, [Nm].
    REAL(C_Double)                             :: GenTqMeas                    ! Measured generator torque [Nm]
    REAL(C_Double)                             :: GenArTq                      ! Electrical generator torque, for above-rated PI-control [Nm].
    REAL(C_Double)                             :: GenBrTq                      ! Electrical generator torque, for below-rated PI-control [Nm].
    REAL(C_Double)                             :: IPC_PitComF(3)               ! Commanded pitch of each blade as calculated by the individual pitch controller, F stands for low-pass filtered [rad].
    REAL(C_Double)                             :: PC_KP                        ! Proportional gain for pitch controller at rated pitch (zero) [s].
    REAL(C_Double)                             :: PC_KI                        ! Integral gain for pitch controller at rated pitch (zero) [-].
    REAL(C_Double)                             :: PC_KD                        ! Differential gain for pitch controller at rated pitch (zero) [-].
    REAL(C_Double)                             :: PC_TF                        ! First-order filter parameter for derivative action
    REAL(C_Double)                             :: PC_MaxPit                    ! Maximum pitch setting in pitch controller (variable) [rad].
    REAL(C_Double)                             :: PC_MinPit                    ! Minimum pitch setting in pitch controller (variable) [rad].
    REAL(C_Double)                             :: PC_PitComT                   ! Total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(C_Double)                             :: PC_PitComTF                   ! Filtered Total command pitch based on the sum of the proportional and integral terms [rad].
    REAL(C_Double)                             :: PC_PitComT_IPC(3)            ! Total command pitch based on the sum of the proportional and integral terms, including IPC term [rad].
    REAL(C_Double)                             :: PC_PwrErr                    ! Power error with respect to rated power [W]
    REAL(C_Double)                             :: PC_SineExcitation            ! Sine contribution to pitch signal
    REAL(C_Double)                             :: PC_SpdErr                    ! Current speed error (pitch control) [rad/s].
    INTEGER(4)                          :: PC_State                     ! State of the pitch control system
    REAL(C_Double)                             :: PitCom(3)                    ! Commanded pitch of each blade the last time the controller was called [rad].
    REAL(C_Double)                             :: SS_DelOmegaF                 ! Filtered setpoint shifting term defined in setpoint smoother [rad/s].
    REAL(C_Double)                             :: TestType                     ! Test variable, no use
    REAL(C_Double)                             :: VS_MaxTq                     ! Maximum allowable generator torque [Nm].
    REAL(C_Double)                             :: VS_LastGenTrq                ! Commanded electrical generator torque the last time the controller was called [Nm].
    REAL(C_Double)                             :: VS_LastGenPwr                ! Commanded electrical generator torque the last time the controller was called [Nm].
    REAL(C_Double)                             :: VS_MechGenPwr                ! Mechanical power on the generator axis [W]
    REAL(C_Double)                             :: VS_SpdErrAr                  ! Current speed error for region 2.5 PI controller (generator torque control) [rad/s].
    REAL(C_Double)                             :: VS_SpdErrBr                  ! Current speed error for region 1.5 PI controller (generator torque control) [rad/s].
    REAL(C_Double)                             :: VS_SpdErr                    ! Current speed error for tip-speed-ratio tracking controller (generator torque control) [rad/s].
    INTEGER(4)                          :: VS_State                     ! State of the torque control system
    REAL(C_Double)                             :: VS_Rgn3Pitch                 ! Pitch angle at which the state machine switches to region 3, [rad].
    REAL(C_Double)                             :: WE_Vw                        ! Estimated wind speed [m/s]
    REAL(C_Double)                             :: WE_Vw_F                      ! Filtered estimated wind speed [m/s]
    REAL(C_Double)                             :: WE_VwI                       ! Integrated wind speed quantity for estimation [m/s]
    REAL(C_Double)                             :: WE_VwIdot                    ! Differentiated integrated wind speed quantity for estimation [m/s]
    REAL(C_Double)                             :: VS_LastGenTrqF               ! Differentiated integrated wind speed quantity for estimation [m/s]
    REAL(C_Double)                             :: Y_AccErr                     ! Accumulated yaw error [rad].
    REAL(C_Double)                             :: Y_ErrLPFFast                 ! Filtered yaw error by fast low pass filter [rad].
    REAL(C_Double)                             :: Y_ErrLPFSlow                 ! Filtered yaw error by slow low pass filter [rad].
    REAL(C_Double)                             :: Y_MErr                       ! Measured yaw error, measured + setpoint [rad].
    REAL(C_Double)                             :: Y_YawEndT                    ! Yaw end time [s]. Indicates the time up until which yaw is active with a fixed rate
    LOGICAL(1)                          :: SD                           ! Shutdown, .FALSE. if inactive, .TRUE. if active
    REAL(C_Double)                             :: Fl_PitCom                           ! Shutdown, .FALSE. if inactive, .TRUE. if active
    REAL(C_Double)                             :: NACIMU_FA_AccF
    REAL(C_Double)                             :: FA_AccF
    REAL(C_Double)                             :: Flp_Angle(3)                 ! Flap Angle (rad)
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
    REAL(C_Double), DIMENSION(:), ALLOCATABLE      :: TSR_vec
    REAL(C_Double), DIMENSION(:), ALLOCATABLE      :: Beta_vec
    REAL(C_Double), DIMENSION(:,:), ALLOCATABLE    :: Cp_mat
    REAL(C_Double), DIMENSION(:,:), ALLOCATABLE    :: Ct_mat
    REAL(C_Double), DIMENSION(:,:), ALLOCATABLE    :: Cq_mat
END TYPE PerformanceData

TYPE, PUBLIC :: DebugVariables                                          
! Variables used for debug purposes
    REAL(C_Double)                             :: WE_Cp                        ! Cp that WSE uses to determine aerodynamic torque[-]
    REAL(C_Double)                             :: WE_b                         ! Pitch that WSE uses to determine aerodynamic torque[-]
    REAL(C_Double)                             :: WE_w                         ! Rotor Speed that WSE uses to determine aerodynamic torque[-]
    REAL(C_Double)                             :: WE_t                         ! Torque that WSE uses[-]
    REAL(C_Double)                             :: WE_Vm                        ! Mean wind speed component in WSE [m/s]
    REAL(C_Double)                             :: WE_Vt                        ! Turbulent wind speed component in WSE [m/s]
    REAL(C_Double)                             :: WE_lambda                    ! TSR in WSE [rad]
    !
    REAL(C_Double)                             :: PC_PICommand                 

END TYPE DebugVariables

TYPE, PUBLIC :: ErrorVariables
    ! Error Catching
    INTEGER(4)                      :: size_avcMSG
    INTEGER(C_INT)                  :: aviFAIL             ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
    ! CHARACTER(:), ALLOCATABLE  :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
    CHARACTER(:), ALLOCATABLE       :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
END TYPE ErrorVariables

END MODULE ROSCO_Types
