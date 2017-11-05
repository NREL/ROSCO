MODULE DRC_Types

IMPLICIT NONE

TYPE, PUBLIC :: ControlParameters
	REAL(4)								:: CornerFreq					! Corner frequency (-3dB point) in the first-order low-pass filter, [rad/s]
	INTEGER(4)							:: LoggingLevel					! 0 = write no debug files, 1 = write standard output .dbg-file, 2 = write standard output .dbg-file and complete avrSWAP-array .dbg2-file
	REAL(4)								:: IPC_KI						! Integral gain for the individual pitch controller, [-]. 8E-10
	INTEGER(4)							:: IPC_ControlMode				! Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) on = 1/off = 0
	REAL(4)								:: IPC_omegaHP					! High-pass filter cut-in frequency used to separate yaw-by-IPC contribution from blade load reduction contribution, [rad/s].
	REAL(4)								:: IPC_omegaLP					! Low-pass filter corner frequency for the individual pitch controller, [rad/s].	
	REAL(4)								:: IPC_omegaNotch				! Notch filter corner frequency for the individual pitch controller, [rad/s].
	REAL(4)								:: IPC_phi						! Phase offset added to the azimuth angle for the individual pitch controller, [rad].
	REAL(4)								:: IPC_zetaHP					! High-pass filter damping value, [-].
	REAL(4)								:: IPC_zetaLP					! Low-pass filter damping factor for the individual pitch controller, [-].	
	REAL(4)								:: IPC_zetaNotch				! Notch filter damping factor for the individual pitch controller, [-].
	INTEGER(4)							:: PC_GS_n						! Amount of gain-scheduling table entries
	REAL(4), DIMENSION(:), ALLOCATABLE	:: PC_GS_angles					! Gain-schedule table: pitch angles
	REAL(4), DIMENSION(:), ALLOCATABLE	:: PC_GS_kp						! Gain-schedule table: pitch controller kp gains
	REAL(4), DIMENSION(:), ALLOCATABLE	:: PC_GS_ki						! Gain-schedule table: pitch controller ki gains
	REAL(4), DIMENSION(:), ALLOCATABLE	:: PC_GS_kd						! Gain-schedule table: pitch controller kd gains
	REAL(4), DIMENSION(:), ALLOCATABLE	:: PC_GS_tf						! Gain-schedule table: pitch controller tf gains (derivative filter)
	REAL(4)								:: PC_MaxPit					! Maximum physical pitch limit, [rad].
	REAL(4)								:: PC_MinPit					! Minimum physical pitch limit, [rad].
	REAL(4)								:: PC_MaxRat					! Maximum pitch rate (in absolute value) in pitch controller, [rad/s].
	REAL(4)								:: PC_MinRat					! Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
	REAL(4)								:: PC_RefSpd					! Desired (reference) HSS speed for pitch controller, [rad/s].
	REAL(4)								:: PC_SetPnt					! Record 5: Below-rated pitch angle set-point (deg) [used only with Bladed Interface]
	REAL(4)								:: PC_Switch					! Angle above lowest minimum pitch angle for switch [rad]
	INTEGER(4)							:: VS_ControlMode				! Generator torque control mode in above rated conditions, 0 = constant torque / 1 = constant power
	REAL(4)								:: VS_CtInSp					! Transitional generator speed (HSS side) between regions 1 and 1 1/2, [rad/s].
	REAL(4)								:: VS_GenTrqArSatMax			! Above rated generator torque PI control saturation, [Nm] -- 212900
	REAL(4)								:: VS_MaxOM						! Optimal mode maximum speed, [rad/s].
	REAL(4)								:: VS_MaxRat					! Maximum torque rate (in absolute value) in torque controller, [Nm/s].
	REAL(4)								:: VS_MaxTq						! Maximum generator torque in Region 3 (HSS side), [Nm]. -- chosen to be 10% above VS_RtTq
	REAL(4)								:: VS_MinTq						! Minimum generator (HSS side), [Nm].
	REAL(4)								:: VS_MinOM						! Optimal mode minimum speed, [rad/s]
	REAL(4)								:: VS_Rgn2K						! Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2
	REAL(4)								:: VS_RtPwr						! Wind turbine rated power [W]
	REAL(4)								:: VS_RtTq						! Rated torque, [Nm].
	REAL(4)								:: VS_RtSpd						! Rated generator speed [rad/s]
	INTEGER(4)							:: VS_n							! Number of controller gains
	REAL(4), DIMENSION(:), ALLOCATABLE	:: VS_KP						! Proportional gain for generator PI torque controller, used in the transitional 2.5 region
	REAL(4), DIMENSION(:), ALLOCATABLE	:: VS_KI						! Integral gain for generator PI torque controller, used in the transitional 2.5 region
	INTEGER(4)							:: Y_ControlMode				! Yaw control mode: (0 = no yaw control, 1 = yaw rate control, 2 = yaw-by-IPC)
	REAL(4)								:: Y_ErrThresh					! Error threshold [rad]. Turbine begins to yaw when it passes this. (104.71975512) -- 1.745329252
	INTEGER(4)							:: Y_IPC_n						! Number of controller gains (yaw-by-IPC)
	REAL(4), DIMENSION(:), ALLOCATABLE	:: Y_IPC_KP						! Yaw-by-IPC proportional controller gain Kp
	REAL(4), DIMENSION(:), ALLOCATABLE	:: Y_IPC_KI						! Yaw-by-IPC integral controller gain Ki
	REAL(4)								:: Y_MErrSet					! Yaw alignment error, setpoint [rad]
	REAL(4)								:: Y_omegaLPFast				! Corner frequency fast low pass filter, 1.0 [Hz]
	REAL(4)								:: Y_omegaLPSlow				! Corner frequency slow low pass filter, 1/60 [Hz]
	REAL(4)								:: Y_Rate						! Yaw rate [rad/s]
	
	REAL(4)								:: PC_RtTq99					! 99% of the rated torque value, using for switching between pitch and torque control, [Nm].
	REAL(4)								:: VS_Rgn2MaxTq					! Maximum torque at the end of the below-rated region 2, [Nm]
	REAL(4)								:: VS_Rgn2MinTq					! Minimum torque at the beginning of the below-rated region 2, [Nm]
	REAL(4)								:: VS_Rgn3MP					! Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, [rad].
END TYPE ControlParameters

TYPE, PUBLIC :: LocalVariables
	! From avrSWAP
	INTEGER(4)							:: iStatus
	REAL(4)								:: Time
	REAL(4)								:: DT
	REAL(4)								:: VS_GenPwr
	REAL(4)								:: GenSpeed
	REAL(4)								:: Y_M
	REAL(4)								:: HorWindV
	REAL(4)								:: rootMOOP(3)
	REAL(4)								:: BlPitch(3)
	REAL(4)								:: Azimuth
	INTEGER(4)							:: NumBl
	
	! Internal controller variables
	REAL(4)								:: GenSpeedF					! Filtered HSS (generator) speed [rad/s].
	REAL(4)								:: GenTrq						! Electrical generator torque, [Nm].
	REAL(4)								:: GenTrqAr						! Electrical generator torque, for above-rated PI-control [Nm].
	REAL(4)								:: GenTrqBr						! Electrical generator torque, for below-rated PI-control [Nm].
	INTEGER(4)							:: GlobalState					! Current global state to determine the behavior of the different controllers [-].
	REAL(4)								:: IPC_PitComF(3)				! Commanded pitch of each blade as calculated by the individual pitch controller, F stands for low pass filtered, [rad].
	REAL(4)								:: PC_KP						! Proportional gain for pitch controller at rated pitch (zero), [s].
	REAL(4)								:: PC_KI						! Integral gain for pitch controller at rated pitch (zero), [-].
	REAL(4)								:: PC_KD						! Differential gain for pitch controller at rated pitch (zero), [-].
	REAL(4)								:: PC_TF						! First-order filter parameter for derivative action
	REAL(4)								:: PC_MaxPitVar					! Maximum pitch setting in pitch controller (variable) [rad].
	REAL(4)								:: PC_PitComT					! Total command pitch based on the sum of the proportional and integral terms, [rad].
	REAL(4)								:: PC_PitComT_IPC(3)			! Total command pitch based on the sum of the proportional and integral terms, including IPC term [rad].
	REAL(4)								:: PC_PwrErr					! Power error with respect to rated power [W]
	REAL(4)								:: PC_SpdErr					! Current speed error (pitch control) [rad/s].
	REAL(4)								:: PitCom(3)					! Commanded pitch of each blade the last time the controller was called, [rad].
	INTEGER(4)							:: TestType						! Test variable, no use
	REAL(4)								:: VS_LastGenTrq				! Commanded electrical generator torque the last time the controller was called, [Nm].
	REAL(4)								:: VS_SpdErrAr					! Current speed error (generator torque control) [rad/s].
	REAL(4)								:: VS_SpdErrBr					! Current speed error (generator torque control) [rad/s].
	REAL(4)								:: Y_AccErr						! Accumulated yaw error [rad].
	REAL(4)								:: Y_ErrLPFFast					! Filtered yaw error by fast low pass filter [rad].
	REAL(4)								:: Y_ErrLPFSlow					! Filtered yaw error by slow low pass filter [rad].
	REAL(4)								:: Y_MErr						! Measured yaw error, measured + setpoint [rad].
	REAL(4)								:: Y_YawEndT					! Yaw end time, [s]. Indicates the time up until which yaw is active with a fixed rate
END TYPE LocalVariables

TYPE, PUBLIC :: ObjectInstances
	INTEGER(4)							:: instLPF
	INTEGER(4)							:: instSecLPF
	INTEGER(4)							:: instHPF
	INTEGER(4)							:: instNotchSlopes
	INTEGER(4)							:: instNotch
END TYPE ObjectInstances
END MODULE DRC_Types