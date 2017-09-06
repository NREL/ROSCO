!=======================================================================
! SUBROUTINE DISCON (avrSWAP, from_SC, to_SC, aviFAIL, accINFILE, avcOUTNAME, avcMSG) BIND (C, NAME='DISCON')
SUBROUTINE DISCON (avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG) BIND (C, NAME='DISCON')
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON

   ! 06/09/2017

   ! This Bladed-style DLL controller is used to implement a variable-speed
   ! generator-torque controller, PI collective blade pitch controller, individual pitch
   ! controller and yaw controller for the NREL Offshore 5MW baseline wind turbine.
   ! This routine was extended by S.P. Mulders, J. Hoorneman and J. Govers of TU Delft.
   ! The routine is based on the routine as written by J. Jonkman of NREL/NWTC.

   ! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
   ! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

USE, INTRINSIC	:: ISO_C_Binding
USE				:: FunctionToolbox
USE				:: Filters

IMPLICIT NONE
#ifndef IMPLICIT_DLLEXPORT
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
#endif

!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------

   ! Passed Variables:
!REAL(C_FLOAT), INTENT(IN)		:: from_SC(*)			! DATA from the supercontroller
!REAL(C_FLOAT), INTENT(INOUT)	:: to_SC(*)				! DATA to the supercontroller

REAL(C_FLOAT), INTENT(INOUT)			:: avrSWAP(*)						! The swap array, used to pass data to, and receive data from, the DLL controller.
INTEGER(C_INT), INTENT(INOUT)			:: aviFAIL							! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
CHARACTER(KIND=C_CHAR), INTENT(IN)		:: accINFILE(NINT(avrSWAP(50)))		! The name of the parameter input file
CHARACTER(KIND=C_CHAR), INTENT(IN)		:: avcOUTNAME(NINT(avrSWAP(51)))	! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR), INTENT(INOUT)	:: avcMSG(NINT(avrSWAP(49)))		! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
   ! Local Variables:

REAL(4)						:: Azimuth											! Rotor azimuth angle [rad].
REAL(4)						:: BlPitch(3)										! Current values of the blade pitch angles [rad].
REAL(4), PARAMETER			:: CornerFreq = 0.7853981							! Corner frequency (-3dB point) in the first-order low-pass filter, [rad/s]
REAL(4)						:: DT												! Time step [s].
REAL(4)						:: ElapTime											! Elapsed time since the last call to the controller [s].
REAL(4)						:: GenSpeed											! Current  HSS (generator) speed [rad/s].
REAL(4)						:: GenSpeedF										! Filtered HSS (generator) speed [rad/s].
REAL(4)						:: GenTrq											! Electrical generator torque, [Nm].
REAL(4)						:: HorWindV											! Horizontal wind speed at hub-height, [m/s].
REAL(4), PARAMETER			:: IPC_KI = 8E-10									! Integral gain for the individual pitch controller, [-].
REAL(4), PARAMETER			:: IPC_omegaHP = 0.3141592							! High-pass filter cut-in frequency used to separate yaw-by-IPC contribution from blade load reduction contribution, [rad/s].
REAL(4), PARAMETER			:: IPC_omegaLP = 0.6283185							! Low-pass filter corner frequency for the individual pitch controller, [rad/s].
REAL(4), PARAMETER			:: IPC_omegaNotch = 1.269330365						! Notch filter corner frequency for the individual pitch controller, [rad/s].
REAL(4), PARAMETER			:: IPC_phi = 0.436332313							! Phase offset added to the azimuth angle for the individual pitch controller, [rad].
REAL(4)						:: IPC_PitComF(3)									! Commanded pitch of each blade as calculated by the individual pitch controller, F stands for low pass filtered, [rad].
REAL(4), PARAMETER			:: IPC_zetaHP = 0.70								! High-pass filter damping value, [-].
REAL(4), PARAMETER			:: IPC_zetaLP = 1.0									! Low-pass filter damping factor for the individual pitch controller, [-].
REAL(4), PARAMETER			:: IPC_zetaNotch = 0.5								! Notch filter damping factor for the individual pitch controller, [-].
REAL(4), SAVE				:: LastGenTrq										! Commanded electrical generator torque the last time the controller was called, [Nm].
REAL(4), SAVE				:: LastTime											! Last time this DLL was called, [s].
REAL(4), SAVE				:: LastTimePC										! Last time the pitch  controller was called, [s].
REAL(4), SAVE				:: LastTimeVS										! Last time the torque controller was called, [s].
REAL(4)						:: PC_GK											! Current value of the gain correction factor, used in the gain scheduling law of the pitch controller, [-].
INTEGER(4), SAVE							:: PC_GS_n							! Amount of gain-scheduling table entries
REAL(4), DIMENSION(:), ALLOCATABLE, SAVE	:: PC_GS_angles						! Gain-schedule table: pitch angles
REAL(4), DIMENSION(:), ALLOCATABLE, SAVE	:: PC_GS_kp							! Gain-schedule table: pitch controller kp gainspitch angles
REAL(4), DIMENSION(:), ALLOCATABLE, SAVE	:: PC_GS_ki							! Gain-schedule table: pitch controller ki gainspitch angles
REAL(4)						:: PC_KI											! Integral gain for pitch controller at rated pitch (zero), [-].
REAL(4)						:: PC_KP											! Proportional gain for pitch controller at rated pitch (zero), [s].
REAL(4)						:: PC_MaxPit										! Maximum physical pitch limit, [rad].
REAL(4)						:: PC_MaxPitVar										! Maximum pitch setting in pitch controller (variable) [rad].
REAL(4)						:: PC_MaxRat										! Maximum pitch rate (in absolute value) in pitch controller, [rad/s].
REAL(4)						:: PC_MinPit										! Minimum physical pitch limit, [rad].
REAL(4)						:: PC_MinRat										! Minimum pitch rate (in absolute value) in pitch controller, [rad/s].
REAL(4)						:: PC_RefSpd										! Desired (reference) HSS speed for pitch controller, [rad/s].
REAL(4)						:: PC_RtTq99										! 99% of the rated torque value, using for switching between pitch and torque control, [Nm].
REAL(4)						:: PC_SetPnt										! Fine pitch angle, [rad].
REAL(4)						:: PC_SpdErr										! Current speed error (pitch control) [rad/s].
REAL(4), SAVE				:: PitCom(3)										! Commanded pitch of each blade the last time the controller was called, [rad].
REAL(4), SAVE				:: PitComT											! Total command pitch based on the sum of the proportional and integral terms, [rad].
REAL(4), SAVE				:: PitComT_IPC(3)									! Total command pitch based on the sum of the proportional and integral terms, including IPC term [rad].
REAL(4), PARAMETER			:: R2D = 57.295780									! Factor to convert radians to degrees.
REAL(4)						:: rootMOOP(3)										! Blade root out of plane bending moments, [Nm].
REAL(4), PARAMETER			:: RPS2RPM = 9.5492966								! Factor to convert radians per second to revolutions per minute.
REAL(4)						:: Time												! Current simulation time, [s].
REAL(4), SAVE				:: VS_CtInSp										! Transitional generator speed (HSS side) between regions 1 and 1 1/2, [rad/s].
INTEGER(4), SAVE			:: VS_n												! Number of controller gains
REAL(4), DIMENSION(:), ALLOCATABLE, SAVE	:: VS_KP							! Proportional gain for generator PI torque controller, used in the transitional 2.5 region
REAL(4), DIMENSION(:), ALLOCATABLE, SAVE	:: VS_KI							! Integral gain for generator PI torque controller, used in the transitional 2.5 region
REAL(4)						:: VS_MaxOM											! Optimal mode maximum speed, [rad/s].
REAL(4), SAVE				:: VS_MaxRat										! Maximum torque rate (in absolute value) in torque controller, [Nm/s].
REAL(4)						:: VS_MaxTq											! Maximum generator torque in Region 3 (HSS side), [Nm]. -- chosen to be 10% above VS_RtTq
REAL(4)						:: VS_MinOM											! Optimal mode minimum speed, [rad/s].
REAL(4)						:: VS_Rgn2K											! Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2.
REAL(4), SAVE				:: VS_Rgn2MaxTq										! Maximum torque at the end of the below-rated region 2, [Nm]
REAL(4), SAVE				:: VS_Rgn3MP										! Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, [rad]. -- chosen to be 1.0 degree above PC_SetPnt
REAL(4)						:: VS_RtTq											! Rated torque, [Nm].
REAL(4)						:: VS_RtSpd											! Rated generator speed [rad/s]
REAL(4), SAVE				:: VS_Slope15										! Torque/speed slope of region 1 1/2 cut-in torque ramp , [Nm/(rad/s)].
REAL(4)						:: VS_SpdErr										! Current speed error (generator torque control) [rad/s].
REAL(4), SAVE				:: Y_AccErr											! Accumulated yaw error [rad].
INTEGER(4), SAVE			:: Y_ControlMode									! Yaw control mode: (0 = no yaw control, 1 = yaw rate control, 2 = yaw-by-IPC)
REAL(4)						:: Y_ErrLPFFast										! Filtered yaw error by fast low pass filter [rad].
REAL(4)						:: Y_ErrLPFSlow										! Filtered yaw error by slow low pass filter [rad].
REAL(4), PARAMETER			:: Y_ErrThresh = 1.745329252						! Error threshold [rad]. Turbine begins to yaw when it passes this. (104.71975512).
REAL(4), SAVE				:: Y_YawRate										! Yaw rate [rad/s].
REAL(4)						:: Y_MErr											! Measured yaw error [rad].
REAL(4), PARAMETER			:: Y_omegaLPFast = 1.0								! Corner frequency fast low pass filter, [Hz].
REAL(4), PARAMETER			:: Y_omegaLPSlow = 0.016666667						! Corner frequency slow low pass filter, 1/60 [Hz].
REAL(4), SAVE				:: Y_YawEndT										! Yaw end time, [s]. Indicates the time up until which yaw is active with a fixed rate.

! REAL(4)						:: testValue										! TestValue

INTEGER(4)					:: I												! Generic index.
INTEGER(4)					:: iStatus											! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
INTEGER(4)					:: K												! Loops through blades.
INTEGER(4)					:: NumBl											! Number of blades, [-].
INTEGER(4), PARAMETER		:: UnDb = 85										! I/O unit for the debugging information
INTEGER(4), PARAMETER		:: UnDb2 = 86										! I/O unit for the debugging information
INTEGER(4), PARAMETER		:: Un = 87											! I/O unit for pack/unpack (checkpoint & restart)
INTEGER(4), PARAMETER		:: UnUser = 88										! I/O unit for user defined parameter file
INTEGER(4), PARAMETER		:: UnPitchGains = 89								! I/O unit for user defined pitch gains parameter file

LOGICAL(1), PARAMETER		:: DbgOut = .TRUE.									! Flag to indicate whether to output debugging information

CHARACTER(1), PARAMETER		:: Tab			= CHAR(9)							! The tab character.
CHARACTER(25), PARAMETER	:: FmtDat = "(F8.3,99('"//Tab//"',ES10.3E2,:))	"	! The format of the debugging data
CHARACTER(*), PARAMETER		:: UserFile = 'DISCON.IN'							! Name of the user defined parameter file

CHARACTER(SIZE(accINFILE)-1)	:: InFile										! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcOUTNAME)-1)	:: RootName										! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)		:: ErrMsg										! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]

   ! Load variables from calling program (See Appendix A of Bladed User's Guide):
iStatus			= NINT(avrSWAP(1))
Time			= avrSWAP(2)
DT				= avrSWAP(3)
BlPitch(1)		= avrSWAP(4)
PC_SetPnt		= avrSWAP(5)
PC_MinPit		= avrSWAP(6)
PC_MaxPit		= avrSWAP(7)
PC_MinRat		= avrSWAP(8)
PC_MaxRat		= avrSWAP(9)
VS_Rgn2K		= avrSWAP(16)
VS_MinOM		= avrSWAP(17)
VS_MaxOM		= avrSWAP(18)
VS_RtSpd		= avrSWAP(19)
PC_RefSpd		= avrSWAP(19)
GenSpeed		= avrSWAP(20)
VS_RtTq			= avrSWAP(22)
Y_MErr			= avrSWAP(24)
HorWindV		= avrSWAP(27)
rootMOOP(1)		= avrSWAP(30)
rootMOOP(2)		= avrSWAP(31)
rootMOOP(3)		= avrSWAP(32)
BlPitch(2)		= avrSWAP(33)
BlPitch(3)		= avrSWAP(34)
Azimuth			= avrSWAP(60)
NumBl			= NINT(avrSWAP(61))

PC_RtTq99		= VS_RtTq*0.99
VS_MaxTq		= VS_RtTq*1.1
VS_Rgn2MaxTq	= VS_Rgn2K*VS_MaxOM**2
VS_Rgn3MP		= PC_SetPnt + 1.0/R2D

   ! Convert C character arrays to Fortran strings:

RootName = TRANSFER(avcOUTNAME(1:LEN(RootName)), RootName)
I = INDEX(RootName,C_NULL_CHAR) - 1			! if this has a c null character at the end...
IF (I > 0) RootName = RootName(1:I)			! remove it

InFile = TRANSFER(accINFILE(1:LEN(InFile)),  InFile)
I = INDEX(InFile,C_NULL_CHAR) - 1			! if this has a c null character at the end...
IF (I > 0) InFile = InFile(1:I)				! remove it

   ! Initialize aviFAIL to 0:

aviFAIL      = 0

   ! Read any External Controller Parameters specified in the User Interface
   !   and initialize variables:

IF (iStatus == 0)  THEN  ! .TRUE. if we're on the first call to the DLL

		! Inform users that we are using this user-defined routine:

	aviFAIL = 1
	ErrMsg = 'Running the Delft Research Controller (DRC) for the NREL'		// &
			'5MW baseline wind turbine from DISCON.dll.'					// &
			'Delft Univeristy of Technology'								// &
			'Based on baseline NREL5MW controller by J. Jonkman'

		! Read user defined parameter file

	OPEN(UnUser, file=UserFile)
	DO I = 120, 129
		READ(UnUser, *) avrSWAP(I)
	END DO
	CLOSE(UnUser)
	
	Y_ControlMode	= NINT(avrSWAP(120))
	Y_YawRate		= avrSWAP(121)
	VS_CtInSp		= avrSWAP(122)
	VS_MaxRat		= avrSWAP(123)
	
		! Determine some torque control parameters not specified directly:

	VS_Slope15 = (VS_Rgn2K*VS_MinOM*VS_MinOM)/(VS_MinOM - VS_CtInSp)

	! Initialize the SAVEd variables:
		! NOTE: LastGenTrq, though SAVEd, is initialized in the torque controller
		! below for simplicity, not here.
	
	PitCom		= BlPitch							! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
	Y_AccErr	= 0.0								! This will ensure that the accumulated yaw error starts at zero
	Y_YawEndT	= -1.0								! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing

	LastTime	= Time								! This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
	LastTimePC	= Time - DT							! This will ensure that the pitch  controller is called on the first pass
	LastTimeVS	= Time - DT							! This will ensure that the torque controller is called on the first pass

	!..............................................................................................................................
	! Read gain-scheduled PI pitch controller gains and torque controller gains from file
	!..............................................................................................................................
	OPEN(unit=UnPitchGains, file='PitchGains.IN', status='old', action='read')
	READ(UnPitchGains, *) PC_GS_n
	
	ALLOCATE(PC_GS_angles(PC_GS_n))
	READ(UnPitchGains,*) PC_GS_angles
	
	ALLOCATE(PC_GS_kp(PC_GS_n))
	READ(UnPitchGains,*) PC_GS_kp
	
	ALLOCATE(PC_GS_ki(PC_GS_n))
	READ(UnPitchGains,*) PC_GS_ki
	
	READ(UnPitchGains, *) VS_n
	
	ALLOCATE(VS_KP(VS_n))
	READ(UnPitchGains,*) VS_KP
	
	ALLOCATE(VS_KI(VS_n))
	READ(UnPitchGains,*) VS_KI

	!..............................................................................................................................
	! Check validity of input parameters:
	!..............................................................................................................................


	IF (CornerFreq <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'CornerFreq must be greater than zero.'
	ENDIF

	IF (DT <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'DT must be greater than zero.'
	ENDIF

	IF (VS_CtInSp < 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_CtInSp must not be negative.'
	ENDIF

	IF (VS_MinOM <= VS_CtInSp) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_MinOM must be greater than VS_CtInSp.'
	ENDIF

	IF (VS_MaxRat <= 0.0) THEN
		aviFAIL =  -1
		ErrMsg  = 'VS_MaxRat must be greater than zero.'
	ENDIF

    IF (VS_RtTq < 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_RtTw must not be negative.'
	ENDIF

	IF (VS_Rgn2K < 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_Rgn2K must not be negative.'
	ENDIF

	IF (VS_Rgn2K*VS_RtSpd*VS_RtSpd > VS_RtTq) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_Rgn2K*VS_RtSpd^2 must not be greater than VS_RtTq.'
	ENDIF

	IF (VS_MaxTq < VS_RtTq) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.'
	ENDIF

	IF (VS_KP(1) > 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_KP must be greater than zero.'
	ENDIF

	IF (VS_KI(1) > 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_KI must be greater than zero.'
	ENDIF

	IF (PC_RefSpd <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'PC_RefSpd must be greater than zero.'
	ENDIF

	IF (PC_MaxRat <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'PC_MaxRat must be greater than zero.'
	ENDIF

	IF (PC_MinPit >= PC_MaxPit)  THEN
		aviFAIL = -1
		ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
	ENDIF

	IF (IPC_KI <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_KI must be greater than zero.'
	ENDIF

	IF (IPC_omegaLP <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_omegaLP must be greater than zero.'
	ENDIF

	IF (IPC_omegaNotch <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_omegaNotch must be greater than zero.'
	ENDIF

	IF (IPC_phi <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_phi must be greater than zero.'
	ENDIF

	IF (IPC_zetaLP <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_zetaLP must be greater than zero.'
	ENDIF

	IF (IPC_zetaNotch <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_zetaNotch must be greater than zero.'
	ENDIF

	IF (Y_ErrThresh <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_ErrThresh must be greater than zero.'
	ENDIF

	IF (Y_YawRate <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_YawRate must be greater than zero.'
	ENDIF

	IF (Y_omegaLPFast <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_omegaLPFast must be greater than zero.'
	ENDIF

	IF (Y_omegaLPSlow <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_omegaLPSlow must be greater than zero.'
	ENDIF

	!..............................................................................................................................
	! Initializing debug file
	!..............................................................................................................................

		! If we're debugging, open the debug file and write the header:

	IF (DbgOut) THEN
		OPEN (UnDb, FILE=TRIM(RootName)//'.dbg', STATUS='REPLACE')
		WRITE (UnDb,'(A)')	'   Time '  //Tab//'PitComT  ' //Tab//'PC_KP ' //Tab//'PC_KI  ' //Tab//'Y_MErr  ' //Tab//'rootMOOP(1)  '
		WRITE (UnDb,'(A)')	'   (sec) ' //Tab//'(rad)    '  //Tab//'(-) ' //Tab//'(-)   ' //Tab//'(rad)   ' //Tab//'(?)   '
		
		OPEN(UnDb2, FILE=TRIM(RootName)//'.dbg2', STATUS='REPLACE')
		WRITE(UnDb2,'(/////)')
		WRITE(UnDb2,'(A,85("'//Tab//'AvrSWAP(",I2,")"))')  'Time ', (i,i=1,85)
		WRITE(UnDb2,'(A,85("'//Tab//'(-)"))')  '(s)'
	ENDIF

ENDIF

!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------


IF ((iStatus >= 0) .AND. (aviFAIL >= 0))  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step


		! Abort if the user has not requested a pitch angle actuator (See Appendix A
		!   of Bladed User's Guide):

	IF (NINT(avrSWAP(10)) /= 0)  THEN ! .TRUE. if a pitch angle actuator hasn't been requested
		aviFAIL = -1
		ErrMsg  = 'Pitch angle actuator not requested.'
	ENDIF

		! Set unused outputs to zero (See Appendix A of Bladed User's Guide):

	avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
	avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
	avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
	avrSWAP(65) = 0.0 ! Number of variables returned for logging
	avrSWAP(72) = 0.0 ! Generator start-up resistance
	avrSWAP(79) = 0.0 ! Request for loads: 0=none
	avrSWAP(80) = 0.0 ! Variable slip current status
	avrSWAP(81) = 0.0 ! Variable slip current demand

		! Filter the HSS (generator) speed measurement:
		! Apply Low-Pass Filter
	GenSpeedF = LPFilter(GenSpeed, DT, CornerFreq, iStatus, .FALSE., 1)     ! This is the first instance of LPFilter

	!..............................................................................................................................
	! VARIABLE-SPEED TORQUE CONTROL:
	!..............................................................................................................................

		! Compute the elapsed time since the last call to the controller:

	ElapTime = Time - LastTimeVS
	
		! Compute the generator torque, which depends on which region we are in:
		
	VS_SpdErr = VS_RtSpd - GenSpeedF											! Current speed error
	IF (PitComT >= VS_Rgn3MP) THEN												! We are in region 3 - power is constant
		GenTrq = VS_RtTq
	ELSE
		GenTrq = PIController(VS_SpdErr, VS_KP(1), VS_KI(1), VS_Rgn2MaxTq, VS_RtTq, DT, VS_Rgn2MaxTq, 1)
		IF (GenTrq >= VS_Rgn2MaxTq*1.01) THEN
			CONTINUE
		ELSEIF (GenSpeedF <= VS_CtInSp)  THEN										! We are in region 1 - torque is zero
			GenTrq = 0.0
		ELSEIF (GenSpeedF < VS_MinOM)  THEN										! We are in region 1 1/2 - linear ramp in torque from zero to optimal
			GenTrq = VS_Slope15*(GenSpeedF - VS_CtInSp)
		ELSEIF (GenSpeedF < VS_MaxOM)  THEN										! We are in region 2 - optimal torque is proportional to the square of the generator speed
			GenTrq = VS_Rgn2K*GenSpeedF*GenSpeedF
		ELSE																		! We are in region 2 1/2 - simple induction generator transition region
			GenTrq = VS_Rgn2MaxTq
		END IF
	END IF

		! Saturate the commanded torque using the maximum torque limit:

	GenTrq = MIN(GenTrq, VS_MaxTq)						! Saturate the command using the maximum torque limit

		! Saturate the commanded torque using the torque rate limit:

	IF (iStatus == 0)  LastGenTrq = GenTrq				! Initialize the value of LastGenTrq on the first pass only
	GenTrq = ratelimit(GenTrq, LastGenTrq, -VS_MaxRat, VS_MaxRat, DT)	! Saturate the command using the torque rate limit

		! Reset the values of LastTimeVS and LastGenTrq to the current values:

	LastTimeVS = Time
	LastGenTrq = GenTrq

		! Set the generator contactor status, avrSWAP(35), to main (high speed)
		! variable-speed generator, the torque override to yes, and command the
		! generator torque (See Appendix A of Bladed User's Guide):

	avrSWAP(35) = 1.0          ! Generator contactor status: 1=main (high speed) variable-speed generator
	avrSWAP(56) = 0.0          ! Torque override: 0=yes
	avrSWAP(47) = LastGenTrq   ! Demanded generator torque

	!..............................................................................................................................
	! Pitch control
	!..............................................................................................................................

	IF (GenTrq > PC_RtTq99) THEN
		PC_MaxPitVar = PC_MaxPit
	ELSE
		PC_MaxPitVar = PC_SetPnt
	END IF
		! Compute the elapsed time since the last call to the controller:

	ElapTime = Time - LastTimePC

		! Compute the gain scheduling correction factor based on the previously
		! commanded pitch angle for blade 1:

	PC_KP = interp1d(PC_GS_angles, PC_GS_kp, PitComT)
	PC_KI = interp1d(PC_GS_angles, PC_GS_ki, PitComT)

		! Compute the current speed error and its integral w.r.t. time; saturate the
		! integral term using the pitch angle limits:

	PC_SpdErr = PC_RefSpd - GenSpeedF									! Current speed error

		! Compute the pitch commands associated with the proportional and integral
		!   gains:

	PitComT = PIController(PC_SpdErr, PC_KP, PC_KI, PC_SetPnt, PC_MaxPitVar, ElapTime, PC_SetPnt, 2)

		! Individual pitch control

	CALL IPC(rootMOOP, Azimuth, IPC_phi, Y_MErr, DT, IPC_KI, IPC_omegaHP, IPC_omegaLP, IPC_omegaNotch, IPC_zetaHP, IPC_zetaLP, IPC_zetaNotch, iStatus, Y_ControlMode, NumBl, IPC_PitComF)

		! Combine and saturate all pitch commands:

	DO K = 1,NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
		PitComT_IPC(K) = PitComT + IPC_PitComF(K)							! Add the individual pitch command
		PitComT_IPC(K) = saturate(PitComT_IPC(K), PC_MinPit, PC_MaxPit)			! Saturate the overall command using the pitch angle limits
		
		PitCom(K) = ratelimit(PitComT_IPC(K), BlPitch(K), PC_MinRat, PC_MaxRat, DT)	! Saturate the overall command of blade K using the pitch rate limit
		PitCom(K) = saturate(PitComT_IPC(K), PC_MinPit, PC_MaxPit)						! Saturate the overall command using the pitch angle limits
	ENDDO

		! Set the pitch override to yes and command the pitch demanded from the last
		! call to the controller (See Appendix A of Bladed User's Guide):

	avrSWAP(55) = 0.0			! Pitch override: 0=yes

	avrSWAP(42) = PitCom(1)		! Use the command angles of all blades if using individual pitch
	avrSWAP(43) = PitCom(2)		! "
	avrSWAP(44) = PitCom(3)		! "

	avrSWAP(45) = PitCom(1)		! Use the command angle of blade 1 if using collective pitch

		! Reset the value of LastTimePC to the current value:

	LastTimePC = Time

	!..............................................................................................................................
	! Yaw control
	!..............................................................................................................................
	
	IF (Y_ControlMode == 1) THEN
		avrSWAP(29) = 0									! Yaw control parameter: 0 = yaw rate control
		IF (Time >= Y_YawEndT) THEN											! Check if the turbine is currently yawing
			avrSWAP(48) = 0.0													! Set yaw rate to zero

			Y_ErrLPFFast = LPFilter(Y_MErr, DT, Y_omegaLPFast, iStatus, .FALSE., 2)		! Fast low pass filtered yaw error with a frequency of 1
			Y_ErrLPFSlow = LPFilter(Y_MErr, DT, Y_omegaLPSlow, iStatus, .FALSE., 3)		! Slow low pass filtered yaw error with a frequency of 1/60

			Y_AccErr = Y_AccErr + ElapTime*SIGN(Y_ErrLPFFast**2, Y_ErrLPFFast)	! Integral of the fast low pass filtered yaw error

			IF (ABS(Y_AccErr) >= Y_ErrThresh) THEN								! Check if accumulated error surpasses the threshold
				Y_YawEndT = ABS(Y_ErrLPFSlow/Y_YawRate) + Time					! Yaw to compensate for the slow low pass filtered error
			END IF
		ELSE
			avrSWAP(48) = SIGN(Y_YawRate, Y_MErr)		! Set yaw rate to predefined yaw rate, the sign of the error is copied to the rate
			Y_ErrLPFFast = LPFilter(Y_MErr, DT, Y_omegaLPFast, iStatus, .TRUE., 2)		! Fast low pass filtered yaw error with a frequency of 1
			Y_ErrLPFSlow = LPFilter(Y_MErr, DT, Y_omegaLPSlow, iStatus, .TRUE., 3)		! Slow low pass filtered yaw error with a frequency of 1/60
			Y_AccErr = 0.0								! "
		END IF
	END IF
	!..............................................................................................................................
		! Output debugging information if requested:

	IF (DbgOut)  THEN
		WRITE (UnDb,FmtDat)		Time,	PitComT,	PC_KP,	PC_KI,	Y_MErr,	rootMOOP(1)
		WRITE (UnDb2,FmtDat)	Time, avrSWAP(1:85)
	END IF

		! Reset the value of LastTime to the current value:
	LastTime = Time
	
ENDIF

avcMSG = TRANSFER(TRIM(ErrMsg)//C_NULL_CHAR, avcMSG, SIZE(avcMSG))

RETURN

END SUBROUTINE DISCON
