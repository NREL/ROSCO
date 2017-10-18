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
USE				:: ReadParameters
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

REAL(4), SAVE				:: LastGenTrq										! Commanded electrical generator torque the last time the controller was called, [Nm].
REAL(4), SAVE				:: PitComT											! Total command pitch based on the sum of the proportional and integral terms, [rad].
REAL(4), SAVE				:: PitComT_IPC(3)									! Total command pitch based on the sum of the proportional and integral terms, including IPC term [rad].
REAL(4), SAVE				:: Y_AccErr											! Accumulated yaw error [rad].
REAL(4), SAVE				:: Y_YawEndT										! Yaw end time, [s]. Indicates the time up until which yaw is active with a fixed rate
REAL(4), SAVE				:: testValue										! TestValue

INTEGER(4)					:: I												! Generic index.
INTEGER(4)					:: K												! Loops through blades.
INTEGER(4), PARAMETER		:: UnDb = 85										! I/O unit for the debugging information
INTEGER(4), PARAMETER		:: UnDb2 = 86										! I/O unit for the debugging information

LOGICAL(1), PARAMETER		:: DbgOut = .FALSE.									! Flag to indicate whether to output debugging information

CHARACTER(1), PARAMETER		:: Tab			= CHAR(9)							! The tab character.
CHARACTER(25), PARAMETER	:: FmtDat = "(F8.3,99('"//Tab//"',ES10.3E2,:))	"	! The format of the debugging data

CHARACTER(SIZE(avcOUTNAME)-1)	:: RootName										! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)		:: ErrMsg										! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]



   ! Read avrSWAP array into derived types/variables
CALL ReadAvrSWAP(avrSWAP)

   ! Initialize aviFAIL to 0:
aviFAIL = 0

   ! Read any External Controller Parameters specified in the User Interface
   !   and initialize variables:
IF (LocalVar%iStatus == 0)  THEN  ! .TRUE. if we're on the first call to the DLL
	
		! Inform users that we are using this user-defined routine:
	aviFAIL = 1
	ErrMsg = '                                                          '//NEW_LINE('A')// &
			 'Running the Delft Research Controller (DRC)               '//NEW_LINE('A')// &
			 'A wind turbine controller for use in the scientific field '//NEW_LINE('A')// &
			 'Written by S.P. Mulders, Jan-Willem van Wingerden         '//NEW_LINE('A')// &
			 'Delft University of Technology, The Netherlands           '//NEW_LINE('A')// &
			 'Visit our GitHub-page to contribute to this project:      '//NEW_LINE('A')// &
			 'https://github.com/TUDelft-DataDrivenControl              '
	
	CALL ReadControlParameterFileSub()

		! Initialize testValue (debugging variable)
	! testValue = 0.4

	! Initialize the SAVEd variables:
		! NOTE: LastGenTrq, though SAVEd, is initialized in the torque controller
		! below for simplicity, not here.
	LocalVar%PitCom		= LocalVar%BlPitch							! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
	Y_AccErr	= 0.0								! This will ensure that the accumulated yaw error starts at zero
	Y_YawEndT	= -1.0								! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing

	!..............................................................................................................................
	! Check validity of input parameters:
	!..............................................................................................................................

	IF (CntrPar%CornerFreq <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'CornerFreq must be greater than zero.'
	ENDIF

	IF (LocalVar%DT <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'DT must be greater than zero.'
	ENDIF

	IF (CntrPar%VS_CtInSp < 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_CtInSp must not be negative.'
	ENDIF

	IF (CntrPar%VS_MinOM <= CntrPar%VS_CtInSp) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_MinOM must be greater than VS_CtInSp.'
	ENDIF

	IF (CntrPar%VS_MaxRat <= 0.0) THEN
		aviFAIL =  -1
		ErrMsg  = 'VS_MaxRat must be greater than zero.'
	ENDIF

    IF (CntrPar%VS_RtTq < 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_RtTw must not be negative.'
	ENDIF

	IF (CntrPar%VS_Rgn2K < 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_Rgn2K must not be negative.'
	ENDIF

	IF (CntrPar%VS_MaxTq < CntrPar%VS_RtTq) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.'
	ENDIF

	IF (CntrPar%VS_KP(1) > 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_KP must be greater than zero.'
	ENDIF

	IF (CntrPar%VS_KI(1) > 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'VS_KI must be greater than zero.'
	ENDIF

	IF (CntrPar%PC_RefSpd <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'PC_RefSpd must be greater than zero.'
	ENDIF

	IF (CntrPar%PC_MaxRat <= 0.0) THEN
		aviFAIL = -1
		ErrMsg  = 'PC_MaxRat must be greater than zero.'
	ENDIF

	IF (CntrPar%PC_MinPit >= CntrPar%PC_MaxPit)  THEN
		aviFAIL = -1
			ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
	ENDIF

	IF (CntrPar%IPC_KI <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_KI must be greater than zero.'
	ENDIF
    
	IF (CntrPar%IPC_omegaLP <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_omegaLP must be greater than zero.'
	ENDIF
    
	IF (CntrPar%IPC_omegaNotch <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_omegaNotch must be greater than zero.'
	ENDIF
    
	IF (CntrPar%IPC_phi <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_phi must be greater than zero.'
	ENDIF
    
	IF (CntrPar%IPC_zetaLP <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_zetaLP must be greater than zero.'
	ENDIF
    
	IF (CntrPar%IPC_zetaNotch <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'IPC_zetaNotch must be greater than zero.'
	ENDIF
    
	IF (CntrPar%Y_ErrThresh <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_ErrThresh must be greater than zero.'
	ENDIF
    
	IF (CntrPar%Y_Rate <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'CntrPar%Y_Rate must be greater than zero.'
	ENDIF
    
	IF (CntrPar%Y_omegaLPFast <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_omegaLPFast must be greater than zero.'
	ENDIF
    
	IF (CntrPar%Y_omegaLPSlow <= 0.0)  THEN
		aviFAIL = -1
		ErrMsg  = 'Y_omegaLPSlow must be greater than zero.'
	ENDIF

	!..............................................................................................................................
	! Initializing debug file
	!..............................................................................................................................

		! If we're debugging, open the debug file and write the header:
	IF (CntrPar%LoggingLevel > 0) THEN
		OPEN (UnDb, FILE=TRIM(RootName)//'.dbg', STATUS='REPLACE')
		WRITE (UnDb,'(A)')	'   LocalVar%Time '  //Tab//'PitComT  ' //Tab//'LocalVar%PC_SpdErr  ' //Tab//'LocalVar%PC_KP ' //Tab//'LocalVar%PC_KI  ' //Tab//'LocalVar%Y_M  ' //Tab//'LocalVar%rootMOOP(1)  '//Tab//'VS_RtPwr  '//Tab//'LocalVar%GenTrq'
		WRITE (UnDb,'(A)')	'   (sec) ' //Tab//'(rad)    '  //Tab//'(rad/s) '//Tab//'(-) ' //Tab//'(-)   ' //Tab//'(rad)   ' //Tab//'(?)   ' //Tab//'(W)   '//Tab//'(Nm)  '
	END IF
	
	IF (CntrPar%LoggingLevel > 1) THEN
		OPEN(UnDb2, FILE=TRIM(RootName)//'.dbg2', STATUS='REPLACE')
		WRITE(UnDb2,'(/////)')
		WRITE(UnDb2,'(A,85("'//Tab//'AvrSWAP(",I2,")"))')  'LocalVar%Time ', (i,i=1,85)
		WRITE(UnDb2,'(A,85("'//Tab//'(-)"))')  '(s)'
	END IF

ENDIF

!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------


IF ((LocalVar%iStatus >= 0) .AND. (aviFAIL >= 0))  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step
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
	LocalVar%GenSpeedF = SecLPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%CornerFreq, 0.7, LocalVar%iStatus, 1)     ! This is the first instance of a second order LPFilter

		! Calculate yaw-alignment error
	LocalVar%Y_MErr = LocalVar%Y_M + CntrPar%Y_MErrSet
	
	!..............................................................................................................................
	! VARIABLE-SPEED TORQUE CONTROL:
	!..............................................................................................................................
	
		! Compute the generator torque, which depends on which region we are in:
		
	LocalVar%VS_SpdErrAr = CntrPar%VS_RtSpd - LocalVar%GenSpeedF					! Current speed error - Above-rated PI-control
	LocalVar%VS_SpdErrBr = CntrPar%VS_MinOM - LocalVar%GenSpeedF					! Current speed error - Below-rated PI-control
	IF (PitComT >= CntrPar%VS_Rgn3MP) THEN						! We are in region 3
		LocalVar%GenTrqAr = PIController(LocalVar%VS_SpdErrAr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_Rgn2MaxTq, CntrPar%VS_GenTrqArSatMax, LocalVar%DT, CntrPar%VS_GenTrqArSatMax, .TRUE., 1)
		LocalVar%GenTrqBr = PIController(LocalVar%VS_SpdErrBr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MinTq, CntrPar%VS_Rgn2MinTq, LocalVar%DT, CntrPar%VS_Rgn2MinTq, .TRUE., 4)
		IF (CntrPar%VS_ControlMode == 1) THEN					! Constant power tracking
			LocalVar%GenTrq = CntrPar%VS_RtPwr/LocalVar%GenSpeedF
		ELSE											! Constant torque tracking
			LocalVar%GenTrq = CntrPar%VS_RtTq
		END IF
	ELSE
		LocalVar%GenTrqAr = PIController(LocalVar%VS_SpdErrAr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_Rgn2MaxTq, CntrPar%VS_GenTrqArSatMax, LocalVar%DT, CntrPar%VS_Rgn2MaxTq, .FALSE., 1)
		LocalVar%GenTrqBr = PIController(LocalVar%VS_SpdErrBr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MinTq, CntrPar%VS_Rgn2MinTq, LocalVar%DT, CntrPar%VS_Rgn2MinTq, .FALSE., 4)
		IF (LocalVar%GenTrqAr >= CntrPar%VS_Rgn2MaxTq*1.01) THEN
			LocalVar%GenTrq = LocalVar%GenTrqAr
			CONTINUE
		ELSEIF (LocalVar%GenTrqBr <= CntrPar%VS_Rgn2MinTq*0.99) THEN								! We are in region 1 1/2
			LocalVar%GenTrq = LocalVar%GenTrqBr
			CONTINUE
		ELSEIF (LocalVar%GenSpeedF < CntrPar%VS_MaxOM)  THEN										! We are in region 2 - optimal torque is proportional to the square of the generator speed
			LocalVar%GenTrq = CntrPar%VS_Rgn2K*LocalVar%GenSpeedF*LocalVar%GenSpeedF
		ELSE																		! We are in region 2 1/2 - simple induction generator transition region
			LocalVar%GenTrq = CntrPar%VS_Rgn2MaxTq
		END IF
	END IF

		! Saturate the commanded torque using the maximum torque limit:

	LocalVar%GenTrq = MIN(LocalVar%GenTrq, CntrPar%VS_MaxTq)						! Saturate the command using the maximum torque limit

		! Saturate the commanded torque using the torque rate limit:
	IF (LocalVar%iStatus == 0)  LastGenTrq = LocalVar%GenTrq				! Initialize the value of LastGenTrq on the first pass only
	LocalVar%GenTrq = ratelimit(LocalVar%GenTrq, LastGenTrq, -CntrPar%VS_MaxRat, CntrPar%VS_MaxRat, LocalVar%DT)	! Saturate the command using the torque rate limit

		! Reset the value of LastGenTrq to the current values:
	LastGenTrq = LocalVar%GenTrq

		! Set the generator contactor status, avrSWAP(35), to main (high speed)
		! variable-speed generator, the torque override to yes, and command the
		! generator torque (See Appendix A of Bladed User's Guide):
	avrSWAP(35) = 1.0          ! Generator contactor status: 1=main (high speed) variable-speed generator
	avrSWAP(56) = 0.0          ! Torque override: 0=yes
	avrSWAP(47) = LastGenTrq   ! Demanded generator torque

	!..............................................................................................................................
	! Pitch control
	!..............................................................................................................................

	IF (CntrPar%VS_ControlMode == 0 .AND. LocalVar%GenTrq >= CntrPar%PC_RtTq99) THEN
		LocalVar%PC_MaxPitVar = CntrPar%PC_MaxPit
	ELSEIF (CntrPar%VS_ControlMode == 1 .AND. LocalVar%GenTrqAr >= CntrPar%VS_GenTrqArSatMax*0.99) THEN
		LocalVar%PC_MaxPitVar = CntrPar%PC_MaxPit
	ELSE
		LocalVar%PC_MaxPitVar = CntrPar%PC_SetPnt
	END IF

		! Compute the gain scheduling correction factor based on the previously
		! commanded pitch angle for blade 1:
	LocalVar%PC_KP = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_kp, PitComT)
	LocalVar%PC_KI = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_ki, PitComT)
	LocalVar%PC_KD = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_kd, PitComT)
	LocalVar%PC_TF = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_tf, PitComT)

		! Compute the current speed error and its integral w.r.t. time; saturate the
		! integral term using the pitch angle limits:
	LocalVar%PC_SpdErr = CntrPar%PC_RefSpd - LocalVar%GenSpeedF					! Speed error
	LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr					! Power error

		! Compute the pitch commands associated with the proportional and integral
		!   gains:
	! PitComT = NotchFilter(PC_SpdErr, LocalVar%DT, 1.59, 0.01, 0.2, LocalVar%iStatus, 1)
	PitComT = PIController(LocalVar%PC_SpdErr, LocalVar%PC_KP, LocalVar%PC_KI, CntrPar%PC_SetPnt, LocalVar%PC_MaxPitVar, LocalVar%DT, CntrPar%PC_SetPnt, .FALSE., 2) ! + DFController(LocalVar%PC_SpdErr, LocalVar%PC_KD, LocalVar%PC_TF, LocalVar%DT, 1)
	IF (CntrPar%VS_ControlMode == 1) THEN
		PitComT = PitComT + PIController(LocalVar%PC_PwrErr, -4.0E-09, -4.0E-09, CntrPar%PC_SetPnt, LocalVar%PC_MaxPitVar, LocalVar%DT, CntrPar%PC_SetPnt, .FALSE., 5)
	END IF
	
		! Individual pitch control
	IF ((CntrPar%IPC_ControlMode == 1) .OR. (CntrPar%Y_ControlMode == 2)) THEN
		CALL IPC(LocalVar%rootMOOP, LocalVar%Azimuth, CntrPar%IPC_phi, LocalVar%Y_MErr, LocalVar%DT, CntrPar%IPC_KI, CntrPar%Y_IPC_KP, CntrPar%Y_IPC_KI, CntrPar%IPC_omegaHP, CntrPar%IPC_omegaLP, CntrPar%IPC_omegaNotch, CntrPar%IPC_zetaHP, CntrPar%IPC_zetaLP, CntrPar%IPC_zetaNotch, LocalVar%iStatus, CntrPar%IPC_ControlMode, CntrPar%Y_ControlMode, LocalVar%NumBl, LocalVar%IPC_PitComF)
	ELSE
		LocalVar%IPC_PitComF = 0.0
	END IF

		! Combine and saturate all pitch commands:
	DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
		PitComT_IPC(K) = PitComT + LocalVar%IPC_PitComF(K)									! Add the individual pitch command
		PitComT_IPC(K) = saturate(PitComT_IPC(K), CntrPar%PC_MinPit, CntrPar%PC_MaxPit)				! Saturate the overall command using the pitch angle limits
		
		! PitCom(K) = ratelimit(PitComT_IPC(K), LocalVar%BlPitch(K), PC_MinRat, PC_MaxRat, LocalVar%DT)	! Saturate the overall command of blade K using the pitch rate limit
		LocalVar%PitCom(K) = saturate(PitComT_IPC(K), CntrPar%PC_MinPit, CntrPar%PC_MaxPit)					! Saturate the overall command using the pitch angle limits
		LocalVar%PitCom(K) = LPFilter(LocalVar%PitCom(K), LocalVar%DT, CntrPar%CornerFreq, LocalVar%iStatus, .FALSE., K+3)
	END DO

		! Set the pitch override to yes and command the pitch demanded from the last
		! call to the controller (See Appendix A of Bladed User's Guide):

	avrSWAP(55) = 0.0						! Pitch override: 0=yes

	avrSWAP(42) = LocalVar%PitCom(1)		! Use the command angles of all blades if using individual pitch
	avrSWAP(43) = LocalVar%PitCom(2)		! "
	avrSWAP(44) = LocalVar%PitCom(3)		! "

	avrSWAP(45) = LocalVar%PitCom(1)		! Use the command angle of blade 1 if using collective pitch

	!..............................................................................................................................
	! Yaw control
	!..............................................................................................................................
	
	IF (CntrPar%Y_ControlMode == 1) THEN
		avrSWAP(29) = 0									! Yaw control parameter: 0 = yaw rate control
		IF (LocalVar%Time >= Y_YawEndT) THEN											! Check if the turbine is currently yawing
			avrSWAP(48) = 0.0													! Set yaw rate to zero

			LocalVar%Y_ErrLPFFast = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPFast, LocalVar%iStatus, .FALSE., 2)		! Fast low pass filtered yaw error with a frequency of 1
			LocalVar%Y_ErrLPFSlow = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPSlow, LocalVar%iStatus, .FALSE., 3)		! Slow low pass filtered yaw error with a frequency of 1/60

			Y_AccErr = Y_AccErr + LocalVar%DT*SIGN(LocalVar%Y_ErrLPFFast**2, LocalVar%Y_ErrLPFFast)	! Integral of the fast low pass filtered yaw error

			IF (ABS(Y_AccErr) >= CntrPar%Y_ErrThresh) THEN								! Check if accumulated error surpasses the threshold
				Y_YawEndT = ABS(LocalVar%Y_ErrLPFSlow/CntrPar%Y_Rate) + LocalVar%Time					! Yaw to compensate for the slow low pass filtered error
			END IF
		ELSE
			avrSWAP(48) = SIGN(CntrPar%Y_Rate, LocalVar%Y_MErr)		! Set yaw rate to predefined yaw rate, the sign of the error is copied to the rate
			LocalVar%Y_ErrLPFFast = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPFast, LocalVar%iStatus, .TRUE., 2)		! Fast low pass filtered yaw error with a frequency of 1
			LocalVar%Y_ErrLPFSlow = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPSlow, LocalVar%iStatus, .TRUE., 3)		! Slow low pass filtered yaw error with a frequency of 1/60
			Y_AccErr = 0.0								! "
		END IF
	END IF
	!..............................................................................................................................
		! Output debugging information if requested:
	IF (CntrPar%LoggingLevel > 0) THEN
		WRITE (UnDb,FmtDat)		LocalVar%Time,	PitComT,	LocalVar%PC_SpdErr,	LocalVar%PC_KP,	LocalVar%PC_KI,	LocalVar%Y_MErr,	LocalVar%rootMOOP(1), CntrPar%VS_RtPwr, LocalVar%GenTrq
	END IF
	IF (CntrPar%LoggingLevel > 1) THEN
		WRITE (UnDb2,FmtDat)	LocalVar%Time, avrSWAP(1:85)
	END IF

	
ENDIF

avcMSG = TRANSFER(TRIM(ErrMsg)//C_NULL_CHAR, avcMSG, SIZE(avcMSG))

RETURN

END SUBROUTINE DISCON
