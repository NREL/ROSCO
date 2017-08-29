!=======================================================================
! SUBROUTINE DISCON ( avrSWAP, from_SC, to_SC, aviFAIL, accINFILE, avcOUTNAME, avcMSG ) BIND (C, NAME='DISCON')
SUBROUTINE DISCON ( avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG ) BIND (C, NAME='DISCON')
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON

   ! 18/08/2017

   ! This Bladed-style DLL controller is used to implement a variable-speed
   ! generator-torque controller, PI collective blade pitch controller, individual pitch
   ! controller and yaw controller for the NREL Offshore 5MW baseline wind turbine.
   ! This routine was extended by S. Mulders, J. Hoorneman and J. Govers of TU Delft.
   ! The routine is based on the routine as written by J. Jonkman of NREL/NWTC for use
   ! in the IEA Annex XXIII OC3 studies.

   ! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
   ! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

   ! Note that gfortran v5.x on Mac produces compiler errors with the DLLEXPORT attribute,
   ! so the compiler directive IMPLICIT_DLLEXPORT is added.

USE, INTRINSIC  :: ISO_C_Binding
USE             :: FunctionToolbox
USE             :: Filters

IMPLICIT NONE
#ifndef IMPLICIT_DLLEXPORT
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
#endif


!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------


   ! Passed Variables:
!REAL(C_FLOAT),          INTENT(IN   ) :: from_SC   (*)  ! DATA from the supercontroller
!REAL(C_FLOAT),          INTENT(INOUT) :: to_SC     (*)  ! DATA to the supercontroller


REAL(C_FLOAT),          INTENT(INOUT) :: avrSWAP   (*)                  ! The swap array, used to pass data to, and receive data from, the DLL controller.
INTEGER(C_INT),         INTENT(INOUT) :: aviFAIL                        ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
CHARACTER(KIND=C_CHAR), INTENT(IN)    :: accINFILE (NINT(avrSWAP(50)))  ! The name of the parameter input file
CHARACTER(KIND=C_CHAR), INTENT(IN)    :: avcOUTNAME(NINT(avrSWAP(51)))  ! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcMSG    (NINT(avrSWAP(49)))  ! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.


   ! Local Variables:

REAL(4)                      :: BlPitch (3)                                   	! Current values of the blade pitch angles [rad].
REAL(4), PARAMETER           :: CornerFreq    	=	0.7853981                  	! Corner frequency (-3dB point) in the recursive, single-pole, low-pass filter [rad/s]. -- chosen to be 1/4 the blade edgewise natural frequency ( 1/4 of approx. 1 [Hz] = 0.25 [Hz] = 1.570796 [rad/s]).
REAL(4)                      :: DT                                              ! Time step [s].
REAL(4)                      :: ElapTime                                        ! Elapsed time since the last call to the controller [s].
REAL(4)                      :: GenSpeed                                        ! Current  HSS (generator) speed [rad/s].
REAL(4)                      :: GenSpeedF                                       ! Filtered HSS (generator) speed [rad/s].
REAL(4)                      :: GenTrq                                          ! Electrical generator torque, [Nm].
REAL(4)                      :: HorWindV                                        ! Horizontal wind speed at hub-height, [m/s].
REAL(4)                      :: IPC_aziAngle                                 	! Rotor azimuth angle [rad].
REAL(4), PARAMETER           :: IPC_KI          =	0.0000000008                ! Integral gain for the individual pitch controller, [-].
REAL(4), PARAMETER           :: IPC_KNotch      =	1                           ! Notch filter gain for the individual pitch controller, [-].
REAL(4), PARAMETER           :: IPC_omegaLP     =	1000.0                      ! Low pass filter corner frequency for the individual pitch controller, [Hz].
REAL(4), PARAMETER           :: IPC_omegaNotch  =	1.269330365                 ! Notch filter corner frequency for the individual pitch controller, [Hz].
REAL(4), PARAMETER           :: IPC_phi         =	0.436332313                 ! Phase offset added to the azimuth angle for the individual pitch controller, [rad].
REAL(4)                      :: IPC_PitComF (3)                                 ! Commanded pitch of each blade as calculated by the individual pitch controller, F stands for low pass filtered, [rad].
REAL(4), PARAMETER           :: IPC_zetaLP      =	1.0                         ! Low pass filter damping factor for the individual pitch controller, [-].
REAL(4), PARAMETER           :: IPC_zetaNotch   =	0.5                         ! Notch filter damping factor for the individual pitch controller, [-].
REAL(4), SAVE                :: IntSpdErr                                       ! Current integral of speed error w.r.t. time, [rad].
REAL(4), SAVE                :: LastGenTrq                                      ! Commanded electrical generator torque the last time the controller was called, [Nm].
REAL(4), SAVE                :: LastTime                                        ! Last time this DLL was called, [s].
REAL(4), SAVE                :: LastTimePC                                      ! Last time the pitch  controller was called, [s].
REAL(4), SAVE                :: LastTimeVS                                      ! Last time the torque controller was called, [s].
REAL(4)                      :: PC_GK                                           ! Current value of the gain correction factor, used in the gain scheduling law of the pitch controller, [-].
REAL(4), PARAMETER           :: PC_KI         	=	0.008068634               	! Integral gain for pitch controller at rated pitch (zero), [-].
REAL(4), PARAMETER           :: PC_KK         	=	0.1099965                 	! Pitch angle where the the derivative of the aerodynamic power w.r.t. pitch has increased by a factor of two relative to the derivative at rated pitch (zero), [rad].
REAL(4), PARAMETER           :: PC_KP         	=	0.01882681                	! Proportional gain for pitch controller at rated pitch (zero), [s].
REAL(4), PARAMETER           :: PC_MaxPit     	=	1.570796                  	! Maximum pitch setting in pitch controller, [rad].
REAL(4), PARAMETER           :: PC_MaxRat     	=	0.1396263                 	! Maximum pitch  rate (in absolute value) in pitch  controller, [rad/s].
REAL(4)                      :: PC_MinPit                                       ! Minimum pitch setting in pitch controller, [rad].
REAL(4), PARAMETER           :: PC_RefSpd     	=	122.9096                    ! Desired (reference) HSS speed for pitch controller, [rad/s].
REAL(4)                      :: PC_SetPnt                                       ! Pitch set point used as minimum, probably set to zero, [rad].
REAL(4), SAVE                :: PitCom (3)                                   	! Commanded pitch of each blade the last time the controller was called, [rad].
REAL(4)                      :: PitComI                                         ! Integral term of command pitch, [rad].
REAL(4)                      :: PitComP                                         ! Proportional term of command pitch, [rad].
REAL(4)                      :: PitComT (3)                                     ! Total command pitch based on the sum of the proportional and integral terms, [rad].
REAL(4)                      :: PitRate (3)                                   	! Pitch rates of each blade based on the current pitch angles and current pitch command, [rad/s].
REAL(4), PARAMETER           :: R2D           	=	57.295780                  	! Factor to convert radians to degrees.
REAL(4)                      :: rootMOOP (3)                                    ! Blade root out of plane bending moments, [Nm].
REAL(4), PARAMETER           :: RPS2RPM       	=	9.5492966                 	! Factor to convert radians per second to revolutions per minute.
REAL(4)                      :: SpdErr                                          ! Current speed error, [rad/s].
REAL(4)                      :: Time                                            ! Current simulation time, [s].
REAL(4)                      :: TrqRate                                         ! Torque rate based on the current and last torque commands, [Nm/s].
REAL(4), PARAMETER           :: VS_CtInSp     	=	70.16224                   	! Transitional generator speed (HSS side) between regions 1 and 1 1/2, [rad/s].
REAL(4), PARAMETER           :: VS_MaxRat     	=	15000.0                   	! Maximum torque rate (in absolute value) in torque controller, [Nm/s].
REAL(4), PARAMETER           :: VS_MaxTq      	=	47402.91                  	! Maximum generator torque in Region 3 (HSS side), [Nm]. -- chosen to be 10% above VS_RtTq = 43.09355kNm
REAL(4), PARAMETER           :: VS_Rgn2K      	=	2.332287                  	! Generator torque constant in Region 2 (HSS side), N-m/(rad/s)^2.
REAL(4), PARAMETER           :: VS_Rgn2Sp     	=	91.21091                  	! Transitional generator speed (HSS side) between regions 1 1/2 and 2, [rad/s].
REAL(4), PARAMETER           :: VS_Rgn3MP     	=	0.01745329                	! Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, [rad]. -- chosen to be 1.0 degree above PC_MinPit
REAL(4), PARAMETER           :: VS_RtGnSp     	=	121.6805                    ! Rated generator speed (HSS side), [rad/s]. -- chosen to be 99% of PC_RefSpd
REAL(4)                      :: VS_RtTq                                         ! Rated torque, [Nm].
REAL(4), PARAMETER           :: VS_RtPwr      	=	5296610.0                   ! Rated generator generator power in Region 3, [W]. -- chosen to be 5MW divided by the electrical generator efficiency of 94.4%
REAL(4), SAVE                :: VS_Slope15                                      ! Torque/speed slope of region 1 1/2 cut-in torque ramp , [Nm/(rad/s)].
REAL(4), SAVE                :: VS_Slope25                                      ! Torque/speed slope of region 2 1/2 induction generator, [Nm/(rad/s)].
REAL(4), PARAMETER           :: VS_SlPc       	=	10.0                       	! Rated generator slip percentage in Region 2 1/2, [%].
REAL(4), SAVE                :: VS_SySp                                         ! Synchronous speed of region 2 1/2 induction generator, [rad/s].
REAL(4), SAVE                :: VS_TrGnSp                                       ! Transitional generator speed (HSS side) between regions 2 and 2 1/2, [rad/s].
REAL(4), SAVE                :: Y_AccErr										! Accumulated yaw error [rad].
REAL(4)                      :: Y_ErrLPFFast									! Filtered yaw error by fast low pass filter [rad].
REAL(4)                      :: Y_ErrLPFSlow									! Filtered yaw error by slow low pass filter [rad].
REAL(4), PARAMETER           :: Y_ErrThresh   	=	1.745329252               	! Error threshold [rad]. Turbine begins to yaw when it passes this. (104.71975512).
REAL(4), PARAMETER           :: Y_YawRate     	=	0.005235988               	! Yaw rate [rad/s].
REAL(4)                      :: Y_MErr                                          ! Measured yaw error [rad].
REAL(4), PARAMETER           :: Y_omegaLPFast 	=	1.0							! Corner frequency fast low pass filter, [Hz].
REAL(4), PARAMETER           :: Y_omegaLPSlow 	=	0.016666667					! Corner frequency slow low pass filter, 1/60 [Hz].
REAL(4), SAVE                :: Y_YawEndT										! Yaw end time, [s]. Indicates the time up until which the yaws with a fixed rate.

INTEGER(4)                   :: ErrStat
INTEGER(4)                   :: I                                               ! Generic index.
INTEGER(4)                   :: iStatus                                         ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
INTEGER(4)                   :: K                                               ! Loops through blades.
INTEGER(4)                   :: NumBl                                           ! Number of blades, [-].
INTEGER(4), PARAMETER        :: UnDb          = 85                              ! I/O unit for the debugging information
INTEGER(4), PARAMETER        :: UnDb2         = 86                              ! I/O unit for the debugging information
INTEGER(4), PARAMETER        :: Un            = 87                              ! I/O unit for pack/unpack (checkpoint & restart)
INTEGER(4), PARAMETER        :: UnUser        = 88                              ! I/O unit for user defined parameter file

LOGICAL(1), PARAMETER        :: DbgOut     = .FALSE.                          	! Flag to indicate whether to output debugging information

CHARACTER(   1), PARAMETER   :: Tab           = CHAR( 9 )                       ! The tab character.
CHARACTER(  25), PARAMETER   :: FmtDat    = "(F8.3,99('"//Tab//"',ES10.3E2,:))"	! The format of the debugging data
CHARACTER(   *), PARAMETER   :: UserFile      = 'DISCON.IN'                     ! Name of the user defined parameter file

CHARACTER(SIZE(accINFILE)-1) :: InFile                                          ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcOUTNAME)-1):: RootName                                        ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)    :: ErrMsg                                          ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]


   ! Load variables from calling program (See Appendix A of Bladed User's Guide):

BlPitch  (1) =       avrSWAP( 4)
BlPitch  (2) =       avrSWAP(33)
BlPitch  (3) =       avrSWAP(34)
DT           =       avrSWAP( 3)
GenSpeed     =       avrSWAP(20)
HorWindV     =       avrSWAP(27)
IPC_aziAngle =       avrSWAP(60)
iStatus      = NINT( avrSWAP( 1) )
NumBl        = NINT( avrSWAP(61) )
PC_MinPit    =       avrSWAP( 6)
PC_SetPnt    =       avrSWAP( 5)
rootMOOP (1) =       avrSWAP(30)
rootMOOP (2) =       avrSWAP(31)
rootMOOP (3) =       avrSWAP(32)
Time         =       avrSWAP( 2)
Y_MErr       =       avrSWAP(24)
VS_RtTq      =       avrSWAP(22)

!print *, 'from_sc: ', from_sc(1:4)
!to_sc(1) = 5.0;
!to_sc(2) = 2.0;

   ! Convert C character arrays to Fortran strings:

RootName = TRANSFER( avcOUTNAME(1:LEN(RootName)), RootName )
I = INDEX(RootName,C_NULL_CHAR) - 1       ! if this has a c null character at the end...
IF ( I > 0 ) RootName = RootName(1:I)     ! remove it

InFile = TRANSFER( accINFILE(1:LEN(InFile)),  InFile )
I = INDEX(InFile,C_NULL_CHAR) - 1         ! if this has a c null character at the end...
IF ( I > 0 ) InFile = InFile(1:I)         ! remove it

   ! Initialize aviFAIL to 0:

aviFAIL      = 0


   ! Read any External Controller Parameters specified in the User Interface
   !   and initialize variables:

IF ( iStatus == 0 )  THEN  ! .TRUE. if we're on the first call to the DLL

		! Inform users that we are using this user-defined routine:

	aviFAIL  = 1
	ErrMsg   = 'Running with torque and pitch control of the NREL offshore '// &
			  '5MW baseline wind turbine from DISCON.dll as written by J. '// &
			  'Jonkman of NREL/NWTC for use in the IEA Annex XXIII OC3 '   // &
			  'studies.'

		! Read user defined parameter file

	OPEN( UnUser, file=UserFile)
	DO I = 120, 129
		READ( UnUser, *) avrSWAP(I)
	END DO
	CLOSE(UnUser)

		! Determine some torque control parameters not specified directly:

	VS_SySp    	= VS_RtGnSp/( 1.0 +  0.01*VS_SlPc )
	VS_Slope15 	= ( VS_Rgn2K*VS_Rgn2Sp*VS_Rgn2Sp )/( VS_Rgn2Sp - VS_CtInSp )
	VS_Slope25 	= ( VS_RtPwr/VS_RtGnSp           )/( VS_RtGnSp - VS_SySp   )
	IF ( VS_Rgn2K == 0.0 )  THEN  ! .TRUE. if the Region 2 torque is flat, and thus, the denominator in the ELSE condition is zero
	  VS_TrGnSp = VS_SySp
	ELSE                          ! .TRUE. if the Region 2 torque is quadratic with speed
	  VS_TrGnSp = ( VS_Slope25 - SQRT( VS_Slope25*( VS_Slope25 - 4.0*VS_Rgn2K*VS_SySp ) ) )/( 2.0*VS_Rgn2K )
	ENDIF

		! Initialize the SAVEd variables:
		! NOTE: LastGenTrq, though SAVEd, is initialized in the torque controller
		!       below for simplicity, not here.

	PC_GK      = 1.0/( 1.0 + PitCom(1)/PC_KK )   ! This will ensure that the pitch angle is unchanged if the initial SpdErr is zero
	IntSpdErr  = PitCom(1)/( PC_GK*PC_KI )       ! This will ensure that the pitch angle is unchanged if the initial SpdErr is zero
	PitCom     = BlPitch                         ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
	Y_AccErr   = 0.0                             ! This will ensure that the accumulated yaw error starts at zero
	Y_YawEndT  = -1.0                            ! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing

	LastTime   = Time                            ! This will ensure that generator speed filter will use the initial value of the generator speed on the first pass
	LastTimePC = Time - DT                       ! This will ensure that the pitch  controller is called on the first pass
	LastTimeVS = Time - DT                       ! This will ensure that the torque controller is called on the first pass


	!..............................................................................................................................
	! Check validity of input parameters:
	!..............................................................................................................................


	IF ( CornerFreq <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'CornerFreq must be greater than zero.'
	ENDIF

	IF ( DT     <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'DT must be greater than zero.'
	ENDIF

	IF ( VS_CtInSp <  0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_CtInSp must not be negative.'
	ENDIF

	IF ( VS_Rgn2Sp <= VS_CtInSp )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_Rgn2Sp must be greater than VS_CtInSp.'
	ENDIF

	IF ( VS_TrGnSp <  VS_Rgn2Sp )  THEN
	  aviFAIL = -1
	  ErrMsg = 'VS_TrGnSp must not be less than VS_Rgn2Sp.'
	ENDIF

	IF ( VS_SlPc   <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_SlPc must be greater than zero.'
	ENDIF

	IF ( VS_MaxRat <= 0.0 )  THEN
	  aviFAIL =  -1
	  ErrMsg  = 'VS_MaxRat must be greater than zero.'
	ENDIF

	IF ( VS_RtPwr  <  0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_RtPwr must not be negative.'
	ENDIF

    IF ( VS_RtTq  <  0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_RtTw must not be negative.'
	ENDIF

	IF ( VS_Rgn2K  <  0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_Rgn2K must not be negative.'
	ENDIF

	IF ( VS_Rgn2K*VS_RtGnSp*VS_RtGnSp > VS_RtPwr/VS_RtGnSp )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_Rgn2K*VS_RtGnSp^2 must not be greater than VS_RtPwr/VS_RtGnSp.'
	ENDIF

	IF ( VS_MaxTq                     < VS_RtPwr/VS_RtGnSp )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'VS_RtPwr/VS_RtGnSp must not be greater than VS_MaxTq.'
	ENDIF

	IF ( PC_KI     <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'PC_KI must be greater than zero.'
	ENDIF

	IF ( PC_KK     <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'PC_KK must be greater than zero.'
	ENDIF

	IF ( PC_KP     <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'PC_KP must be greater than zero.'
	ENDIF

	IF ( PC_RefSpd <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'PC_RefSpd must be greater than zero.'
	ENDIF

	IF ( PC_MaxRat <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'PC_MaxRat must be greater than zero.'
	ENDIF

	IF ( PC_MinPit >= PC_MaxPit )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
	ENDIF

	IF ( IPC_KI <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_KI must be greater than zero.'
	ENDIF

	IF ( IPC_KNotch <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_KNotch must be greater than zero.'
	ENDIF

	IF ( IPC_omegaLP <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_omegaLP must be greater than zero.'
	ENDIF

	IF ( IPC_omegaNotch <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_omegaNotch must be greater than zero.'
	ENDIF

	IF ( IPC_phi <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_phi must be greater than zero.'
	ENDIF

	IF ( IPC_zetaLP <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_zetaLP must be greater than zero.'
	ENDIF

	IF ( IPC_zetaNotch <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'IPC_zetaNotch must be greater than zero.'
	ENDIF

    IF ( Y_ErrThresh <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'Y_ErrThresh must be greater than zero.'
	ENDIF

	IF ( Y_YawRate <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'Y_YawRate must be greater than zero.'
	ENDIF

	IF ( Y_omegaLPFast <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'Y_omegaLPFast must be greater than zero.'
	ENDIF

	IF ( Y_omegaLPSlow <= 0.0 )  THEN
	  aviFAIL = -1
	  ErrMsg  = 'Y_omegaLPSlow must be greater than zero.'
	ENDIF


	!..............................................................................................................................
	! Initializing debug file
	!..............................................................................................................................

		! If we're debugging, open the debug file and write the header:

	IF ( DbgOut )  THEN

	  OPEN ( UnDb, FILE=TRIM( RootName )//'.dbg', STATUS='REPLACE' )

		WRITE (UnDb,'(/////)')
		WRITE (UnDb,'(A)')  '   Time '  //Tab//'ElapTime  ' //Tab//'HorWindV ' //Tab//'GenSpeed  ' //Tab//'GenSpeedF ' //Tab//'RelSpdErr ' //Tab// &
						  'SpdErr    '  //Tab//'IntSpdErr ' //Tab//'PC_GK    ' //Tab//'PitComP   ' //Tab//'PitComI   ' //Tab//'MErr      ' //Tab// &
						  'PitRate1  '  //Tab//'PitRate2  ' //Tab//'PitRate3 ' //Tab//'PitCom1   ' //Tab//'PitCom2   ' //Tab//'PitCom3   ' //Tab// &
						  'BlPitch1  '  //Tab//'BlPitch2  ' //Tab//'BlPitch3 ' //Tab//'rootMOOP1 ' //Tab//'rootMOOP2 ' //Tab//'rootMOOP3 ' //Tab// &
						  'PitComF1  '  //Tab//'PitComF2  ' //Tab//'PitComF3 ' //Tab//'PitComT1  ' //Tab//'PitComT2  ' //Tab//'PitComT3  ' //Tab// &
						  'ErrLPFFast ' //Tab//'ErrLPFSlow' //Tab//'Y_AccErr ' //Tab//'Y_YawEndT '

		WRITE (UnDb,'(A)')  '   (sec) ' //Tab//'(sec)    '   //Tab//'(m/sec) ' //Tab//'(rpm)   '   //Tab//'(rpm)   '   //Tab//'(%)     '   //Tab// &
						  '(rad/s)   '  //Tab//'(rad)    '   //Tab//'(-)     ' //Tab//'(deg)   '   //Tab//'(deg)   '   //Tab//'(deg)   '   //Tab// &
						  '(deg/s)   '  //Tab//'(deg/s)  '   //Tab//'(deg/s) ' //Tab//'(deg)   '   //Tab//'(deg)   '   //Tab//'(deg)   '   //Tab// &
						  '(deg)     '  //Tab//'(deg)    '   //Tab//'(deg)   ' //Tab//'(Nm)    '   //Tab//'(Nm)    '   //Tab//'(Nm)    '   //Tab// &
						  '(deg)     '  //Tab//'(deg)    '   //Tab//'(deg)   ' //Tab//'(deg)   '   //Tab//'(deg)   '   //Tab//'(deg)   '   //Tab// &
						  '(deg)     '  //Tab//'(deg)    '   //Tab//'(deg*s) ' //Tab//'(sec)   '

		OPEN ( UnDb2, FILE=TRIM( RootName )//'.dbg2', STATUS='REPLACE' )
		WRITE (UnDb2,'(/////)')

		WRITE (UnDb2,'(A,85("'//Tab//'AvrSWAP(",I2,")"))')  'Time ', (i,i=1,85)
		WRITE (UnDb2,'(A,85("'//Tab//'(-)"))')  '(s)'

	ENDIF


ENDIF


!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------


IF ( ( iStatus >= 0 ) .AND. ( aviFAIL >= 0 ) )  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step


		! Abort if the user has not requested a pitch angle actuator (See Appendix A
		!   of Bladed User's Guide):

	IF ( NINT(avrSWAP(10)) /= 0 )  THEN ! .TRUE. if a pitch angle actuator hasn't been requested
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
	GenSpeedF = LPFilter( GenSpeed, DT, CornerFreq, iStatus, 1)     ! This is the first instance of LPFilter


	!..............................................................................................................................
	! VARIABLE-SPEED TORQUE CONTROL:
	!..............................................................................................................................


		! Compute the elapsed time since the last call to the controller:

	ElapTime = Time - LastTimeVS

		! Compute the generator torque, which depends on which region we are in:

	IF ( (   GenSpeedF >= VS_RtGnSp ) .OR. (  PitCom(1) >= VS_Rgn3MP ) )  THEN ! We are in region 3 - power is constant
		GenTrq = VS_RtTq
	ELSEIF ( GenSpeedF <= VS_CtInSp )  THEN                                    ! We are in region 1 - torque is zero
		GenTrq = 0.0
	ELSEIF ( GenSpeedF <  VS_Rgn2Sp )  THEN                                    ! We are in region 1 1/2 - linear ramp in torque from zero to optimal
		GenTrq = VS_Slope15*( GenSpeedF - VS_CtInSp )
	ELSEIF ( GenSpeedF <  VS_TrGnSp )  THEN                                    ! We are in region 2 - optimal torque is proportional to the square of the generator speed
		GenTrq = VS_Rgn2K*GenSpeedF*GenSpeedF
	ELSE                                                                       ! We are in region 2 1/2 - simple induction generator transition region
		GenTrq = VS_Slope25*( GenSpeedF - VS_SySp   )
	ENDIF


		! Saturate the commanded torque using the maximum torque limit:

	GenTrq  = MIN( GenTrq , VS_MaxTq  )						! Saturate the command using the maximum torque limit


		! Saturate the commanded torque using the torque rate limit:

	IF ( iStatus == 0 )  LastGenTrq = GenTrq				! Initialize the value of LastGenTrq on the first pass only
	TrqRate = ( GenTrq - LastGenTrq )/ElapTime				! Torque rate (unsaturated)
	TrqRate = saturate(TrqRate,-VS_MaxRat,VS_MaxRat)		! Saturate the torque rate using its maximum absolute value
	GenTrq  = LastGenTrq + TrqRate*ElapTime					! Saturate the command using the torque rate limit


		! Reset the values of LastTimeVS and LastGenTrq to the current values:

	LastTimeVS = Time
	LastGenTrq = GenTrq


		! Set the generator contactor status, avrSWAP(35), to main (high speed)
		!   variable-speed generator, the torque override to yes, and command the
		!   generator torque (See Appendix A of Bladed User's Guide):

	avrSWAP(35) = 1.0          ! Generator contactor status: 1=main (high speed) variable-speed generator
	avrSWAP(56) = 0.0          ! Torque override: 0=yes
	avrSWAP(47) = LastGenTrq   ! Demanded generator torque


	!..............................................................................................................................
	! Pitch control
	!..............................................................................................................................


		! Compute the elapsed time since the last call to the controller:

	ElapTime = Time - LastTimePC

		! Compute the gain scheduling correction factor based on the previously
		!   commanded pitch angle for blade 1:

	PC_GK = 1.0/( 1.0 + PitCom(1)/PC_KK )


		! Compute the current speed error and its integral w.r.t. time; saturate the
		!   integral term using the pitch angle limits:

	SpdErr    = GenSpeedF - PC_RefSpd									! Current speed error
	IntSpdErr = IntSpdErr + SpdErr*ElapTime								! Current integral of speed error w.r.t. time
	IntSpdErr = saturate(IntSpdErr,PC_SetPnt/( PC_GK*PC_KI ),&
											PC_MaxPit/( PC_GK*PC_KI )	)	! Saturate the integral term using the pitch angle limits, converted to integral speed error limits


		! Compute the pitch commands associated with the proportional and integral
		!   gains:

	PitComP   = PC_GK*PC_KP*   SpdErr									! Proportional term
	PitComI   = PC_GK*PC_KI*IntSpdErr									! Integral term (saturated)


		! Individual pitch control

	CALL IPC(rootMOOP, IPC_aziAngle, DT, IPC_KI, IPC_KNotch, IPC_omegaLP, IPC_omegaNotch, IPC_phi, IPC_zetaLP, IPC_zetaNotch, iStatus, NumBl, IPC_PitComF)


        ! Combine and saturate all pitch commands:

	DO K = 1,NumBl ! Loop through all blades

		 PitComT (K)  = PitComP + PitComI                    			! Overall command (unsaturated)
		 PitComT (K)  = saturate(PitComT(K),PC_SetPnt,PC_MaxPit)		! Saturate the overall command using the pitch set point
		 PitComT (K)  = PitComT(K) + IPC_PitComF(K)                     ! Add the individual pitch command
		 PitComT (K)  = saturate(PitComT(K),PC_MinPit,PC_MaxPit)        ! Saturate the overall command using the pitch angle limits

		 PitRate(K) = ( PitComT(K) - BlPitch(K) )/ElapTime				! Pitch rate of blade K (unsaturated)
		 PitRate(K) = saturate( PitRate(K), -1.0*PC_MaxRat, PC_MaxRat )	! Saturate the pitch rate of blade K using its maximum absolute value
		 PitCom (K) = BlPitch(K) + PitRate(K)*ElapTime                  ! Saturate the overall command of blade K using the pitch rate limit

		 PitCom (K) = saturate( PitCom(K), PC_MinPit, PC_MaxPit )		! Saturate the overall command using the pitch angle limits

	ENDDO

		! Set the pitch override to yes and command the pitch demanded from the last
		!   call to the controller (See Appendix A of Bladed User's Guide):

	avrSWAP(55) = 0.0       ! Pitch override: 0=yes

	avrSWAP(42) = PitCom(1) ! Use the command angles of all blades if using individual pitch
	avrSWAP(43) = PitCom(2) ! "
	avrSWAP(44) = PitCom(3) ! "

	avrSWAP(45) = PitCom(1) ! Use the command angle of blade 1 if using collective pitch

		! Reset the value of LastTimePC to the current value:

	  LastTimePC = Time


	!..............................................................................................................................
	! Yaw control
	!..............................................................................................................................


	avrSWAP(29)	= 0				    ! Yaw control parameter: 0 = yaw rate control

	IF ( Y_YawEndT <= Time) THEN    ! Check if the turbine is currently yawing
		avrSWAP(48) = 0.0                                                   ! Set yaw rate to zero

		Y_ErrLPFFast    = LPFilter( Y_MErr, DT, Y_omegaLPFast, iStatus, 2)  ! Fast low pass filtered yaw error with a frequency of 1
		Y_ErrLPFSlow    = LPFilter( Y_MErr, DT, Y_omegaLPSlow, iStatus, 3)  ! Slow low pass filtered yaw error with a frequency of 1/60

		Y_AccErr = Y_AccErr + ElapTime*SIGN(Y_ErrLPFFast**2,Y_ErrLPFFast)   ! Integral of the fast low pass filtered yaw error

		IF ( ABS(Y_AccErr) >= Y_ErrThresh ) THEN                            ! Check if accumulated error surpasses the threshold
			Y_YawEndT   = ABS(Y_ErrLPFSlow/Y_YawRate) + Time                ! Yaw to compensate for the slow low pass filtered error
		END IF
	ELSE
		avrSWAP(48)		= SIGN(Y_YawRate,Y_MErr)    ! Set yaw rate to predefined yaw rate, the sign of the error is copied to the rate
		Y_ErrLPFFast    = 0.0                       ! Reset all errors
		Y_ErrLPFSlow    = 0.0                       ! "
		Y_AccErr        = 0.0                       ! "
	END IF


	!..............................................................................................................................


		! Output debugging information if requested:

	IF ( DbgOut )  THEN
		WRITE (UnDb,FmtDat)  Time,			    ElapTime,		HorWindV,	GenSpeed*RPS2RPM,	GenSpeedF*RPS2RPM,	100.0*SpdErr/PC_RefSpd, &
							 SpdErr,		    IntSpdErr,		PC_GK,	    PitComP*R2D,		PitComI*R2D,		Y_MErr*R2D,             &
							 PitRate*R2D,								    PitCom*R2D,														&
							 BlPitch*R2D,								    rootMOOP,														&
							 IPC_PitComF*R2D,							    PitComT*R2D,		                                            &
							 Y_ErrLPFFast*R2D,  Y_ErrLPFSlow*R2D,			Y_AccErr*R2D,       Y_YawEndT

		WRITE (UnDb2,FmtDat) Time, avrSWAP(1:85)
	END IF


	!..............................................................................................................................


		! Reset the value of LastTime to the current value:

	LastTime = Time


!------------------------------------------------------------------------------------------------------------------------------
! Save and load saved variables in case of crash
!------------------------------------------------------------------------------------------------------------------------------

ELSEIF ( iStatus == -8 )  THEN

		! pack
	OPEN( Un, FILE=TRIM( InFile ), STATUS='UNKNOWN', FORM='UNFORMATTED' , ACCESS='STREAM', IOSTAT=ErrStat, ACTION='WRITE' )

	IF ( ErrStat /= 0 ) THEN
		ErrMsg  = 'Cannot open file "'//TRIM( InFile )//'". Another program may have locked it for writing.'
		aviFAIL = -1
	ELSE

			! write all static variables to the checkpoint file (inverse of unpack):
		WRITE( Un, IOSTAT=ErrStat ) GenSpeedF               ! Filtered HSS (generator) speed, [rad/s].
		WRITE( Un, IOSTAT=ErrStat ) IntSpdErr               ! Current integral of speed error w.r.t. time, [rad].
		WRITE( Un, IOSTAT=ErrStat ) LastGenTrq              ! Commanded electrical generator torque the last time the controller was called, [Nm].
		WRITE( Un, IOSTAT=ErrStat ) LastTime                ! Last time this DLL was called, [s].
		WRITE( Un, IOSTAT=ErrStat ) LastTimePC              ! Last time the pitch  controller was called, [s].
		WRITE( Un, IOSTAT=ErrStat ) LastTimeVS              ! Last time the torque controller was called, [s].
		WRITE( Un, IOSTAT=ErrStat ) PitCom                  ! Commanded pitch of each blade the last time the controller was called, [rad].
		WRITE( Un, IOSTAT=ErrStat ) VS_Slope15              ! Torque/speed slope of region 1 1/2 cut-in torque ramp , [Nm/(rad/s)].
		WRITE( Un, IOSTAT=ErrStat ) VS_Slope25              ! Torque/speed slope of region 2 1/2 induction generator, [Nm/(rad/s)].
		WRITE( Un, IOSTAT=ErrStat ) VS_SySp                 ! Synchronous speed of region 2 1/2 induction generator, [rad/s].
		WRITE( Un, IOSTAT=ErrStat ) VS_TrGnSp               ! Transitional generator speed (HSS side) between regions 2 and 2 1/2, [rad/s].

		CLOSE ( Un )

	END IF

ELSEIF( iStatus == -9 ) THEN

		!unpack
	OPEN( Un, FILE=TRIM( InFile ), STATUS='OLD', FORM='UNFORMATTED', ACCESS='STREAM', IOSTAT=ErrStat, ACTION='READ' )

	IF ( ErrStat /= 0 ) THEN
		aviFAIL = -1
		ErrMsg  = ' Cannot open file "'//TRIM( InFile )//'" for reading. Another program may have locked.'
	ELSE

			! READ all static variables from the restart file (inverse of pack):
		READ( Un, IOSTAT=ErrStat ) GenSpeedF               ! Filtered HSS (generator) speed, [rad/s].
		READ( Un, IOSTAT=ErrStat ) IntSpdErr               ! Current integral of speed error w.r.t. time, [rad].
		READ( Un, IOSTAT=ErrStat ) LastGenTrq              ! Commanded electrical generator torque the last time the controller was called, [Nm].
		READ( Un, IOSTAT=ErrStat ) LastTime                ! Last time this DLL was called, [s].
		READ( Un, IOSTAT=ErrStat ) LastTimePC              ! Last time the pitch  controller was called, [s].
		READ( Un, IOSTAT=ErrStat ) LastTimeVS              ! Last time the torque controller was called, [s].
		READ( Un, IOSTAT=ErrStat ) PitCom                  ! Commanded pitch of each blade the last time the controller was called, [rad].
		READ( Un, IOSTAT=ErrStat ) VS_Slope15              ! Torque/speed slope of region 1 1/2 cut-in torque ramp , [Nm/(rad/s)].
		READ( Un, IOSTAT=ErrStat ) VS_Slope25              ! Torque/speed slope of region 2 1/2 induction generator, [Nm/(rad/s)].
		READ( Un, IOSTAT=ErrStat ) VS_SySp                 ! Synchronous speed of region 2 1/2 induction generator, [rad/s].
		READ( Un, IOSTAT=ErrStat ) VS_TrGnSp               ! Transitional generator speed (HSS side) between regions 2 and 2 1/2, [rad/s].

		CLOSE ( Un )
	ENDIF

ENDIF

avcMSG = TRANSFER( TRIM(ErrMsg)//C_NULL_CHAR, avcMSG, SIZE(avcMSG) )

RETURN

END SUBROUTINE DISCON
