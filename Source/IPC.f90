!-------------------------------------------------------------------------------------------------------------------------------
! Individual pitch control subroutine
SUBROUTINE IPC(rootMOOP, aziAngle, phi, Y_MErr, DT, KInter, Y_IPC_KP, Y_IPC_KI, omegaHP, omegaLP, omegaNotch, zetaHP, zetaLP, zetaNotch, iStatus, IPC_ControlMode, Y_ControlMode, NumBl, PitComIPCF, objInst)
!...............................................................................................................................

	USE :: FunctionToolbox
	USE :: Filters
	USE DRC_Types, ONLY : ObjectInstances

	IMPLICIT NONE

	!------------------------------------------------------------------------------------------------------------------------------
	! Variable declaration and initialization
	!------------------------------------------------------------------------------------------------------------------------------

		! Inputs

	REAL(4), INTENT(IN)		:: aziAngle							! Rotor azimuth angle
	REAL(4), INTENT(IN)		:: DT								! Time step
	REAL(4), INTENT(IN)		:: KInter							! Gain for the integrator
	REAL(4), INTENT(IN)		:: omegaHP							! High-pass filter cut-in frequency
	REAL(4), INTENT(IN)		:: omegaLP							! Low-pass filter cut-off frequency
	REAL(4), INTENT(IN)		:: omegaNotch						! Notch filter frequency
	REAL(4), INTENT(IN)		:: phi								! Phase offset added to the azimuth angle
	REAL(4), INTENT(IN)		:: rootMOOP(3)						! Root out of plane bending moments of each blade
	REAL(4), INTENT(IN)		:: Y_MErr							! Yaw alignment error, measured [rad]
	REAL(4), INTENT(IN)		:: zetaHP							! High-pass filter damping value
	REAL(4), INTENT(IN)		:: zetaLP							! Low-pass filter damping value
	REAL(4), INTENT(IN)		:: zetaNotch						! Notch filter damping value
	INTEGER(4), INTENT(IN)	:: iStatus							! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
	INTEGER(4), INTENT(IN)	:: NumBl							! Number of turbine blades
	INTEGER(4), INTENT(IN)	:: Y_ControlMode					! Yaw control mode
	REAL(4), INTENT(IN)		:: Y_IPC_KP							! Yaw-by-IPC proportional controller gain Kp
	REAL(4), INTENT(IN)		:: Y_IPC_KI							! Yaw-by-IPC integral controller gain Ki
	INTEGER(4), INTENT(IN)	:: IPC_ControlMode					! Turn Individual Pitch Control (IPC) for fatigue load reductions (pitch contribution) on = 1/off = 0

		! Outputs

	REAL(4), INTENT(OUT)	:: PitComIPCF(3)					! Filtered pitch angle of each rotor blade
	
		! Inputs/outputs
		
	TYPE(ObjectInstances), INTENT(INOUT)	:: objInst

		! Local variables

	REAL(4), PARAMETER		:: PI = 3.14159265359				! Mathematical constant pi
	REAL(4)					:: rootMOOPF(3), PitComIPC(3)		! 
	INTEGER(4)				:: K								! Integer used to loop through turbine blades
	REAL(4)					:: axisTilt, axisYaw, axisYawF		! Direct axis and quadrature axis outputted by Coleman transform
	REAL(4), SAVE			:: IntAxisTilt, IntAxisYaw			! Integral of the direct axis and quadrature axis
	REAL(4)					:: IntAxisYawIPC					! IPC contribution with yaw-by-IPC component
	REAL(4)					:: Y_MErrF, Y_MErrF_IPC				! Unfiltered and filtered yaw alignment error [rad]
	REAL(4)					:: PitComIPC_woYaw(3)

	!------------------------------------------------------------------------------------------------------------------------------
	! Body
	!------------------------------------------------------------------------------------------------------------------------------
	! Calculates the commanded pitch angles.
	! NOTE: if it is required for this subroutine to be used multiple times (for 1p and 2p IPC for example), the saved variables
	! IntAxisTilt and IntAxisYaw need to be modified so that they support multiple instances (see LPFilter in the Filters module).
	!------------------------------------------------------------------------------------------------------------------------------
		! Filter rootMOOPs with notch filter

	DO K = 1,NumBl
		! Instances 1-3 of the Notch Filter are reserved for this routine.
		rootMOOPF(K) = rootMOOP(K)	! Notch filter currently not in use
		!rootMOOPF(K) = WHICHNOTCHFILTERFUNCTION(rootMOOP(K), DT, omegaNotch, zetaNotch, iStatus, K) !! CHECK
	END DO

		! Calculate commanded IPC pitch angles
	!CALL CalculatePitCom(rootMOOPF, aziAngle, Y_MErr, DT, KInter, Y_IPC_KP, Y_IPC_KI, omegaHP, omegaLP, zetaHP, zetaLP, phi, iStatus, IPC_ControlMode, Y_ControlMode, PitComIPC, objInst)

		! Initialization
			! Set integrals to be 0 in the first time step

	IF(iStatus==0)  THEN
		IntAxisTilt = 0.0
		IntAxisYaw = 0.0
	END IF

		! Body
			! Pass rootMOOPs through the Coleman transform to get the direct and quadrature axis

	CALL ColemanTransform(rootMOOP, aziAngle, axisTilt, axisYaw)

		! High-pass filter the MBC yaw component and filter yaw alignment error, and compute the yaw-by-IPC contribution
	
	IF (Y_ControlMode == 2) THEN
		axisYawF = HPFilter(axisYaw, DT, omegaHP, iStatus, .FALSE., objInst%instHPF)
		Y_MErrF = SecLPFilter(Y_MErr, DT, omegaLP, zetaLP, iStatus, .FALSE., objInst%instSecLPF)
		Y_MErrF_IPC = PIController(Y_MErrF, Y_IPC_KP, Y_IPC_KI, -100.0, 100.0, DT, 0.0, .FALSE., 3)
	ELSE
		axisYawF = axisYaw
		Y_MErrF = 0.0
	END IF
	
		! Integrate the signal and multiply with the IPC gain
	IF (IPC_ControlMode == 1) THEN
		IntAxisTilt	= IntAxisTilt + DT * KInter * axisTilt
		IntAxisYaw	= IntAxisYaw + DT * KInter * axisYawF
	ELSE
		IntAxisTilt = 0.0
		IntAxisYaw = 0.0
	END IF
	
		! Add the yaw-by-IPC contribution

	IntAxisYawIPC = IntAxisYaw + Y_MErrF_IPC

		! Pass direct and quadrature axis through the inverse Coleman transform to get the commanded pitch angles
	
	CALL ColemanTransformInverse(IntAxisTilt, IntAxisYawIPC, aziAngle, phi, PitComIPC)

	! Filter PitComIPC with second order low pass filter

	DO K = 1,NumBl
		! Instances 1-3 of the Second order Low-Pass Filter are reserved for this routine.
		! PitComIPCF(K) = SecLPFilter(PitComIPC(K), DT, omegaLP, zetaLP, iStatus, K)
		PitComIPCF(K) = PitComIPC(K)
	END DO

CONTAINS

	!-------------------------------------------------------------------------------------------------------------------------------
	!The Coleman or d-q axis transformation transforms the root out of plane bending moments of each turbine blade
	!to a direct axis and a quadrature axis
	SUBROUTINE ColemanTransform(rootMOOP, aziAngle, axisTilt, axisYaw)
	!...............................................................................................................................

		IMPLICIT NONE

			! Inputs

		REAL(4), INTENT(IN)		:: rootMOOP(3)						! Root out of plane bending moments of each blade
		REAL(4), INTENT(IN)		:: aziAngle							! Rotor azimuth angle

			! Outputs

		REAL(4), INTENT(OUT)	:: axisTilt, axisYaw				! Direct axis and quadrature axis outputted by this transform

			! Local

		REAL(4), PARAMETER		:: phi2 = 2.0/3.0*PI				! Phase difference from first to second blade
		REAL(4), PARAMETER		:: phi3 = 4.0/3.0*PI				! Phase difference from first to third blade

			! Body

		axisTilt	= 2.0/3.0 * (cos(aziAngle)*rootMOOP(1) + cos(aziAngle+phi2)*rootMOOP(2) + cos(aziAngle+phi3)*rootMOOP(3))
		axisYaw		= 2.0/3.0 * (sin(aziAngle)*rootMOOP(1) + sin(aziAngle+phi2)*rootMOOP(2) + sin(aziAngle+phi3)*rootMOOP(3))

	END SUBROUTINE ColemanTransform
	!-------------------------------------------------------------------------------------------------------------------------------
	!The inverse Coleman or d-q axis transformation transforms the direct axis and quadrature axis
	!back to root out of plane bending moments of each turbine blade
	SUBROUTINE ColemanTransformInverse(axisTilt, axisYaw, aziAngle, phi, PitComIPC)
	!...............................................................................................................................

		IMPLICIT NONE

			! Inputs

		REAL(4), INTENT(IN)		:: axisTilt, axisYaw			! Direct axis and quadrature axis
		REAL(4), INTENT(IN)		:: aziAngle 						! Rotor azimuth angle
		REAL(4), INTENT(IN)		:: phi								! Phase shift added to the azimuth angle

			! Outputs

		REAL(4), INTENT(OUT)	:: PitComIPC (3)					! Root out of plane bending moments of each blade

			! Local

		REAL(4), PARAMETER		:: phi2 = 2.0/3.0*PI				! Phase difference from first to second blade
		REAL(4), PARAMETER		:: phi3 = 4.0/3.0*PI				! Phase difference from first to third blade

			! Body

		PitComIPC(1) = cos(aziAngle+phi)*axisTilt + sin(aziAngle+phi)*axisYaw
		PitComIPC(2) = cos(aziAngle+phi+phi2)*axisTilt + sin(aziAngle+phi+phi2)*axisYaw
		PitComIPC(3) = cos(aziAngle+phi+phi3)*axisTilt + sin(aziAngle+phi+phi3)*axisYaw

	END SUBROUTINE ColemanTransformInverse
	!-------------------------------------------------------------------------------------------------------------------------------
END SUBROUTINE IPC
