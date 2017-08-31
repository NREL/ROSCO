!-------------------------------------------------------------------------------------------------------------------------------
! Individual pitch control subroutine
SUBROUTINE IPC(rootMOOP, aziAngle, DT, KInter, KNotch, omegaLP, omegaNotch, phi, zetaLP, zetaNotch, iStatus, NumBl, PitComIPCF)
!...............................................................................................................................

	USE Filters

	IMPLICIT NONE


	!------------------------------------------------------------------------------------------------------------------------------
	! Variable declaration and initialization
	!------------------------------------------------------------------------------------------------------------------------------


		! Inputs

	REAL(4), INTENT(IN)		:: aziAngle							! Rotor azimuth angle
	REAL(4), INTENT(IN)		:: DT								! Time step
	REAL(4), INTENT(IN)		:: KInter							! Gain for the integrator
	REAL(4), INTENT(IN)		:: KNotch							! Gain for the notch filter
	REAL(4), INTENT(IN)		:: omegaLP							!
	REAL(4), INTENT(IN)		:: omegaNotch						!
	REAL(4), INTENT(IN)		:: phi								! Phase offset added to the azimuth angle
	REAL(4), INTENT(IN)		:: rootMOOP(3)						! Root out of plane bending moments of each blade
	REAL(4), INTENT(IN)		:: zetaLP							!
	REAL(4), INTENT(IN)		:: zetaNotch						!
	INTEGER, INTENT(IN)		:: iStatus							! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
	INTEGER, INTENT(IN)		:: NumBl							! Number of turbine blades

		! Outputs

	REAL(4), INTENT(OUT)	:: PitComIPCF(3)					! Filtered pitch angle of each rotor blade

		! Local variables

	REAL(4), PARAMETER		:: PI = 3.14159265359				! Mathematical constant pi
	REAL(4)					:: rootMOOPF (3), PitComIPC (3)		! 
	INTEGER					:: K								! Integer used to loop through turbine blades


	!------------------------------------------------------------------------------------------------------------------------------
	! Body
	!------------------------------------------------------------------------------------------------------------------------------


		! Filter rootMOOPs with notch filter

	DO K = 1,NumBl
		! Instances 1-3 of the Notch Filter are reserved for this routine.
		rootMOOPF(K) = rootMOOP(K)	! Notch filter currently not in use
		!rootMOOPF(K) = NotchFilter(rootMOOP(K), DT, KNotch, omegaNotch, zetaNotch, iStatus, K)
	END DO

		! Calculate commanded IPC pitch angles

	CALL CalculatePitCom(rootMOOPF, aziAngle, DT, KInter, phi, iStatus, PitComIPC)

		! Filter PitComIPC with second order low pass filter

	DO K = 1,NumBl
		! Instances 1-3 of the Second order Low-Pass Filter are reserved for this routine.
		PitComIPCF(K) = SecLPFilter(PitComIPC(K), DT, omegaLP, zetaLP, iStatus, K)
	END DO


	!------------------------------------------------------------------------------------------------------------------------------
	! Subroutines
	!------------------------------------------------------------------------------------------------------------------------------


CONTAINS
	!-------------------------------------------------------------------------------------------------------------------------------
	! Calculates the commanded pitch angles.
	! NOTE: if it is required for this subroutine to be used multiple times (for 1p and 2p IPC for example), the saved variables
	! IntAxisDirect and IntAxisQuadr need to be modified so that they support multiple instances (see LPFilter in the Filters module).
	SUBROUTINE CalculatePitCom(rootMOOP, aziAngle, DT, KInter, phi, iStatus, PitComIPC)
	!...............................................................................................................................

		IMPLICIT NONE

			! Inputs

		REAL(4), INTENT(IN)		:: rootMOOP (3)						! Root out of plane bending moments of each blade
		REAL(4), INTENT(IN)		:: aziAngle							! Rotor azimuth angle
		REAL(4), INTENT(IN)		:: DT								! The time step
		REAL(4), INTENT(IN)		:: KInter							! Integrator gain
		REAL(4), INTENT(IN)		:: phi								! Phase offset added to the azimuth angle
		INTEGER, INTENT(IN)		:: iStatus							! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.

			! Outputs

		REAL(4), INTENT(OUT)	:: PitComIPC(3)						! Commended pitch angle of each rotor blade

			! Local variables

		REAL(4)					:: axisDirect, axisQuadr			! Direct axis and quadrature axis outputted by Coleman transform
		REAL(4), SAVE			:: IntAxisDirect, IntAxisQuadr		! Integral of the direct axis and quadrature axis

			! Initialization

			! Set integrals to be 0 in the first time step

		IF(iStatus==0)  THEN
			IntAxisDirect = 0.0
			IntAxisQuadr = 0.0
		END IF

			! Body

			! Pass rootMOOPs through the Coleman transform to get the direct and quadrature axis

		CALL ColemanTransform(rootMOOP, aziAngle, axisDirect, axisQuadr)

			! Multiply with gain and take the integral

		IntAxisDirect	= IntAxisDirect + DT * KInter * axisDirect
		IntAxisQuadr	= IntAxisQuadr + DT * KInter * axisQuadr

			! Pass direct and quadrature axis through the inverse Coleman transform to get the commanded pitch angles

		CALL ColemanTransformInverse(IntAxisDirect, IntAxisQuadr, aziAngle, phi, PitComIPC)

	END SUBROUTINE CalculatePitCom
	!-------------------------------------------------------------------------------------------------------------------------------
	!The Coleman or d-q axis transformation transforms the root out of plane bending moments of each turbine blade
	!to a direct axis and a quadrature axis
	SUBROUTINE ColemanTransform(rootMOOP, aziAngle, axisDirect, axisQuadr)
	!...............................................................................................................................

		IMPLICIT NONE

			! Inputs

		REAL(4), INTENT(IN)		:: rootMOOP (3)						! Root out of plane bending moments of each blade
		REAL(4), INTENT(IN)		:: aziAngle							! Rotor azimuth angle

			! Outputs

		REAL(4), INTENT(OUT)	:: axisDirect, axisQuadr			! Direct axis and quadrature axis outputted by this transform

			! Local

		REAL(4), PARAMETER		:: phi2 = 2.0/3.0*PI				! Phase difference from first to second blade
		REAL(4), PARAMETER		:: phi3 = 4.0/3.0*PI    			! Phase difference from first to third blade

			! Body

		axisDirect	= 2.0/3.0 * (cos(aziAngle)*rootMOOP(1) + cos(aziAngle+phi2)*rootMOOP(2) + cos(aziAngle+phi3)*rootMOOP(3))
		axisQuadr	= 2.0/3.0 * (sin(aziAngle)*rootMOOP(1) + sin(aziAngle+phi2)*rootMOOP(2) + sin(aziAngle+phi3)*rootMOOP(3))

	END SUBROUTINE ColemanTransform
	!-------------------------------------------------------------------------------------------------------------------------------
	!The inverse Coleman or d-q axis transformation transforms the direct axis and quadrature axis
	!back to root out of plane bending moments of each turbine blade
	SUBROUTINE ColemanTransformInverse(axisDirect, axisQuadr, aziAngle, phi, PitComIPC)
	!...............................................................................................................................

		IMPLICIT NONE

			! Inputs

		REAL(4), INTENT(IN)		:: axisDirect, axisQuadr			! Direct axis and quadrature axis
		REAL(4), INTENT(IN)		:: aziAngle 						! Rotor azimuth angle
		REAL(4), INTENT(IN)		:: phi								! Phase shift added to the azimuth angle

			! Outputs

		REAL(4), INTENT(OUT)	:: PitComIPC (3)					! Root out of plane bending moments of each blade

			! Local

		REAL(4), PARAMETER		:: phi2 = 2.0/3.0*PI				! Phase difference from first to second blade
		REAL(4), PARAMETER		:: phi3 = 4.0/3.0*PI				! Phase difference from first to third blade

			! Body

		PitComIPC(1) = cos(aziAngle+phi)*axisDirect + sin(aziAngle+phi)*axisQuadr
		PitComIPC(2) = cos(aziAngle+phi+phi2)*axisDirect + sin(aziAngle+phi+phi2)*axisQuadr
		PitComIPC(3) = cos(aziAngle+phi+phi3)*axisDirect + sin(aziAngle+phi+phi3)*axisQuadr

	END SUBROUTINE ColemanTransformInverse
	!-------------------------------------------------------------------------------------------------------------------------------
END SUBROUTINE IPC
