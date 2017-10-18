MODULE ReadParameters

	USE DRC_Types
	IMPLICIT NONE
	
	TYPE(ControlParameters), SAVE	:: CntrPar
	TYPE(LocalVariables)			:: LocalVar
	
CONTAINS
	!..............................................................................................................................
	! Read all constant control parameters from ControllerParameters.in parameter file
	!..............................................................................................................................
	SUBROUTINE ReadControlParameterFileSub()

		INTEGER(4), PARAMETER		:: UnControllerParameters = 89
		OPEN(unit=UnControllerParameters, file='ControllerParameters.in', status='old', action='read')
		
		!------------------- GENERAL CONSTANTS -------------------
		READ(UnControllerParameters, *) CntrPar%CornerFreq
		READ(UnControllerParameters, *) CntrPar%LoggingLevel
		
		!------------------- IPC CONSTANTS -----------------------
		READ(UnControllerParameters, *) CntrPar%IPC_KI
		READ(UnControllerParameters, *) CntrPar%IPC_ControlMode
		READ(UnControllerParameters, *) CntrPar%IPC_omegaHP
		READ(UnControllerParameters, *) CntrPar%IPC_omegaLP
		READ(UnControllerParameters, *) CntrPar%IPC_omegaNotch
		READ(UnControllerParameters, *) CntrPar%IPC_phi
		READ(UnControllerParameters, *) CntrPar%IPC_zetaHP
		READ(UnControllerParameters, *) CntrPar%IPC_zetaLP
		READ(UnControllerParameters, *) CntrPar%IPC_zetaNotch
		
		!------------------- PITCH CONSTANTS -----------------------
		READ(UnControllerParameters, *) CntrPar%PC_GS_n
		
		ALLOCATE(CntrPar%PC_GS_angles(CntrPar%PC_GS_n))
		READ(UnControllerParameters,*) CntrPar%PC_GS_angles
		
		ALLOCATE(CntrPar%PC_GS_kp(CntrPar%PC_GS_n))
		READ(UnControllerParameters,*) CntrPar%PC_GS_kp
		
		ALLOCATE(CntrPar%PC_GS_ki(CntrPar%PC_GS_n))
		READ(UnControllerParameters,*) CntrPar%PC_GS_ki
		
		ALLOCATE(CntrPar%PC_GS_kd(CntrPar%PC_GS_n))
		READ(UnControllerParameters,*) CntrPar%PC_GS_kd
		
		ALLOCATE(CntrPar%PC_GS_tf(CntrPar%PC_GS_n))
		READ(UnControllerParameters,*) CntrPar%PC_GS_tf
		
		READ(UnControllerParameters, *) CntrPar%PC_MaxPit
		READ(UnControllerParameters, *) CntrPar%PC_MinPit
		READ(UnControllerParameters, *) CntrPar%PC_MaxRat
		READ(UnControllerParameters, *) CntrPar%PC_MinRat
		READ(UnControllerParameters, *) CntrPar%PC_RefSpd
		READ(UnControllerParameters, *) CntrPar%PC_SetPnt
		READ(UnControllerParameters, *) CntrPar%PC_Switch
		
		!------------------- TORQUE CONSTANTS -----------------------
		READ(UnControllerParameters, *) CntrPar%VS_ControlMode
		READ(UnControllerParameters, *) CntrPar%VS_CtInSp
		READ(UnControllerParameters, *) CntrPar%VS_GenTrqArSatMax
		READ(UnControllerParameters, *) CntrPar%VS_MaxOM
		READ(UnControllerParameters, *) CntrPar%VS_MaxRat
		READ(UnControllerParameters, *) CntrPar%VS_MaxTq
		READ(UnControllerParameters, *) CntrPar%VS_MinTq
		READ(UnControllerParameters, *) CntrPar%VS_MinOM
		READ(UnControllerParameters, *) CntrPar%VS_Rgn2K
		READ(UnControllerParameters, *) CntrPar%VS_RtPwr
		READ(UnControllerParameters, *) CntrPar%VS_RtTq
		READ(UnControllerParameters, *) CntrPar%VS_RtSpd
		READ(UnControllerParameters, *) CntrPar%VS_n
		
		ALLOCATE(CntrPar%VS_KP(CntrPar%VS_n))
		READ(UnControllerParameters,*) CntrPar%VS_KP
		
		ALLOCATE(CntrPar%VS_KI(CntrPar%VS_n))
		READ(UnControllerParameters,*) CntrPar%VS_KI
		
		!------------------- YAW CONSTANTS -----------------------
		READ(UnControllerParameters, *) CntrPar%Y_ControlMode
		READ(UnControllerParameters, *) CntrPar%Y_ErrThresh
		READ(UnControllerParameters, *) CntrPar%Y_IPC_n
		
		ALLOCATE(CntrPar%Y_IPC_KP(CntrPar%Y_IPC_n))
		READ(UnControllerParameters,*) CntrPar%Y_IPC_KP
		
		ALLOCATE(CntrPar%Y_IPC_KI(CntrPar%Y_IPC_n))
		READ(UnControllerParameters,*) CntrPar%Y_IPC_KI
		
		READ(UnControllerParameters, *) CntrPar%Y_MErrSet
		READ(UnControllerParameters, *) CntrPar%Y_omegaLPFast
		READ(UnControllerParameters, *) CntrPar%Y_omegaLPSlow
		READ(UnControllerParameters, *) CntrPar%Y_Rate
		
		!------------------- CALCULATED CONSTANTS -----------------------
		CntrPar%PC_RtTq99		= CntrPar%VS_RtTq*0.99
		CntrPar%VS_Rgn2MinTq	= CntrPar%VS_Rgn2K*CntrPar%VS_MinOM**2
		CntrPar%VS_Rgn2MaxTq	= CntrPar%VS_Rgn2K*CntrPar%VS_MaxOM**2
		CntrPar%VS_Rgn3MP		= CntrPar%PC_SetPnt + CntrPar%PC_Switch
		
		CLOSE(UnControllerParameters)
	END SUBROUTINE ReadControlParameterFileSub
	
	SUBROUTINE ReadAvrSWAP(avrSWAP)
	
		USE, INTRINSIC	:: ISO_C_Binding
		REAL(C_FLOAT), INTENT(INOUT)	:: avrSWAP(*)	! The swap array, used to pass data to, and receive data from, the DLL controller.
		
		! Load variables from calling program (See Appendix A of Bladed User's Guide):
		LocalVar%iStatus			= NINT(avrSWAP(1))
		LocalVar%Time				= avrSWAP(2)
		LocalVar%DT				= avrSWAP(3)
		LocalVar%BlPitch(1)		= avrSWAP(4)
		LocalVar%VS_GenPwr		= avrSWAP(15)
		LocalVar%GenSpeed			= avrSWAP(20)
		LocalVar%Y_M				= avrSWAP(24)
		LocalVar%HorWindV			= avrSWAP(27)
		LocalVar%rootMOOP(1)		= avrSWAP(30)
		LocalVar%rootMOOP(2)		= avrSWAP(31)
		LocalVar%rootMOOP(3)		= avrSWAP(32)
		LocalVar%BlPitch(2)		= avrSWAP(33)
		LocalVar%BlPitch(3)		= avrSWAP(34)
		LocalVar%Azimuth			= avrSWAP(60)
		LocalVar%NumBl			= NINT(avrSWAP(61))
	END SUBROUTINE ReadAvrSWAP
END MODULE ReadParameters