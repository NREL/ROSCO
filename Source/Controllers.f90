MODULE Controllers

	USE, INTRINSIC	:: ISO_C_Binding
	USE FunctionToolbox
	USE Filters
	
	IMPLICIT NONE
	
CONTAINS	
	SUBROUTINE PitchControl(avrSWAP, CntrPar, LocalVar, objInst)
	
		USE DRC_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
	   
	   ! Local Variables:	
		REAL(C_FLOAT), INTENT(INOUT)	:: avrSWAP(*)	! The swap array, used to pass data to, and receive data from, the DLL controller.
		INTEGER(4)						:: K			! Loops through blades.
		
		TYPE(ControlParameters), INTENT(INOUT)	:: CntrPar
		TYPE(LocalVariables), INTENT(INOUT)		:: LocalVar
		TYPE(ObjectInstances), INTENT(INOUT)	:: objInst
	
		!..............................................................................................................................
		! Pitch control
		!..............................................................................................................................
		! Set the pitch override to yes
		avrSWAP(55) = 0.0						! Pitch override: 0=yes
		
		IF (CntrPar%VS_ControlMode == 0 .AND. LocalVar%GenTrq >= CntrPar%PC_RtTq99) THEN
			LocalVar%PC_MaxPitVar = CntrPar%PC_MaxPit
		ELSEIF (CntrPar%VS_ControlMode == 1 .AND. LocalVar%GenTrqAr >= CntrPar%VS_GenTrqArSatMax*0.99) THEN
			LocalVar%PC_MaxPitVar = CntrPar%PC_MaxPit
		ELSE
			LocalVar%PC_MaxPitVar = CntrPar%PC_SetPnt
		END IF
		
		! Compute the gain scheduling correction factor based on the previously
		! commanded pitch angle for blade 1:
		LocalVar%PC_KP = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_kp, LocalVar%PC_PitComT)
		LocalVar%PC_KI = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_ki, LocalVar%PC_PitComT)
		LocalVar%PC_KD = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_kd, LocalVar%PC_PitComT)
		LocalVar%PC_TF = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_tf, LocalVar%PC_PitComT)
	
		! Compute the current speed error and its integral w.r.t. time; saturate the
		! integral term using the pitch angle limits:
		LocalVar%PC_SpdErr = CntrPar%PC_RefSpd - LocalVar%GenSpeedF					! Speed error
		LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr					! Power error
		LocalVar%Y_MErr = LocalVar%Y_M + CntrPar%Y_MErrSet							! Yaw-alignment error
			
		! Compute the pitch commands associated with the proportional and integral
		!   gains:
		LocalVar%PC_PitComT = PIController(LocalVar%PC_SpdErr, LocalVar%PC_KP, LocalVar%PC_KI, CntrPar%PC_SetPnt, LocalVar%PC_MaxPitVar, LocalVar%DT, CntrPar%PC_SetPnt, .FALSE., 2) ! + DFController(LocalVar%PC_SpdErr, LocalVar%PC_KD, LocalVar%PC_TF, LocalVar%DT, 1)
		IF (CntrPar%VS_ControlMode == 1) THEN
			LocalVar%PC_PitComT = LocalVar%PC_PitComT + PIController(LocalVar%PC_PwrErr, CntrPar%PC_ConstP_KP(1), CntrPar%PC_ConstP_KI(1), CntrPar%PC_SetPnt, LocalVar%PC_MaxPitVar, LocalVar%DT, CntrPar%PC_SetPnt, .FALSE., 5)
		END IF
		
		! Individual pitch control
		IF ((CntrPar%IPC_ControlMode == 1) .OR. (CntrPar%Y_ControlMode == 2)) THEN
			CALL IPC(CntrPar, LocalVar, objInst)
		ELSE
			LocalVar%IPC_PitComF = 0.0 ! THIS IS AN ARRAY!!
		END IF
	
		! Combine and saturate all pitch commands:
		DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
			LocalVar%PC_PitComT_IPC(K) = LocalVar%PC_PitComT + LocalVar%IPC_PitComF(K)									! Add the individual pitch command
			LocalVar%PC_PitComT_IPC(K) = saturate(LocalVar%PC_PitComT_IPC(K), CntrPar%PC_MinPit, CntrPar%PC_MaxPit)				! Saturate the overall command using the pitch angle limits
			
			! PitCom(K) = ratelimit(LocalVar%PC_PitComT_IPC(K), LocalVar%BlPitch(K), PC_MinRat, PC_MaxRat, LocalVar%DT)	! Saturate the overall command of blade K using the pitch rate limit
			LocalVar%PitCom(K) = saturate(LocalVar%PC_PitComT_IPC(K), CntrPar%PC_MinPit, CntrPar%PC_MaxPit)					! Saturate the overall command using the pitch angle limits
			LocalVar%PitCom(K) = LPFilter(LocalVar%PitCom(K), LocalVar%DT, CntrPar%CornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF)
		END DO
		
		! Command the pitch demanded from the last
		! call to the controller (See Appendix A of Bladed User's Guide):
		avrSWAP(42) = LocalVar%PitCom(1)		! Use the command angles of all blades if using individual pitch
		avrSWAP(43) = LocalVar%PitCom(2)		! "
		avrSWAP(44) = LocalVar%PitCom(3)		! "
		avrSWAP(45) = LocalVar%PitCom(1)		! Use the command angle of blade 1 if using collective pitch
	END SUBROUTINE PitchControl
	
	SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst)
	
		USE DRC_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
	
		REAL(C_FLOAT), INTENT(INOUT)				:: avrSWAP(*)	! The swap array, used to pass data to, and receive data from, the DLL controller.
	
		TYPE(ControlParameters), INTENT(INOUT)	:: CntrPar
		TYPE(LocalVariables), INTENT(INOUT)	:: LocalVar
		TYPE(ObjectInstances), INTENT(INOUT)	:: objInst
		
		!..............................................................................................................................
		! VARIABLE-SPEED TORQUE CONTROL:
		!..............................................................................................................................
		avrSWAP(35) = 1.0          ! Generator contactor status: 1=main (high speed) variable-speed generator
		avrSWAP(56) = 0.0          ! Torque override: 0=yes
		
		! Filter the HSS (generator) speed measurement:
		! Apply Low-Pass Filter
		LocalVar%GenSpeedF = SecLPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%CornerFreq, 0.7, LocalVar%iStatus, .FALSE., objInst%instSecLPF)     ! This is the first instance of a second order LPFilter
		
		! Compute the generator torque, which depends on which region we are in:
			
		LocalVar%VS_SpdErrAr = CntrPar%VS_RtSpd - LocalVar%GenSpeedF					! Current speed error - Above-rated PI-control
		LocalVar%VS_SpdErrBr = CntrPar%VS_MinOM - LocalVar%GenSpeedF					! Current speed error - Below-rated PI-control
		IF (LocalVar%PC_PitComT >= CntrPar%VS_Rgn3MP) THEN						! We are in region 3
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
		IF (LocalVar%iStatus == 0)  LocalVar%VS_LastGenTrq = LocalVar%GenTrq				! Initialize the value of LocalVar%VS_LastGenTrq on the first pass only
		LocalVar%GenTrq = ratelimit(LocalVar%GenTrq, LocalVar%VS_LastGenTrq, -CntrPar%VS_MaxRat, CntrPar%VS_MaxRat, LocalVar%DT)	! Saturate the command using the torque rate limit
	
		! Reset the value of LocalVar%VS_LastGenTrq to the current values:
		LocalVar%VS_LastGenTrq = LocalVar%GenTrq
	
		! Set the generator contactor status, avrSWAP(35), to main (high speed)
		! variable-speed generator, the torque override to yes, and command the
		! generator torque (See Appendix A of Bladed User's Guide):
		avrSWAP(47) = LocalVar%VS_LastGenTrq   ! Demanded generator torque
	END SUBROUTINE VariableSpeedControl
	
	SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst)
	
		USE DRC_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
	
		REAL(C_FLOAT), INTENT(INOUT)				:: avrSWAP(*)	! The swap array, used to pass data to, and receive data from, the DLL controller.
	
		TYPE(ControlParameters), INTENT(INOUT)	:: CntrPar
		TYPE(LocalVariables), INTENT(INOUT)	:: LocalVar
		TYPE(ObjectInstances), INTENT(INOUT)	:: objInst
		
		!..............................................................................................................................
		! Yaw control
		!..............................................................................................................................
		
		IF (CntrPar%Y_ControlMode == 1) THEN
			avrSWAP(29) = 0									! Yaw control parameter: 0 = yaw rate control
			IF (LocalVar%Time >= LocalVar%Y_YawEndT) THEN											! Check if the turbine is currently yawing
				avrSWAP(48) = 0.0													! Set yaw rate to zero
	
				LocalVar%Y_ErrLPFFast = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPFast, LocalVar%iStatus, .FALSE., objInst%instLPF)		! Fast low pass filtered yaw error with a frequency of 1
				LocalVar%Y_ErrLPFSlow = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPSlow, LocalVar%iStatus, .FALSE., objInst%instLPF)		! Slow low pass filtered yaw error with a frequency of 1/60
	
				LocalVar%Y_AccErr = LocalVar%Y_AccErr + LocalVar%DT*SIGN(LocalVar%Y_ErrLPFFast**2, LocalVar%Y_ErrLPFFast)	! Integral of the fast low pass filtered yaw error
	
				IF (ABS(LocalVar%Y_AccErr) >= CntrPar%Y_ErrThresh) THEN								! Check if accumulated error surpasses the threshold
					LocalVar%Y_YawEndT = ABS(LocalVar%Y_ErrLPFSlow/CntrPar%Y_Rate) + LocalVar%Time					! Yaw to compensate for the slow low pass filtered error
				END IF
			ELSE
				avrSWAP(48) = SIGN(CntrPar%Y_Rate, LocalVar%Y_MErr)		! Set yaw rate to predefined yaw rate, the sign of the error is copied to the rate
				LocalVar%Y_ErrLPFFast = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPFast, LocalVar%iStatus, .TRUE., objInst%instLPF)		! Fast low pass filtered yaw error with a frequency of 1
				LocalVar%Y_ErrLPFSlow = LPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%Y_omegaLPSlow, LocalVar%iStatus, .TRUE., objInst%instLPF)		! Slow low pass filtered yaw error with a frequency of 1/60
				LocalVar%Y_AccErr = 0.0								! "
			END IF
		END IF
	END SUBROUTINE YawRateControl
	
	SUBROUTINE IPC(CntrPar, LocalVar, objInst)
		!-------------------------------------------------------------------------------------------------------------------------------
		! Individual pitch control subroutine
		!
		! Variable declaration and initialization
		!------------------------------------------------------------------------------------------------------------------------------
		USE DRC_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
		
		! Local variables
		REAL(4)					:: PitComIPC(3)
		INTEGER(4)				:: K								! Integer used to loop through turbine blades
		REAL(4)					:: axisTilt, axisYaw, axisYawF		! Direct axis and quadrature axis outputted by Coleman transform
		REAL(4), SAVE			:: IntAxisTilt, IntAxisYaw			! Integral of the direct axis and quadrature axis
		REAL(4)					:: IntAxisYawIPC					! IPC contribution with yaw-by-IPC component
		REAL(4)					:: Y_MErrF, Y_MErrF_IPC				! Unfiltered and filtered yaw alignment error [rad]
		REAL(4)					:: PitComIPC_woYaw(3)
	
		TYPE(ControlParameters), INTENT(INOUT)	:: CntrPar
		TYPE(LocalVariables), INTENT(INOUT)		:: LocalVar
		TYPE(ObjectInstances), INTENT(INOUT)	:: objInst
		
		!------------------------------------------------------------------------------------------------------------------------------
		! Body
		!------------------------------------------------------------------------------------------------------------------------------
		! Calculates the commanded pitch angles.
		! NOTE: if it is required for this subroutine to be used multiple times (for 1p and 2p IPC for example), the saved variables
		! IntAxisTilt and IntAxisYaw need to be modified so that they support multiple instances (see LPFilter in the Filters module).
		!------------------------------------------------------------------------------------------------------------------------------
			! Filter rootMOOPs with notch filter
	
		!DO K = 1,LocalVar%NumBl
			! Instances 1-3 of the Notch Filter are reserved for this routine.
		!	rootMOOPF(K) = LocalVar%rootMOOP(K)	! Notch filter currently not in use
		!END DO
	
			! Initialization
				! Set integrals to be 0 in the first time step
	
		IF(LocalVar%iStatus==0)  THEN
			IntAxisTilt = 0.0
			IntAxisYaw = 0.0
		END IF
	
		! Pass rootMOOPs through the Coleman transform to get the direct and quadrature axis
		CALL ColemanTransform(LocalVar%rootMOOP, LocalVar%Azimuth, axisTilt, axisYaw)
	
		! High-pass filter the MBC yaw component and filter yaw alignment error, and compute the yaw-by-IPC contribution
		IF (CntrPar%Y_ControlMode == 2) THEN
			axisYawF = HPFilter(axisYaw, LocalVar%DT, CntrPar%IPC_omegaHP, LocalVar%iStatus, .FALSE., objInst%instHPF)
			Y_MErrF = SecLPFilter(LocalVar%Y_MErr, LocalVar%DT, CntrPar%IPC_omegaLP, CntrPar%IPC_zetaLP, LocalVar%iStatus, .FALSE., objInst%instSecLPF)
			Y_MErrF_IPC = PIController(Y_MErrF, CntrPar%Y_IPC_KP(1), CntrPar%Y_IPC_KI(1), -CntrPar%Y_IPC_IntSat, CntrPar%Y_IPC_IntSat, LocalVar%DT, 0.0, .FALSE., 3)
		ELSE
			axisYawF = axisYaw
			Y_MErrF = 0.0
		END IF
		
		! Integrate the signal and multiply with the IPC gain
		IF (CntrPar%IPC_ControlMode == 1) THEN
			IntAxisTilt	= IntAxisTilt + LocalVar%DT * CntrPar%IPC_KI * axisTilt
			IntAxisYaw = IntAxisYaw + LocalVar%DT * CntrPar%IPC_KI * axisYawF
			IntAxisTilt = saturate(IntAxisTilt, -CntrPar%IPC_IntSat, CntrPar%IPC_IntSat)
			IntAxisYaw = saturate(IntAxisYaw, -CntrPar%IPC_IntSat, CntrPar%IPC_IntSat)
		ELSE
			IntAxisTilt = 0.0
			IntAxisYaw = 0.0
		END IF
		
		! Add the yaw-by-IPC contribution
		IntAxisYawIPC = IntAxisYaw + Y_MErrF_IPC
	
		! Pass direct and quadrature axis through the inverse Coleman transform to get the commanded pitch angles
		CALL ColemanTransformInverse(IntAxisTilt, IntAxisYawIPC, LocalVar%Azimuth, CntrPar%IPC_phi, PitComIPC)
	
		! Filter PitComIPC with second order low pass filter
		DO K = 1,LocalVar%NumBl
			! Instances 1-3 of the Second order Low-Pass Filter are reserved for this routine.
			! LocalVar%IPC_PitComF(K) = SecLPFilter(PitComIPC(K), LocalVar%DT, CntrPar%IPC_omegaLP, CntrPar%IPC_zetaLP, LocalVar%iStatus, K)
			LocalVar%IPC_PitComF(K) = PitComIPC(K)
		END DO
	END SUBROUTINE IPC
END MODULE Controllers