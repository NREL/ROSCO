! This module contains additional blocks to supplement the primary controllers used in the Controllers module
!
! Many of these have I/O flags as a part of the DISCON input file 
!
! Blocks (Subroutines):
!       State Machine: determine the state of the wind turbine to specify the corresponding control actions
!       WindSpeedEstimator: Estimate wind speed
!       SetpointSmoother: Modify generator torque and blade pitch controller setpoints in transition region

MODULE ControllerBlocks

USE Constants
USE Filters
USE Functions

IMPLICIT NONE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    ! State machines, determines the state of the wind turbine to specify the corresponding control actions
    ! States:
    ! - VS/PC_State = 0, Error state, for debugging purposes (VS) / No pitch control active, pitch constant at fine-pitch (PC)
    ! - VS_State = 1, Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
    ! - VS_State = 2, Region 2, operation, maximum rotor power efficiency (Cp-max) tracking, keep TSR constant at a fixed fine-pitch angle
    ! - VS_State = 3, Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
    ! - VS_State = 4 + PC_State = 1, above-rated operation using pitch control (constant torque mode)
    ! - VS_State = 5 + PC_State = 2, above-rated operation using pitch and torque control (constant power mode)
    SUBROUTINE StateMachine(CntrPar, LocalVar)
        USE DRC_Types, ONLY : LocalVariables, ControlParameters
        IMPLICIT NONE
    
            ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
        
        ! Initialize State machine if first call
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL

            IF (LocalVar%PitCom(1) >= CntrPar%VS_Rgn3Pitch) THEN ! We are in region 3
                IF (CntrPar%VS_ControlMode == 1) THEN ! Constant power tracking
                    LocalVar%VS_State = 5
                    LocalVar%PC_State = 2
                ELSE ! Constant torque tracking
                    LocalVar%VS_State = 4
                    LocalVar%PC_State = 1
                END IF
            ELSE ! We are in Region 2
                LocalVar%VS_State = 2
                LocalVar%PC_State = 0
            END IF

        ! Operational States
        ELSE
            ! Pitch controller state machine
            IF (CntrPar%PC_ControlMode == 1) THEN
                LocalVar%PC_State = 1
            ELSE 
                LocalVar%PC_State = 0
            END IF
            
            ! Torque control state machine
            IF (LocalVar%PC_PitComT >= CntrPar%VS_Rgn3Pitch) THEN 
                ! Region 3
                IF (CntrPar%VS_ControlMode == 1) THEN ! Constant power tracking
                    LocalVar%VS_State = 5
                ELSE ! Constant torque tracking
                    LocalVar%VS_State = 4
                END IF
            ELSE
                ! Region 2 1/2 - active PI torque control
                IF (LocalVar%GenArTq >= CntrPar%VS_MaxOMTq*1.01) THEN 
                    LocalVar%VS_State = 3
                ! Region 1 1/2
                ELSEIF (LocalVar%GenBrTq <= CntrPar%VS_MinOMTq*0.99) THEN 
                    LocalVar%VS_State = 1
                ! Region 2 - optimal torque is proportional to the square of the generator speed
                ELSEIF (LocalVar%GenSpeedF < CntrPar%VS_RefSpd)  THEN 
                    LocalVar%VS_State = 2
                ! Error state, Debug
                ELSE 
                    LocalVar%VS_State = 0
                END IF
            END IF
        END IF
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar)
        USE DRC_Types, ONLY : LocalVariables, ControlParameters
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN) :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar 
        
        ! Body
        LocalVar%WE_VwIdot = CntrPar%WE_Gamma/CntrPar%WE_Jtot*(LocalVar%VS_LastGenTrq*CntrPar%WE_GearboxRatio - AeroDynTorque(LocalVar, CntrPar))
        
        LocalVar%WE_VwI = LocalVar%WE_VwI + LocalVar%WE_VwIdot*LocalVar%DT
        LocalVar%WE_Vw = LocalVar%WE_VwI + CntrPar%WE_Gamma*LocalVar%RotSpeed
        
    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
        USE DRC_Types!, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst

        Real(4), PARAMETER           :: SS_Mode = 1                         ! Gain Bias Mode, 0 = no gain bais, 1 = gain bias-.
        Real(4), PARAMETER           :: SS_VSGainBias   = 30                ! Variable speed torque controller gain bias, (rad/s)/(rad).
        Real(4), PARAMETER           :: SS_PCGainBias   = 0.0001            ! Collective pitch controller gain bias, (rad/s)/(Nm).
        Real(4), PARAMETER           :: F_SSCornerFreq = 0.1                ! Cornering frequency of first order low pass filter for the gain bias signal, Hz.
        Real(4)                      :: DelOmega                            ! Reference generator speed shift, rad/s.
        
        ! Setpoint Smoothing
        IF ( SS_Mode == 1) THEN
            ! Find setpoint shift amount
            DelOmega = (LocalVar%BlPitch(1) - CntrPar%PC_MinPit)*SS_VSGainBias - (CntrPar%VS_RtTq - LocalVar%VS_LastGenTrq)*SS_PCGainBias
            ! Filter
            LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, F_SSCornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF) 
        ELSE
            LocalVar%SS_DelOmegaF = 0
        ENDIF

    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks