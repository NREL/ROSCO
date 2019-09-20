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
    SUBROUTINE StateMachine(CntrPar, LocalVar)
    ! State machine, determines the state of the wind turbine to specify the corresponding control actions
    ! PC States:
    !       PC_State = 0, No pitch control active, BldPitch = PC_MinPit
    !       PC_State = 1, Active PI blade pitch control enabled
    ! VS States
    !       VS_State = 0, Error state, for debugging purposes, GenTq = VS_RtTq
    !       VS_State = 1, Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
    !       VS_State = 2, Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
    !       VS_State = 3, Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
    !       VS_State = 4, above-rated operation using pitch control (constant torque mode)
    !       VS_State = 5, above-rated operation using pitch and torque control (constant power mode)
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
            ! --- Pitch controller state machine ---
            IF (CntrPar%PC_ControlMode == 1) THEN
                LocalVar%PC_State = 1
            ELSE 
                LocalVar%PC_State = 0
            END IF
            
            ! --- Torque control state machine ---
            IF (LocalVar%PC_PitComT >= CntrPar%VS_Rgn3Pitch) THEN           ! Region 3
                IF (CntrPar%VS_ControlMode == 1) THEN 
                    LocalVar%VS_State = 5 ! Constant power tracking
                ELSE 
                    LocalVar%VS_State = 4 ! Constant torque tracking
                END IF
            ELSE
                IF (LocalVar%GenArTq >= CntrPar%VS_MaxOMTq*1.01) THEN       ! Region 2 1/2 - active PI torque control
                    LocalVar%VS_State = 3                 
                ELSEIF (LocalVar%GenSpeedF < CntrPar%VS_RefSpd)  THEN       ! Region 2 - optimal torque is proportional to the square of the generator speed
                
                    LocalVar%VS_State = 2
                ELSEIF (LocalVar%GenBrTq <= CntrPar%VS_MinOMTq*0.99) THEN   ! Region 1 1/2
                
                    LocalVar%VS_State = 1
                ELSE                                                        ! Error state, Debug
                    LocalVar%VS_State = 0
                END IF
            END IF
        END IF
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst)
    ! Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
    !       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
    !       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 
        USE DRC_Types!, ONLY : LocalVariables, ControlParameters, Obj
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        ! Allocate Variables
        REAL(4)             :: F_WECornerFreq ! Corner frequency (-3dB point) for first order low pass filter for measured hub height wind speed [Hz]
        ! Define Variables
        F_WECornerFreq = 0.04  ! Fix to 20 second time constant for now
        
        ! Define wind speed estimate
        IF (CntrPar%WE_Mode == 1) THEN      
            ! Inversion and Invariance Filter implementation
            LocalVar%WE_VwIdot = CntrPar%WE_Gamma/CntrPar%WE_Jtot*(LocalVar%VS_LastGenTrq*CntrPar%WE_GearboxRatio - AeroDynTorque(LocalVar, CntrPar))
            LocalVar%WE_VwI = LocalVar%WE_VwI + LocalVar%WE_VwIdot*LocalVar%DT
            LocalVar%WE_Vw = LocalVar%WE_VwI + CntrPar%WE_Gamma*LocalVar%RotSpeed
        ELSE                                
            ! Filter wind speed at hub height as directly passed from OpenFAST
            LocalVar%WE_Vw = LPFilter(LocalVar%HorWindV, LocalVar%DT, F_WECornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF)
        END IF 

    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
    ! Setpoint smoother modifies controller reference in order to separate generator torque and blade pitch control actions
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement setpoint smoothing
        USE DRC_Types!, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        ! Allocate Variables
        Real(4)                      :: DelOmega                            ! Reference generator speed shift, rad/s.
        
        ! Setpoint Smoothing
        IF ( CntrPar%SS_Mode == 1) THEN
            ! Find setpoint shift amount
            DelOmega = (LocalVar%BlPitch(1) - CntrPar%PC_MinPit)*CntrPar%SS_VSGainBias - (CntrPar%VS_RtTq - LocalVar%VS_LastGenTrq)*CntrPar%SS_PCGainBias
            ! Filter
            LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, CntrPar%F_SSCornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF) 
        ELSE
            LocalVar%SS_DelOmegaF = 0 ! No setpoint smoothing
        ENDIF

    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks