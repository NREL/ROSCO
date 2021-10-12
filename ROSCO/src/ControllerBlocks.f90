! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains additional routines and functions to supplement the primary controllers used in the Controllers module

MODULE ControllerBlocks

USE, INTRINSIC :: ISO_C_Binding
USE Constants
USE Filters
USE Functions

IMPLICIT NONE

CONTAINS
! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE Constants
        ! Allocate variables
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst

        REAL(8)                                         :: VS_RefSpd        ! Referece speed for variable speed torque controller, [rad/s] 
        REAL(8)                                         :: PC_RefSpd        ! Referece speed for pitch controller, [rad/s] 
        REAL(8)                                         :: Omega_op         ! Optimal TSR-tracking generator speed, [rad/s]

        ! ----- Calculate yaw misalignment error -----
        LocalVar%Y_MErr = LocalVar%Y_M + CntrPar%Y_MErrSet ! Yaw-alignment error
        
        ! ----- Pitch controller speed and power error -----
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF < 0) THEN
            PC_RefSpd = CntrPar%PC_RefSpd - LocalVar%SS_DelOmegaF
        ELSE
            PC_RefSpd = CntrPar%PC_RefSpd
        ENDIF

        LocalVar%PC_SpdErr = PC_RefSpd - LocalVar%GenSpeedF            ! Speed error
        LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr             ! Power error
        
        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]
        IF ((CntrPar%VS_ControlMode == 2) .OR. (CntrPar%VS_ControlMode == 3)) THEN
            VS_RefSpd = (CntrPar%VS_TSRopt * LocalVar%We_Vw_F / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio
            VS_RefSpd = saturate(VS_RefSpd,CntrPar%VS_MinOMSpd, CntrPar%VS_RefSpd)
        ELSE
            VS_RefSpd = CntrPar%VS_RefSpd
        ENDIF 
        
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF > 0) THEN
            VS_RefSpd = VS_RefSpd - LocalVar%SS_DelOmegaF
        ENDIF

        ! Force zero torque in shutdown mode
        IF (LocalVar%SD) THEN
            VS_RefSpd = CntrPar%VS_MinOMSpd
        ENDIF

        ! Force minimum rotor speed
        VS_RefSpd = max(VS_RefSpd, CntrPar%VS_MinOmSpd)

        ! TSR-tracking reference error
        IF ((CntrPar%VS_ControlMode == 2) .OR. (CntrPar%VS_ControlMode == 3)) THEN
            LocalVar%VS_SpdErr = VS_RefSpd - LocalVar%GenSpeedF
        ENDIF

        ! Define transition region setpoint errors
        LocalVar%VS_SpdErrAr = VS_RefSpd - LocalVar%GenSpeedF               ! Current speed error - Region 2.5 PI-control (Above Rated)
        LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF     ! Current speed error - Region 1.5 PI-control (Below Rated)
        
        ! Region 3 minimum pitch angle for state machine
        LocalVar%VS_Rgn3Pitch = LocalVar%PC_MinPit + CntrPar%PC_Switch

    END SUBROUTINE ComputeVariablesSetpoints
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
    !       VS_State = 6, Tip-Speed-Ratio tracking PI controller
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        
        ! Initialize State machine if first call
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL

            IF (LocalVar%PitCom(1) >= LocalVar%VS_Rgn3Pitch) THEN ! We are in region 3
                IF (CntrPar%VS_ControlMode == 1) THEN ! Constant power tracking
                    LocalVar%VS_State = 5
                    LocalVar%PC_State = 1
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
            IF (LocalVar%PC_PitComT >= LocalVar%VS_Rgn3Pitch) THEN       

                IF (CntrPar%VS_ControlMode == 1) THEN                   ! Region 3
                    LocalVar%VS_State = 5 ! Constant power tracking
                ELSE 
                    LocalVar%VS_State = 4 ! Constant torque tracking
                END IF
            ELSE
                IF (LocalVar%GenArTq >= CntrPar%VS_MaxOMTq*1.01) THEN       ! Region 2 1/2 - active PI torque control
                    LocalVar%VS_State = 3                 
                ELSEIF ((LocalVar%GenSpeedF < CntrPar%VS_RefSpd) .AND. &
                        (LocalVar%GenBrTq >= CntrPar%VS_MinOMTq)) THEN       ! Region 2 - optimal torque is proportional to the square of the generator speed
                    LocalVar%VS_State = 2
                ELSEIF (LocalVar%GenBrTq < CntrPar%VS_MinOMTq) THEN   ! Region 1 1/2
                
                    LocalVar%VS_State = 1
                ELSE                                                        ! Error state, Debug
                    LocalVar%VS_State = 0
                END IF
            END IF
        END IF
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    ! Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
    !       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
    !       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, DebugVariables, ErrorVariables
        USE ieee_arithmetic
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)       :: PerfData
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        ! Allocate Variables
        REAL(8)                 :: F_WECornerFreq   ! Corner frequency (-3dB point) for first order low pass filter for measured hub height wind speed [Hz]

        !       Only used in EKF, if WE_Mode = 2
        REAL(8), SAVE           :: om_r             ! Estimated rotor speed [rad/s]
        REAL(8), SAVE           :: v_t              ! Estimated wind speed, turbulent component [m/s]
        REAL(8), SAVE           :: v_m              ! Estimated wind speed, 10-minute averaged [m/s]
        REAL(8), SAVE           :: v_h              ! Combined estimated wind speed [m/s]
        REAL(8)                 :: L                ! Turbulent length scale parameter [m]
        REAL(8)                 :: Ti               ! Turbulent intensity, [-]

        !           - operating conditions
        REAL(8)                 :: A_op             ! Estimated operational system pole [UNITS!]
        REAL(8)                 :: Cp_op            ! Estimated operational Cp [-]
        REAL(8)                 :: Tau_r            ! Estimated rotor torque [Nm]
        REAL(8)                 :: a                ! wind variance
        REAL(8)                 :: lambda           ! tip-speed-ratio [rad]
        REAL(8)                 :: RotSpeed         ! Rotor Speed [rad], locally

        !           - Covariance matrices
        REAL(8), DIMENSION(3,3)         :: F        ! First order system jacobian 
        REAL(8), DIMENSION(3,3), SAVE   :: P        ! Covariance estiamte 
        REAL(8), DIMENSION(1,3)         :: H        ! Output equation jacobian 
        REAL(8), DIMENSION(3,1), SAVE   :: xh       ! Estimated state matrix
        REAL(8), DIMENSION(3,1)         :: dxh      ! Estimated state matrix deviation from previous timestep
        REAL(8), DIMENSION(3,3)         :: Q        ! Process noise covariance matrix
        REAL(8), DIMENSION(1,1)         :: S        ! Innovation covariance 
        REAL(8), DIMENSION(3,1), SAVE   :: K        ! Kalman gain matrix
        REAL(8)                         :: R_m      ! Measurement noise covariance [(rad/s)^2]
        
        CHARACTER(*), PARAMETER                 :: RoutineName = 'WindSpeedEstimator'

        ! ---- Debug Inputs ------
        DebugVar%WE_b   = LocalVar%PC_PitComTF*R2D
        DebugVar%WE_w   = LocalVar%RotSpeedF
        DebugVar%WE_t   = LocalVar%VS_LastGenTrqF

        ! ---- Define wind speed estimate ---- 
        
        ! Inversion and Invariance Filter implementation
        IF (CntrPar%WE_Mode == 1) THEN      
            ! Compute AeroDynTorque
            Tau_r = AeroDynTorque(LocalVar, CntrPar, PerfData, ErrVar)

            LocalVar%WE_VwIdot = CntrPar%WE_Gamma/CntrPar%WE_Jtot*(LocalVar%VS_LastGenTrq*CntrPar%WE_GearboxRatio - Tau_r)
            LocalVar%WE_VwI = LocalVar%WE_VwI + LocalVar%WE_VwIdot*LocalVar%DT
            LocalVar%WE_Vw = LocalVar%WE_VwI + CntrPar%WE_Gamma*LocalVar%RotSpeedF

        ! Extended Kalman Filter (EKF) implementation
        ELSEIF (CntrPar%WE_Mode == 2) THEN
            ! Define contant values
            L = 6.0 * CntrPar%WE_BladeRadius
            Ti = 0.18
            R_m = 0.02
            H = RESHAPE((/1.0 , 0.0 , 0.0/),(/1,3/))
            ! Define matrices to be filled
            F = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            Q = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            IF (LocalVar%iStatus == 0) THEN
                ! Initialize recurring values
                om_r = max(LocalVar%RotSpeedF, EPSILON(1.0))
                v_t = 0.0
                v_m = LocalVar%HorWindV
                v_h = LocalVar%HorWindV
                lambda = max(LocalVar%RotSpeed, EPSILON(1.0)) * CntrPar%WE_BladeRadius/v_h
                xh = RESHAPE((/om_r, v_t, v_m/),(/3,1/))
                P = RESHAPE((/0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0/),(/3,3/))
                K = RESHAPE((/0.0,0.0,0.0/),(/3,1/))
                Cp_op   = 0.25  ! initialize so debug output doesn't give *****
                
            ELSE
                ! Find estimated operating Cp and system pole
                A_op = interp1d(CntrPar%WE_FOPoles_v,CntrPar%WE_FOPoles,v_h,ErrVar)

                ! TEST INTERP2D
                lambda = max(LocalVar%RotSpeed, EPSILON(1.0)) * CntrPar%WE_BladeRadius/v_h
                Cp_op = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, LocalVar%BlPitch(1)*R2D, lambda , ErrVar)
                Cp_op = max(0.0,Cp_op)
                
                ! Update Jacobian
                F(1,1) = A_op
                F(1,2) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/om_r * 3.0 * Cp_op * v_h**2.0
                F(1,3) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/om_r * 3.0 * Cp_op * v_h**2.0
                F(2,2) = - PI * v_m/(2.0*L)
                F(2,3) = - PI * v_t/(2.0*L)
                
                ! Update process noise covariance
                Q(1,1) = 0.00001
                Q(2,2) =(PI * (v_m**3.0) * (Ti**2.0)) / L
                Q(3,3) = (2.0**2.0)/600.0
                
                ! Prediction update
                Tau_r = AeroDynTorque(LocalVar, CntrPar, PerfData, ErrVar)
                a = PI * v_m/(2.0*L)
                dxh(1,1) = 1.0/CntrPar%WE_Jtot * (Tau_r - CntrPar%WE_GearboxRatio * LocalVar%VS_LastGenTrqF)
                dxh(2,1) = -a*v_t
                dxh(3,1) = 0.0
                
                xh = xh + LocalVar%DT * dxh ! state update
                P = P + LocalVar%DT*(MATMUL(F,P) + MATMUL(P,TRANSPOSE(F)) + Q - MATMUL(K * R_m, TRANSPOSE(K))) 
                
                ! Measurement update
                S = MATMUL(H,MATMUL(P,TRANSPOSE(H))) + R_m        ! NJA: (H*T*H') \approx 0
                K = MATMUL(P,TRANSPOSE(H))/S(1,1)
                xh = xh + K*(LocalVar%RotSpeedF - om_r)
                P = MATMUL(identity(3) - MATMUL(K,H),P)
                
                
                ! Wind Speed Estimate
                om_r = max(xh(1,1), EPSILON(1.0))
                v_t = xh(2,1)
                v_m = xh(3,1)
                v_h = v_t + v_m
                LocalVar%WE_Vw = v_m + v_t

                IF (ieee_is_nan(v_h)) THEN
                    om_r = LocalVar%RotSpeedF
                    v_t = 0.0
                    v_m = LocalVar%HorWindV
                    v_h = LocalVar%HorWindV
                    LocalVar%WE_Vw = v_m + v_t
                ENDIF

            ENDIF
            ! Debug Outputs
            DebugVar%WE_Cp = Cp_op
            DebugVar%WE_Vm = v_m
            DebugVar%WE_Vt = v_t
            DebugVar%WE_lambda = lambda
        ELSE        
            ! Define Variables
            F_WECornerFreq = 0.20944  ! Fix to 30 second time constant for now    

            ! Filter wind speed at hub height as directly passed from OpenFAST
            LocalVar%WE_Vw = LPFilter(LocalVar%HorWindV, LocalVar%DT, F_WECornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF)
        ENDIF 

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
    ! Setpoint smoother modifies controller reference in order to separate generator torque and blade pitch control actions
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement setpoint smoothing
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        ! Allocate Variables
        REAL(8)                      :: DelOmega                            ! Reference generator speed shift, rad/s.
        
        ! ------ Setpoint Smoothing ------
        IF ( CntrPar%SS_Mode == 1) THEN
            ! Find setpoint shift amount
            DelOmega = ((LocalVar%PC_PitComT - LocalVar%PC_MinPit)/0.524) * CntrPar%SS_VSGain - ((CntrPar%VS_RtPwr - LocalVar%VS_LastGenPwr))/CntrPar%VS_RtPwr * CntrPar%SS_PCGain ! Normalize to 30 degrees for now
            DelOmega = DelOmega * CntrPar%PC_RefSpd
            ! Filter
            LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, CntrPar%F_SSCornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF) 
        ELSE
            LocalVar%SS_DelOmegaF = 0 ! No setpoint smoothing
        ENDIF

    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar) 
    ! PitchSaturation defines a minimum blade pitch angle based on a lookup table provided by DISCON.IN
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement pitch saturation
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, DebugVariables, ErrorVariables
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        CHARACTER(*),               PARAMETER           :: RoutineName = 'PitchSaturation'

        ! Define minimum blade pitch angle as a function of estimated wind speed
        PitchSaturation = interp1d(CntrPar%PS_WindSpeeds, CntrPar%PS_BldPitchMin, LocalVar%WE_Vw_F, ErrVar)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END FUNCTION PitchSaturation
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION Shutdown(LocalVar, CntrPar, objInst) 
    ! PeakShaving defines a minimum blade pitch angle based on a lookup table provided by DISON.IN
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement setpoint smoothing
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        
        ! Local Variables 
        REAL(8)                                         :: SD_BlPitchF
        ! Initialize Shutdown Varible
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%SD = .FALSE.
        ENDIF

        ! See if we should shutdown
        IF (.NOT. LocalVar%SD ) THEN
            ! Filter pitch signal
            SD_BlPitchF = LPFilter(LocalVar%PC_PitComT, LocalVar%DT, CntrPar%SD_CornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF)
            
            ! Go into shutdown if above max pit
            IF (SD_BlPitchF > CntrPar%SD_MaxPit) THEN
                LocalVar%SD  = .TRUE.
            ELSE
                LocalVar%SD  = .FALSE.
            ENDIF 
        ENDIF

        ! Pitch Blades to 90 degrees at max pitch rate if in shutdown mode
        IF (LocalVar%SD) THEN
            Shutdown = LocalVar%BlPitch(1) + CntrPar%PC_MaxRat*LocalVar%DT
            IF (MODULO(LocalVar%Time, 10.0) == 0) THEN
                print *, ' ** SHUTDOWN MODE **'
            ENDIF
        ELSE
            Shutdown = LocalVar%PC_PitComT
        ENDIF

        
    END FUNCTION Shutdown
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks
