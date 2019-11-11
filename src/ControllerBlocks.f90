! This module contains additional blocks to supplement the primary controllers used in the Controllers module
!
! Many of these have I/O flags as a part of the DISCON input file 
!
! Blocks (Subroutines):
!       State Machine: determine the state of the wind turbine to specify the corresponding control actions
!       WindSpeedEstimator: Estimate wind speed
!       SetpointSmoother: Modify generator torque and blade pitch controller setpoints in transition region

MODULE ControllerBlocks

USE, INTRINSIC :: ISO_C_Binding
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
    !       VS_State = 6, Tip-Speed-Ratio tracking PI controller
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN)          :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        
        ! Initialize State machine if first call
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL

            IF (LocalVar%PitCom(1) >= CntrPar%VS_Rgn3Pitch) THEN ! We are in region 3
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
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData)
    ! Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
    !       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
    !       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN)          :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)       :: PerfData
        ! Allocate Variables
        REAL(4)                 :: F_WECornerFreq   ! Corner frequency (-3dB point) for first order low pass filter for measured hub height wind speed [Hz]

        !       Only used in EKF, if WE_Mode = 2
        REAL(4), SAVE           :: om_r             ! Estimated rotor speed [rad/s]
        REAL(4), SAVE           :: v_t              ! Estimated wind speed, turbulent component [m/s]
        REAL(4), SAVE           :: v_m              ! Estimated wind speed, 10-minute averaged [m/s]
        REAL(4), SAVE           :: v_h              ! Combined estimated wind speed [m/s]
        REAL(4)                 :: L                ! Turbulent length scale parameter [m]
        REAL(4)                 :: Ti               ! Turbulent intensity, [-]
        ! REAL(4), DIMENSION(3,3) :: I
        !           - operating conditions
        REAL(4)                 :: A_op             ! Estimated operational system pole [UNITS!]
        REAL(4)                 :: Cp_op            ! Estimated operational Cp [-]
        REAL(4)                 :: Tau_r            ! Estimated rotor torque [Nm]
        REAL(4)                 :: a                ! wind variance
        REAL(4)                 :: lambda           ! tip-speed-ratio [rad]

        ! REAL(4), DIMENSION(1,23)    :: WE_EKF_Vref
        ! REAL(4)                 :: WE_EKF_Vref(23)
        ! REAL(4)                 :: WE_EKF_Aref(23)
        ! REAL(4)                 :: WE_EKF_Cpref(23)
        !           - Covariance matrices
        REAL(4), DIMENSION(3,3)         :: F        ! First order system jacobian 
        REAL(4), DIMENSION(3,3), SAVE   :: P        ! Covariance estiamte 
        REAL(4), DIMENSION(1,3)         :: H        ! Output equation jacobian 
        REAL(4), DIMENSION(3,1), SAVE   :: xh       ! Estimated state matrix
        REAL(4), DIMENSION(3,1)         :: dxh      ! Estimated state matrix deviation from previous timestep
        REAL(4), DIMENSION(3,3)         :: Q        ! Process noise covariance matrix
        REAL(4), DIMENSION(1,1)         :: S        ! Innovation covariance 
        REAL(4), DIMENSION(3,1), SAVE   :: K        ! Kalman gain matrix
        REAL(4)                         :: R_m      ! Measurement noise covariance [(rad/s)^2]
        REAL(4)                         :: temp     ! temp variable

        ! WE_EKF_Vref = (/ 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 /)
        ! WE_EKF_Aref = (/ -0.0203, -0.0270, -0.0338, -0.0405, -0.0473, -0.0540, -0.0608, -0.0675, -0.0743, -0.0671, -0.0939, -0.1257, -0.1601, -0.1973, -0.2364, -0.2783, -0.3223, -0.3678, -0.4153, -0.4632, -0.5122, -0.5629, -0.6194 /)
        ! WE_EKF_Cpref = (/ 0.4957, 0.4957, 0.4957, 0.4957, 0.4957, 0.4957, 0.4957, 0.4957, 0.4957, 0.4193, 0.3298, 0.2641, 0.2147, 0.1769, 0.1475, 0.1242, 0.1056, 0.0906, 0.0782, 0.0680, 0.0596, 0.0524, 0.0464 /)

        
        ! ---- Define wind speed estimate ---- 
        
        ! Inversion and Invariance Filter implementation
        IF (CntrPar%WE_Mode == 1) THEN      
            LocalVar%WE_VwIdot = CntrPar%WE_Gamma/CntrPar%WE_Jtot*(LocalVar%VS_LastGenTrq*CntrPar%WE_GearboxRatio - AeroDynTorque(LocalVar, CntrPar, PerfData))
            LocalVar%WE_VwI = LocalVar%WE_VwI + LocalVar%WE_VwIdot*LocalVar%DT
            LocalVar%WE_Vw = LocalVar%WE_VwI + CntrPar%WE_Gamma*LocalVar%RotSpeed

        ! Extended Kalman Filter (EKF) implementation
        ELSEIF (CntrPar%WE_Mode == 2) THEN
            ! Define contant values
            L = 4.0 * CntrPar%WE_BladeRadius
            Ti = 0.2
            R_m = 0.02
            H = RESHAPE((/1.0 , 0.0 , 0.0/),(/1,3/))
            ! Define matrices to be filled
            F = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            Q = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            IF (LocalVar%iStatus == 0) THEN
                ! Initialize recurring values
                om_r = LocalVar%RotSpeed
                v_t = 0.0
                v_m = LocalVar%HorWindV
                v_h = LocalVar%HorWindV
                xh = RESHAPE((/om_r, v_t, v_m/),(/3,1/))
                P = RESHAPE((/0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0/),(/3,3/))
                K = RESHAPE((/0.0,0.0,0.0/),(/3,1/))
            ELSE
                ! Update estimated values
                ! v_h = v_m + v_t
                a = PI * v_m/(2.0*L)

                ! Find estimated operating Cp and system pole
                A_op = interp1d(CntrPar%WE_FOPoles_v,CntrPar%WE_FOPoles,v_h)

                ! TEST INTERP2D
                lambda = LocalVar%RotSpeed * CntrPar%WE_BladeRadius/v_h
                Cp_op = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, LocalVar%BlPitch(1)*R2D, lambda )
                Cp_op = max(0.0,Cp_op)
                
                ! Update Jacobian
                F(1,1) = A_op
                F(1,2) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * Cp_op * 3.0 * v_h**2.0 * 1.0/om_r
                F(1,3) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * Cp_op * 3.0 * v_h**2.0 * 1.0/om_r
                F(2,2) = PI * v_m/(2.0*L)
                F(2,3) = PI * v_t/(2.0*L)

                ! Update process noise covariance
                Q(1,1) = 0.00001
                Q(2,2) =(PI * (v_m**3.0) * (Ti**2.0)) / L
                Q(3,3) = (2.0**2.0)/600.0

                ! Prediction update
                ! Tau_r = 0.5 * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**3 * Cp_op * v_h**2 * 1.0/(lambda)
                Tau_r = AeroDynTorque(LocalVar,CntrPar,PerfData)
                dxh(1,1) = 1.0/CntrPar%WE_Jtot * (Tau_r - CntrPar%WE_GearboxRatio * LocalVar%VS_LastGenTrq)
                dxh(2,1) = -a*v_t
                dxh(3,1) = 0.0
                
                xh = xh + LocalVar%DT * dxh ! state update
                P = P + LocalVar%DT*(MATMUL(F,P) + MATMUL(P,TRANSPOSE(F)) + Q - MATMUL(K * R_m, TRANSPOSE(K))) 

                ! Measurement update
                S = MATMUL(H,MATMUL(P,TRANSPOSE(H))) + R_m        ! NJA: (H*T*H') \approx 0
                K = MATMUL(P,TRANSPOSE(H))/S(1,1)
                xh = xh + K*(LocalVar%GenSpeedF/CntrPar%WE_GearboxRatio - xh(1,1))
                P = MATMUL(identity(3) - MATMUL(K,H),P)
                
                ! Wind Speed Estimate
                ! xh(1,1) = saturate(xh(1,1),0.0, CntrPar%PC_RefSpd * 1.5)
                ! xh(2,1) = saturate(xh(2,1),-10.0, 10.0)
                ! xh(3,1) = saturate(xh(3,1),0.0, MAXVAL(WE_EKF_Vref))
                om_r = xh(1,1)
                v_t = xh(2,1)
                v_m = xh(3,1)
                v_h = v_t + v_m
                LocalVar%TestType = v_m + v_t
                LocalVar%WE_Vw = v_m + v_t
                ! LocalVar%WE_Vw = LPFilter(v_m + v_t,LocalVar%DT,0.5,LocalVar%iStatus,.FALSE.,objInst%instLPF)
            ENDIF

        ELSE        
            ! Define Variables
            F_WECornerFreq = 0.0333  ! Fix to 30 second time constant for now    

            ! Filter wind speed at hub height as directly passed from OpenFAST
            LocalVar%WE_Vw = LPFilter(LocalVar%HorWindV, LocalVar%DT, F_WECornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF)
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
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        ! Allocate Variables
        Real(4)                      :: DelOmega                            ! Reference generator speed shift, rad/s.
        
        ! ------ Setpoint Smoothing ------
        IF ( CntrPar%SS_Mode == 1) THEN
            ! Find setpoint shift amount
            DelOmega = ((LocalVar%BlPitch(1) - CntrPar%PC_MinPit)/0.524) * CntrPar%SS_VSGain - ((CntrPar%VS_RtPwr - LocalVar%VS_GenPwr))/CntrPar%VS_RtPwr * CntrPar%SS_PCGain ! Normalize to 30 degrees for now
            DelOmega = DelOmega * CntrPar%PC_RefSpd
            ! Filter
            LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, CntrPar%F_SSCornerFreq, LocalVar%iStatus, .FALSE., objInst%instLPF) 
        ELSE
            LocalVar%SS_DelOmegaF = 0 ! No setpoint smoothing
        ENDIF

    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION PeakShaving(LocalVar, CntrPar, objInst) 
    ! PeakShaving defines a minimum blade pitch angle based on a lookup table provided by DISON.IN
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement setpoint smoothing
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        ! Allocate Variables 
        REAL(4)                     :: V_towertop ! Estimated velocity of tower top (m/s)
        REAL(4)                     :: Vhat     ! Estimated wind speed without towertop motion [m/s]
        REAL(4)                     :: Vhatf     ! 30 second low pass filtered Estimated wind speed without towertop motion [m/s]

        ! Account for towertop motions in wind speed estimate
        !       Integrate Towertop Acceleration  
        ! dV_towertop = 
        ! V_towertop = PIController(LocalVar%FA_Acc, 0.0, 1.0, -100.00, 100.00, LocalVar%DT, 0.0, .FALSE., objInst%instPI)

        Vhat = LocalVar%WE_Vw
        Vhatf = LPFilter(Vhat,LocalVar%DT,0.2,LocalVar%iStatus,.FALSE.,objInst%instLPF)
        LocalVar%TestType = Vhatf
        ! Define minimum blade pitch angle as a function of estimated wind speed
        PeakShaving = interp1d(CntrPar%PS_WindSpeeds, CntrPar%PS_BldPitchMin, Vhatf)

    END FUNCTION PeakShaving
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION Shutdown(LocalVar, CntrPar, objInst) 
    ! PeakShaving defines a minimum blade pitch angle based on a lookup table provided by DISON.IN
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement setpoint smoothing
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        ! Allocate Variables 
        REAL(4)                      :: SD_BlPitchF
        ! Initialize Shutdown Varible
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%SD = .FALSE.
        ENDIF

        ! See if we should shutdown
        IF (.NOT. LocalVar%SD ) THEN
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
