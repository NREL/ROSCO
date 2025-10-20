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
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE Constants
        ! Allocate variables
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar


        ! ----- Pitch controller speed and power error -----
        
        ! Power reference tracking generator speed
        IF (CntrPar%PRC_Mode == 1) THEN
            LocalVar%PRC_WSE_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%PRC_LPF_Freq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 
            LocalVar%PC_RefSpd_PRC = interp1d(CntrPar%PRC_WindSpeeds,CntrPar%PRC_GenSpeeds,LocalVar%PRC_WSE_F,ErrVar)
        ELSE
            LocalVar%PC_RefSpd_PRC = CntrPar%PC_RefSpd
        ENDIF
        
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF < 0) THEN
            LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC - LocalVar%SS_DelOmegaF
        ELSE
            LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC
        ENDIF

        ! Compute error for pitch controller
        LocalVar%PC_RefSpd = LocalVar%PC_RefSpd_SS!-LocalVar%Del_Beta        
        LocalVar%PC_SpdErr = LocalVar%PC_RefSpd - LocalVar%GenSpeedF            ! Speed error
        LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr             ! Power error, unused
                
        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]
        IF (CntrPar%VS_ControlMode == 2) THEN
            LocalVar%VS_RefSpd_TSR = (CntrPar%VS_TSRopt * LocalVar%We_Vw_F / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio
        ELSEIF (CntrPar%VS_ControlMode == 3) THEN
            LocalVar%VS_GenPwrF = LPFilter(LocalVar%VS_GenPwr, LocalVar%DT,CntrPar%VS_PwrFiltF, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 
            LocalVar%VS_RefSpd_TSR = (LocalVar%VS_GenPwrF/CntrPar%VS_Rgn2K)**(1./3.) ! Genspeed reference that doesnt depend on wind speed estimate (https://doi.org/10.2172/1259805)
        ELSE
            LocalVar%VS_RefSpd_TSR = CntrPar%VS_RefSpd
        ENDIF 

        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_TSR

        ! Exclude reference speeds specified by user
        IF (CntrPar%TRA_Mode > 0) THEN
            CALL RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
        END IF

        ! Saturate torque reference speed between min speed and rated speed
        LocalVar%VS_RefSpd = saturate(LocalVar%VS_RefSpd,CntrPar%VS_MinOMSpd, CntrPar%VS_RefSpd)

        ! Implement power reference rotor speed (overwrites above), convert to generator speed
        IF (CntrPar%PRC_Mode == 1) THEN
            LocalVar%VS_RefSpd = interp1d(CntrPar%PRC_WindSpeeds,CntrPar%PRC_GenSpeeds,LocalVar%WE_Vw_F,ErrVar)
        ENDIF
        
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF > 0) THEN
            LocalVar%VS_RefSpd = LocalVar%VS_RefSpd - LocalVar%SS_DelOmegaF
        ENDIF

        ! Force zero torque in shutdown mode
        IF (LocalVar%SD) THEN
            LocalVar%VS_RefSpd = CntrPar%VS_MinOMSpd
        ENDIF

        ! Force minimum rotor speed
        LocalVar%VS_RefSpd = max(LocalVar%VS_RefSpd, CntrPar%VS_MinOmSpd)

        ! Reference error
        IF ((CntrPar%VS_ControlMode == 2) .OR. (CntrPar%VS_ControlMode == 3)) THEN
            LocalVar%VS_SpdErr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF
        ENDIF

        ! Define transition region setpoint errors
        LocalVar%VS_SpdErrAr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF               ! Current speed error - Region 2.5 PI-control (Above Rated)
        LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF     ! Current speed error - Region 1.5 PI-control (Below Rated)
        
        ! Region 3 minimum pitch angle for state machine
        LocalVar%VS_Rgn3Pitch = LocalVar%PC_MinPit + CntrPar%PC_Switch

        ! Debug Vars
        DebugVar%VS_RefSpd = LocalVar%VS_RefSpd
        DebugVar%PC_RefSpd = LocalVar%PC_RefSpd


    END SUBROUTINE ComputeVariablesSetpoints
	!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE AdaptiveEnvelopeProtectionSystem(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    !Adaptive Envelope Protection System is used to keep the turbine within a pre-defined Thrust Limit value. It predicts the turbine thrust force based on wind and turbine states and calculates
	!a theorical wind speed, referred to as envelope wind speed. By comparing the envelope wind speed and the actual wind speed, it decides the excessive thrust force, and applies an avoidance action
	!generating an extra blade pitch signal, which changes of blade pitch reference (output of the blade pitch controller), i.e. blade pitch angles. In the current implementation here, 
	!rather than the real wind and thrust force of the turbine, wind and thrust estimates are used in the protection algorithm.

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
            
        INTEGER, PARAMETER :: n = 6            ! Dimension of the vector: Weght and del
        REAL(DbKi) :: T_err                    ! Thrust Prediction Error
        REAL(DbKi) :: P                        ! Solution of Lyapunov Equation
        REAL(DbKi) :: a_s =-0.2401             ! Approximate value for a
        REAL(DbKi) :: b_s = 0.0175             ! Approximate value for b
        REAL(DbKi) :: RotSpeed                 ! Rotor Speed [rad], locally
        REAL(DbKi) :: Thrst                    ! Turbine Thrust [MN], locally
        REAL(DbKi) :: Uenv                     ! Envelope Wind Speed		
        REAL(DbKi) :: Delta                    ! Neural Network Output
        REAL(DbKi) :: Thrst_es                 ! Thrust Estimate
        REAL(DbKi) :: Pre_Thrst_es             ! Previous Time Step Thrust Estimate
        REAL(DbKi) :: Thrst_es_dt              ! Derivative of Thrust Estimate
        REAL(DbKi) :: Del_Beta                 ! Adaptive Envelope Protection System (AEPS) output, (Extra Blade Pitch Angle)        
        REAL(DbKi) :: Weght(n), del(n)         ! Vectors for Neural Network (NN) Weights and del
        REAL(DbKi) :: dWeght_dt(n)             ! Derivative of NN Weights          
        INTEGER    :: m, i, j
        REAL(DbKi) :: Adp, Tdot                ! Adaptation Output, Derivative of Thrust Estimate
        REAL(DbKi) :: D1(3), D2(2), bias, Uold, Ea, Es
                
        CHARACTER(*), PARAMETER                 :: RoutineName = 'AdaptiveEnvelopeProtectionSystem'
               
            IF (LocalVar%iStatus == 0) THEN

                ! Initializing the vector for Neural Network Weights
                ! Weght = [0, 0, 0, 0, 0, 0]
                Weght = [0.000007139, 0.0000214171, 0.000001874, 0.00000562229, 0.000005398, 0.00001619]

                ! Initialize the Delta
                LocalVar%Delta = 0.0      

                ! Initial guess for Uenv
                LocalVar%Uenv = 10  

                ! Randomly large relative approximation for Uenv error
                Ea = 0.1  

            ENDIF
			
                ! NOTE: Calculation of the Thrust (Estimate) of the controlled turbine is realized under the subroutine of WindSpeedEstimator
            
                ! Adaptive EPS Algorithm Starts from Here

                P = 0.5 / CntrPar%Kc ! Solution of Lyapunov Equation (-K)'*P+P*(-K)=-I
                
                    ! Kronecker product calculation (del = D1 ⊗ D2)
                    bias = 1.0                    
                    D1 = [bias, LocalVar%Thrst, LocalVar%RotSpeedF]  !Define D1 vector (length l)                    
                    D2 = [bias, LocalVar%We_Vw]                      ! Define D2 vector (length s)
                    
                    ! Compute the Kronecker product
                    do i = 1, 3   ! l = 3
                        do j = 1, 2   ! s = 2
                            del((i-1)*2 + j) = D1(i) * D2(j)         ! Compute Kronecker product
                        end do
                    end do

                    ! Apply sigmoid to each element
                    do i = 1, 3
                        do j = 1, 2
                            del((i-1)*2 + j) = 1.0_DbKi / (1.0_DbKi + exp(-del((i-1)*2 + j)))
                        end do
                    end do
                   
                    ! Compute the Derivative of Weight
                    do i = 1, n                        
                        dWeght_dt(i) = CntrPar%gamma * (del(i) * LocalVar%T_err * P - CntrPar%ke * Weght(i) * abs(LocalVar%T_err))
                    end do

                    ! Update Weght using Euler's method
                    Weght = Weght + dWeght_dt * LocalVar%DT

                    ! Compute Delta                  
                    do i = 1, n
                        LocalVar%Delta = LocalVar%Delta + Weght(i) * del(i)
                    end do

                    ! Compute the Derivative of Thrust estimate (Thrst_es)                    
                    Thrst_es_dt = a_s * LocalVar%Thrst_es + b_s * LocalVar%We_Vw + LocalVar%Delta + CntrPar%Kc * LocalVar%T_err                  
                                      
                    ! Update Thrst_es using Euler's method
                    LocalVar%Thrst_es = LocalVar%Thrst_es + Thrst_es_dt * LocalVar%DT !Thrst_es in Mega Newton
                    LocalVar%Thrst_esN=LocalVar%Thrst_es * (10**6) !Thrst_es in Newton

                    ! Update the error between Thurst and its estimate                    
                    LocalVar%T_err = LocalVar%Thrst - LocalVar%Thrst_es

                    ! Calculate the Adaptation                    
                    LocalVar%Adp = LocalVar%Delta + CntrPar%Kc * LocalVar%T_err
                   
                    ! Calculate the Derivative of Thrust Estimate
                    Pre_Thrst_es = LocalVar%Thrst_es - Thrst_es_dt * LocalVar%DT
                    LocalVar%Tdot = (LocalVar%Thrst_es - Pre_Thrst_es) / LocalVar%DT

                    if (CntrPar%ASO_Mode == 0) then
                       LocalVar%Del_Beta = 0

                    !else if (CntrPar%ASO_Mode == 1) then
                    !else if ((CntrPar%ASO_Mode == 1) and (LocalVar%time >= 80)) then
                    else if (CntrPar%ASO_Mode == 1 .and. LocalVar%time >= 80) then

                       ! Detecting the Excessive Thrust Force and Generating Extra Blade Pitch Output
                       !if (LocalVar%We_Vw - LocalVar%Uenv >= 0) then
                       if (LocalVar%We_Vw - LocalVar%Uenv >= -1.0) then  
                           !LocalVar%Del_Beta = CntrPar%e_dp * (LocalVar%We_Vw - LocalVar%Uenv)                                                                                            
                           LocalVar%Del_Beta = CntrPar%e_dp * abs(LocalVar%We_Vw - (LocalVar%Uenv-1.0))                                                  
                          
                       else 
                           LocalVar%Del_Beta = 0
                       end if   
                                                           
                    else if (CntrPar%ASO_Mode == 2) then                                       
                    
                       if (LocalVar%Thrst - CntrPar%PreDf_Thrst >= -0.05*CntrPar%PreDf_Thrst) then
                            LocalVar%Del_Beta = CntrPar%e_dp * abs(LocalVar%Thrst - (CntrPar%PreDf_Thrst-0.05*CntrPar%PreDf_Thrst))                       
                       else 
                           LocalVar%Del_Beta = 0
                       end if

                    else if (CntrPar%ASO_Mode == 3) then

                        print *, "Design ASCOS system"

                    end if     
                                        
                    ! Envelope wind speed calculation                  
                    Es = 0.01      ! Uenv tolerance                    
                    LocalVar%Uold = LocalVar%Uenv
                    m = 0          ! Iteration counter

                    ! Iteration loop for calculating Envelope Wind Speed, Uenv
                    do while (Ea > Es)
                        
                        LocalVar%Uenv = -(1.0 / b_s) * (a_s * CntrPar%PreDf_Thrst + LocalVar%Adp - LocalVar%Tdot)
                        Ea = abs((LocalVar%Uenv - LocalVar%Uold) / LocalVar%Uenv)
                        LocalVar%Uold = LocalVar%Uenv
                        m = m + 1
                        
                    end do                                                       
                                                        
                ! Adaptive EPS Algorithm Ends Here

    END SUBROUTINE AdaptiveEnvelopeProtectionSystem
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
                IF (CntrPar%VS_ConstPower == 1) THEN ! Constant power tracking
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

                IF (CntrPar%VS_ConstPower == 1) THEN                   ! Region 3
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
        REAL(DbKi)                 :: F_WECornerFreq   ! Corner frequency (-3dB point) for first order low pass filter for measured hub height wind speed [Hz]

        !       Only used in EKF, if WE_Mode = 2
        REAL(DbKi)                 :: L                ! Turbulent length scale parameter [m]
        REAL(DbKi)                 :: Ti               ! Turbulent intensity, [-]
        ! REAL(DbKi), DIMENSION(3,3) :: I
        !           - operating conditions
        REAL(DbKi)                 :: A_op             ! Estimated operational system pole [UNITS!]
        REAL(DbKi)                 :: Cp_op            ! Estimated operational Cp [-]
        REAL(DbKi)                 :: Ct_op            ! Estimated operational Ct [-]
        REAL(DbKi)                 :: Tau_r            ! Estimated rotor torque [Nm]
        REAL(DbKi)                 :: a                ! wind variance
        REAL(DbKi)                 :: lambda           ! tip-speed-ratio [rad]
        REAL(DbKi)                 :: RotSpeed         ! Rotor Speed [rad], locally

        !           - Covariance matrices
        REAL(DbKi), DIMENSION(3,3)         :: F        ! First order system jacobian 
        REAL(DbKi), DIMENSION(1,3)         :: H        ! Output equation jacobian 
        REAL(DbKi), DIMENSION(3,1)         :: dxh      ! Estimated state matrix deviation from previous timestep
        REAL(DbKi), DIMENSION(3,3)         :: Q        ! Process noise covariance matrix
        REAL(DbKi), DIMENSION(1,1)         :: S        ! Innovation covariance 
        REAL(DbKi)                         :: R_m      ! Measurement noise covariance [(rad/s)^2]

        REAL(DbKi)              :: WE_Inp_Pitch
        REAL(DbKi)              :: WE_Inp_Torque
        REAL(DbKi)              :: WE_Inp_Speed

        real :: alp! Thrst, dt
        real :: cut_frequency
               
        CHARACTER(*), PARAMETER                 :: RoutineName = 'WindSpeedEstimator'
		
    
        ! Saturate inputs to WSE
        IF (LocalVar%RotSpeedF < 0.25 * CntrPar%VS_MinOMSpd / CntrPar%WE_GearboxRatio) THEN
            WE_Inp_Speed = 0.25 * CntrPar%VS_MinOMSpd / CntrPar%WE_GearboxRatio + EPSILON(1.0_DbKi)  ! If this is 0, could cause problems...
        ELSE
            WE_Inp_Speed = LocalVar%RotSpeedF
        END IF

        IF (LocalVar%BlPitchCMeas < CntrPar%PC_MinPit) THEN
            WE_Inp_Pitch = CntrPar%PC_MinPit
        ELSE
            WE_Inp_Pitch = LocalVar%BlPitchCMeas
        END IF

        IF (LocalVar%VS_LastGenTrqF < 0.0001 * CntrPar%VS_RtTq) THEN
            WE_Inp_Torque = 0.0001 * CntrPar%VS_RtTq
        ELSE
            WE_Inp_Torque = LocalVar%VS_LastGenTrqF
        END IF

        ! ---- Debug Inputs ------
        DebugVar%WE_b   = WE_Inp_Pitch
        DebugVar%WE_w   = WE_Inp_Speed
        DebugVar%WE_t   = WE_Inp_Torque

        ! ---- Define wind speed estimate ---- 
        
        ! Inversion and Invariance Filter implementation
        IF (CntrPar%WE_Mode == 1) THEN      
            ! Compute AeroDynTorque
            Tau_r = AeroDynTorque(LocalVar%RotSpeedF, LocalVar%BlPitchCMeas, LocalVar, CntrPar, PerfData, ErrVar)

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
                LocalVar%WE%om_r = WE_Inp_Speed
                LocalVar%WE%v_t = 0.0
                LocalVar%WE%v_m = max(LocalVar%HorWindV, 3.0_DbKi)   ! avoid divide by 0 below if HorWindV is 0, which some AMRWind setups create
                LocalVar%WE%v_h = max(LocalVar%HorWindV, 3.0_DbKi)   ! avoid divide by 0 below if HorWindV is 0, which some AMRWind setups create
                LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t   ! Initialize WE_Vw to aviod divide by zero
                lambda = WE_Inp_Speed * CntrPar%WE_BladeRadius/LocalVar%WE%v_h
                LocalVar%WE%xh = RESHAPE((/LocalVar%WE%om_r, LocalVar%WE%v_t, LocalVar%WE%v_m/),(/3,1/))
                LocalVar%WE%P = RESHAPE((/0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0/),(/3,3/))
                LocalVar%WE%K = RESHAPE((/0.0,0.0,0.0/),(/3,1/))
                Cp_op   = 0.25  ! initialize so debug output doesn't give *****

            ELSE

                ! Find estimated operating Cp and system pole
                A_op = interp1d(CntrPar%WE_FOPoles_v,CntrPar%WE_FOPoles,LocalVar%WE%v_h,ErrVar)

                ! TEST INTERP2D
                lambda = max(WE_Inp_Speed, EPSILON(1.0_DbKi)) * CntrPar%WE_BladeRadius/LocalVar%WE%v_h
                Cp_op = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, WE_Inp_Pitch*R2D, lambda , ErrVar)
                Cp_op = max(0.0,Cp_op)

                ! Calculate the operational Ct value of the controlled turbine
                Ct_op = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Ct_mat, WE_Inp_Pitch*R2D, lambda , ErrVar)
                Ct_op = max(0.0,Ct_op)
                                
                ! Calculate the Thrust Estimate of the controlled turbine                
                LocalVar%ThrstN=0.5 *CntrPar%WE_RhoAir * (LocalVar%WE_Vw**2) * PI * (CntrPar%WE_BladeRadius**2) * Ct_op !Estimated Thrust using CT surface (Newton)
                LocalVar%Thrst=0.5 * 0.000001 *CntrPar%WE_RhoAir * (LocalVar%WE_Vw**2) * PI * (CntrPar%WE_BladeRadius**2) * Ct_op !Estimated Thrust using CT surface (Mega Newton)
              
                ! Update Jacobian
                F(1,1) = A_op
                F(1,2) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * Cp_op * LocalVar%WE%v_h**2.0
                F(1,3) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * Cp_op * LocalVar%WE%v_h**2.0
                F(2,2) = - PI * LocalVar%WE%v_m/(2.0*L)
                F(2,3) = - PI * LocalVar%WE%v_t/(2.0*L)
                
                ! Update process noise covariance
                Q(1,1) = 0.00001
                Q(2,2) =(PI * (LocalVar%WE%v_m**3.0) * (Ti**2.0)) / L
                Q(3,3) = (2.0**2.0)/600.0
                
                ! Prediction update
                Tau_r = AeroDynTorque(WE_Inp_Speed, WE_Inp_Pitch, LocalVar, CntrPar, PerfData, ErrVar)
                a = PI * LocalVar%WE%v_m/(2.0*L)
                dxh(1,1) = 1.0/CntrPar%WE_Jtot * (Tau_r - CntrPar%WE_GearboxRatio * WE_Inp_Torque)
                dxh(2,1) = -a*LocalVar%WE%v_t
                dxh(3,1) = 0.0
                
                LocalVar%WE%xh = LocalVar%WE%xh + LocalVar%DT * dxh ! state update
                LocalVar%WE%P = LocalVar%WE%P + LocalVar%DT*(MATMUL(F,LocalVar%WE%P) + MATMUL(LocalVar%WE%P,TRANSPOSE(F)) + Q - MATMUL(LocalVar%WE%K * R_m, TRANSPOSE(LocalVar%WE%K))) 
                
                ! Measurement update
                S = MATMUL(H,MATMUL(LocalVar%WE%P,TRANSPOSE(H))) + R_m        ! NJA: (H*T*H') \approx 0
                LocalVar%WE%K = MATMUL(LocalVar%WE%P,TRANSPOSE(H))/S(1,1)
                LocalVar%WE%xh = LocalVar%WE%xh + LocalVar%WE%K*(WE_Inp_Speed - LocalVar%WE%om_r)
                LocalVar%WE%P = MATMUL(identity(3) - MATMUL(LocalVar%WE%K,H),LocalVar%WE%P)
                
                
                ! Wind Speed Estimate
                LocalVar%WE%om_r = max(LocalVar%WE%xh(1,1), EPSILON(1.0_DbKi))
                LocalVar%WE%v_t = LocalVar%WE%xh(2,1)
                LocalVar%WE%v_m = LocalVar%WE%xh(3,1)
                LocalVar%WE%v_h = LocalVar%WE%v_t + LocalVar%WE%v_m
                LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t

                IF (ieee_is_nan(LocalVar%WE%v_h)) THEN
                    LocalVar%WE%om_r = WE_Inp_Speed
                    LocalVar%WE%v_t = 0.0
                    LocalVar%WE%v_m = LocalVar%HorWindV
                    LocalVar%WE%v_h = LocalVar%HorWindV
                    LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t
                ENDIF

            ENDIF
            ! Debug Outputs
            DebugVar%WE_Cp = Cp_op
            DebugVar%WE_Vm = LocalVar%WE%v_m
            DebugVar%WE_Vt = LocalVar%WE%v_t
            DebugVar%WE_lambda = lambda
        ELSE        
            ! Filter wind speed at hub height as directly passed from OpenFAST
            LocalVar%WE_Vw = LPFilter(LocalVar%HorWindV, LocalVar%DT, CntrPar%F_WECornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
        ENDIF 
        DebugVar%WE_Vw = LocalVar%WE_Vw
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
        REAL(DbKi)                      :: DelOmega                            ! Reference generator speed shift, rad/s.
        
        ! ------ Setpoint Smoothing ------
        IF ( CntrPar%SS_Mode == 1) THEN
            ! Find setpoint shift amount
            DelOmega = ((LocalVar%PC_PitComT - LocalVar%PC_MinPit)/0.524) * CntrPar%SS_VSGain - ((CntrPar%VS_RtPwr - LocalVar%VS_LastGenPwr))/CntrPar%VS_RtPwr * CntrPar%SS_PCGain ! Normalize to 30 degrees for now
            DelOmega = DelOmega * CntrPar%PC_RefSpd
            ! Filter
            LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, CntrPar%F_SSCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 
        ELSE
            LocalVar%SS_DelOmegaF = 0 ! No setpoint smoothing
        ENDIF

    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar) 
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
    REAL(DbKi) FUNCTION Shutdown(LocalVar, CntrPar, objInst) 
    ! Shutdown controller 
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        
        ! Local Variables 
        REAL(DbKi)                                      :: SD_BlPitchF
        ! Initialize Shutdown Varible
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%SD = .FALSE.
        ENDIF

        ! See if we should shutdown
        IF (.NOT. LocalVar%SD ) THEN
            ! Filter pitch signal
            SD_BlPitchF = LPFilter(LocalVar%PC_PitComT, LocalVar%DT, CntrPar%SD_CornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
            
            ! Go into shutdown if above max pit
            IF (SD_BlPitchF > CntrPar%SD_MaxPit) THEN
                LocalVar%SD  = .TRUE.
            ELSE
                LocalVar%SD  = .FALSE.
            ENDIF 
        ENDIF

        ! Pitch Blades to 90 degrees at max pitch rate if in shutdown mode
        IF (LocalVar%SD) THEN
            Shutdown = LocalVar%BlPitchCMeas + CntrPar%PC_MaxRat*LocalVar%DT
            IF (MODULO(LocalVar%Time, 10.0_DbKi) == 0) THEN
                print *, ' ** SHUTDOWN MODE **'
            ENDIF
        ELSE
            Shutdown = LocalVar%PC_PitComT
        ENDIF

        
    END FUNCTION Shutdown
!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar) 
    ! Reference speed exclusion:
    !   Changes torque controllerr reference speed to avoid specified frequencies by a prescribed bandwidth
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(DebugVariables),      INTENT(INOUT)        :: DebugVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst

        
        REAL(DbKi)                             :: VS_RefSpeed_LSS
        
        ! Get LSS Ref speed
        VS_RefSpeed_LSS = LocalVar%VS_RefSpd/CntrPar%WE_GearboxRatio

        IF ((VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2) .AND. &
            (VS_RefSpeed_LSS < CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2)) THEN
            ! In hysteresis zone, hold reference speed
            LocalVar%FA_Hist = 1 ! Set negative hysteris if ref < exclusion band
        ELSE
            LocalVar%FA_Hist = 0
        END IF

        ! Initialize last reference speed state
        IF (LocalVar%restart) THEN
            ! If starting in hist band
            IF (LocalVar%FA_Hist > 0) THEN
                IF (VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed) THEN
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2
                ELSE
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2
                ENDIF
            ELSE
                LocalVar%TRA_LastRefSpd = LocalVar%VS_RefSpd
            END IF
        END IF 


        IF (LocalVar%FA_Hist > 0) THEN
            LocalVar%VS_RefSpd_TRA = LocalVar%TRA_LastRefSpd
        ELSE
            LocalVar%VS_RefSpd_TRA = LocalVar%VS_RefSpd
        END IF

        ! Save last reference speed       
        LocalVar%TRA_LastRefSpd = LocalVar%VS_RefSpd_TRA

        ! Rate limit reference speed
        LocalVar%VS_RefSpd_RL = ratelimit(LocalVar%VS_RefSpd_TRA, -CntrPar%TRA_RateLimit, CntrPar%TRA_RateLimit, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL)
        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_RL * CntrPar%WE_GearboxRatio
       
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks
