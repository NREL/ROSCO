! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! This module contains all the filters and related subroutines

! Filters:
!       LPFilter: Low-pass filter
!       SecLPFilter: Second order low-pass filter
!       HPFilter: High-pass filter
!       NotchFilter: Notch filter
!       NotchFilterSlopes: Notch Filter with descending slopes
!       PreFilterMeasuredSignals: Pre-filter signals during each run iteration

MODULE Filters
!...............................................................................................................................
    USE Constants
    IMPLICIT NONE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst)
    ! Discrete time Low-Pass Filter of the form:
    !                               Continuous Time Form:   H(s) = CornerFreq/(1 + CornerFreq)
    !                               Discrete Time Form:     H(z) = (b1z + b0) / (a1*z + a0)
    !
        USE ROSCO_Types, ONLY : FilterParameters
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 

        REAL(8), INTENT(IN)         :: InputSignal
        REAL(8), INTENT(IN)         :: DT                       ! time step [s]
        REAL(8), INTENT(IN)         :: CornerFreq               ! corner frequency [rad/s]
        INTEGER(4), INTENT(IN)      :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER(4), INTENT(INOUT)   :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)      :: reset                    ! Reset the filter to the input signal

            ! Initialization
        IF ((iStatus == 0) .OR. reset) THEN   
            FP%lpf1_OutputSignalLast(inst) = InputSignal
            FP%lpf1_InputSignalLast(inst) = InputSignal
            FP%lpf1_a1(inst) = 2 + CornerFreq*DT
            FP%lpf1_a0(inst) = CornerFreq*DT - 2
            FP%lpf1_b1(inst) = CornerFreq*DT
            FP%lpf1_b0(inst) = CornerFreq*DT
        ENDIF

        ! Define coefficients

        ! Filter
        LPFilter = 1.0/FP%lpf1_a1(inst) * (-FP%lpf1_a0(inst)*FP%lpf1_OutputSignalLast(inst) + FP%lpf1_b1(inst)*InputSignal + FP%lpf1_b0(inst)*FP%lpf1_InputSignalLast(inst))

        ! Save signals for next time step
        FP%lpf1_InputSignalLast(inst)  = InputSignal
        FP%lpf1_OutputSignalLast(inst) = LPFilter
        inst = inst + 1

    END FUNCTION LPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst)
    ! Discrete time Low-Pass Filter of the form:
    !                               Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
    !                               Discrete Time From:     H(z) = (b2*z^2 + b1*z + b0) / (a2*z^2 + a1*z + a0)
        USE ROSCO_Types, ONLY : FilterParameters
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 
        REAL(8), INTENT(IN)         :: InputSignal
        REAL(8), INTENT(IN)         :: DT                       ! time step [s]
        REAL(8), INTENT(IN)         :: CornerFreq               ! corner frequency [rad/s]
        REAL(8), INTENT(IN)         :: Damp                     ! Dampening constant
        INTEGER(4), INTENT(IN)      :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER(4), INTENT(INOUT)   :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)      :: reset                    ! Reset the filter to the input signal

        ! Initialization
        IF ((iStatus == 0) .OR. reset )  THEN
            FP%lpf2_OutputSignalLast1(inst)  = InputSignal
            FP%lpf2_OutputSignalLast2(inst)  = InputSignal
            FP%lpf2_InputSignalLast1(inst)   = InputSignal
            FP%lpf2_InputSignalLast2(inst)   = InputSignal
            
            ! Coefficients
            FP%lpf2_a2(inst) = DT**2.0*CornerFreq**2.0 + 4.0 + 4.0*Damp*CornerFreq*DT
            FP%lpf2_a1(inst) = 2.0*DT**2.0*CornerFreq**2.0 - 8.0
            FP%lpf2_a0(inst) = DT**2.0*CornerFreq**2.0 + 4.0 - 4.0*Damp*CornerFreq*DT
            FP%lpf2_b2(inst) = DT**2.0*CornerFreq**2.0
            FP%lpf2_b1(inst) = 2.0*DT**2.0*CornerFreq**2.0
            FP%lpf2_b0(inst) = DT**2.0*CornerFreq**2.0
        ENDIF

        ! Filter
        SecLPFilter = 1.0/FP%lpf2_a2(inst) * (FP%lpf2_b2(inst)*InputSignal + FP%lpf2_b1(inst)*FP%lpf2_InputSignalLast1(inst) + FP%lpf2_b0(inst)*FP%lpf2_InputSignalLast2(inst) - FP%lpf2_a1(inst)*FP%lpf2_OutputSignalLast1(inst) - FP%lpf2_a0(inst)*FP%lpf2_OutputSignalLast2(inst))

        ! Save signals for next time step
        FP%lpf2_InputSignalLast2(inst)   = FP%lpf2_InputSignalLast1(inst)
        FP%lpf2_InputSignalLast1(inst)   = InputSignal
        FP%lpf2_OutputSignalLast2(inst)  = FP%lpf2_OutputSignalLast1(inst)
        FP%lpf2_OutputSignalLast1(inst)  = SecLPFilter

        inst = inst + 1

    END FUNCTION SecLPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION HPFilter( InputSignal, DT, CornerFreq, FP, iStatus, reset, inst)
    ! Discrete time High-Pass Filter
        USE ROSCO_Types, ONLY : FilterParameters
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 

        REAL(8), INTENT(IN)     :: InputSignal
        REAL(8), INTENT(IN)     :: DT                       ! time step [s]
        REAL(8), INTENT(IN)     :: CornerFreq               ! corner frequency [rad/s]
        INTEGER, INTENT(IN)     :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(INOUT)  :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)  :: reset                    ! Reset the filter to the input signal
        ! Local
        REAL(8)                 :: K                        ! Constant gain

        ! Initialization
        IF ((iStatus == 0) .OR. reset)  THEN
            FP%hpf_OutputSignalLast(inst) = InputSignal
            FP%hpf_InputSignalLast(inst) = InputSignal
        ENDIF
        K = 2.0 / DT

        ! Body
        HPFilter = K/(CornerFreq + K)*InputSignal - K/(CornerFreq + K)*FP%hpf_InputSignalLast(inst) - (CornerFreq - K)/(CornerFreq + K)*FP%hpf_OutputSignalLast(inst)

        ! Save signals for next time step
        FP%HPF_InputSignalLast(inst)   = InputSignal
        FP%HPF_OutputSignalLast(inst)  = HPFilter
        inst = inst + 1

    END FUNCTION HPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst)
    ! Discrete time inverted Notch Filter with descending slopes, G = CornerFreq*s/(Damp*s^2+CornerFreq*s+Damp*CornerFreq^2)
        USE ROSCO_Types, ONLY : FilterParameters
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 

        REAL(8), INTENT(IN)     :: InputSignal
        REAL(8), INTENT(IN)     :: DT                       ! time step [s]
        REAL(8), INTENT(IN)     :: CornerFreq               ! corner frequency [rad/s]
        REAL(8), INTENT(IN)     :: Damp                     ! Dampening constant
        INTEGER, INTENT(IN)     :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(INOUT)  :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)  :: reset                    ! Reset the filter to the input signal

        ! Initialization
        IF ((iStatus == 0) .OR. reset) THEN
            FP%nfs_OutputSignalLast1(inst)  = InputSignal
            FP%nfs_OutputSignalLast2(inst)  = InputSignal
            FP%nfs_InputSignalLast1(inst)   = InputSignal
            FP%nfs_InputSignalLast2(inst)   = InputSignal
            FP%nfs_b2(inst) = 2.0 * DT * CornerFreq
            FP%nfs_b0(inst) = -FP%nfs_b2(inst)
            FP%nfs_a2(inst) = Damp*DT**2.0*CornerFreq**2.0 + 2.0*DT*CornerFreq + 4.0*Damp
            FP%nfs_a1(inst) = 2.0*Damp*DT**2.0*CornerFreq**2.0 - 8.0*Damp
            FP%nfs_a0(inst) = Damp*DT**2.0*CornerFreq**2.0 - 2*DT*CornerFreq + 4.0*Damp
        ENDIF

        NotchFilterSlopes = 1.0/FP%nfs_a2(inst) * (FP%nfs_b2(inst)*InputSignal + FP%nfs_b0(inst)*FP%nfs_InputSignalLast2(inst) &
                            - FP%nfs_a1(inst)*FP%nfs_OutputSignalLast1(inst)  - FP%nfs_a0(inst)*FP%nfs_OutputSignalLast2(inst))

        ! Save signals for next time step
        FP%nfs_InputSignalLast2(inst)   = FP%nfs_InputSignalLast1(inst)
        FP%nfs_InputSignalLast1(inst)   = InputSignal          !Save input signal for next time step
        FP%nfs_OutputSignalLast2(inst)  = FP%nfs_OutputSignalLast1(inst)      !Save input signal for next time step
        FP%nfs_OutputSignalLast1(inst)  = NotchFilterSlopes
        inst = inst + 1

    END FUNCTION NotchFilterSlopes
!-------------------------------------------------------------------------------------------------------------------------------
    REAL FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst)
    ! Discrete time Notch Filter 
    !                               Continuous Time Form: G(s) = (s^2 + 2*omega*betaNum*s + omega^2)/(s^2 + 2*omega*betaDen*s + omega^2)
    !                               Discrete Time Form:   H(z) = (b2*z^2 +b1*z^2 + b0*z)/((z^2 +a1*z^2 + a0*z))
        USE ROSCO_Types, ONLY : FilterParameters
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 

        REAL(8), INTENT(IN)     :: InputSignal
        REAL(8), INTENT(IN)     :: DT                       ! time step [s]
        REAL(8), INTENT(IN)     :: omega                    ! corner frequency [rad/s]
        REAL(8), INTENT(IN)     :: betaNum                  ! Dampening constant in numerator of filter transfer function
        REAL(8), INTENT(IN)     :: betaDen                  ! Dampening constant in denominator of filter transfer function
        INTEGER, INTENT(IN)     :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(INOUT)  :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)  :: reset                    ! Reset the filter to the input signal
        ! Local
        REAL(8)                 :: K                        ! Constant gain

        ! Initialization
        K = 2.0/DT
        IF ((iStatus == 0) .OR. reset) THEN
            FP%nf_OutputSignalLast1(inst)  = InputSignal
            FP%nf_OutputSignalLast2(inst)  = InputSignal
            FP%nf_InputSignalLast1(inst)   = InputSignal
            FP%nf_InputSignalLast2(inst)   = InputSignal
            FP%nf_b2(inst) = (K**2.0 + 2.0*omega*BetaNum*K + omega**2.0)/(K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
            FP%nf_b1(inst) = (2.0*omega**2.0 - 2.0*K**2.0)  / (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0);
            FP%nf_b0(inst) = (K**2.0 - 2.0*omega*BetaNum*K + omega**2.0) / (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
            FP%nf_a1(inst) = (2.0*omega**2.0 - 2.0*K**2.0)  / (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
            FP%nf_a0(inst) = (K**2.0 - 2.0*omega*BetaDen*K + omega**2.0)/ (K**2.0 + 2.0*omega*BetaDen*K + omega**2.0)
        ENDIF
        
        ! Body
        NotchFilter = FP%nf_b2(inst)*InputSignal + FP%nf_b1(inst)*FP%nf_InputSignalLast1(inst) + FP%nf_b0(inst)*FP%nf_InputSignalLast2(inst) - FP%nf_a1(inst)*FP%nf_OutputSignalLast1(inst) - FP%nf_a0(inst)*FP%nf_OutputSignalLast2(inst)

        ! Save signals for next time step
        FP%nf_InputSignalLast2(inst)   = FP%nf_InputSignalLast1(inst)
        FP%nf_InputSignalLast1(inst)   = InputSignal                  ! Save input signal for next time step
        FP%nf_OutputSignalLast2(inst)  = FP%nf_OutputSignalLast1(inst)      ! Save input signal for next time step
        FP%nf_OutputSignalLast1(inst)  = NotchFilter
        inst = inst + 1

    END FUNCTION NotchFilter
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE PreFilterMeasuredSignals(CntrPar, LocalVar, objInst)
    ! Prefilter measured wind turbine signals to separate the filtering from the actual control actions

        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        
        TYPE(ControlParameters), INTENT(INOUT)      :: CntrPar
        TYPE(LocalVariables),    INTENT(INOUT)      :: LocalVar
        TYPE(ObjectInstances),   INTENT(INOUT)      :: objInst

        ! Filter the HSS (generator) and LSS (rotor) speed measurement:
        ! Apply Low-Pass Filter (choice between first- and second-order low-pass filter)
        IF (CntrPar%F_LPFType == 1) THEN
            LocalVar%GenSpeedF = LPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
            LocalVar%RotSpeedF = LPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
        ELSEIF (CntrPar%F_LPFType == 2) THEN   
            LocalVar%GenSpeedF = SecLPFilter(LocalVar%GenSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, CntrPar%F_LPFDamping, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Second-order low-pass filter on generator speed
            LocalVar%RotSpeedF = SecLPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%F_LPFCornerFreq, CntrPar%F_LPFDamping, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Second-order low-pass filter on generator speed
        ENDIF
        ! Apply Notch Fitler
        IF (CntrPar%F_NotchType == 1 .OR. CntrPar%F_NotchType == 3) THEN
            LocalVar%GenSpeedF = NotchFilter(LocalVar%GenSpeedF, LocalVar%DT, CntrPar%F_NotchCornerFreq, CntrPar%F_NotchBetaNumDen(1), CntrPar%F_NotchBetaNumDen(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
        ENDIF

        ! Filtering the tower fore-aft acceleration signal 
        IF (CntrPar%Fl_Mode == 1) THEN
            ! Force to start at 0
            LocalVar%NacIMU_FA_AccF = SecLPFilter(LocalVar%NacIMU_FA_Acc, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Fixed Damping
            LocalVar%FA_AccF = SecLPFilter(LocalVar%FA_Acc, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF) ! Fixed Damping
            LocalVar%NacIMU_FA_AccF = HPFilter(LocalVar%NacIMU_FA_AccF, LocalVar%DT, 0.0167, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instHPF) 
            LocalVar%FA_AccF = HPFilter(LocalVar%FA_AccF, LocalVar%DT, 0.0167, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instHPF) 
            
            IF (CntrPar%F_NotchType >= 2) THEN
                LocalVar%NACIMU_FA_AccF = NotchFilter(LocalVar%NacIMU_FA_AccF, LocalVar%DT, CntrPar%F_NotchCornerFreq, CntrPar%F_NotchBetaNumDen(1), CntrPar%F_NotchBetaNumDen(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch) ! Fixed Damping
                LocalVar%FA_AccF = NotchFilter(LocalVar%FA_AccF, LocalVar%DT, CntrPar%F_NotchCornerFreq, CntrPar%F_NotchBetaNumDen(1), CntrPar%F_NotchBetaNumDen(2), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch) ! Fixed Damping
            ENDIF
        ENDIF

        LocalVar%FA_AccHPF = HPFilter(LocalVar%FA_Acc, LocalVar%DT, CntrPar%FA_HPFCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instHPF)
        
        ! Filter Wind Speed Estimator Signal
        LocalVar%We_Vw_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT, 0.209, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) ! 30 second time constant


        ! Control commands (used by WSE, mostly)
        LocalVar%VS_LastGenTrqF = SecLPFilter(LocalVar%VS_LastGenTrq, LocalVar%DT, CntrPar%F_LPFCornerFreq, 0.7, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF)
        LocalVar%PC_PitComTF    = SecLPFilter(LocalVar%PC_PitComT, LocalVar%DT, CntrPar%F_LPFCornerFreq*0.25, 0.7, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF)

    END SUBROUTINE PreFilterMeasuredSignals
    END MODULE Filters
