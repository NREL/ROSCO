! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Read and set the parameters used by the controller

MODULE ReadSetParameters

    USE, INTRINSIC :: ISO_C_Binding

    USE Constants
    USE Functions
    USE SysSubs

    IMPLICIT NONE

    ! Global Variables
    LOGICAL, PARAMETER     :: DEBUG_PARSING = .FALSE.      ! debug flag to output parsing information, set up Echo file later
    
    INTERFACE ParseInput                                                         ! Parses a character variable name and value from a string.
        MODULE PROCEDURE ParseInput_Str                                             ! Parses a character string from a string.
        MODULE PROCEDURE ParseInput_Dbl                                             ! Parses a double-precision REAL from a string.
        MODULE PROCEDURE ParseInput_Int                                             ! Parses an INTEGER from a string.
        ! MODULE PROCEDURE ParseInput_Log                                             ! Parses an LOGICAL from a string.
    END INTERFACE

    INTERFACE ParseAry                                                         ! Parse an array of numbers from a string.
        MODULE PROCEDURE ParseDbAry                                             ! Parse an array of double-precision REAL values.
        MODULE PROCEDURE ParseInAry                                             ! Parse an array of whole numbers.
    END INTERFACE



CONTAINS
 ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar)
        USE ROSCO_Types, ONLY : LocalVariables

        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        
        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LocalVar%iStatus            = NINT(avrSWAP(1))
        LocalVar%Time               = avrSWAP(2)
        LocalVar%DT                 = avrSWAP(3)
        LocalVar%VS_MechGenPwr      = avrSWAP(14)
        LocalVar%VS_GenPwr          = avrSWAP(15)
        LocalVar%GenSpeed           = avrSWAP(20)
        LocalVar%RotSpeed           = avrSWAP(21)
        LocalVar%GenTqMeas          = avrSWAP(23)
        LocalVar%Y_M                = avrSWAP(24)
        LocalVar%HorWindV           = avrSWAP(27)
        LocalVar%rootMOOP(1)        = avrSWAP(30)
        LocalVar%rootMOOP(2)        = avrSWAP(31)
        LocalVar%rootMOOP(3)        = avrSWAP(32)
        LocalVar%FA_Acc             = avrSWAP(53)
        LocalVar%NacIMU_FA_Acc      = avrSWAP(83)
        LocalVar%Azimuth            = avrSWAP(60)
        LocalVar%NumBl              = NINT(avrSWAP(61))

        ! --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does not...
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%BlPitch(1) = avrSWAP(4)
            LocalVar%BlPitch(2) = avrSWAP(33)
            LocalVar%BlPitch(3) = avrSWAP(34)
        ELSE
            LocalVar%BlPitch(1) = LocalVar%PitCom(1)
            LocalVar%BlPitch(2) = LocalVar%PitCom(2)
            LocalVar%BlPitch(3) = LocalVar%PitCom(3)      
        ENDIF

        IF (LocalVar%iStatus == 0) THEN
            LocalVar%restart = .True.
        ELSE
            LocalVar%restart = .False.
        ENDIF

    END SUBROUTINE ReadAvrSWAP    
! -----------------------------------------------------------------------------------
    ! Define parameters for control actions
    SUBROUTINE SetParameters(avrSWAP, accINFILE, size_avcMSG, CntrPar, LocalVar, objInst, PerfData, ErrVar)
                
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData, ErrorVariables
        
        REAL(ReKi),                 INTENT(INOUT)   :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(C_CHAR),          INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file

        INTEGER(IntKi),                 INTENT(IN   )   :: size_avcMSG
        TYPE(ControlParameters),    INTENT(INOUT)   :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)   :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)   :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar

        
        INTEGER(IntKi)                              :: K    ! Index used for looping through blades.

        CHARACTER(*),               PARAMETER       :: RoutineName = 'SetParameters'

        
        
        ! Error Catching Variables
        ! Set ErrVar%aviFAIL to 0 in each iteration:
        ErrVar%aviFAIL = 0
        ! ALLOCATE(ErrVar%ErrMsg(size_avcMSG-1))
        ErrVar%size_avcMSG  = size_avcMSG
        
        ! Initialize all filter instance counters at 1
        objInst%instLPF         = 1
        objInst%instSecLPF      = 1
        objInst%instHPF         = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch       = 1
        objInst%instPI          = 1
        
        ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
        avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
        avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
        avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
        avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
        avrSWAP(55) = 0.0 ! Pitch override: 0=yes
        avrSWAP(56) = 0.0 ! Torque override: 0=yes
        avrSWAP(65) = 0.0 ! Number of variables returned for logging
        avrSWAP(72) = 0.0 ! Generator start-up resistance
        avrSWAP(79) = 0.0 ! Request for loads: 0=none
        avrSWAP(80) = 0.0 ! Variable slip current status
        avrSWAP(81) = 0.0 ! Variable slip current demand
        
        ! Read any External Controller Parameters specified in the User Interface
        !   and initialize variables:
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Inform users that we are using this user-defined routine:
            ! ErrVar%aviFAIL = 1
            write (*,*) '                                                                              '//NEW_LINE('A')// &
                        '------------------------------------------------------------------------------'//NEW_LINE('A')// &
                        'Running ROSCO-'//TRIM(rosco_version)//NEW_LINE('A')// &
                        'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
                        'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
                        '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
                        '------------------------------------------------------------------------------'
            ! Specifically save accINFILE info (DISCON.IN)
            LocalVar%ACC_INFILE_SIZE = NINT(avrSWAP(50))
            Allocate(LocalVar%ACC_INFILE(LocalVar%ACC_INFILE_SIZE))
            LocalVar%ACC_INFILE = accINFILE

            ! Read Control Parameter File
            CALL ReadControlParameterFileSub(CntrPar, accINFILE, NINT(avrSWAP(50)),ErrVar)
            ! If there's been an file reading error, don't continue
            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
                RETURN
            ENDIF

            IF (CntrPar%WE_Mode > 0) THEN
                CALL READCpFile(CntrPar,PerfData,ErrVar)
            ENDIF
        
            ! Initialize the SAVED variables:
            LocalVar%PitCom     = LocalVar%BlPitch ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
            LocalVar%Y_AccErr   = 0.0  ! This will ensure that the accumulated yaw error starts at zero
            LocalVar%Y_YawEndT  = -1.0 ! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing
            
            ! Wind speed estimator initialization
            LocalVar%WE_Vw      = LocalVar%HorWindV
            LocalVar%WE_VwI     = LocalVar%WE_Vw - CntrPar%WE_Gamma*LocalVar%RotSpeed
            
            ! Setpoint Smoother initialization to zero
            LocalVar%SS_DelOmegaF = 0

            ! Generator Torque at K omega^2 or rated
            IF (LocalVar%GenSpeed > 0.98 * CntrPar%PC_RefSpd) THEN
                LocalVar%GenTq = CntrPar%VS_RtTq
            ELSE
                LocalVar%GenTq = min(CntrPar%VS_RtTq, CntrPar%VS_Rgn2K*LocalVar%GenSpeed*LocalVar%GenSpeed)
            ENDIF            
            LocalVar%VS_LastGenTrq = LocalVar%GenTq       
            LocalVar%VS_MaxTq      = CntrPar%VS_MaxTq
            
            ! Check validity of input parameters:
            CALL CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)

            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            ENDIF
            

        ENDIF
    END SUBROUTINE SetParameters
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadControlParameterFileSub(CntrPar, accINFILE, accINFILE_size,ErrVar)!, accINFILE_size)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters, ErrorVariables

        INTEGER(IntKi)                                  :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size),  INTENT(IN   )       :: accINFILE(accINFILE_size)    ! DISCON input filename
        INTEGER(IntKi), PARAMETER                       :: UnControllerParameters = 89  ! Unit number to open file
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar                      ! Control parameter type
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar                      ! Control parameter type

        INTEGER(IntKi)                                  :: CurLine 

        CHARACTER(1024)                                 :: OL_String                    ! Open description loop string
        INTEGER(IntKi)                                  :: OL_Count                     ! Number of open loop channels

        CHARACTER(1024)                                 :: PriPath        ! Path name of the primary DISCON file


        CHARACTER(*),               PARAMETER           :: RoutineName = 'ReadControlParameterFileSub'

        CurLine = 1

        ! Get primary path of DISCON.IN file (accINFILE(1) here)
        CALL GetPath( accINFILE(1), PriPath )     ! Input files will be relative to the path where the primary input file is located.

        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')
        
        !----------------------- HEADER ------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------------- DEBUG --------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        CALL ParseInput(UnControllerParameters,CurLine,'LoggingLevel',accINFILE(1),CntrPar%LoggingLevel,ErrVar)

        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------- CONTROLLER FLAGS ---------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'F_LPFType',accINFILE(1),CntrPar%F_LPFType,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'F_NotchType',accINFILE(1),CntrPar%F_NotchType,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'IPC_ControlMode',accINFILE(1),CntrPar%IPC_ControlMode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_ControlMode',accINFILE(1),CntrPar%VS_ControlMode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_ControlMode',accINFILE(1),CntrPar%PC_ControlMode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_ControlMode',accINFILE(1),CntrPar%Y_ControlMode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'SS_Mode',accINFILE(1),CntrPar%SS_Mode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'WE_Mode',accINFILE(1),CntrPar%WE_Mode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PS_Mode',accINFILE(1),CntrPar%PS_Mode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'SD_Mode',accINFILE(1),CntrPar%SD_Mode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'FL_Mode',accINFILE(1),CntrPar%FL_Mode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Flp_Mode',accINFILE(1),CntrPar%Flp_Mode,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'OL_Mode',accINFILE(1),CntrPar%OL_Mode,ErrVar)

        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------- FILTER CONSTANTS ---------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'F_LPFCornerFreq',accINFILE(1),CntrPar%F_LPFCornerFreq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'F_LPFDamping',accINFILE(1),CntrPar%F_LPFDamping,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'F_NotchCornerFreq',accINFILE(1),CntrPar%F_NotchCornerFreq,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'F_NotchBetaNumDen', CntrPar%F_NotchBetaNumDen, 2, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'F_SSCornerFreq',accINFILE(1),CntrPar%F_SSCornerFreq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'F_WECornerFreq',accINFILE(1),CntrPar%F_WECornerFreq,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'F_FlCornerFreq', CntrPar%F_FlCornerFreq, 2, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'F_FlHighPassFreq',accINFILE(1),CntrPar%F_FlHighPassFreq,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'F_FlpCornerFreq', CntrPar%F_FlpCornerFreq, 2, accINFILE(1), ErrVar )
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------- BLADE PITCH CONTROLLER CONSTANTS -----------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_GS_n',accINFILE(1),CntrPar%PC_GS_n,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'PC_GS_angles', CntrPar%PC_GS_angles, CntrPar%PC_GS_n, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'PC_GS_KP', CntrPar%PC_GS_KP, CntrPar%PC_GS_n, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'PC_GS_KI', CntrPar%PC_GS_KI, CntrPar%PC_GS_n, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'PC_GS_KD', CntrPar%PC_GS_KD, CntrPar%PC_GS_n, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'PC_GS_TF', CntrPar%PC_GS_TF, CntrPar%PC_GS_n, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'PC_MaxPit',accINFILE(1),CntrPar%PC_MaxPit,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_MinPit',accINFILE(1),CntrPar%PC_MinPit,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_MaxRat',accINFILE(1),CntrPar%PC_MaxRat,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_MinRat',accINFILE(1),CntrPar%PC_MinRat,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_RefSpd',accINFILE(1),CntrPar%PC_RefSpd,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_FinePit',accINFILE(1),CntrPar%PC_FinePit,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PC_Switch',accINFILE(1),CntrPar%PC_Switch,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !------------------- IPC CONSTANTS -----------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine) 
        CALL ParseAry(UnControllerParameters, CurLine, 'IPC_Vramp', CntrPar%IPC_Vramp, 2, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'IPC_IntSat',accINFILE(1),CntrPar%IPC_IntSat,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'IPC_KP', CntrPar%IPC_KP, 2, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'IPC_KI', CntrPar%IPC_KI, 2, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'IPC_aziOffset', CntrPar%IPC_aziOffset, 2, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'IPC_CornerFreqAct',accINFILE(1),CntrPar%IPC_CornerFreqAct,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !------------ VS TORQUE CONTROL CONSTANTS ----------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_GenEff',accINFILE(1),CntrPar%VS_GenEff,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_ArSatTq',accINFILE(1),CntrPar%VS_ArSatTq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_MaxRat',accINFILE(1),CntrPar%VS_MaxRat,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_MaxTq',accINFILE(1),CntrPar%VS_MaxTq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_MinTq',accINFILE(1),CntrPar%VS_MinTq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_MinOMSpd',accINFILE(1),CntrPar%VS_MinOMSpd,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_Rgn2K',accINFILE(1),CntrPar%VS_Rgn2K,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_RtPwr',accINFILE(1),CntrPar%VS_RtPwr,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_RtTq',accINFILE(1),CntrPar%VS_RtTq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_RefSpd',accINFILE(1),CntrPar%VS_RefSpd,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'VS_n',accINFILE(1),CntrPar%VS_n,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'VS_KP', CntrPar%VS_KP, CntrPar%VS_n, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'VS_KI', CntrPar%VS_KI, CntrPar%VS_n, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'VS_TSRopt',accINFILE(1),CntrPar%VS_TSRopt,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !------- Setpoint Smoother --------------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'SS_VSGain',accINFILE(1),CntrPar%SS_VSGain,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'SS_PCGain',accINFILE(1),CntrPar%SS_PCGain,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine) 

        !------------ WIND SPEED ESTIMATOR CONTANTS --------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'WE_BladeRadius',accINFILE(1),CntrPar%WE_BladeRadius,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'WE_CP_n',accINFILE(1),CntrPar%WE_CP_n,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'WE_CP', CntrPar%WE_CP, CntrPar%WE_CP_n, accINFILE(1), ErrVar, .FALSE. )
        CALL ParseInput(UnControllerParameters,CurLine,'WE_Gamma',accINFILE(1),CntrPar%WE_Gamma,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'WE_GearboxRatio',accINFILE(1),CntrPar%WE_GearboxRatio,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'WE_Jtot',accINFILE(1),CntrPar%WE_Jtot,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'WE_RhoAir',accINFILE(1),CntrPar%WE_RhoAir,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'PerfFileName',accINFILE(1),CntrPar%PerfFileName,ErrVar)        
        CALL ParseAry(UnControllerParameters, CurLine, 'PerfTableSize', CntrPar%PerfTableSize, 2, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'WE_FOPoles_N',accINFILE(1),CntrPar%WE_FOPoles_N,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'WE_FOPoles_v', CntrPar%WE_FOPoles_v, CntrPar%WE_FOPoles_N, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'WE_FOPoles', CntrPar%WE_FOPoles, CntrPar%WE_FOPoles_N, accINFILE(1), ErrVar )
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !-------------- YAW CONTROLLER CONSTANTS -----------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_ErrThresh',accINFILE(1),CntrPar%Y_ErrThresh,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_IPC_IntSat',accINFILE(1),CntrPar%Y_IPC_IntSat,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_IPC_n',accINFILE(1),CntrPar%Y_IPC_n,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'Y_IPC_KP', CntrPar%Y_IPC_KP, CntrPar%Y_IPC_n, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'Y_IPC_KI', CntrPar%Y_IPC_KI, CntrPar%Y_IPC_n, accINFILE(1), ErrVar )
        CALL ParseInput(UnControllerParameters,CurLine,'Y_IPC_omegaLP',accINFILE(1),CntrPar%Y_IPC_omegaLP,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_IPC_zetaLP',accINFILE(1),CntrPar%Y_IPC_zetaLP,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_MErrSet',accINFILE(1),CntrPar%Y_MErrSet,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_omegaLPFast',accINFILE(1),CntrPar%Y_omegaLPFast,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_omegaLPSlow',accINFILE(1),CntrPar%Y_omegaLPSlow,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Y_Rate',accINFILE(1),CntrPar%Y_Rate,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !------------ FORE-AFT TOWER DAMPER CONSTANTS ------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)   
        CALL ParseInput(UnControllerParameters,CurLine,'FA_KI',accINFILE(1),CntrPar%FA_KI,ErrVar)
        ! Don't check this name until we make an API change
        CALL ParseInput(UnControllerParameters,CurLine,'FA_HPFCornerFreq',accINFILE(1),CntrPar%FA_HPFCornerFreq,ErrVar,.FALSE.)
        CALL ParseInput(UnControllerParameters,CurLine,'FA_IntSat',accINFILE(1),CntrPar%FA_IntSat,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)      

        !------------ PEAK SHAVING ------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)   
        CALL ParseInput(UnControllerParameters,CurLine,'PS_BldPitchMin_N',accINFILE(1),CntrPar%PS_BldPitchMin_N,ErrVar)
        CALL ParseAry(UnControllerParameters, CurLine, 'PS_WindSpeeds', CntrPar%PS_WindSpeeds, CntrPar%PS_BldPitchMin_N, accINFILE(1), ErrVar )
        CALL ParseAry(UnControllerParameters, CurLine, 'PS_BldPitchMin', CntrPar%PS_BldPitchMin, CntrPar%PS_BldPitchMin_N, accINFILE(1), ErrVar )
        CALL ReadEmptyLine(UnControllerParameters,CurLine) 

        !------------ SHUTDOWN ------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)   
        CALL ParseInput(UnControllerParameters,CurLine,'SD_MaxPit',accINFILE(1),CntrPar%SD_MaxPit,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'SD_CornerFreq',accINFILE(1),CntrPar%SD_CornerFreq,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)      

        !------------ FLOATING ------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'Fl_Kp',accINFILE(1),CntrPar%Fl_Kp,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine) 

        !------------ Flaps ------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)   
        CALL ParseInput(UnControllerParameters,CurLine,'Flp_Angle',accINFILE(1),CntrPar%Flp_Angle,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Flp_Kp',accINFILE(1),CntrPar%Flp_Kp,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Flp_Ki',accINFILE(1),CntrPar%Flp_Ki,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Flp_MaxPit',accINFILE(1),CntrPar%Flp_MaxPit,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)   

        !------------ Open loop input ------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)   
        CALL ParseInput(UnControllerParameters,CurLine,'OL_Filename',accINFILE(1),CntrPar%OL_Filename,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Ind_Breakpoint',accINFILE(1),CntrPar%Ind_Breakpoint,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Ind_BldPitch',accINFILE(1),CntrPar%Ind_BldPitch,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Ind_GenTq',accINFILE(1),CntrPar%Ind_GenTq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Ind_YawRate',accINFILE(1),CntrPar%Ind_YawRate,ErrVar)

        ! Fix Paths (add relative paths if called from another dir)
        IF (PathIsRelative(CntrPar%PerfFileName)) CntrPar%PerfFileName = TRIM(PriPath)//TRIM(CntrPar%PerfFileName)
        IF (PathIsRelative(CntrPar%OL_Filename)) CntrPar%OL_Filename = TRIM(PriPath)//TRIM(CntrPar%OL_Filename)
        
        ! Read open loop input, if desired
        IF (CntrPar%OL_Mode == 1) THEN
            OL_String = ''      ! Display string
            OL_Count  = 1
            IF (CntrPar%Ind_BldPitch > 0) THEN
                OL_String   = TRIM(OL_String)//' BldPitch '
                OL_Count    = OL_Count + 1
            ENDIF

            IF (CntrPar%Ind_GenTq > 0) THEN
                OL_String   = TRIM(OL_String)//' GenTq '
                OL_Count    = OL_Count + 1
            ENDIF

            IF (CntrPar%Ind_YawRate > 0) THEN
                OL_String   = TRIM(OL_String)//' YawRate '
                OL_Count    = OL_Count + 1
            ENDIF

            PRINT *, 'ROSCO: Implementing open loop control for'//TRIM(OL_String)
            CALL Read_OL_Input(CntrPar%OL_Filename,110_IntKi,OL_Count,CntrPar%OL_Channels, ErrVar)

            CntrPar%OL_Breakpoints = CntrPar%OL_Channels(:,CntrPar%Ind_Breakpoint)

            IF (CntrPar%Ind_BldPitch > 0) THEN
                CntrPar%OL_BldPitch = CntrPar%OL_Channels(:,CntrPar%Ind_BldPitch)
            ENDIF

            IF (CntrPar%Ind_GenTq > 0) THEN
                CntrPar%OL_GenTq = CntrPar%OL_Channels(:,CntrPar%Ind_GenTq)
            ENDIF

            IF (CntrPar%Ind_YawRate > 0) THEN
                CntrPar%OL_YawRate = CntrPar%OL_Channels(:,CntrPar%Ind_YawRate)
            ENDIF
        END IF

        ! Debugging outputs (echo someday)
        ! write(400,*) CntrPar%OL_YawRate

        ! END OF INPUT FILE    

        ! Close Input File
        CLOSE(UnControllerParameters)

        
        !------------------- CALCULATED CONSTANTS -----------------------
        CntrPar%PC_RtTq99 = CntrPar%VS_RtTq*0.99
        CntrPar%VS_MinOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_MinOMSpd**2
        CntrPar%VS_MaxOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_RefSpd**2
        
        !------------------- HOUSEKEEPING -----------------------
        CntrPar%PerfFileName = TRIM(CntrPar%PerfFileName)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF


    END SUBROUTINE ReadControlParameterFileSub
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadCpFile(CntrPar,PerfData, ErrVar)
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters, ErrorVariables
        
        TYPE(ControlParameters),    INTENT(INOUT)   :: CntrPar
        TYPE(PerformanceData),      INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar
        
        ! Local variables
        INTEGER(IntKi), PARAMETER                       :: UnPerfParameters = 89
        INTEGER(IntKi)                                  :: i ! iteration index

        INTEGER(IntKi)                                  :: CurLine 
        CHARACTER(*), PARAMETER                     :: RoutineName = 'ReadCpFile'
        REAL(DbKi), DIMENSION(:), ALLOCATABLE          :: TmpPerf

        CurLine = 1

        OPEN(unit=UnPerfParameters, file=TRIM(CntrPar%PerfFileName), status='old', action='read') ! Should put input file into DISCON.IN
        
        ! ----------------------- Axis Definitions ------------------------
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ParseAry(UnPerfParameters, CurLine, 'Pitch angle vector', PerfData%Beta_vec, CntrPar%PerfTableSize(1), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ParseAry(UnPerfParameters, CurLine, 'TSR vector', PerfData%TSR_vec, CntrPar%PerfTableSize(2), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)

        ! ----------------------- Read Cp, Ct, Cq, Tables ------------------------
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) ! Input file should contains wind speed information here - unneeded for now
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cp_mat(i,:) ! Read Cp table
        END DO
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Ct_mat(i,:) ! Read Ct table
        END DO
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cq_mat(i,:) ! Read Cq table
        END DO

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
    
    END SUBROUTINE ReadCpFile
    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution
    SUBROUTINE CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
    
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(IN   )       :: LocalVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar
        INTEGER(IntKi),                 INTENT(IN   )       :: size_avcMSG
        REAL(ReKi),              INTENT(IN   )       :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        
        CHARACTER(*), PARAMETER                         :: RoutineName = 'CheckInputs'
        ! Local
        
        !..............................................................................................................................
        ! Check validity of input parameters:
        !..............................................................................................................................

        !------- DEBUG ------------------------------------------------------------

        ! LoggingLevel
        IF ((CntrPar%LoggingLevel < 0) .OR. (CntrPar%LoggingLevel > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'LoggingLevel must be 0 - 3.'
        ENDIF

        !------- CONTROLLER FLAGS -------------------------------------------------

        ! F_LPFType
        IF ((CntrPar%F_LPFType < 1) .OR. (CntrPar%F_LPFType > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_LPFType must be 1 or 2.'
        ENDIF

        ! F_NotchType
        IF ((CntrPar%F_NotchType < 0) .OR. (CntrPar%F_NotchType > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_NotchType must be 0, 1, 2, or 3.'
        ENDIF

        ! F_NotchType
        IF ((CntrPar%F_NotchType < 0) .OR. (CntrPar%F_NotchType > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_NotchType must be 0, 1, or 2.'
        ENDIF

        ! IPC_ControlMode
        IF ((CntrPar%IPC_ControlMode < 0) .OR. (CntrPar%IPC_ControlMode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_ControlMode must be 0, 1, or 2.'
        ENDIF

        ! VS_ControlMode
        IF ((CntrPar%VS_ControlMode < 0) .OR. (CntrPar%VS_ControlMode > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_ControlMode must be 0, 1, 2, or 3.'
        ENDIF

        ! PC_ControlMode
        IF ((CntrPar%PC_ControlMode < 0) .OR. (CntrPar%PC_ControlMode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_ControlMode must be 0 or 1.'
        ENDIF

        ! Y_ControlMode
        IF ((CntrPar%Y_ControlMode < 0) .OR. (CntrPar%Y_ControlMode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Y_ControlMode must be 0, 1 or 2.'
        ENDIF

        IF ((CntrPar%IPC_ControlMode > 0) .AND. (CntrPar%Y_ControlMode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously'
        ENDIF

        ! SS_Mode
        IF ((CntrPar%SS_Mode < 0) .OR. (CntrPar%SS_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SS_Mode must be 0 or 1.'
        ENDIF

        ! WE_Mode
        IF ((CntrPar%WE_Mode < 0) .OR. (CntrPar%WE_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_Mode must be 0, 1, or 2.'
        ENDIF

        ! PS_Mode
        IF ((CntrPar%PS_Mode < 0) .OR. (CntrPar%PS_Mode > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PS_Mode must be 0 or 1.'
        ENDIF

        ! SD_Mode
        IF ((CntrPar%SD_Mode < 0) .OR. (CntrPar%SD_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SD_Mode must be 0 or 1.'
        ENDIF

        ! Fl_Mode
        IF ((CntrPar%Fl_Mode < 0) .OR. (CntrPar%Fl_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Fl_Mode must be 0, 1, or 2.'
        ENDIF

        ! Flp_Mode
        IF ((CntrPar%Flp_Mode < 0) .OR. (CntrPar%Flp_Mode > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Flp_Mode must be 0, 1, 2, or 3.'
        ENDIF

        IF ((CntrPar%IPC_ControlMode > 0) .AND. (CntrPar%Flp_Mode > 0)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg   = 'ROSCO does not currently support IPC_ControlMode and Flp_Mode > 0'
        ENDIF
        !------- FILTERS ----------------------------------------------------------
        
        ! F_LPFCornerFreq
        IF (CntrPar%F_LPFCornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_LPFCornerFreq must be greater than zero.'
        ENDIF

        ! F_LPFDamping
        IF (CntrPar%F_LPFType == 2) THEN
            IF (CntrPar%F_LPFDamping <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_LPFDamping must be greater than zero.'
            ENDIF
        ENDIF

        ! Notch Filter Params
        IF (CntrPar%F_NotchType > 0) THEN

            ! F_NotchCornerFreq
            IF (CntrPar%F_NotchCornerFreq <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_NotchCornerFreq must be greater than zero.'
            ENDIF

            ! F_NotchBetaNumDen(2)
            IF (CntrPar%F_NotchBetaNumDen(2) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_NotchBetaNumDen(2) must be greater than zero.'
            ENDIF
        ENDIF

        ! F_SSCornerFreq
        IF (CntrPar%F_SSCornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_SSCornerFreq must be greater than zero.'
        ENDIF

        ! F_WECornerFreq
        IF (CntrPar%F_WECornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_WECornerFreq must be greater than zero.'
        ENDIF

        ! F_FlHighPassFreq
        IF (CntrPar%F_FlHighPassFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_FlHighPassFreq must be greater than zero.'
        ENDIF

        IF (CntrPar%Fl_Mode > 0) THEN
            ! F_FlCornerFreq(1)  (frequency)
            IF (CntrPar%F_FlCornerFreq(1) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlCornerFreq(1) must be greater than zero.'
            ENDIF

            ! F_FlCornerFreq(2)  (damping)
            IF (CntrPar%F_FlCornerFreq(2) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlCornerFreq(2) must be greater than zero.'
            ENDIF
        ENDIF

        IF (CntrPar%Flp_Mode > 0) THEN
            ! F_FlpCornerFreq(1)  (frequency)
            IF (CntrPar%F_FlpCornerFreq(1) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlpCornerFreq(1) must be greater than zero.'
            ENDIF

            ! F_FlpCornerFreq(2)  (damping)
            IF (CntrPar%F_FlpCornerFreq(2) < 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlpCornerFreq(2) must be greater than or equal to zero.'
            ENDIF
        ENDIF
                     
        
        !------- BLADE PITCH CONTROL ----------------------------------------------

        ! PC_GS_n
        IF (CntrPar%PC_GS_n <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_GS_n must be greater than 0'
        ENDIF

        ! PC_GS_angles
        IF (.NOT. NonDecreasing(CntrPar%PC_GS_angles)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_GS_angles must be non-decreasing'
        ENDIF

        ! PC_GS_KP and PC_GS_KI
        ! I'd like to throw warnings if these are positive

        ! PC_MinPit and PC_MaxPit
        IF (CntrPar%PC_MinPit >= CntrPar%PC_MaxPit)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
        ENDIF

        ! PC_RefSpd
        IF (CntrPar%PC_RefSpd <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_RefSpd must be greater than zero.'
        ENDIF
        
        ! PC_MaxRat
        IF (CntrPar%PC_MaxRat <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_MaxRat must be greater than zero.'
        ENDIF

        ! PC_MinRat
        IF (CntrPar%PC_MinRat >= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_MinRat must be less than zero.'
        ENDIF

        !------- INDIVIDUAL PITCH CONTROL -----------------------------------------

        IF (CntrPar%IPC_CornerFreqAct < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Corner frequency of IPC actuator model must be positive, or set to 0 to disable.'
        ENDIF

        IF (CntrPar%IPC_KI(1) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KI(1) must be zero or greater than zero.'
        ENDIF
        
        IF (CntrPar%IPC_KI(2) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KI(2) must be zero or greater than zero.'
        ENDIF

        IF (CntrPar%IPC_KI(1) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KP(1) must be zero or greater than zero.'
        ENDIF
        
        IF (CntrPar%IPC_KI(2) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KP(2) must be zero or greater than zero.'
        ENDIF

        !------- VS TORQUE CONTROL ------------------------------------------------
       
        IF (CntrPar%VS_MaxRat <= 0.0) THEN
            ErrVar%aviFAIL =  -1
            ErrVar%ErrMsg  = 'VS_MaxRat must be greater than zero.'
        ENDIF
        
        
        
        ! VS_Rgn2K
        IF (CntrPar%VS_Rgn2K < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_Rgn2K must not be negative.'
        ENDIF
        
        ! VS_RtTq
        IF (CntrPar%VS_MaxTq < CntrPar%VS_RtTq) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.'
        ENDIF

        ! VS_RtPwr
        IF (CntrPar%VS_RtPwr < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_RtPwr must not be negative.'
        ENDIF

        ! VS_RtTq
        IF (CntrPar%VS_RtTq < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_RtTq must not be negative.'
        ENDIF

        ! VS_KP
        IF (CntrPar%VS_KP(1) > 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_KP must be less than zero.'
        ENDIF
        
        ! VS_KI
        IF (CntrPar%VS_KI(1) > 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_KI must be less than zero.'
        ENDIF

        ! VS_TSRopt
        IF (CntrPar%VS_TSRopt < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_TSRopt must be greater than zero.'
        ENDIF
        
        !------- SETPOINT SMOOTHER ---------------------------------------------

        ! SS_VSGain
        IF (CntrPar%SS_VSGain < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SS_VSGain must be greater than zero.'
        ENDIF

        ! SS_PCGain
        IF (CntrPar%SS_PCGain < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SS_PCGain must be greater than zero.'
        ENDIF
        
        !------- WIND SPEED ESTIMATOR ---------------------------------------------

        ! WE_BladeRadius
        IF (CntrPar%WE_BladeRadius < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_BladeRadius must be greater than zero.'
        ENDIF

        ! WE_GearboxRatio
        IF (CntrPar%WE_GearboxRatio < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_GearboxRatio must be greater than zero.'
        ENDIF

        ! WE_Jtot
        IF (CntrPar%WE_Jtot < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_Jtot must be greater than zero.'
        ENDIF

        ! WE_RhoAir
        IF (CntrPar%WE_RhoAir < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_RhoAir must be greater than zero.'
        ENDIF

        ! PerfTableSize(1)
        IF (CntrPar%PerfTableSize(1) < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PerfTableSize(1) must be greater than zero.'
        ENDIF

        ! PerfTableSize(2)
        IF (CntrPar%PerfTableSize(2) < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PerfTableSize(2) must be greater than zero.'
        ENDIF

        ! WE_FOPoles_N
        IF (CntrPar%WE_FOPoles_N < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_FOPoles_N must be greater than zero.'
        ENDIF

        ! WE_FOPoles_v
        IF (.NOT. NonDecreasing(CntrPar%WE_FOPoles_v)) THEN
            ErrVar%aviFAIL = -1
            write(400,*) CntrPar%WE_FOPoles_v
            ErrVar%ErrMsg  = 'WE_FOPoles_v must be non-decreasing.'
        ENDIF



        ! ---- Yaw Control ----
        IF (CntrPar%Y_ControlMode > 0) THEN
            IF (CntrPar%Y_IPC_omegaLP <= 0.0)  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'Y_IPC_omegaLP must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_IPC_zetaLP <= 0.0)  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'Y_IPC_zetaLP must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_ErrThresh <= 0.0)  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'Y_ErrThresh must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_Rate <= 0.0)  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'CntrPar%Y_Rate must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_omegaLPFast <= 0.0)  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'Y_omegaLPFast must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_omegaLPSlow <= 0.0)  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'Y_omegaLPSlow must be greater than zero.'
            ENDIF
        ENDIF

        !------- MINIMUM PITCH SATURATION -------------------------------------------
        IF (CntrPar%PS_Mode > 0) THEN

            ! PS_BldPitchMin_N
            IF (CntrPar%PS_BldPitchMin_N < 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'PS_BldPitchMin_N must be greater than zero.'
            ENDIF

            ! PS_WindSpeeds
            IF (.NOT. NonDecreasing(CntrPar%PS_WindSpeeds)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'PS_WindSpeeds must be non-decreasing.'
            ENDIF


        ENDIF

        ! --- Floating Control ---
        IF (CntrPar%Fl_Mode > 0) THEN
            IF (CntrPar%F_NotchType < 1 .OR. CntrPar%F_NotchCornerFreq == 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'F_NotchType and F_NotchCornerFreq must be specified for Fl_Mode greater than zero.'
            ENDIF
        ENDIF

        ! --- Open loop control ---
        IF (((CntrPar%Ind_Breakpoint) < 0) .OR. &
        (CntrPar%Ind_BldPitch < 0) .OR. &
        (CntrPar%Ind_GenTq < 0) .OR. &
        (CntrPar%Ind_YawRate < 0)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = 'All open loop control indices must be greater than zero'
        ENDIF
            

        
        ! Abort if the user has not requested a pitch angle actuator (See Appendix A
        ! of Bladed User's Guide):
        IF (NINT(avrSWAP(10)) /= 0)  THEN ! .TRUE. if a pitch angle actuator hasn't been requested
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Pitch angle actuator not requested.'
        ENDIF
        
        IF (NINT(avrSWAP(28)) == 0 .AND. ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Y_ControlMode > 1))) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1.'
        ENDIF

        ! DT
        IF (LocalVar%DT <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'DT must be greater than zero.'
        ENDIF

        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE CheckInputs
    
    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Int(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(20)                           :: Words       (2)               ! The two "words" parsed from the line

        INTEGER(IntKi),             INTENT(INOUT)   :: Variable   ! Variable
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Int

    !=======================================================================
    ! Parse double input, this is a copy of ParseInput_Int and a change in the variable definitions
    subroutine ParseInput_Dbl(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(20)                           :: Words       (2)               ! The two "words" parsed from the line
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        REAL(DbKi),             INTENT(INOUT)      :: Variable   ! Variable
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
            print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Dbl

    !=======================================================================
    ! Parse string input, this is a copy of ParseInput_Int and a change in the variable definitions
    subroutine ParseInput_Str(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(200)                          :: Words       (2)               ! The two "words" parsed from the line
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        CHARACTER(*),           INTENT(INOUT)   :: Variable   ! Variable
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            if (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),'(A)',IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid STRING value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Str

!=======================================================================
!> This subroutine parses the specified line of text for AryLen REAL values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
    SUBROUTINE ParseDbAry ( Un, LineNum, AryName, Ary, AryLen, FileName, ErrVar, CheckName )

        USE ROSCO_Types, ONLY : ErrorVariables

        ! Arguments declarations.
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

        REAL(DbKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

        INTEGER(IntKi),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
        CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


        CHARACTER(*),           INTENT(IN   )   :: AryName                       !< The array name we are trying to fill.

        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input

        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName


        ! Local declarations.

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.
        INTEGER(IntKi)                              :: i

        CHARACTER(200), ALLOCATABLE             :: Words_Ary       (:)               ! The array "words" parsed from the line.
        CHARACTER(1024)                         :: Debug_String 
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseDbAry'
        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN
            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Allocate array and handle errors
            ALLOCATE ( Ary(AryLen) , STAT=ErrStatLcl )
            IF ( ErrStatLcl /= 0 ) THEN
                IF ( ALLOCATED(Ary) ) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( AryName )//' array; array was already allocated.'
                ELSE
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( AryName )//' array.'
                END IF
            END IF
        
            ! Allocate words array
            ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Fatal error allocating memory for the Words array.'
                CALL Cleanup()
                RETURN
            ENDIF

            ! Separate line string into AryLen + 1 words, should include variable name
            CALL GetWords ( Line, Words_Ary, AryLen + 1 )  

            ! Debug Output
            IF (DEBUG_PARSING) THEN
                Debug_String = ''
                DO i = 1,AryLen+1
                    Debug_String = TRIM(Debug_String)//TRIM(Words_Ary(i))
                    IF (i < AryLen + 1) THEN
                        Debug_String = TRIM(Debug_String)//','
                    END IF
                END DO
                print *, 'Read: '//TRIM(Debug_String)//' on line ', LineNum
            END IF

            ! Check that Variable Name is at the end of Words, will also check length of array
            IF (CheckName_) THEN
                CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), AryName, FileName, LineNum, ErrVar )
            END IF
        
            ! Read array
            READ (Line,*,IOSTAT=ErrStatLcl)  Ary
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                                //TRIM( FileName )//'".'//NewLine//  &
                                ' >> The "'//TRIM( AryName )//'" array was not assigned valid REAL values on line #' &
                                //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                                //'    "'//TRIM( Line )//'"' 
                RETURN
                CALL Cleanup()         
            ENDIF

        !  IF ( PRESENT(UnEc) )  THEN
        !     IF ( UnEc > 0 )  WRITE (UnEc,'(A)')  TRIM( FileInfo%Lines(LineNum) )
        !  END IF

            LineNum = LineNum + 1
            CALL Cleanup()
        ENDIF

        RETURN

        !=======================================================================
        CONTAINS
        !=======================================================================
            SUBROUTINE Cleanup ( )

                ! This subroutine cleans up the parent routine before exiting.

                ! Deallocate the Words array if it had been allocated.

                IF ( ALLOCATED( Words_Ary ) ) DEALLOCATE( Words_Ary )


                RETURN

            END SUBROUTINE Cleanup

  END SUBROUTINE ParseDbAry

  !=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
  SUBROUTINE ParseInAry ( Un, LineNum, AryName, Ary, AryLen, FileName, ErrVar, CheckName )

    USE ROSCO_Types, ONLY : ErrorVariables

    ! Arguments declarations.
    INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
    INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

    INTEGER(IntKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

    INTEGER(IntKi),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
    CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


    CHARACTER(*),           INTENT(IN   )   :: AryName                       !< The array name we are trying to fill.

    TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input

    LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

    ! Local declarations.

    CHARACTER(1024)                         :: Line
    INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.
    INTEGER(IntKi)                              :: i

    CHARACTER(200), ALLOCATABLE             :: Words_Ary       (:)               ! The array "words" parsed from the line.
    CHARACTER(1024)                         :: Debug_String 
    CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInAry'

    LOGICAL                                 :: CheckName_

    ! Figure out if we're checking the name, default to .TRUE.
    CheckName_ = .TRUE.
    if (PRESENT(CheckName)) CheckName_ = CheckName    

    ! If we've already failed, don't read anything
    IF (ErrVar%aviFAIL >= 0) THEN
        ! Read the whole line as a string
        READ(Un, '(A)') Line

        ! Allocate array and handle errors
        ALLOCATE ( Ary(AryLen) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 ) THEN
            IF ( ALLOCATED(Ary) ) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( AryName )//' array; array was already allocated.'
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( AryName )//' array.'
            END IF
        END IF
    
        ! Allocate words array
        ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':Fatal error allocating memory for the Words array.'
            CALL Cleanup()
            RETURN
        ENDIF

        ! Separate line string into AryLen + 1 words, should include variable name
        CALL GetWords ( Line, Words_Ary, AryLen + 1 )  

        ! Debug Output
        IF (DEBUG_PARSING) THEN
            Debug_String = ''
            DO i = 1,AryLen+1
                Debug_String = TRIM(Debug_String)//TRIM(Words_Ary(i))
                IF (i < AryLen + 1) THEN
                    Debug_String = TRIM(Debug_String)//','
                END IF
            END DO
            print *, 'Read: '//TRIM(Debug_String)//' on line ', LineNum
        END IF

        ! Check that Variable Name is at the end of Words, will also check length of array
        IF (CheckName_) THEN
            CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), AryName, FileName, LineNum, ErrVar )
        END IF
    
        ! Read array
        READ (Line,*,IOSTAT=ErrStatLcl)  Ary
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                            //TRIM( FileName )//'".'//NewLine//  &
                            ' >> The "'//TRIM( AryName )//'" array was not assigned valid REAL values on line #' &
                            //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                            //'    "'//TRIM( Line )//'"' 
            RETURN
            CALL Cleanup()         
        ENDIF

    !  IF ( PRESENT(UnEc) )  THEN
    !     IF ( UnEc > 0 )  WRITE (UnEc,'(A)')  TRIM( FileInfo%Lines(LineNum) )
    !  END IF

        LineNum = LineNum + 1
        CALL Cleanup()
    ENDIF

    RETURN

    !=======================================================================
    CONTAINS
    !=======================================================================
        SUBROUTINE Cleanup ( )

            ! This subroutine cleans up the parent routine before exiting.

            ! Deallocate the Words array if it had been allocated.

            IF ( ALLOCATED( Words_Ary ) ) DEALLOCATE( Words_Ary )


            RETURN

        END SUBROUTINE Cleanup

END SUBROUTINE ParseInAry

!=======================================================================
 !> This subroutine checks the data to be parsed to make sure it finds
    !! the expected variable name and an associated value.
SUBROUTINE ChkParseData ( Words, ExpVarName, FileName, FileLineNum, ErrVar )

    USE ROSCO_Types, ONLY : ErrorVariables


        ! Arguments declarations.
    TYPE(ErrorVariables),         INTENT(INOUT)          :: ErrVar   ! Current line of input

    INTEGER(IntKi), INTENT(IN)             :: FileLineNum                   !< The number of the line in the file being parsed.
    INTEGER(IntKi)                        :: NameIndx                      !< The index into the Words array that points to the variable name.

    CHARACTER(*),   INTENT(IN)             :: ExpVarName                    !< The expected variable name.
    CHARACTER(*),   INTENT(IN)             :: Words       (2)               !< The two words to be parsed from the line.

    CHARACTER(*),   INTENT(IN)             :: FileName                      !< The name of the file being parsed.


        ! Local declarations.

    CHARACTER(20)                          :: ExpUCVarName                  ! The uppercase version of ExpVarName.
    CHARACTER(20)                          :: FndUCVarName                  ! The uppercase version of the word being tested.




        ! Convert the found and expected names to uppercase.

    FndUCVarName = Words(1)
    ExpUCVarName = ExpVarName

    CALL Conv2UC ( FndUCVarName )
    CALL Conv2UC ( ExpUCVarName )

    ! See which word is the variable name.  Generate an error if it is the first
        
    IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )  THEN
        NameIndx = 1
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = ' >> A fatal error occurred when parsing data from "'//TRIM( FileName ) &
                            //'".'//NewLine//' >> The variable "'//TRIM( Words(1) )//'" was not assigned a valid value on line #' &
                            //TRIM( Int2LStr( FileLineNum ) )//'.' 
        RETURN
    ELSE
        FndUCVarName = Words(2)
        CALL Conv2UC ( FndUCVarName )
        IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )  THEN
        NameIndx = 2
        ELSE
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = ' >> A fatal error occurred when parsing data from "'//TRIM( FileName ) &
                            //'".'//NewLine//' >> The variable "'//TRIM( ExpVarName )//'" was not assigned a valid value on line #' &
                            //TRIM( Int2LStr( FileLineNum ) )//'.' 
        RETURN
        ENDIF
    ENDIF


END SUBROUTINE ChkParseData 

!=======================================================================
subroutine ReadEmptyLine(Un,CurLine)
    INTEGER(IntKi),         INTENT(IN   )          :: Un   ! Input file unit
    INTEGER(IntKi),         INTENT(INOUT)          :: CurLine   ! Current line of input

    CHARACTER(1024)                            :: Line

    READ(Un, '(A)') Line
    CurLine = CurLine + 1

END subroutine ReadEmptyLine

!=======================================================================
!> This subroutine is used to get the NumWords "words" from a line of text.
!! It uses spaces, tabs, commas, semicolons, single quotes, and double quotes ("whitespace")
!! as word separators. If there aren't NumWords in the line, the remaining array elements will remain empty.
!! Use CountWords (nwtc_io::countwords) to count the number of words in a line.
SUBROUTINE GetWords ( Line, Words, NumWords )

    ! Argument declarations.

    INTEGER, INTENT(IN)          :: NumWords                                     !< The number of words to look for.

    CHARACTER(*), INTENT(IN)     :: Line                                         !< The string to search.
    CHARACTER(*), INTENT(OUT)    :: Words(NumWords)                              !< The array of found words.


        ! Local declarations.

    INTEGER                      :: Ch                                           ! Character position within the string.
    INTEGER                      :: IW                                           ! Word index.
    INTEGER                      :: NextWhite                                    ! The location of the next whitespace in the string.
    CHARACTER(1), PARAMETER       :: Tab      = CHAR( 9 ) 



        ! Let's prefill the array with blanks.

    DO IW=1,NumWords
        Words(IW) = ' '
    END DO ! IW


        ! Let's make sure we have text on this line.

    IF ( LEN_TRIM( Line ) == 0 )  RETURN


        ! Parse words separated by any combination of spaces, tabs, commas,
        ! semicolons, single quotes, and double quotes ("whitespace").

    Ch = 0
    IW = 0

    DO

        NextWhite = SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )

        IF ( NextWhite > 1 )  THEN

        IW        = IW + 1
        Words(IW) = Line(Ch+1:Ch+NextWhite-1)

        IF ( IW == NumWords )  EXIT

        Ch = Ch + NextWhite

        ELSE IF ( NextWhite == 1 )  THEN

        Ch = Ch + 1

        CYCLE

        ELSE

        EXIT

        END IF

    END DO


    RETURN
END SUBROUTINE GetWords
!=======================================================================
!> Let's parse the path name from the name of the given file.
!! We'll count everything before (and including) the last "\" or "/".
SUBROUTINE GetPath ( GivenFil, PathName )

    ! Argument declarations.

 CHARACTER(*), INTENT(IN)     :: GivenFil                                     !< The name of the given file.
 CHARACTER(*), INTENT(OUT)    :: PathName                                     !< The path name of the given file (based solely on the GivenFil text string).


    ! Local declarations.

 INTEGER                      :: I                                            ! DO index for character position.


    ! Look for path separators

 I = INDEX( GivenFil, '\', BACK=.TRUE. )
 I = MAX( I, INDEX( GivenFil, '/', BACK=.TRUE. ) )

 IF ( I == 0 ) THEN
    ! we don't have a path specified, return '.'
    PathName = '.'//PathSep
 ELSE
    PathName = GivenFil(:I)
 END IF


 RETURN
 END SUBROUTINE GetPath
!=======================================================================
!> This routine determines if the given file name is absolute or relative.
!! We will consider an absolute path one that satisfies one of the
!! following four criteria:
!!     1. It contains ":/"
!!     2. It contains ":\"
!!     3. It starts with "/"
!!     4. It starts with "\"
!!   
!! All others are considered relative.
 FUNCTION PathIsRelative ( GivenFil )

    ! Argument declarations.

 CHARACTER(*), INTENT(IN)     :: GivenFil                                            !< The name of the given file.
 LOGICAL                      :: PathIsRelative                                      !< The function return value

 

    ! Determine if file name begins with an absolute path name or if it is relative 
    !    note that Doxygen has serious issues if you use the single quote instead of  
    !    double quote characters in the strings below:

 PathIsRelative = .FALSE.

 IF ( ( INDEX( GivenFil, ":/") == 0 ) .AND. ( INDEX( GivenFil, ":\") == 0 ) ) THEN   ! No drive is specified (by ":\" or ":/")

    IF ( INDEX( "/\", GivenFil(1:1) ) == 0 ) THEN                                    ! The file name doesn't start with "\" or "/"

       PathIsRelative = .TRUE.

    END IF

 END IF

 RETURN
 END FUNCTION PathIsRelative
!=======================================================================
! ------------------------------------------------------
    ! Read Open Loop Control Inputs
    ! 
    ! Timeseries or lookup tables of the form
    ! index (time or wind speed)   channel_1 \t channel_2 \t channel_3 ...
    ! This could be used to read any group of data of unspecified length ...
SUBROUTINE Read_OL_Input(OL_InputFileName, Unit_OL_Input, NumChannels, Channels, ErrVar)

    USE ROSCO_Types, ONLY : ErrorVariables

    CHARACTER(1024), INTENT(IN)                             :: OL_InputFileName    ! DISCON input filename
    INTEGER(IntKi), INTENT(IN)                              :: Unit_OL_Input 
    INTEGER(IntKi), INTENT(IN)                              :: NumChannels     ! Number of open loop channels being defined
    ! REAL(DbKi), INTENT(OUT), DIMENSION(:), ALLOCATABLE      :: Breakpoints    ! Breakpoints of open loop Channels
    REAL(DbKi), INTENT(OUT), DIMENSION(:,:), ALLOCATABLE    :: Channels         ! Open loop channels
    TYPE(ErrorVariables),         INTENT(INOUT)          :: ErrVar   ! Current line of input


    LOGICAL                                                 :: FileExists
    INTEGER                                                 :: IOS                                                 ! I/O status of OPEN.
    CHARACTER(1024)                                         :: Line              ! Temp variable for reading whole line from file
    INTEGER(IntKi)                                          :: NumComments
    INTEGER(IntKi)                                          :: NumDataLines
    REAL(DbKi)                                              :: TmpData(NumChannels)  ! Temp variable for reading all columns from a line
    CHARACTER(15)                                           :: NumString

    INTEGER(IntKi)                                          :: I,J

    CHARACTER(*),               PARAMETER                   :: RoutineName = 'Read_OL_Input'

    !-------------------------------------------------------------------------------------------------
    ! Read from input file, borrowed (read: copied) from (Open)FAST team...thanks!
    !-------------------------------------------------------------------------------------------------

    !-------------------------------------------------------------------------------------------------
    ! Open the file for reading
    !-------------------------------------------------------------------------------------------------

    INQUIRE (FILE = OL_InputFileName, EXIST = FileExists)

    IF ( .NOT. FileExists) THEN
        ErrVar%aviFAIL = -1
        ErrVar%ErrMsg = TRIM(OL_InputFileName)// ' does not exist'

    ELSE

        OPEN( Unit_OL_Input, FILE=TRIM(OL_InputFileName), STATUS='OLD', FORM='FORMATTED', IOSTAT=IOS, ACTION='READ' )

        IF (IOS /= 0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = 'Cannot open '//TRIM(OL_InputFileName)
        
        ELSE
            ! Do all the stuff!
            !-------------------------------------------------------------------------------------------------
            ! Find the number of comment lines
            !-------------------------------------------------------------------------------------------------

            LINE = '!'                          ! Initialize the line for the DO WHILE LOOP
            NumComments = -1                    ! the last line we read is not a comment, so we'll initialize this to -1 instead of 0

            DO WHILE ( (INDEX( LINE, '!' ) > 0) .OR. (INDEX( LINE, '#' ) > 0) .OR. (INDEX( LINE, '%' ) > 0) ) ! Lines containing "!" are treated as comment lines
                NumComments = NumComments + 1
                
                READ(Unit_OL_Input,'( A )',IOSTAT=IOS) LINE

                ! NWTC_IO has some error catching here that we'll skip for now
        
            END DO !WHILE

            !-------------------------------------------------------------------------------------------------
            ! Find the number of data lines
            !-------------------------------------------------------------------------------------------------

            NumDataLines = 0

            READ(LINE,*,IOSTAT=IOS) ( TmpData(I), I=1,NumChannels ) ! this line was read when we were figuring out the comment lines; let's make sure it contains

            DO WHILE (IOS == 0)  ! read the rest of the file (until an error occurs)
                NumDataLines = NumDataLines + 1
                
                READ(Unit_OL_Input,*,IOSTAT=IOS) ( TmpData(I), I=1,NumChannels )
            
            END DO !WHILE
        
        
            IF (NumDataLines < 1) THEN
                WRITE (NumString,'(I11)')  NumComments
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'Error: '//TRIM(NumString)//' comment lines were found in the uniform wind file, '// &
                            'but the first data line does not contain the proper format.'
                CLOSE(Unit_OL_Input)
            END IF

            !-------------------------------------------------------------------------------------------------
            ! Allocate arrays for the uniform wind data
            !-------------------------------------------------------------------------------------------------
            ALLOCATE(Channels(NumDataLines,NumChannels))

            !-------------------------------------------------------------------------------------------------
            ! Rewind the file (to the beginning) and skip the comment lines
            !-------------------------------------------------------------------------------------------------

            REWIND( Unit_OL_Input )

            DO I=1,NumComments
                READ(Unit_OL_Input,'( A )',IOSTAT=IOS) LINE
            END DO !I
        
            !-------------------------------------------------------------------------------------------------
            ! Read the data arrays
            !-------------------------------------------------------------------------------------------------
        
            DO I=1,NumDataLines
            
                READ(Unit_OL_Input,*,IOSTAT=IOS) ( TmpData(J), J=1,NumChannels )

                IF (IOS > 0) THEN
                    CLOSE(Unit_OL_Input)
                END IF

                Channels(I,:)        = TmpData
        
            END DO !I     
        END IF
    END IF

    IF (ErrVar%aviFAIL < 0) THEN
        ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    ENDIF

END SUBROUTINE Read_OL_Input

END MODULE ReadSetParameters
