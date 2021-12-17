! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! This handles all of ROSCO's input/output files ~WITH THE EXCPETION~ of reading the primary
! input files. The DISCON.IN and Cp_Ct_Cq.txt files are handed in ReadSetParameters.f90

MODULE ROSCO_IO
    USE, INTRINSIC :: ISO_C_Binding
    USE ROSCO_Types
    USE ReadSetParameters
    IMPLICIT NONE
CONTAINS
    SUBROUTINE WriteRestartFile(LocalVar, CntrPar, objInst, RootName, size_avcOUTNAME)
        USE ROSCO_Types, ONLY : LocalVariables, ObjectInstances, ControlParameters
        
        TYPE(LocalVariables), INTENT(IN)                :: LocalVar
        TYPE(ControlParameters), INTENT(INOUT)          :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT)            :: objInst
        INTEGER(4), INTENT(IN)                          :: size_avcOUTNAME
        CHARACTER(size_avcOUTNAME-1), INTENT(IN)        :: RootName 
        
        INTEGER(4), PARAMETER        :: Un = 87             ! I/O unit for pack/unpack (checkpoint & restart)
        INTEGER(4)                   :: I                   ! Generic index.
        CHARACTER(128)               :: InFile              ! Input checkpoint file
        INTEGER(4)                   :: ErrStat
        CHARACTER(128)               :: ErrMsg              
        CHARACTER(128)               :: n_t_global          ! timestep number as a string

        WRITE(n_t_global, '(I0.0)' ), NINT(LocalVar%Time/LocalVar%DT)
        InFile = RootName(1:size_avcOUTNAME-5)//TRIM( n_t_global )//'.RO.chkp'
        OPEN(unit=Un, FILE=TRIM(InFile), STATUS='UNKNOWN', FORM='UNFORMATTED' , ACCESS='STREAM', IOSTAT=ErrStat, ACTION='WRITE' )

        IF ( ErrStat /= 0 ) THEN
            ErrMsg  = 'Cannot open file "'//TRIM( InFile )//'". Another program may have locked it for writing.'
            
        ELSE
            ! From AVR SWAP
            WRITE( Un, IOSTAT=ErrStat) LocalVar%iStatus
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Time
            WRITE( Un, IOSTAT=ErrStat) LocalVar%DT
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_GenPwr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%GenSpeed
            WRITE( Un, IOSTAT=ErrStat) LocalVar%RotSpeed
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_M
            WRITE( Un, IOSTAT=ErrStat) LocalVar%HorWindV
            WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%BlPitch(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%BlPitch(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%BlPitch(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Azimuth
            WRITE( Un, IOSTAT=ErrStat) LocalVar%NumBl
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_Acc
            WRITE( Un, IOSTAT=ErrStat) LocalVar%NacIMU_FA_Acc
            ! Internal Control Variables
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPFI
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%RotSpeedF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%GenSpeedF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%GenTq
            WRITE( Un, IOSTAT=ErrStat) LocalVar%GenTqMeas
            WRITE( Un, IOSTAT=ErrStat) LocalVar%GenArTq
            WRITE( Un, IOSTAT=ErrStat) LocalVar%GenBrTq
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisTilt_1P
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisYaw_1P
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisTilt_2P
            WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisYaw_2P
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_KP
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_KI
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_KD
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_TF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_MaxPit
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_MinPit
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_Last
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComTF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_PwrErr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_SpdErr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_State
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%SS_DelOmegaF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%TestType
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_MaxTq
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenTrq
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenPwr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_MechGenPwr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrAr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrBr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_State
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_Rgn3Pitch
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_Vw
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_Vw_F
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_VwI
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE_VwIdot
            WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenTrqF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_AccErr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFFast
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFSlow
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_MErr
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Y_YawEndT
            WRITE( Un, IOSTAT=ErrStat) LocalVar%SD
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Fl_PitCom
            WRITE( Un, IOSTAT=ErrStat) LocalVar%NACIMU_FA_AccF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_AccF
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%RootMyb_Last(1)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%RootMyb_Last(2)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%RootMyb_Last(3)
            WRITE( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE_SIZE
            WRITE( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE
            WRITE( Un, IOSTAT=ErrStat) LocalVar%restart

            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%om_r
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%v_t
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%v_m
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%v_h
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%P
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%xh
            WRITE( Un, IOSTAT=ErrStat) LocalVar%WE%K

            WRITE( Un, IOSTAT=ErrStat) objInst%instLPF
            WRITE( Un, IOSTAT=ErrStat) objInst%instSecLPF
            WRITE( Un, IOSTAT=ErrStat) objInst%instHPF
            WRITE( Un, IOSTAT=ErrStat) objInst%instNotchSlopes
            WRITE( Un, IOSTAT=ErrStat) objInst%instNotch
            WRITE( Un, IOSTAT=ErrStat) objInst%instPI

            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_a1
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_a0
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_b1
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_b0
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_InputSignalLast
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_OutputSignalLast
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_a2
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_a1
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_a0
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_b2
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_b1
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_b0
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_InputSignalLast2
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_OutputSignalLast2
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_InputSignalLast1
            WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_OutputSignalLast1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%hpf_InputSignalLast
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%hpf_OutputSignalLast
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_b2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_b0
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_a2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_a1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nfs_a0
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_b2
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_b1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_b0
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_a1
            WRITE( un, IOSTAT=ErrStat) LocalVar%FP%nf_a0

            WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm
            WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast
            WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm2
            WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast2

            CLOSE ( Un )

        ENDIF
        
    END SUBROUTINE WriteRestartFile

    SUBROUTINE ReadRestartFile(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, size_avcOUTNAME, ErrVar)
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, ErrorVariables
        
        TYPE(LocalVariables), INTENT(INOUT)    :: LocalVar
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        TYPE(ObjectInstances), INTENT(INOUT)   :: objInst
        TYPE(PerformanceData), INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables), INTENT(INOUT)    :: ErrVar


        REAL(C_FLOAT), INTENT(IN)                   :: avrSWAP(*)   
        INTEGER(4), INTENT(IN)                      :: size_avcOUTNAME
        CHARACTER(size_avcOUTNAME-1), INTENT(IN)    :: RootName     
        
        INTEGER(4), PARAMETER        :: Un            = 87         ! I/O unit for pack/unpack (checkpoint & restart)
        INTEGER(4)                   :: I                          ! Generic index.  
        INTEGER(4)                   :: ErrStat  
        CHARACTER(128)               :: ErrMsg 
        CHARACTER(128)               :: n_t_global                 ! timestep number as a string
        CHARACTER(128)               :: InFile                     ! Name of ROSCO checkpoint file

        WRITE(n_t_global, '(I0.0)' ), NINT(avrSWAP(2)/avrSWAP(3))
        InFile = RootName(1:size_avcOUTNAME-5)//TRIM( n_t_global )//'.RO.chkp'
        OPEN(unit=Un, FILE=TRIM(InFile), STATUS='UNKNOWN', FORM='UNFORMATTED' , ACCESS='STREAM', IOSTAT=ErrStat, ACTION='READ' )

        IF ( ErrStat /= 0 ) THEN
            ErrMsg  = 'Cannot open file "'//TRIM( InFile )//'". Another program may have locked it for writing.'
            
        ELSE
            ! From AVR SWAP
            READ( Un, IOSTAT=ErrStat) LocalVar%iStatus
            READ( Un, IOSTAT=ErrStat) LocalVar%Time
            READ( Un, IOSTAT=ErrStat) LocalVar%DT
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_GenPwr
            READ( Un, IOSTAT=ErrStat) LocalVar%GenSpeed
            READ( Un, IOSTAT=ErrStat) LocalVar%RotSpeed
            READ( Un, IOSTAT=ErrStat) LocalVar%Y_M
            READ( Un, IOSTAT=ErrStat) LocalVar%HorWindV
            READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%BlPitch(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%BlPitch(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%BlPitch(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%Azimuth
            READ( Un, IOSTAT=ErrStat) LocalVar%NumBl
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_Acc
            READ( Un, IOSTAT=ErrStat) LocalVar%NacIMU_FA_Acc
            ! Internal Control Variables
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPF
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_AccHPFI
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_PitCom(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%RotSpeedF
            READ( Un, IOSTAT=ErrStat) LocalVar%GenSpeedF
            READ( Un, IOSTAT=ErrStat) LocalVar%GenTq
            READ( Un, IOSTAT=ErrStat) LocalVar%GenTqMeas
            READ( Un, IOSTAT=ErrStat) LocalVar%GenArTq
            READ( Un, IOSTAT=ErrStat) LocalVar%GenBrTq
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_PitComF(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisTilt_1P
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisYaw_1P
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisTilt_2P
            READ( Un, IOSTAT=ErrStat) LocalVar%IPC_IntAxisYaw_2P
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_KP
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_KI
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_KD
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_TF
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_MaxPit
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_MinPit
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_Last
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComTF
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PitComT_IPC(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_PwrErr
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_SpdErr
            READ( Un, IOSTAT=ErrStat) LocalVar%PC_State
            READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%SS_DelOmegaF
            READ( Un, IOSTAT=ErrStat) LocalVar%TestType
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_MaxTq
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenTrq
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenPwr
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_MechGenPwr
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrAr
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErrBr
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_SpdErr
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_State
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_Rgn3Pitch
            READ( Un, IOSTAT=ErrStat) LocalVar%WE_Vw
            READ( Un, IOSTAT=ErrStat) LocalVar%WE_Vw_F
            READ( Un, IOSTAT=ErrStat) LocalVar%WE_VwI
            READ( Un, IOSTAT=ErrStat) LocalVar%WE_VwIdot
            READ( Un, IOSTAT=ErrStat) LocalVar%VS_LastGenTrqF
            READ( Un, IOSTAT=ErrStat) LocalVar%Y_AccErr
            READ( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFFast
            READ( Un, IOSTAT=ErrStat) LocalVar%Y_ErrLPFSlow
            READ( Un, IOSTAT=ErrStat) LocalVar%Y_MErr
            READ( Un, IOSTAT=ErrStat) LocalVar%Y_YawEndT
            READ( Un, IOSTAT=ErrStat) LocalVar%SD
            READ( Un, IOSTAT=ErrStat) LocalVar%Fl_PitCom
            READ( Un, IOSTAT=ErrStat) LocalVar%NACIMU_FA_AccF
            READ( Un, IOSTAT=ErrStat) LocalVar%FA_AccF
            READ( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%Flp_Angle(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%RootMyb_Last(1)
            READ( Un, IOSTAT=ErrStat) LocalVar%RootMyb_Last(2)
            READ( Un, IOSTAT=ErrStat) LocalVar%RootMyb_Last(3)
            READ( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE_SIZE
            ALLOCATE(LocalVar%ACC_INFILE(LocalVar%ACC_INFILE_SIZE))
            READ( Un, IOSTAT=ErrStat) LocalVar%ACC_INFILE
            READ( Un, IOSTAT=ErrStat) LocalVar%restart

            READ( Un, IOSTAT=ErrStat) LocalVar%WE%om_r
            READ( Un, IOSTAT=ErrStat) LocalVar%WE%v_t
            READ( Un, IOSTAT=ErrStat) LocalVar%WE%v_m
            READ( Un, IOSTAT=ErrStat) LocalVar%WE%v_h
            READ( Un, IOSTAT=ErrStat) LocalVar%WE%P
            READ( Un, IOSTAT=ErrStat) LocalVar%WE%xh
            READ( Un, IOSTAT=ErrStat) LocalVar%WE%K

            READ( Un, IOSTAT=ErrStat) objInst%instLPF
            READ( Un, IOSTAT=ErrStat) objInst%instSecLPF
            READ( Un, IOSTAT=ErrStat) objInst%instHPF
            READ( Un, IOSTAT=ErrStat) objInst%instNotchSlopes
            READ( Un, IOSTAT=ErrStat) objInst%instNotch
            READ( Un, IOSTAT=ErrStat) objInst%instPI

            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_a1
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_a0
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_b1
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_b0
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_InputSignalLast
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf1_OutputSignalLast
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_a2
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_a1
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_a0
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_b2
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_b1
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_b0
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_InputSignalLast2
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_OutputSignalLast2
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_InputSignalLast1
            READ( Un, IOSTAT=ErrStat) LocalVar%FP%lpf2_OutputSignalLast1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%hpf_InputSignalLast
            READ( un, IOSTAT=ErrStat) LocalVar%FP%hpf_OutputSignalLast
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_b2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_b0
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_a2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_a1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nfs_a0
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_b2
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_b1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_b0
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_a1
            READ( un, IOSTAT=ErrStat) LocalVar%FP%nf_a0

            READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm
            READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast
            READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm2
            READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast2

            CLOSE ( Un )
        ENDIF

        ! Read Parameter files
        CALL ReadControlParameterFileSub(CntrPar, LocalVar%ACC_INFILE, LocalVar%ACC_INFILE_SIZE, ErrVar)
        IF (CntrPar%WE_Mode > 0) THEN
            CALL READCpFile(CntrPar, PerfData, ErrVar)
        ENDIF
        
    END SUBROUTINE ReadRestartFile

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Debug(LocalVar, CntrPar, DebugVar, avrSWAP, RootName, size_avcOUTNAME)
    ! Debug routine, defines what gets printed to DEBUG.dbg if LoggingLevel = 1
    
        USE, INTRINSIC  :: ISO_C_Binding
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables
        
        IMPLICIT NONE
    
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(IN)        :: LocalVar
        TYPE(DebugVariables), INTENT(IN)        :: DebugVar
    
        INTEGER(IntKi), INTENT(IN)                      :: size_avcOUTNAME
        INTEGER(IntKi)                                  :: I , nDebugOuts               ! Generic index.
        CHARACTER(1), PARAMETER                     :: Tab = CHAR(9)                        ! The tab character.
        CHARACTER(29), PARAMETER                    :: FmtDat = "(F10.3,TR5,99(ES10.3E2,TR5:))"   ! The format of the debugging data
        INTEGER(IntKi), PARAMETER                       :: UnDb = 85        ! I/O unit for the debugging information
        INTEGER(IntKi), PARAMETER                       :: UnDb2 = 86       ! I/O unit for the debugging information, avrSWAP
        REAL(ReKi), INTENT(INOUT)                :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(size_avcOUTNAME-1), INTENT(IN)    :: RootName     ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
        CHARACTER(200)                              :: Version      ! git version of ROSCO
        CHARACTER(10)                               :: DebugOutStr1,  DebugOutStr2, DebugOutStr3, DebugOutStr4, DebugOutStr5, &
                                                         DebugOutStr6, DebugOutStr7, DebugOutStr8, DebugOutStr9, DebugOutStr10, &
                                                         DebugOutStr11, DebugOutStr12, DebugOutStr13, DebugOutStr14, DebugOutStr15, & 
                                                         DebugOutStr16, DebugOutStr17, DebugOutStr18, DebugOutStr19, DebugOutStr20                                                           
        CHARACTER(10)                               :: DebugOutUni1,  DebugOutUni2, DebugOutUni3, DebugOutUni4, DebugOutUni5, &
                                                         DebugOutUni6, DebugOutUni7, DebugOutUni8, DebugOutUni9, DebugOutUni10, &
                                                         DebugOutUni11, DebugOutUni12, DebugOutUni13, DebugOutUni14, DebugOutUni15, & 
                                                         DebugOutUni16, DebugOutUni17, DebugOutUni18, DebugOutUni19, DebugOutUni20 
        CHARACTER(10), ALLOCATABLE                  :: DebugOutStrings(:), DebugOutUnits(:)
        REAL(DbKi), ALLOCATABLE                        :: DebugOutData(:)

        ! Set up Debug Strings and Data
        ! Note that Debug strings have 10 character limit
        nDebugOuts = 18
        ALLOCATE(DebugOutData(nDebugOuts))
        !                 Header                            Unit                                Variable
        ! Filters
        DebugOutStr1   = 'FA_AccF';     DebugOutUni1   = '(rad/s^2)';      DebugOutData(1)   = LocalVar%NacIMU_FA_AccF
        DebugOutStr2   = 'FA_AccR';     DebugOutUni2   = '(rad/s^2)';  DebugOutData(2)   = LocalVar%NacIMU_FA_Acc
        DebugOutStr3  = 'RotSpeed';     DebugOutUni3  = '(rad/s)';     DebugOutData(3)  = LocalVar%RotSpeed
        DebugOutStr4  = 'RotSpeedF';    DebugOutUni4  = '(rad/s)';     DebugOutData(4)  = LocalVar%RotSpeedF
        DebugOutStr5  = 'GenSpeed';     DebugOutUni5  = '(rad/s)';     DebugOutData(5)  = LocalVar%GenSpeed
        DebugOutStr6  = 'GenSpeedF';    DebugOutUni6  = '(rad/s)';     DebugOutData(6)  = LocalVar%GenSpeedF
        ! Floating
        DebugOutStr7  = 'FA_Acc';        DebugOutUni7  = '(m/s^2)';    DebugOutData(7)  = LocalVar%FA_Acc
        DebugOutStr8  = 'Fl_Pitcom';     DebugOutUni8  = '(rad)';      DebugOutData(8)  = LocalVar%Fl_Pitcom
        DebugOutStr9  = 'PC_MinPit';     DebugOutUni9  = '(rad)';      DebugOutData(9)  = LocalVar%PC_MinPit
        DebugOutStr10  = 'SS_dOmF';      DebugOutUni10  = '(rad/s)';   DebugOutData(10)  = LocalVar%SS_DelOmegaF
        ! WSE
        DebugOutStr11  = 'WE_Vw';        DebugOutUni11  = '(m/s)';     DebugOutData(11)  = LocalVar%WE_Vw
        DebugOutStr12  = 'WE_b';         DebugOutUni12  = '(deg)';     DebugOutData(12)  = DebugVar%WE_b
        DebugOutStr13  = 'WE_t';         DebugOutUni13  = '(Nm)';      DebugOutData(13)  = DebugVar%WE_t
        DebugOutStr14  = 'WE_w';         DebugOutUni14  = '(rad/s)';   DebugOutData(14)  = DebugVar%WE_w
        DebugOutStr15  = 'WE_Vm';        DebugOutUni15  = '(m/s)';     DebugOutData(15)  = DebugVar%WE_Vm
        DebugOutStr16  = 'WE_Vt';        DebugOutUni16  = '(m/s)';     DebugOutData(16)  = DebugVar%WE_Vt
        DebugOutStr17  = 'WE_lambda';    DebugOutUni17  = '(-)';   DebugOutData(17)  = DebugVar%WE_lambda
        DebugOutStr18  = 'WE_Cp';        DebugOutUni18  = '(-)';       DebugOutData(18)  = DebugVar%WE_Cp

        Allocate(DebugOutStrings(nDebugOuts))
        Allocate(DebugOutUnits(nDebugOuts))
        DebugOutStrings =   [CHARACTER(10)  :: DebugOutStr1, DebugOutStr2, DebugOutStr3, DebugOutStr4, &
                                                DebugOutStr5, DebugOutStr6, DebugOutStr7, DebugOutStr8, &
                                                DebugOutStr9, DebugOutStr10, DebugOutStr11, DebugOutStr12, &
                                                DebugOutStr13, DebugOutStr14, DebugOutStr15, DebugOutStr16, &
                                                DebugOutStr17, DebugOutStr18]
        DebugOutUnits =     [CHARACTER(10)  :: DebugOutUni1, DebugOutUni2, DebugOutUni3, DebugOutUni4, &
                                                DebugOutUni5, DebugOutUni6, DebugOutUni7, DebugOutUni8, &
                                                DebugOutUni9, DebugOutUni10, DebugOutUni11, DebugOutUni12, &
                                                DebugOutUni13, DebugOutUni14, DebugOutUni15, DebugOutUni1, &
                                                DebugOutUni17, DebugOutUni18]
        
        ! Initialize debug file
        IF ((LocalVar%iStatus == 0) .OR. (LocalVar%iStatus == -8))  THEN  ! .TRUE. if we're on the first call to the DLL
        ! If we're debugging, open the debug file and write the header:
            ! Note that the headers will be Truncated to 10 characters!!
            IF (CntrPar%LoggingLevel > 0) THEN
                OPEN(unit=UnDb, FILE=RootName(1:size_avcOUTNAME-5)//'RO.dbg')
                WRITE (UnDb,*)  'Generated on '//CurDate()//' at '//CurTime()//' using ROSCO-'//TRIM(rosco_version)
                WRITE (UnDb,'(99(a10,TR5:))') 'Time',   DebugOutStrings
                WRITE (UnDb,'(99(a10,TR5:))') '(sec)',  DebugOutUnits
            END IF
            
            IF (CntrPar%LoggingLevel > 1) THEN 
                OPEN(unit=UnDb2, FILE=RootName(1:size_avcOUTNAME-5)//'RO.dbg2')
                WRITE(UnDb2,'(/////)')
                WRITE(UnDb2,'(A,85("'//Tab//'AvrSWAP(",I2,")"))')  'LocalVar%Time ', (i,i=1,85)
                WRITE(UnDb2,'(A,85("'//Tab//'(-)"))')  '(s)'
            END IF
        ELSE
            ! Print simulation status, every 10 seconds
            IF (MODULO(LocalVar%Time, 10.0_DbKi) == 0) THEN
                WRITE(*, 100) LocalVar%GenSpeedF*RPS2RPM, LocalVar%BlPitch(1)*R2D, avrSWAP(15)/1000.0, LocalVar%WE_Vw ! LocalVar%Time !/1000.0
                100 FORMAT('Generator speed: ', f6.1, ' RPM, Pitch angle: ', f5.1, ' deg, Power: ', f7.1, ' kW, Est. wind Speed: ', f5.1, ' m/s')
            END IF
            
        ENDIF

        ! Write debug files
        IF (CntrPar%LoggingLevel > 0) THEN
            WRITE (UnDb,FmtDat)  LocalVar%Time, DebugOutData
        END IF

        IF (CntrPar%LoggingLevel > 1) THEN
            WRITE (UnDb2,FmtDat)    LocalVar%Time, avrSWAP(1:85)
        END IF

    END SUBROUTINE Debug

END MODULE ROSCO_IO