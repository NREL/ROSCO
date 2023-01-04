! ROSCO IO
! This file is automatically generated by write_registry.py using ROSCO v2.6.0
! For any modification to the registry, please edit the rosco_types.yaml accordingly
 
MODULE ROSCO_IO
    USE, INTRINSIC :: ISO_C_Binding
    USE ROSCO_Types
    USE ReadSetParameters
    USE Constants
IMPLICIT NONE

CONTAINS

SUBROUTINE WriteRestartFile(LocalVar, CntrPar, ErrVar, objInst, RootName, size_avcOUTNAME)
    TYPE(LocalVariables), INTENT(IN)                :: LocalVar
    TYPE(ControlParameters), INTENT(INOUT)          :: CntrPar
    TYPE(ObjectInstances), INTENT(INOUT)            :: objInst
    TYPE(ErrorVariables), INTENT(INOUT)             :: ErrVar
    INTEGER(IntKi), INTENT(IN)                      :: size_avcOUTNAME
    CHARACTER(size_avcOUTNAME-1), INTENT(IN)        :: RootName 
    
    INTEGER(IntKi)               :: Un                  ! I/O unit for pack/unpack (checkpoint & restart)
    INTEGER(IntKi)               :: I                   ! Generic index.
    CHARACTER(128)               :: InFile              ! Input checkpoint file
    INTEGER(IntKi)               :: ErrStat
    CHARACTER(128)               :: ErrMsg              
    CHARACTER(128)               :: n_t_global          ! timestep number as a string

    WRITE(n_t_global, '(I0.0)' ) NINT(LocalVar%Time/LocalVar%DT)
    InFile = TRIM(RootName)//TRIM( n_t_global )//'.RO.chkp'
    CALL GetNewUnit(Un, ErrVar)
    OPEN(unit=Un, FILE=TRIM(InFile), STATUS='UNKNOWN', FORM='UNFORMATTED' , ACCESS='STREAM', IOSTAT=ErrStat, ACTION='WRITE' )

    IF ( ErrStat /= 0 ) THEN
        ErrMsg  = 'Cannot open file //TRIM( InFile )//. Another program may have locked it for writing.'

    ELSE
        WRITE( Un, IOSTAT=ErrStat) LocalVar%iStatus
        WRITE( Un, IOSTAT=ErrStat) LocalVar%Time
        WRITE( Un, IOSTAT=ErrStat) LocalVar%DT
        WRITE( Un, IOSTAT=ErrStat) LocalVar%VS_GenPwr
        WRITE( Un, IOSTAT=ErrStat) LocalVar%GenSpeed
        WRITE( Un, IOSTAT=ErrStat) LocalVar%RotSpeed
        WRITE( Un, IOSTAT=ErrStat) LocalVar%NacHeading
        WRITE( Un, IOSTAT=ErrStat) LocalVar%NacVane
        WRITE( Un, IOSTAT=ErrStat) LocalVar%HorWindV
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(3)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOPF(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOPF(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rootMOOPF(3)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%BlPitch(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%BlPitch(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%BlPitch(3)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%Azimuth
        WRITE( Un, IOSTAT=ErrStat) LocalVar%NumBl
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_Acc
        WRITE( Un, IOSTAT=ErrStat) LocalVar%NacIMU_FA_Acc
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
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisTilt_1P
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisYaw_1P
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisTilt_2P
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisYaw_2P
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_KI(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_KI(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_KP(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%IPC_KP(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PC_State
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PitCom(3)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PitComAct(1)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PitComAct(2)
        WRITE( Un, IOSTAT=ErrStat) LocalVar%PitComAct(3)
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
        WRITE( Un, IOSTAT=ErrStat) LocalVar%SD
        WRITE( Un, IOSTAT=ErrStat) LocalVar%Fl_PitCom
        WRITE( Un, IOSTAT=ErrStat) LocalVar%NACIMU_FA_AccF
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_AccF
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FA_Hist
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
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%hpf_InputSignalLast
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%hpf_OutputSignalLast
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_b2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_b0
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_a2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_a1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_a0
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_b2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_b1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_b0
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_a1
        WRITE( Un, IOSTAT=ErrStat) LocalVar%FP%nf_a0
        WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm
        WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast
        WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast2
        WRITE( Un, IOSTAT=ErrStat) LocalVar%rlP%LastSignal
        WRITE( Un, IOSTAT=ErrStat) objInst%instLPF
        WRITE( Un, IOSTAT=ErrStat) objInst%instSecLPF
        WRITE( Un, IOSTAT=ErrStat) objInst%instHPF
        WRITE( Un, IOSTAT=ErrStat) objInst%instNotchSlopes
        WRITE( Un, IOSTAT=ErrStat) objInst%instNotch
        WRITE( Un, IOSTAT=ErrStat) objInst%instPI
        WRITE( Un, IOSTAT=ErrStat) objInst%instRL
        Close ( Un )
    ENDIF
END SUBROUTINE WriteRestartFile

 
SUBROUTINE ReadRestartFile(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, size_avcOUTNAME, zmqVar, ErrVar)
    TYPE(LocalVariables), INTENT(INOUT)             :: LocalVar
    TYPE(ControlParameters), INTENT(INOUT)          :: CntrPar
    TYPE(ObjectInstances), INTENT(INOUT)            :: objInst
    TYPE(PerformanceData), INTENT(INOUT)            :: PerfData
    TYPE(ErrorVariables), INTENT(INOUT)             :: ErrVar
    TYPE(ZMQ_Variables), INTENT(INOUT)              :: zmqVar
    REAL(ReKi), INTENT(IN)                          :: avrSWAP(*)
    INTEGER(IntKi), INTENT(IN)                      :: size_avcOUTNAME
    CHARACTER(size_avcOUTNAME-1), INTENT(IN)        :: RootName 
    
    INTEGER(IntKi)               :: Un                  ! I/O unit for pack/unpack (checkpoint & restart)
    INTEGER(IntKi)               :: I                   ! Generic index.
    CHARACTER(128)               :: InFile              ! Input checkpoint file
    INTEGER(IntKi)               :: ErrStat
    CHARACTER(128)               :: ErrMsg              
    CHARACTER(128)               :: n_t_global          ! timestep number as a string

    WRITE(n_t_global, '(I0.0)' ) NINT(avrSWAP(2)/avrSWAP(3))
    InFile = TRIM(RootName)//TRIM( n_t_global )//'.RO.chkp'
    CALL GetNewUnit(Un, ErrVar)
    OPEN(unit=Un, FILE=TRIM(InFile), STATUS='UNKNOWN', FORM='UNFORMATTED' , ACCESS='STREAM', IOSTAT=ErrStat, ACTION='READ' )

    IF ( ErrStat /= 0 ) THEN
        ErrMsg  = 'Cannot open file //TRIM( InFile )//. Another program may have locked it for writing.'

    ELSE
        READ( Un, IOSTAT=ErrStat) LocalVar%iStatus
        READ( Un, IOSTAT=ErrStat) LocalVar%Time
        READ( Un, IOSTAT=ErrStat) LocalVar%DT
        READ( Un, IOSTAT=ErrStat) LocalVar%VS_GenPwr
        READ( Un, IOSTAT=ErrStat) LocalVar%GenSpeed
        READ( Un, IOSTAT=ErrStat) LocalVar%RotSpeed
        READ( Un, IOSTAT=ErrStat) LocalVar%NacHeading
        READ( Un, IOSTAT=ErrStat) LocalVar%NacVane
        READ( Un, IOSTAT=ErrStat) LocalVar%HorWindV
        READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOP(3)
        READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOPF(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOPF(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%rootMOOPF(3)
        READ( Un, IOSTAT=ErrStat) LocalVar%BlPitch(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%BlPitch(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%BlPitch(3)
        READ( Un, IOSTAT=ErrStat) LocalVar%Azimuth
        READ( Un, IOSTAT=ErrStat) LocalVar%NumBl
        READ( Un, IOSTAT=ErrStat) LocalVar%FA_Acc
        READ( Un, IOSTAT=ErrStat) LocalVar%NacIMU_FA_Acc
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
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisTilt_1P
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisYaw_1P
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisTilt_2P
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_AxisYaw_2P
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_KI(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_KI(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_KP(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%IPC_KP(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%PC_State
        READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%PitCom(3)
        READ( Un, IOSTAT=ErrStat) LocalVar%PitComAct(1)
        READ( Un, IOSTAT=ErrStat) LocalVar%PitComAct(2)
        READ( Un, IOSTAT=ErrStat) LocalVar%PitComAct(3)
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
        READ( Un, IOSTAT=ErrStat) LocalVar%SD
        READ( Un, IOSTAT=ErrStat) LocalVar%Fl_PitCom
        READ( Un, IOSTAT=ErrStat) LocalVar%NACIMU_FA_AccF
        READ( Un, IOSTAT=ErrStat) LocalVar%FA_AccF
        READ( Un, IOSTAT=ErrStat) LocalVar%FA_Hist
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
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%hpf_InputSignalLast
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%hpf_OutputSignalLast
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_OutputSignalLast2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_InputSignalLast2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_b2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_b0
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_a2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_a1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nfs_a0
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_OutputSignalLast2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_InputSignalLast2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_b2
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_b1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_b0
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_a1
        READ( Un, IOSTAT=ErrStat) LocalVar%FP%nf_a0
        READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm
        READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast
        READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITerm2
        READ( Un, IOSTAT=ErrStat) LocalVar%piP%ITermLast2
        READ( Un, IOSTAT=ErrStat) LocalVar%rlP%LastSignal
        READ( Un, IOSTAT=ErrStat) objInst%instLPF
        READ( Un, IOSTAT=ErrStat) objInst%instSecLPF
        READ( Un, IOSTAT=ErrStat) objInst%instHPF
        READ( Un, IOSTAT=ErrStat) objInst%instNotchSlopes
        READ( Un, IOSTAT=ErrStat) objInst%instNotch
        READ( Un, IOSTAT=ErrStat) objInst%instPI
        READ( Un, IOSTAT=ErrStat) objInst%instRL
        Close ( Un )
    ENDIF
    ! Read Parameter files
    CALL ReadControlParameterFileSub(CntrPar, zmqVar, LocalVar%ACC_INFILE, LocalVar%ACC_INFILE_SIZE, ErrVar)
    IF (CntrPar%WE_Mode > 0) THEN
        CALL READCpFile(CntrPar, PerfData, ErrVar)
    ENDIF
END SUBROUTINE ReadRestartFile

 
SUBROUTINE Debug(LocalVar, CntrPar, DebugVar, ErrVar, avrSWAP, RootName, size_avcOUTNAME)
! Debug routine, defines what gets printed to DEBUG.dbg if LoggingLevel = 1

    TYPE(ControlParameters), INTENT(IN) :: CntrPar
    TYPE(LocalVariables), INTENT(IN) :: LocalVar
    TYPE(DebugVariables), INTENT(IN) :: DebugVar
    TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar

    INTEGER(IntKi), INTENT(IN)      :: size_avcOUTNAME
    INTEGER(IntKi)                  :: I , nDebugOuts, nLocalVars   ! Generic index.
    CHARACTER(1), PARAMETER         :: Tab = CHAR(9)                ! The tab character.
    CHARACTER(29), PARAMETER        :: FmtDat = "(F20.5,TR5,99(ES20.5E2,TR5:))"   ! The format of the debugging data
    INTEGER(IntKi), SAVE            :: UnDb                         ! I/O unit for the debugging information
    INTEGER(IntKi), SAVE            :: UnDb2                        ! I/O unit for the debugging information, avrSWAP
    INTEGER(IntKi), SAVE            :: UnDb3                        ! I/O unit for the debugging information, avrSWAP
    REAL(ReKi), INTENT(INOUT)       :: avrSWAP(*)                   ! The swap array, used to pass data to, and receive data from, the DLL controller.
    CHARACTER(size_avcOUTNAME-1), INTENT(IN) :: RootName            ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
    CHARACTER(200)                  :: Version                      ! git version of ROSCO
    CHARACTER(15), ALLOCATABLE      :: DebugOutStrings(:), DebugOutUnits(:)
    REAL(DbKi), ALLOCATABLE         :: DebugOutData(:)
 
    CHARACTER(15), ALLOCATABLE      :: LocalVarOutStrings(:)
    REAL(DbKi), ALLOCATABLE         :: LocalVarOutData(:)
 
    nDebugOuts = 26
    Allocate(DebugOutData(nDebugOuts))
    Allocate(DebugOutStrings(nDebugOuts))
    Allocate(DebugOutUnits(nDebugOuts))
    DebugOutData(1) = DebugVar%WE_Cp
    DebugOutData(2) = DebugVar%WE_b
    DebugOutData(3) = DebugVar%WE_w
    DebugOutData(4) = DebugVar%WE_t
    DebugOutData(5) = DebugVar%WE_Vm
    DebugOutData(6) = DebugVar%WE_Vt
    DebugOutData(7) = DebugVar%WE_Vw
    DebugOutData(8) = DebugVar%WE_lambda
    DebugOutData(9) = DebugVar%PC_PICommand
    DebugOutData(10) = DebugVar%GenSpeedF
    DebugOutData(11) = DebugVar%RotSpeedF
    DebugOutData(12) = DebugVar%NacIMU_FA_AccF
    DebugOutData(13) = DebugVar%FA_AccF
    DebugOutData(14) = DebugVar%Fl_PitCom
    DebugOutData(15) = DebugVar%PC_MinPit
    DebugOutData(16) = DebugVar%axisTilt_1P
    DebugOutData(17) = DebugVar%axisYaw_1P
    DebugOutData(18) = DebugVar%axisTilt_2P
    DebugOutData(19) = DebugVar%axisYaw_2P
    DebugOutData(20) = DebugVar%VS_RefSpeed_Excl
    DebugOutData(21) = DebugVar%VS_RefSpeed
    DebugOutData(22) = DebugVar%YawRateCom
    DebugOutData(23) = DebugVar%NacHeadingTarget
    DebugOutData(24) = DebugVar%NacVaneOffset
    DebugOutData(25) = DebugVar%Yaw_Err
    DebugOutData(26) = DebugVar%YawState
    DebugOutStrings = [CHARACTER(15) ::  'WE_Cp', 'WE_b', 'WE_w', 'WE_t', 'WE_Vm', & 
                                      'WE_Vt', 'WE_Vw', 'WE_lambda', 'PC_PICommand', 'GenSpeedF', & 
                                      'RotSpeedF', 'NacIMU_FA_AccF', 'FA_AccF', 'Fl_PitCom', 'PC_MinPit', & 
                                      'axisTilt_1P', 'axisYaw_1P', 'axisTilt_2P', 'axisYaw_2P', 'VS_RefSpeed_Excl', & 
                                      'VS_RefSpeed', 'YawRateCom', 'NacHeadingTarget', 'NacVaneOffset', 'Yaw_Err', & 
                                      'YawState']
    DebugOutUnits = [CHARACTER(15) ::  '[-]', '[-]', '[-]', '[-]', '[m/s]', & 
                                      '[m/s]', '[m/s]', '[rad]', '[rad]', '[rad/s]', & 
                                      '[rad/s]', '[rad/s]', '[m/s]', '[rad]', '[rad]', & 
                                      '[N/A]', '[N/A]', '[N/A]', '[N/A]', '[N/A]', & 
                                      '[N/A]', '[rad/s]', '[deg]', '[deg]', '[deg]', & 
                                      '[N/A]']
    nLocalVars = 70
    Allocate(LocalVarOutData(nLocalVars))
    Allocate(LocalVarOutStrings(nLocalVars))
    LocalVarOutData(1) = LocalVar%iStatus
    LocalVarOutData(2) = LocalVar%Time
    LocalVarOutData(3) = LocalVar%DT
    LocalVarOutData(4) = LocalVar%VS_GenPwr
    LocalVarOutData(5) = LocalVar%GenSpeed
    LocalVarOutData(6) = LocalVar%RotSpeed
    LocalVarOutData(7) = LocalVar%NacHeading
    LocalVarOutData(8) = LocalVar%NacVane
    LocalVarOutData(9) = LocalVar%HorWindV
    LocalVarOutData(10) = LocalVar%rootMOOP(1)
    LocalVarOutData(11) = LocalVar%rootMOOPF(1)
    LocalVarOutData(12) = LocalVar%BlPitch(1)
    LocalVarOutData(13) = LocalVar%Azimuth
    LocalVarOutData(14) = LocalVar%NumBl
    LocalVarOutData(15) = LocalVar%FA_Acc
    LocalVarOutData(16) = LocalVar%NacIMU_FA_Acc
    LocalVarOutData(17) = LocalVar%FA_AccHPF
    LocalVarOutData(18) = LocalVar%FA_AccHPFI
    LocalVarOutData(19) = LocalVar%FA_PitCom(1)
    LocalVarOutData(20) = LocalVar%RotSpeedF
    LocalVarOutData(21) = LocalVar%GenSpeedF
    LocalVarOutData(22) = LocalVar%GenTq
    LocalVarOutData(23) = LocalVar%GenTqMeas
    LocalVarOutData(24) = LocalVar%GenArTq
    LocalVarOutData(25) = LocalVar%GenBrTq
    LocalVarOutData(26) = LocalVar%IPC_PitComF(1)
    LocalVarOutData(27) = LocalVar%PC_KP
    LocalVarOutData(28) = LocalVar%PC_KI
    LocalVarOutData(29) = LocalVar%PC_KD
    LocalVarOutData(30) = LocalVar%PC_TF
    LocalVarOutData(31) = LocalVar%PC_MaxPit
    LocalVarOutData(32) = LocalVar%PC_MinPit
    LocalVarOutData(33) = LocalVar%PC_PitComT
    LocalVarOutData(34) = LocalVar%PC_PitComT_Last
    LocalVarOutData(35) = LocalVar%PC_PitComTF
    LocalVarOutData(36) = LocalVar%PC_PitComT_IPC(1)
    LocalVarOutData(37) = LocalVar%PC_PwrErr
    LocalVarOutData(38) = LocalVar%PC_SpdErr
    LocalVarOutData(39) = LocalVar%IPC_AxisTilt_1P
    LocalVarOutData(40) = LocalVar%IPC_AxisYaw_1P
    LocalVarOutData(41) = LocalVar%IPC_AxisTilt_2P
    LocalVarOutData(42) = LocalVar%IPC_AxisYaw_2P
    LocalVarOutData(43) = LocalVar%IPC_KI(1)
    LocalVarOutData(44) = LocalVar%IPC_KP(1)
    LocalVarOutData(45) = LocalVar%PC_State
    LocalVarOutData(46) = LocalVar%PitCom(1)
    LocalVarOutData(47) = LocalVar%PitComAct(1)
    LocalVarOutData(48) = LocalVar%SS_DelOmegaF
    LocalVarOutData(49) = LocalVar%TestType
    LocalVarOutData(50) = LocalVar%VS_MaxTq
    LocalVarOutData(51) = LocalVar%VS_LastGenTrq
    LocalVarOutData(52) = LocalVar%VS_LastGenPwr
    LocalVarOutData(53) = LocalVar%VS_MechGenPwr
    LocalVarOutData(54) = LocalVar%VS_SpdErrAr
    LocalVarOutData(55) = LocalVar%VS_SpdErrBr
    LocalVarOutData(56) = LocalVar%VS_SpdErr
    LocalVarOutData(57) = LocalVar%VS_State
    LocalVarOutData(58) = LocalVar%VS_Rgn3Pitch
    LocalVarOutData(59) = LocalVar%WE_Vw
    LocalVarOutData(60) = LocalVar%WE_Vw_F
    LocalVarOutData(61) = LocalVar%WE_VwI
    LocalVarOutData(62) = LocalVar%WE_VwIdot
    LocalVarOutData(63) = LocalVar%VS_LastGenTrqF
    LocalVarOutData(64) = LocalVar%Fl_PitCom
    LocalVarOutData(65) = LocalVar%NACIMU_FA_AccF
    LocalVarOutData(66) = LocalVar%FA_AccF
    LocalVarOutData(67) = LocalVar%FA_Hist
    LocalVarOutData(68) = LocalVar%Flp_Angle(1)
    LocalVarOutData(69) = LocalVar%RootMyb_Last(1)
    LocalVarOutData(70) = LocalVar%ACC_INFILE_SIZE
    LocalVarOutStrings = [CHARACTER(15) ::  'iStatus', 'Time', 'DT', 'VS_GenPwr', 'GenSpeed', & 
                                      'RotSpeed', 'NacHeading', 'NacVane', 'HorWindV', 'rootMOOP', & 
                                      'rootMOOPF', 'BlPitch', 'Azimuth', 'NumBl', 'FA_Acc', & 
                                      'NacIMU_FA_Acc', 'FA_AccHPF', 'FA_AccHPFI', 'FA_PitCom', 'RotSpeedF', & 
                                      'GenSpeedF', 'GenTq', 'GenTqMeas', 'GenArTq', 'GenBrTq', & 
                                      'IPC_PitComF', 'PC_KP', 'PC_KI', 'PC_KD', 'PC_TF', & 
                                      'PC_MaxPit', 'PC_MinPit', 'PC_PitComT', 'PC_PitComT_Last', 'PC_PitComTF', & 
                                      'PC_PitComT_IPC', 'PC_PwrErr', 'PC_SpdErr', 'IPC_AxisTilt_1P', 'IPC_AxisYaw_1P', & 
                                      'IPC_AxisTilt_2P', 'IPC_AxisYaw_2P', 'IPC_KI', 'IPC_KP', 'PC_State', & 
                                      'PitCom', 'PitComAct', 'SS_DelOmegaF', 'TestType', 'VS_MaxTq', & 
                                      'VS_LastGenTrq', 'VS_LastGenPwr', 'VS_MechGenPwr', 'VS_SpdErrAr', 'VS_SpdErrBr', & 
                                      'VS_SpdErr', 'VS_State', 'VS_Rgn3Pitch', 'WE_Vw', 'WE_Vw_F', & 
                                      'WE_VwI', 'WE_VwIdot', 'VS_LastGenTrqF', 'Fl_PitCom', 'NACIMU_FA_AccF', & 
                                      'FA_AccF', 'FA_Hist', 'Flp_Angle', 'RootMyb_Last', 'ACC_INFILE_SIZE' & 
                                     ]
    ! Initialize debug file
    IF ((LocalVar%iStatus == 0) .OR. (LocalVar%iStatus == -9))  THEN ! .TRUE. if we're on the first call to the DLL
        IF (CntrPar%LoggingLevel > 0) THEN
            CALL GetNewUnit(UnDb, ErrVar)
            OPEN(unit=UnDb, FILE=TRIM(RootName)//'.RO.dbg')
            WRITE(UnDb, *)  'Generated on '//CurDate()//' at '//CurTime()//' using ROSCO-'//TRIM(rosco_version)
            WRITE(UnDb, '(99(a20,TR5:))') 'Time',   DebugOutStrings
            WRITE(UnDb, '(99(a20,TR5:))') '(sec)',  DebugOutUnits
        END IF

        IF (CntrPar%LoggingLevel > 1) THEN
            CALL GetNewUnit(UnDb2, ErrVar)
            OPEN(unit=UnDb2, FILE=TRIM(RootName)//'.RO.dbg2')
            WRITE(UnDb2, *)  'Generated on '//CurDate()//' at '//CurTime()//' using ROSCO-'//TRIM(rosco_version)
            WRITE(UnDb2, '(99(a20,TR5:))') 'Time',   LocalVarOutStrings
            WRITE(UnDb2, '(99(a20,TR5:))')
        END IF

        IF (CntrPar%LoggingLevel > 2) THEN
            CALL GetNewUnit(UnDb3, ErrVar)
            OPEN(unit=UnDb3, FILE=TRIM(RootName)//'.RO.dbg3')
            WRITE(UnDb3,'(/////)')
            WRITE(UnDb3,'(A,85("'//Tab//'AvrSWAP(",I2,")"))')  'LocalVar%Time ', (i,i=1, 85)
            WRITE(UnDb3,'(A,85("'//Tab//'(-)"))')  '(s)'
        END IF
    END IF
        ! Print simulation status, every 10 seconds
    IF (MODULO(LocalVar%Time, 10.0_DbKi) == 0) THEN
        WRITE(*, 100) LocalVar%GenSpeedF*RPS2RPM, LocalVar%BlPitch(1)*R2D, avrSWAP(15)/1000.0, LocalVar%WE_Vw
        100 FORMAT('Generator speed: ', f6.1, ' RPM, Pitch angle: ', f5.1, ' deg, Power: ', f7.1, ' kW, Est. wind Speed: ', f5.1, ' m/s')
    END IF

    ! Process DebugOutData, LocalVarOutData
    ! Remove very small numbers that cause ******** outputs
    DO I = 1,SIZE(DebugOutData)
        IF (ABS(DebugOutData(I)) < 1E-10) THEN
            DebugOutData(I) = 0
        END IF
    END DO
    
    DO I = 1,SIZE(LocalVarOutData)
        IF (ABS(LocalVarOutData(I)) < 1E-10) THEN
            LocalVarOutData(I) = 0
        END IF
    END DO
    
    ! Write debug files
    IF(CntrPar%LoggingLevel > 0) THEN
        WRITE (UnDb, FmtDat)  LocalVar%Time, DebugOutData
    END IF

    IF(CntrPar%LoggingLevel > 1) THEN
        WRITE (UnDb2, FmtDat)  LocalVar%Time, LocalVarOutData
    END IF

    IF(CntrPar%LoggingLevel > 2) THEN
        WRITE (UnDb3, FmtDat)    LocalVar%Time, avrSWAP(1: 85)
    END IF

END SUBROUTINE Debug

END MODULE ROSCO_IO