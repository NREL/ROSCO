! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! High level run script

!=======================================================================
SUBROUTINE DISCON(avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG) BIND (C, NAME='DISCON')
! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

USE, INTRINSIC  :: ISO_C_Binding
USE             :: ROSCO_Types
USE             :: ReadSetParameters
USE             :: ControllerBlocks
USE             :: Controllers
USE             :: Constants
USE             :: Filters
USE             :: Functions
USE             :: ExtControl
USE             :: ROSCO_IO
USE             :: ZeroMQInterface

IMPLICIT NONE
! Enable .dll export
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
#endif

!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------

! Passed Variables:
!REAL(ReKi), INTENT(IN)      :: from_SC(*)       ! DATA from the super controller
!REAL(ReKi), INTENT(INOUT)   :: to_SC(*)         ! DATA to the super controller

REAL(ReKi),                  INTENT(INOUT)   :: avrSWAP(*)                       ! The swap array, used to pass data to, and receive data from, the DLL controller.
INTEGER(C_INT),                 INTENT(INOUT)   :: aviFAIL                          ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR),         INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME))                   :: RootName                         ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)                       :: ErrMsg                           ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]

TYPE(ControlParameters),        SAVE           :: CntrPar
TYPE(LocalVariables),           SAVE           :: LocalVar
TYPE(ObjectInstances),          SAVE           :: objInst
TYPE(PerformanceData),          SAVE           :: PerfData
TYPE(DebugVariables),           SAVE           :: DebugVar
TYPE(ErrorVariables),           SAVE           :: ErrVar
TYPE(ExtControlType),           SAVE           :: ExtDLL


CHARACTER(*),                   PARAMETER      :: RoutineName = 'ROSCO'

RootName = TRANSFER(avcOUTNAME, RootName)
CALL GetRoot(RootName,RootName)
!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------

! Check for restart
IF ( (NINT(avrSWAP(1)) == -9) .AND. (aviFAIL >= 0))  THEN ! Read restart files
    CALL ReadRestartFile(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, SIZE(avcOUTNAME), ErrVar)
    IF ( CntrPar%LoggingLevel > 0 ) THEN
        CALL Debug(LocalVar, CntrPar, DebugVar, ErrVar, avrSWAP, RootName, SIZE(avcOUTNAME))
    END IF 
END IF

! Read avrSWAP array into derived types/variables
CALL ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)

! Set Control Parameters
IF (ErrVar%aviFAIL >= 0) THEN
    CALL SetParameters(avrSWAP, accINFILE, SIZE(avcMSG), CntrPar, LocalVar, objInst, PerfData, RootName, ErrVar)
ENDIF

! Call external controller, if desired
IF (CntrPar%Ext_Mode > 0 .AND. ErrVar%aviFAIL >= 0) THEN
    CALL ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
    ! Data from external dll is in ExtDLL%avrSWAP, it's unused in the following code
END IF

! Filter signals
CALL PreFilterMeasuredSignals(CntrPar, LocalVar, DebugVar, objInst, ErrVar)

IF (((LocalVar%iStatus >= 0) .OR. (LocalVar%iStatus <= -8)) .AND. (ErrVar%aviFAIL >= 0))  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step
    IF ((LocalVar%iStatus == -8) .AND. (ErrVar%aviFAIL >= 0))  THEN ! Write restart files
        CALL WriteRestartFile(LocalVar, CntrPar, ErrVar, objInst, RootName, SIZE(avcOUTNAME))    
    ENDIF
    IF (CntrPar%ZMQ_Mode > 0) THEN
        CALL UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
    ENDIF
    
    CALL WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    CALL ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    CALL StateMachine(CntrPar, LocalVar)
    CALL SetpointSmoother(LocalVar, CntrPar, objInst)
    CALL VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
    CALL PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    
    IF (CntrPar%Y_ControlMode > 0) THEN
        CALL YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    END IF
    
    IF (CntrPar%Flp_Mode > 0) THEN
        CALL FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
    END IF

    ! Cable control
    IF (CntrPar%CC_Mode > 0) THEN
        CALL CableControl(avrSWAP,CntrPar,LocalVar, objInst, ErrVar)
    END IF

    ! Structural control
    IF (CntrPar%StC_Mode > 0) THEN
        CALL StructuralControl(avrSWAP,CntrPar,LocalVar, objInst, ErrVar)
    END IF
    
    IF ( CntrPar%LoggingLevel > 0 ) THEN
        CALL Debug(LocalVar, CntrPar, DebugVar, ErrVar, avrSWAP, RootName, SIZE(avcOUTNAME))
    END IF 
ELSEIF ((LocalVar%iStatus == -1) .AND. (CntrPar%ZMQ_Mode > 0)) THEN
        CALL UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
END IF


! Add RoutineName to error message
IF (ErrVar%aviFAIL < 0) THEN
    ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    print * , TRIM(ErrVar%ErrMsg)
ENDIF
ErrMsg = ADJUSTL(TRIM(ErrVar%ErrMsg))
avcMSG = TRANSFER(ErrMsg//C_NULL_CHAR, avcMSG, LEN(ErrMsg)+1)
avcMSG = TRANSFER(ErrMsg//C_NULL_CHAR, avcMSG, SIZE(avcMSG))
aviFAIL = ErrVar%aviFAIL
ErrVar%ErrMsg = ''

RETURN
END SUBROUTINE DISCON
