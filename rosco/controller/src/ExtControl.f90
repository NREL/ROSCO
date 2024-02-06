! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains the primary controller routines

! Subroutines:
!           PitchControl: Blade pitch control high level subroutine
!           VariableSpeedControl: Variable speed generator torque control
!           YawRateControl: Nacelle yaw control
!           IPC: Individual pitch control
!           ForeAftDamping: Tower fore-aft damping control
!           FloatingFeedback: Tower fore-aft feedback for floating offshore wind turbines

MODULE ExtControl

    USE, INTRINSIC :: ISO_C_Binding
    USE Functions
    USE ROSCO_Types
    USE SysSubs
    USE Constants

    IMPLICIT NONE



    ABSTRACT INTERFACE
    SUBROUTINE BladedDLL_Legacy_Procedure ( avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG )  BIND(C)
       USE, INTRINSIC :: ISO_C_Binding

       USE Constants

       REAL(ReKi),             INTENT(INOUT) :: avrSWAP   (*)  !< DATA
       INTEGER(C_INT),         INTENT(INOUT) :: aviFAIL        !< FLAG  (Status set in DLL and returned to simulation code)
       CHARACTER(KIND=C_CHAR), INTENT(IN)    :: accINFILE (*)  !< INFILE
       CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcOUTNAME(*)  !< OUTNAME (in:Simulation RootName; out:Name:Units; of logging channels)
       CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcMSG    (*)  !< MESSAGE (Message from DLL to simulation code [ErrMsg])
    END SUBROUTINE BladedDLL_Legacy_Procedure

    END INTERFACE

CONTAINS

    SUBROUTINE ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
        ! Inputs
        TYPE(ControlParameters), INTENT(INOUT)      :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)         :: LocalVar
        TYPE(ErrorVariables), INTENT(INOUT)         :: ErrVar
        TYPE(ExtControlType), INTENT(INOUT)         :: ExtDLL


        REAL(ReKi), INTENT(INOUT)                   :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from the DLL controller.

        ! Temporary variables
        ! CHARACTER(1024), PARAMETER                  :: ExtDLL_InFile = '/Users/dzalkind/Tools/ROSCO/Test_Cases/IEA-15-240-RWT-UMaineSemi/ServoData/DISCON-UMaineSemi.IN'
        CHARACTER(100), PARAMETER                   :: ExtRootName   = 'external_control'

        ! Local Variables
        CHARACTER(*), PARAMETER                     :: RoutineName = 'ExtController'

        TYPE(ExtDLL_Type), SAVE                     :: DLL_Ext
        INTEGER(IntKi), PARAMETER                   :: max_avr_entries = 2000


        PROCEDURE(BladedDLL_Legacy_Procedure), POINTER :: DLL_Legacy_Subroutine          ! The address of the (legacy DISCON) procedure in the Bladed DLL
        CHARACTER(KIND=C_CHAR)                      :: accINFILE(LEN_TRIM(CntrPar%DLL_InFile)+1)  ! INFILE
        CHARACTER(KIND=C_CHAR)                      :: avcOUTNAME(LEN_TRIM(ExtRootName)+1)   ! OUTNAME (Simulation RootName)
        CHARACTER(KIND=C_CHAR)                      :: avcMSG(LEN(ErrVar%ErrMsg)+1)                ! MESSA


        INTEGER(C_INT)                              :: aviFAIL                        ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.


        ! Initialize strings for external controller
        aviFAIL     = 0
        avcMSG     = TRANSFER( C_NULL_CHAR,                            avcMSG     )
        avcOUTNAME = TRANSFER( TRIM(ExtRootName)//C_NULL_CHAR,   avcOUTNAME )
        accINFILE  = TRANSFER( TRIM(CntrPar%DLL_InFile)//C_NULL_CHAR, accINFILE  )
                
        IF (LocalVar%iStatus == 0) THEN  

            !! Set up DLL, will come from ROSCO input   
            DLL_Ext%FileName = TRIM(CntrPar%DLL_FileName)
            DLL_Ext%ProcName = TRIM(CntrPar%DLL_ProcName)

            PRINT *, "ROSCO is calling an external dynamic library for control input:"
            PRINT *, "DLL_FileName:", TRIM(CntrPar%DLL_FileName)
            PRINT *, "DLL_InFile:", TRIM(CntrPar%DLL_InFile)
            PRINT *, "DLL_ProcName:", TRIM(CntrPar%DLL_ProcName)

            ! Load dynamic library, but first make sure that it's free
            ! CALL FreeDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
            CALL LoadDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
            ALLOCATE(ExtDLL%avrSWAP(max_avr_entries)) !(1:max_avr_entries)

            PRINT *, "Library loaded successfully"

        END IF

        ! Set avrSWAP of external DLL, inputs to external DLL
        ExtDLL%avrSWAP = avrSWAP(1:max_avr_entries)

        ! Set some length parameters
        ExtDLL%avrSWAP(49) = LEN(avcMSG)  + 1                     !> * Record 49: Maximum number of characters in the "MESSAGE" argument (-) [size of ExtErrMsg argument plus 1 (we add one for the C NULL CHARACTER)]
        ExtDLL%avrSWAP(50) = LEN_TRIM(CntrPar%DLL_InFile) +1  !> * Record 50: Number of characters in the "INFILE"  argument (-) [trimmed length of ExtDLL_InFile parameter plus 1 (we add one for the C NULL CHARACTER)]
        ExtDLL%avrSWAP(51) = LEN_TRIM(ExtRootName)   +1  !> * Record 51: Number of characters in the "OUTNAME" argument (-) [trimmed length of ExtRootName parameter plus 1 (we add one for the C NULL CHARACTER)]

        ! Call the DLL (first associate the address from the procedure in the DLL with the subroutine):
        CALL C_F_PROCPOINTER( DLL_Ext%ProcAddr(1), DLL_Legacy_Subroutine) 
        CALL DLL_Legacy_Subroutine (ExtDLL%avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG ) 

        ! Clean up DLL 
        ! CALL FreeDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            print * , TRIM(ErrVar%ErrMsg)
        ENDIF


    END SUBROUTINE ExtController


!=================================================================================================================
!=================================================================================================================
!=================================================================================================================


END MODULE ExtControl
