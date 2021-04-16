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

! Submodules:
!           CheckInputs: Initial condition and input check
!           ComputeVariablesSetpoints: Compute setpoints used by controllers
!           ReadAvrSWAP: Read AvrSWAP array
!           ReadControlParameterFileSub: Read DISCON.IN input file
!           ReadCPFile: Read text file containing Cp Surface
!           SetParameters: Define initial conditions 

MODULE ReadSetParameters

    USE, INTRINSIC :: ISO_C_Binding

USE Constants
USE Functions

    IMPLICIT NONE

CONTAINS
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadControlParameterFileSub(CntrPar, accINFILE, accINFILE_size,ErrVar)!, accINFILE_size)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters, ErrorVariables

        INTEGER(4)                                      :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size),  INTENT(IN   )       :: accINFILE(accINFILE_size)    ! DISCON input filename
        INTEGER(4), PARAMETER                           :: UnControllerParameters = 89  ! Unit number to open file
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar                      ! Control parameter type
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar                      ! Control parameter type

        INTEGER(4)                                      :: CurLine 

        CurLine = 1
       

        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')
        
        !----------------------- HEADER ------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------------- DEBUG --------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        CALL ParseInput_Int(UnControllerParameters,CurLine,'LoggingLevel',accINFILE(1),CntrPar%LoggingLevel,ErrVar)

        ! READ(UnControllerParameters, *) CntrPar%LoggingLevel
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------- CONTROLLER FLAGS ---------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'F_LPFType',accINFILE(1),CntrPar%F_LPFType,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'F_NotchType',accINFILE(1),CntrPar%F_NotchType,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'IPC_ControlMode',accINFILE(1),CntrPar%IPC_ControlMode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'VS_ControlMode',accINFILE(1),CntrPar%VS_ControlMode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'PC_ControlMode',accINFILE(1),CntrPar%PC_ControlMode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'Y_ControlMode',accINFILE(1),CntrPar%Y_ControlMode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'SS_Mode',accINFILE(1),CntrPar%SS_Mode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'WE_Mode',accINFILE(1),CntrPar%WE_Mode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'PS_Mode',accINFILE(1),CntrPar%PS_Mode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'SD_Mode',accINFILE(1),CntrPar%SD_Mode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'FL_Mode',accINFILE(1),CntrPar%FL_Mode,ErrVar)
        CALL ParseInput_Int(UnControllerParameters,CurLine,'Flp_Mode',accINFILE(1),CntrPar%Flp_Mode,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------- FILTER CONSTANTS ---------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        READ(UnControllerParameters, *) CntrPar%F_LPFCornerFreq
        READ(UnControllerParameters, *) CntrPar%F_LPFDamping
        READ(UnControllerParameters, *) CntrPar%F_NotchCornerFreq
        ALLOCATE(CntrPar%F_NotchBetaNumDen(2))
        READ(UnControllerParameters,*) CntrPar%F_NotchBetaNumDen
        READ(UnControllerParameters,*) CntrPar%F_SSCornerFreq
        READ(UnControllerParameters,*) CntrPar%F_FlCornerFreq, CntrPar%F_FlDamping
        READ(UnControllerParameters,*) CntrPar%F_FlpCornerFreq, CntrPar%F_FlpDamping
        READ(UnControllerParameters, *)

        !----------- BLADE PITCH CONTROLLER CONSTANTS -----------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%PC_GS_n
        ALLOCATE(CntrPar%PC_GS_angles(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_angles
        ALLOCATE(CntrPar%PC_GS_KP(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KP
        ALLOCATE(CntrPar%PC_GS_KI(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KI
        ALLOCATE(CntrPar%PC_GS_KD(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KD
        ALLOCATE(CntrPar%PC_GS_TF(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_TF
        READ(UnControllerParameters, *) CntrPar%PC_MaxPit
        READ(UnControllerParameters, *) CntrPar%PC_MinPit
        READ(UnControllerParameters, *) CntrPar%PC_MaxRat
        READ(UnControllerParameters, *) CntrPar%PC_MinRat
        READ(UnControllerParameters, *) CntrPar%PC_RefSpd
        READ(UnControllerParameters, *) CntrPar%PC_FinePit
        READ(UnControllerParameters, *) CntrPar%PC_Switch
        READ(UnControllerParameters, *)

        !------------------- IPC CONSTANTS -----------------------
        READ(UnControllerParameters, *) 
        READ(UnControllerParameters, *) CntrPar%IPC_IntSat
        ALLOCATE(CntrPar%IPC_KI(2))
        READ(UnControllerParameters,*) CntrPar%IPC_KI
        ALLOCATE(CntrPar%IPC_aziOffset(2))
        READ(UnControllerParameters,*) CntrPar%IPC_aziOffset
        READ(UnControllerParameters, *) CntrPar%IPC_CornerFreqAct
        READ(UnControllerParameters, *)

        !------------ VS TORQUE CONTROL CONSTANTS ----------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%VS_GenEff
        READ(UnControllerParameters, *) CntrPar%VS_ArSatTq
        READ(UnControllerParameters, *) CntrPar%VS_MaxRat
        READ(UnControllerParameters, *) CntrPar%VS_MaxTq
        READ(UnControllerParameters, *) CntrPar%VS_MinTq
        READ(UnControllerParameters, *) CntrPar%VS_MinOMSpd
        READ(UnControllerParameters, *) CntrPar%VS_Rgn2K
        READ(UnControllerParameters, *) CntrPar%VS_RtPwr
        READ(UnControllerParameters, *) CntrPar%VS_RtTq
        READ(UnControllerParameters, *) CntrPar%VS_RefSpd
        READ(UnControllerParameters, *) CntrPar%VS_n
        ALLOCATE(CntrPar%VS_KP(CntrPar%VS_n))
        READ(UnControllerParameters,*) CntrPar%VS_KP
        ALLOCATE(CntrPar%VS_KI(CntrPar%VS_n))
        READ(UnControllerParameters,*) CntrPar%VS_KI
        READ(UnControllerParameters,*) CntrPar%VS_TSRopt
        READ(UnControllerParameters, *)

        !------- Setpoint Smoother --------------------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%SS_VSGain
        READ(UnControllerParameters, *) CntrPar%SS_PCGain
        READ(UnControllerParameters, *) 

        !------------ WIND SPEED ESTIMATOR CONTANTS --------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%WE_BladeRadius
        READ(UnControllerParameters, *) CntrPar%WE_CP_n
        ALLOCATE(CntrPar%WE_CP(CntrPar%WE_CP_n))
        READ(UnControllerParameters, *) CntrPar%WE_CP
        READ(UnControllerParameters, *) CntrPar%WE_Gamma
        READ(UnControllerParameters, *) CntrPar%WE_GearboxRatio
        READ(UnControllerParameters, *) CntrPar%WE_Jtot
        READ(UnControllerParameters, *) CntrPar%WE_RhoAir
        READ(UnControllerParameters, *) CntrPar%PerfFileName
        ALLOCATE(CntrPar%PerfTableSize(2))
        READ(UnControllerParameters, *) CntrPar%PerfTableSize
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles_N
        ALLOCATE(CntrPar%WE_FOPoles_v(CntrPar%WE_FOPoles_n))
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles_v
        ALLOCATE(CntrPar%WE_FOPoles(CntrPar%WE_FOPoles_n))
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles
        READ(UnControllerParameters, *)

        !-------------- YAW CONTROLLER CONSTANTS -----------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%Y_ErrThresh
        READ(UnControllerParameters, *) CntrPar%Y_IPC_IntSat
        READ(UnControllerParameters, *) CntrPar%Y_IPC_n
        ALLOCATE(CntrPar%Y_IPC_KP(CntrPar%Y_IPC_n))
        READ(UnControllerParameters,*) CntrPar%Y_IPC_KP
        ALLOCATE(CntrPar%Y_IPC_KI(CntrPar%Y_IPC_n))
        READ(UnControllerParameters,*) CntrPar%Y_IPC_KI
        READ(UnControllerParameters, *) CntrPar%Y_IPC_omegaLP
        READ(UnControllerParameters, *) CntrPar%Y_IPC_zetaLP
        READ(UnControllerParameters, *) CntrPar%Y_MErrSet
        READ(UnControllerParameters, *) CntrPar%Y_omegaLPFast
        READ(UnControllerParameters, *) CntrPar%Y_omegaLPSlow
        READ(UnControllerParameters, *) CntrPar%Y_Rate
        READ(UnControllerParameters, *)

        !------------ FORE-AFT TOWER DAMPER CONSTANTS ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%FA_KI  
        READ(UnControllerParameters, *) CntrPar%FA_HPFCornerFreq
        READ(UnControllerParameters, *) CntrPar%FA_IntSat
        READ(UnControllerParameters, *)      

        !------------ PEAK SHAVING ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%PS_BldPitchMin_N  
        ALLOCATE(CntrPar%PS_WindSpeeds(CntrPar%PS_BldPitchMin_N))
        READ(UnControllerParameters, *) CntrPar%PS_WindSpeeds
        ALLOCATE(CntrPar%PS_BldPitchMin(CntrPar%PS_BldPitchMin_N))
        READ(UnControllerParameters, *) CntrPar%PS_BldPitchMin
        READ(UnControllerParameters, *) 

        !------------ SHUTDOWN ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%SD_MaxPit  
        READ(UnControllerParameters, *) CntrPar%SD_CornerFreq
        READ(UnControllerParameters, *)      

        !------------ FLOATING ------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%Fl_Kp  
        READ(UnControllerParameters, *) 

        !------------ Flaps ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%Flp_Angle  
        READ(UnControllerParameters, *) CntrPar%Flp_Kp  
        READ(UnControllerParameters, *) CntrPar%Flp_Ki  
        READ(UnControllerParameters, *) CntrPar%Flp_MaxPit  
        ! END OF INPUT FILE    
        
        !------------------- CALCULATED CONSTANTS -----------------------
        CntrPar%PC_RtTq99 = CntrPar%VS_RtTq*0.99
        CntrPar%VS_MinOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_MinOMSpd**2
        CntrPar%VS_MaxOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_RefSpd**2
        CLOSE(UnControllerParameters)
        
        !------------------- HOUSEKEEPING -----------------------
        CntrPar%PerfFileName = TRIM(CntrPar%PerfFileName)

        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = 'ReadControlParameterFileSub:'//TRIM(ErrVar%ErrMsg)
        ENDIF


        CONTAINS

        subroutine ParseInput_Int(Un,CurLine,VarName, FileName, Variable,ErrVar)
            USE ROSCO_Types, ONLY : ErrorVariables

            CHARACTER(1024)                         :: Line
            INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
            CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
            CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
            INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
            TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
            CHARACTER(20)                           :: Words       (2)               ! The two "words" parsed from the line

            INTEGER(4),             INTENT(INOUT)   :: Variable   ! Variable
            INTEGER(4)                              :: ErrStatLcl                    ! Error status local to this routine.

            ! Read the whole line as a string
            READ(UnControllerParameters, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read
            ! print *, 'Read: '//TRIM(Words(1))//' and '//Words(2),' on line ', CurLine

            ! Check that Variable Name is in Words
            CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )

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

        END subroutine ParseInput_Int

        subroutine ReadEmptyLine(Un,CurLine)
            INTEGER(4),         INTENT(IN   )          :: Un   ! Input file unit
            INTEGER(4),         INTENT(INOUT)          :: CurLine   ! Current line of input

            CHARACTER(1024)                            :: Line

            READ(UnControllerParameters, '(A)') Line
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

        !> This subroutine checks the data to be parsed to make sure it finds
        !! the expected variable name and an associated value.
        SUBROUTINE ChkParseData ( Words, ExpVarName, FileName, FileLineNum, ErrVar )

            USE ROSCO_Types, ONLY : ErrorVariables


                ! Arguments declarations.
            TYPE(ErrorVariables),         INTENT(INOUT)          :: ErrVar   ! Current line of input

            INTEGER(4), INTENT(IN)             :: FileLineNum                   !< The number of the line in the file being parsed.
            INTEGER(4)                        :: NameIndx                      !< The index into the Words array that points to the variable name.

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
                                    //'".'//NewLine//' >> The variable "'//TRIM( Words(1) )//'" was not assigned a value on line #' &
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
                                    //'".'//NewLine//' >> The variable "'//TRIM( ExpVarName )//'" was not assigned a value on line #' &
                                    //TRIM( Int2LStr( FileLineNum ) )//'.' 
                RETURN
                ENDIF
            ENDIF


        END SUBROUTINE ChkParseData 

    END SUBROUTINE ReadControlParameterFileSub
    ! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE Constants
        ! Allocate variables
        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst

        REAL(8)                                 :: VS_RefSpd        ! Referece speed for variable speed torque controller, [rad/s] 
        REAL(8)                                 :: PC_RefSpd        ! Referece speed for pitch controller, [rad/s] 
        REAL(8)                                 :: Omega_op         ! Optimal TSR-tracking generator speed, [rad/s]
        ! temp
        ! REAL(8)                                 :: VS_TSRop = 7.5

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
    ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar)
        USE ROSCO_Types, ONLY : LocalVariables
    
        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        
        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LocalVar%iStatus = NINT(avrSWAP(1))
        LocalVar%Time = avrSWAP(2)
        LocalVar%DT = avrSWAP(3)
        LocalVar%VS_MechGenPwr = avrSWAP(14)
        LocalVar%VS_GenPwr = avrSWAP(15)
        LocalVar%GenSpeed = avrSWAP(20)
        LocalVar%RotSpeed = avrSWAP(21)
        LocalVar%GenTqMeas = avrSWAP(23)
        LocalVar%Y_M = avrSWAP(24)
        LocalVar%HorWindV = avrSWAP(27)
        LocalVar%rootMOOP(1) = avrSWAP(30)
        LocalVar%rootMOOP(2) = avrSWAP(31)
        LocalVar%rootMOOP(3) = avrSWAP(32)
        LocalVar%FA_Acc = avrSWAP(53)
        LocalVar%NacIMU_FA_Acc = avrSWAP(83)
        LocalVar%Azimuth = avrSWAP(60)
        LocalVar%NumBl = NINT(avrSWAP(61))

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

    END SUBROUTINE ReadAvrSWAP
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
        INTEGER(4),                 INTENT(IN   )       :: size_avcMSG
        REAL(C_FLOAT),              INTENT(IN   )       :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        
        
        ! Local
        
        !..............................................................................................................................
        ! Check validity of input parameters:
        !..............................................................................................................................

        !------- DEBUG ------------------------------------------------------------

        ! LoggingLevel
        IF ((CntrPar%LoggingLevel < 0) .OR. (CntrPar%LoggingLevel > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'LoggingLevel must be 0, 1, or 2.'
        ENDIF

        !------- CONTROLLER FLAGS -------------------------------------------------

        ! F_LPFType
        IF ((CntrPar%F_LPFType < 1) .OR. (CntrPar%F_LPFType > 2)) THEN
            ErrVar%aviFAIL = -1
            PRINT *, CntrPar%F_LPFType
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
        IF ((CntrPar%PS_Mode < 0) .OR. (CntrPar%PS_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PS_Mode must be 0 or 1.'
        ENDIF

        ! SD_Mode
        IF ((CntrPar%SD_Mode < 0) .OR. (CntrPar%SD_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SD_Mode must be 0 or 1.'
        ENDIF

        ! Fl_Mode
        IF ((CntrPar%Fl_Mode < 0) .OR. (CntrPar%Fl_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Fl_Mode must be 0 or 1.'
        ENDIF

        ! Flp_Mode
        IF ((CntrPar%Flp_Mode < 0) .OR. (CntrPar%Flp_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Flp_Mode must be 0, 1, or 2.'
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

            ! F_NotchBetaNumDen(1)
            IF (CntrPar%F_NotchBetaNumDen(1) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_NotchBetaNumDen(1) must be greater than zero.'
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

        ! F_FlCornerFreq(1)  (frequency)
        IF (CntrPar%F_FlCornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_FlCornerFreq(1) must be greater than zero.'
        ENDIF

        ! F_FlCornerFreq(2)  (damping)
        IF (CntrPar%F_FlDamping <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_FlCornerFreq(2) must be greater than zero.'
        ENDIF

        ! F_FlpCornerFreq(1)  (frequency)
        IF (CntrPar%F_FlCornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_FlpCornerFreq(1) must be greater than zero.'
        ENDIF

        ! F_FlpCornerFreq(2)  (damping)
        IF (CntrPar%F_FlDamping <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_FlpCornerFreq(2) must be greater than zero.'
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
            IF (CntrPar%F_NotchType <= 1 .OR. CntrPar%F_NotchCornerFreq == 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'F_NotchType and F_NotchCornerFreq must be specified for Fl_Mode greater than zero.'
            ENDIF
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
            ErrVar%ErrMsg  = 'CheckInputs:'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE CheckInputs
    ! -----------------------------------------------------------------------------------
    ! Define parameters for control actions
    SUBROUTINE SetParameters(avrSWAP, accINFILE, size_avcMSG, CntrPar, LocalVar, objInst, PerfData, ErrVar)
               
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData, ErrorVariables
        
        INTEGER(4), INTENT(IN) :: size_avcMSG
        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        TYPE(PerformanceData), INTENT(INOUT)    :: PerfData
        TYPE(ErrorVariables), INTENT(INOUT)     :: ErrVar

        REAL(C_FLOAT), INTENT(INOUT)            :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(KIND=C_CHAR), INTENT(IN)      :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
        
        INTEGER(4)                              :: K    ! Index used for looping through blades.
        CHARACTER(200)                          :: git_version
        
        ! Error Catching Variables
        ! Set ErrVar%aviFAIL to 0 in each iteration:
        ErrVar%aviFAIL = 0
        ! ALLOCATE(ErrVar%ErrMsg(size_avcMSG-1))
        ErrVar%size_avcMSG  = size_avcMSG
        
        ! Initialize all filter instance counters at 1
        objInst%instLPF = 1
        objInst%instSecLPF = 1
        objInst%instHPF = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch = 1
        objInst%instPI = 1
        
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
            git_version = QueryGitVersion()
            ! ErrMsg = '                                                                              '//NEW_LINE('A')// &
            !          '------------------------------------------------------------------------------'//NEW_LINE('A')// &
            !          'Running a controller implemented through NREL''s ROSCO Toolbox                    '//NEW_LINE('A')// &
            !          'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
            !          'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
            !          '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
            !          'Primary development by (listed alphabetically): Nikhar J. Abbas               '//NEW_LINE('A')// &
            !          '                                                Sebastiaan P. Mulders         '//NEW_LINE('A')// &
            !          '                                                Jan-Willem van Wingerden      '//NEW_LINE('A')// &
            !          'Visit our GitHub-page to contribute to this project:                          '//NEW_LINE('A')// &
            !          'https://github.com/NREL/ROSCO                                                 '//NEW_LINE('A')// &
            !          '------------------------------------------------------------------------------'
            PRINT *,'                                                                              '//NEW_LINE('A')// &
                    '------------------------------------------------------------------------------'//NEW_LINE('A')// &
                    'Running ROSCO-'//TRIM(git_version)//NEW_LINE('A')// &
                    'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
                    'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
                    '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
                    '------------------------------------------------------------------------------'

            CALL ReadControlParameterFileSub(CntrPar, accINFILE, NINT(avrSWAP(50)),ErrVar)

            IF (CntrPar%WE_Mode > 0) THEN
                CALL READCpFile(CntrPar,PerfData)
            ENDIF
            ! Initialize testValue (debugging variable)
            LocalVar%TestType = 0
        
            ! Initialize the SAVED variables:
            LocalVar%PitCom = LocalVar%BlPitch ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
            LocalVar%Y_AccErr = 0.0  ! This will ensure that the accumulated yaw error starts at zero
            LocalVar%Y_YawEndT = -1.0 ! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing
            
            ! Wind speed estimator initialization
            LocalVar%WE_Vw = LocalVar%HorWindV
            LocalVar%WE_VwI = LocalVar%WE_Vw - CntrPar%WE_Gamma*LocalVar%RotSpeed
            
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

            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = 'SetParameters:'//TRIM(ErrVar%ErrMsg)
            ENDIF
            

        ENDIF
    END SUBROUTINE SetParameters
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadCpFile(CntrPar,PerfData)
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters

        INTEGER(4), PARAMETER :: UnPerfParameters = 89
        TYPE(PerformanceData), INTENT(INOUT) :: PerfData
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        ! Local variables
        INTEGER(4)                  :: i ! iteration index
        OPEN(unit=UnPerfParameters, file=TRIM(CntrPar%PerfFileName), status='old', action='read') ! Should put input file into DISCON.IN
        
        ! ----------------------- Axis Definitions ------------------------
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        ALLOCATE(PerfData%Beta_vec(CntrPar%PerfTableSize(1)))
        READ(UnPerfParameters, *) PerfData%Beta_vec
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%TSR_vec(CntrPar%PerfTableSize(2)))
        READ(UnPerfParameters, *) PerfData%TSR_vec

        ! ----------------------- Read Cp, Ct, Cq, Tables ------------------------
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) ! Input file should contains wind speed information here - unneeded for now
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cp_mat(i,:) ! Read Cp table
        END DO
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Ct_mat(i,:) ! Read Ct table
        END DO
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cq_mat(i,:) ! Read Cq table
        END DO
    
    END SUBROUTINE ReadCpFile
END MODULE ReadSetParameters
