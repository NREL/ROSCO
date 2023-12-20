! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Helper functions, primarily borrowed from NWTC_IO, for reading inputs and carrying out other helpful tasks

MODULE ROSCO_Helpers

    USE, INTRINSIC :: ISO_C_Binding
    USE ROSCO_Types
    USE CONSTANTS
    USE SysSubs


    IMPLICIT NONE

    ! Global Variables
    LOGICAL, PARAMETER     :: DEBUG_PARSING = .FALSE.      ! debug flag to output parsing information, set up Echo file later
    
    INTERFACE ParseInput                                                         ! Parses a character variable name and value from a string.
        MODULE PROCEDURE ParseInput_Str                                             ! Parses a character string from a string.
        MODULE PROCEDURE ParseInput_Dbl                                             ! Parses a double-precision REAL from a string.
        MODULE PROCEDURE ParseInput_Int                                             ! Parses an INTEGER from a string.
        MODULE PROCEDURE ParseInput_Int_Opt                                             ! Parses an INTEGER from a string.  Optional input.
        MODULE PROCEDURE ParseInput_Dbl_Opt                                             ! Parses an double-precision REAL from a string.  Optional input.
        MODULE PROCEDURE ParseInput_Str_Opt                                             ! Parses an character string from a string.  Optional input.
        ! MODULE PROCEDURE ParseInput_Log                                             ! Parses an LOGICAL from a string.
    END INTERFACE

    INTERFACE ParseAry                                                         ! Parse an array of numbers from a string.
        MODULE PROCEDURE ParseDbAry                                             ! Parse an array of double-precision REAL values.
        MODULE PROCEDURE ParseInAry                                             ! Parse an array of whole numbers.
        MODULE PROCEDURE ParseInAry_Opt                                         ! Parse an array of whole numbers. Optional inputs.
        MODULE PROCEDURE ParseDbAry_Opt                                         ! Parse an array of double-precision REAL values.  Optional inputs.
    END INTERFACE

    INTEGER(IntKi), PARAMETER       :: MaxLineLength    = 2048      ! characters
    INTEGER(IntKi), PARAMETER       :: MaxParamLength   = 200         ! characters, file paths can be long

CONTAINS

    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Int(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(MaxLineLength)                    :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )       :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )       :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)       :: ErrVar   ! Current line of input
        CHARACTER(MaxParamLength)                   :: Words       (2)               ! The two "words" parsed from the line

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
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Int_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(*),           INTENT(IN   ), DIMENSION(:) :: FileLines   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: FileName   ! Input file unit
        TYPE(ErrorVariables),   INTENT(INOUT)               :: ErrVar   ! Current line of input
        INTEGER(IntKi),         INTENT(INOUT)               :: Variable   ! Variable
        Integer(IntKi), OPTIONAL, INTENT(IN   )               :: UnEc   ! Variable


        ! Flag (usually control mode) specifying whether default is allowed, 0 - yes, nonzero - no
        LOGICAL, OPTIONAL,      INTENT(IN   )        :: AllowDefault   
        
        INTEGER(IntKi)                          :: CurLine   ! Current line of input
        CHARACTER(MaxParamLength)               :: Words       (2)               ! The two "words" parsed from the line
        CHARACTER(MaxParamLength)               :: VarNameUC
        CHARACTER(MaxLineLength)                :: Line
        INTEGER(IntKi)                          :: ErrStatLcl           ! Error status local to this routine.
        INTEGER(IntKi)                          :: I, VarLineIndex                    ! Line indexer
        LOGICAL                                 :: AllowDefault_, FoundLine
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInput_Int_Opt'


        ! Figure out if we allow default
        AllowDefault_ = .TRUE.
        if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            CALL FindLine(FileLines, VarName, FoundLine, Line, CurLine)

            ! Separate line again
            CALL GetWords ( Line, Words, 2 )  

            ! PRINT *, "Line: ", Line

            ! Print warning with default
            IF (.NOT. FoundLine) THEN
                IF (.NOT. AllowDefault_) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( VarName )//'. Please check control modes.'
                    RETURN
                ENDIF

                Variable = 0     ! Default of integer inputs is 0 for now
                PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input file.  Using default value of ", Variable
            ENDIF

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0 .AND. FoundLine) THEN        

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

            IF ( PRESENT(UnEc))  THEN
                IF ( UnEc > 0 )  WRITE (UnEc,*)  CurLine, Tab, VarName, Tab, Variable
            END IF

        END IF

    END subroutine ParseInput_Int_Opt

     !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Dbl_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(*),           INTENT(IN   ), DIMENSION(:) :: FileLines   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: FileName   ! Input file unit
        TYPE(ErrorVariables),   INTENT(INOUT)               :: ErrVar   ! Current line of input
        REAL(DbKi),             INTENT(INOUT)               :: Variable   ! Variable
        Integer(IntKi), OPTIONAL, INTENT(IN   )               :: UnEc   ! Variable

        
        ! Flag (usually control mode) specifying whether default is allowed, 0 - yes, nonzero - no
        LOGICAL, OPTIONAL,      INTENT(IN   )        :: AllowDefault   
        
        INTEGER(IntKi)                          :: CurLine   ! Current line of input
        CHARACTER(MaxParamLength)               :: Words       (2)               ! The two "words" parsed from the line
        CHARACTER(MaxParamLength)               :: VarNameUC
        CHARACTER(MaxLineLength)                :: Line
        INTEGER(IntKi)                          :: ErrStatLcl           ! Error status local to this routine.
        INTEGER(IntKi)                          :: I, VarLineIndex                    ! Line indexer
        LOGICAL                                 :: AllowDefault_, FoundLine
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInput_Dbl_Opt'


        ! Figure out if we allow default
        AllowDefault_ = .TRUE.
        if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            CALL FindLine(FileLines, VarName, FoundLine, Line, CurLine)


            ! Separate line again
            CALL GetWords ( Line, Words, 2 )  

            ! PRINT *, "Line: ", Line

            ! Print warning with default
            IF (.NOT. FoundLine) THEN
                IF (.NOT. AllowDefault_) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( VarName )//'. Please check control modes.'
                    RETURN
                ENDIF

                Variable = 0     ! Default of integer inputs is 0 for now
                PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input file.  Using default value of ", Variable
            ENDIF

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0 .AND. FoundLine) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid REAL value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            IF ( PRESENT(UnEc))  THEN
                IF ( UnEc > 0 )  WRITE (UnEc,*)  CurLine, Tab, VarName, Tab, Variable
            END IF

        END IF

    END subroutine ParseInput_Dbl_Opt

        !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Str_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(*),           INTENT(IN   ), DIMENSION(:) :: FileLines   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: FileName   ! Input file unit
        TYPE(ErrorVariables),   INTENT(INOUT)               :: ErrVar   ! Current line of input
        CHARACTER(*),           INTENT(INOUT)               :: Variable   ! Variable
        Integer(IntKi), OPTIONAL, INTENT(IN   )               :: UnEc   ! Variable

        ! Flag (usually control mode) specifying whether default is allowed, 0 - yes, nonzero - no
        LOGICAL, OPTIONAL,      INTENT(IN   )        :: AllowDefault   
        
        INTEGER(IntKi)                          :: CurLine   ! Current line of input
        CHARACTER(MaxParamLength)               :: Words       (2)               ! The two "words" parsed from the line
        CHARACTER(MaxParamLength)               :: VarNameUC
        CHARACTER(MaxLineLength)                :: Line
        INTEGER(IntKi)                          :: ErrStatLcl           ! Error status local to this routine.
        INTEGER(IntKi)                          :: I, VarLineIndex                    ! Line indexer
        LOGICAL                                 :: AllowDefault_, FoundLine
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInput_Str_Opt'


        ! Figure out if we allow default
        AllowDefault_ = .TRUE.
        if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            CALL FindLine(FileLines, VarName, FoundLine, Line, CurLine)


            ! Separate line again
            CALL GetWords ( Line, Words, 2 )  

            ! PRINT *, "Line: ", TRIM(Line)

            ! Print warning with default
            IF (.NOT. FoundLine) THEN
                IF (.NOT. AllowDefault_) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( VarName )//'. Please check control modes.'
                    RETURN
                ENDIF

                Variable = 'unused'     ! Default of string input is unused for now
                PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input file.  Using default value of ", TRIM(Variable)
            ENDIF

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            IF (ErrVar%aviFAIL >= 0 .AND. FoundLine) THEN        

                ! Read the variable
                READ (Words(1),'(A)',IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            IF ( PRESENT(UnEc))  THEN
                IF ( UnEc > 0 )  WRITE (UnEc,*)  CurLine, Tab, VarName, Tab, Variable
            END IF

        END IF

    END subroutine ParseInput_Str_Opt


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
    SUBROUTINE ParseDbAry ( Un, LineNum, ParamName, Ary, AryLen, FileName, ErrVar, CheckName )

        USE ROSCO_Types, ONLY : ErrorVariables

        ! Arguments declarations.
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

        REAL(DbKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

        INTEGER(IntKi),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
        CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


        CHARACTER(*),           INTENT(IN   )   :: ParamName                       !< The array name we are trying to fill.

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
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( ParamName )//' array; array was already allocated.'
                ELSE
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( ParamName )//' array.'
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
                CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), ParamName, FileName, LineNum, ErrVar )
            END IF
        
            ! Read array
            READ (Line,*,IOSTAT=ErrStatLcl)  Ary
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                                //TRIM( FileName )//'".'//NewLine//  &
                                ' >> The "'//TRIM( ParamName )//'" array was not assigned valid REAL values on line #' &
                                //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                                //'    "'//TRIM( Line )//'"' 
                RETURN
                CALL Cleanup()         
            ENDIF

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
  SUBROUTINE ParseInAry ( Un, LineNum, ParamName, Ary, AryLen, FileName, ErrVar, CheckName )

    USE ROSCO_Types, ONLY : ErrorVariables

    ! Arguments declarations.
    INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
    INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

    INTEGER(IntKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

    INTEGER(IntKi),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
    CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


    CHARACTER(*),           INTENT(IN   )   :: ParamName                       !< The array name we are trying to fill.

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
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( ParamName )//' array; array was already allocated.'
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( ParamName )//' array.'
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
            CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), ParamName, FileName, LineNum, ErrVar )
        END IF
    
        ! Read array
        READ (Line,*,IOSTAT=ErrStatLcl)  Ary
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                            //TRIM( FileName )//'".'//NewLine//  &
                            ' >> The "'//TRIM( ParamName )//'" array was not assigned valid INTEGER values on line #' &
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

!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
  SUBROUTINE ParseInAry_Opt( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )

    USE ROSCO_Types, ONLY : ErrorVariables

    ! Arguments declarations.
    CHARACTER(*),  INTENT(IN   ), DIMENSION(:)     :: FileLines   ! Input file unit
    INTEGER,                INTENT(IN   )          :: AryLen                        !< The length of the array to parse.
    INTEGER(IntKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.
    CHARACTER(*),           INTENT(IN)             :: FileName                      !< The name of the file being parsed.
    CHARACTER(*),           INTENT(IN)             :: ParamName                       !< The array name we are trying to fill.
    TYPE(ErrorVariables),   INTENT(INOUT)          :: ErrVar   ! Current line of input
    Integer(IntKi), OPTIONAL, INTENT(IN   )               :: UnEc   ! Variable
    LOGICAL, OPTIONAL,      INTENT(IN   )        :: AllowDefault   

    ! Local declarations.
    INTEGER(IntKi)                                 :: FinalAryLen                   !< Final array length, non-input
    INTEGER(IntKi)                                 :: LineNum                       !< The number of the line to parse.
    CHARACTER(MaxLineLength)                       :: Line
    INTEGER(IntKi)                                 :: ErrStatLcl                    ! Error status local to this routine.
    INTEGER(IntKi)                                 :: i

    CHARACTER(MaxParamLength), ALLOCATABLE          :: Words_Ary       (:)               ! The array "words" parsed from the line.
    CHARACTER(MaxLineLength)                        :: Debug_String 
    CHARACTER(*), PARAMETER                         :: RoutineName = 'ParseInAry_Opt'
    CHARACTER(MaxParamLength)                       :: ParamNameUC
    LOGICAL                                         :: AllowDefault_, FoundLine

    ! Figure out if we allow default
    AllowDefault_ = .TRUE.
    if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

    ! If we've already failed, don't read anything
    IF (ErrVar%aviFAIL >= 0) THEN
        
        CALL FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)

        ! PRINT *, "Line: ", TRIM(Line)

        ! Minimum array length
        IF (AryLen < 1) THEN
            FinalAryLen = 1
        ELSE
            FinalAryLen = AryLen
        ENDIF

        ! Allocate array and handle errors
         ALLOCATE ( Ary(FinalAryLen) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 ) THEN
            IF ( ALLOCATED(Ary) ) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( ParamName )//' array; array was already allocated.'
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( ParamName )//' array.'
            END IF
        END IF

        ! Print warning with default
        IF (.NOT. FoundLine) THEN
            IF (.NOT. AllowDefault_) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( ParamName )//'. Please check control modes and array length.'
                RETURN
            ENDIF

            Ary = 0     ! Default of allocatable arrays is 0 for now
            PRINT *, "ROSCO Warning: Did not find correct size "//TRIM( ParamName )//" in input file.  Using default value of [", Ary, "]"
        ENDIF

        IF (FoundLine) THEN
    
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
        
            ! Read array
            READ (Line,*,IOSTAT=ErrStatLcl)  Ary
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                                //TRIM( FileName )//'".'//NewLine//  &
                                ' >> The "'//TRIM( ParamName )//'" array was not assigned valid REAL values on line #' &
                                //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                                //'    "'//TRIM( Line )//'"' 
                RETURN
                CALL Cleanup()         
            ENDIF

        ENDIF

        IF ( PRESENT(UnEc))  THEN
            IF ( UnEc > 0 )  WRITE (UnEc,*)  LineNum, Tab, ParamName, Tab, Ary
        END IF

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

END SUBROUTINE ParseInAry_Opt

!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
  SUBROUTINE ParseDbAry_Opt ( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )

    USE ROSCO_Types, ONLY : ErrorVariables

    ! Arguments declarations.
    CHARACTER(*),  INTENT(IN   ), DIMENSION(:)     :: FileLines   ! Input file unit
    INTEGER,                INTENT(IN)             :: AryLen                        !< The length of the array to parse.
    REAL(DbKi), ALLOCATABLE,   INTENT(INOUT)       :: Ary(:)            !< The array to receive the input values.
    CHARACTER(*),           INTENT(IN)             :: FileName                      !< The name of the file being parsed.
    CHARACTER(*),           INTENT(IN)             :: ParamName                       !< The array name we are trying to fill.
    TYPE(ErrorVariables),   INTENT(INOUT)          :: ErrVar   ! Current line of input
    LOGICAL, OPTIONAL,      INTENT(IN   )          :: AllowDefault   
    Integer(IntKi), OPTIONAL, INTENT(IN   )        :: UnEc   ! Variable

    ! Local declarations.
    INTEGER(IntKi)                                 :: LineNum                       !< The number of the line to parse.
    INTEGER(IntKi)                                 :: FinalAryLen                   !< Final array length, non-input
    CHARACTER(MaxLineLength)                       :: Line
    INTEGER(IntKi)                                 :: ErrStatLcl                    ! Error status local to this routine.
    INTEGER(IntKi)                                 :: i

    CHARACTER(MaxParamLength), ALLOCATABLE          :: Words_Ary       (:)               ! The array "words" parsed from the line.
    CHARACTER(MaxLineLength)                        :: Debug_String 
    CHARACTER(*), PARAMETER                         :: RoutineName = 'ParseDbAry_Opt'
    CHARACTER(MaxParamLength)                       :: ParamNameUC, FileLineUC
    LOGICAL                                         :: AllowDefault_, FoundLine

    ! Figure out if we allow default
    AllowDefault_ = .TRUE.
    if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

    ! If we've already failed, don't read anything
    IF (ErrVar%aviFAIL >= 0) THEN
        
        CALL FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)
        
        ! Minimum array length
        IF (AryLen < 1) THEN
            FinalAryLen = 1
        ELSE
            FinalAryLen = AryLen
        ENDIF

        ! Allocate array and handle errors
        ALLOCATE ( Ary(FinalAryLen) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 ) THEN
            IF ( ALLOCATED(Ary) ) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( ParamName )//' array; array was already allocated.'
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( ParamName )//' array.'
            END IF
        END IF

        ! Print warning with default
        IF (.NOT. FoundLine) THEN
            IF (.NOT. AllowDefault_) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( ParamName )//'. Please check control modes and array length.'
                RETURN
            ENDIF

            Ary = 0     ! Default of allocatable arrays is 0 for now
            PRINT *, "ROSCO Warning: Did not find correct size "//TRIM( ParamName )//" in input file.  Using default value of [", Ary, "]"
        ENDIF

        IF (FoundLine) THEN
    
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
        
            ! Read array
            READ (Line,*,IOSTAT=ErrStatLcl)  Ary
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                                //TRIM( FileName )//'".'//NewLine//  &
                                ' >> The "'//TRIM( ParamName )//'" array was not assigned valid REAL values on line #' &
                                //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                                //'    "'//TRIM( Line )//'"' 
                RETURN
                CALL Cleanup()         
            ENDIF

        ENDIF

        IF ( PRESENT(UnEc))  THEN
            IF ( UnEc > 0 )  WRITE (UnEc,*)  LineNum, Tab, ParamName, Tab, Ary
        END IF

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

END SUBROUTINE ParseDbAry_Opt



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

SUBROUTINE FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)
    CHARACTER(*),   INTENT(IN   ), DIMENSION(:)    :: FileLines   ! Input file unit
    CHARACTER(*),   INTENT(IN)                     :: ParamName                       !< The array name we are trying to fill.
    INTEGER(IntKi), INTENT(OUT)                    :: LineNum                       !< The number of the line to parse.
    LOGICAL, INTENT(OUT)                    :: FoundLine
    INTEGER(IntKi), OPTIONAL,      INTENT(IN   )          :: AryLen
    CHARACTER(MaxLineLength), INTENT(OUT)          :: Line
    
    
    CHARACTER(MaxParamLength), ALLOCATABLE  :: Words(:)               ! The two "words" parsed from the line

    CHARACTER(MaxParamLength)                      :: ParamNameUC, FileLineUC
    INTEGER(IntKi)                                 :: I, WordInd

    IF (.NOT. PRESENT(AryLen)) THEN
        WordInd = 2
    ELSE
        WordInd = AryLen + 1
    ENDIF

    ALLOCATE(Words(WordInd))   ! TODO: check error
    
    ! Make name uppercase
    ParamNameUC = ParamName
    CALL Conv2UC(ParamNameUC)
    
    ! Search for line in FileLines
    FoundLine = .FALSE.
    LineNum = 0
    DO I = 1,SIZE(FileLines)

        ! Separate line string into 2 words
        CALL GetWords ( FileLines(I), Words, WordInd )  
        
        ! Make FileLines uppercase
        FileLineUC = Words(WordInd)
        CALL Conv2UC(FileLineUC)

        ! WRITE(500,*) Words

        ! PRINT *, TRIM(ParamNameUC), '==', TRIM(FileLineUC), '=', TRIM(FileLineUC)==TRIM(ParamNameUC)
        
        IF (FileLineUC == ParamNameUC) THEN
            Line = FileLines(I)
            LineNum = I
            FoundLine = .TRUE.
        END IF
    END DO

END subroutine FindLine

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
!> Let's parse the root file name from the name of the given file.
!! We'll count everything after the last period as the extension.
!! Borrowed from NWTC_IO...thanks!

   SUBROUTINE GetRoot ( GivenFil, RootName )

      ! Argument declarations.

   CHARACTER(*), INTENT(IN)     :: GivenFil                                     !< The name of the given file.
   CHARACTER(*), INTENT(OUT)    :: RootName                                     !< The parsed root name of the given file.


      ! Local declarations.

   INTEGER                      :: I                                            ! DO index for character position.



      ! Deal with a couple of special cases.

   IF ( ( TRIM( GivenFil ) == "." ) .OR. (  TRIM( GivenFil ) == ".." ) )  THEN
      RootName = TRIM( GivenFil )
      RETURN
   END IF


      ! More-normal cases.

   DO I=LEN_TRIM( GivenFil ),1,-1


      IF ( GivenFil(I:I) == '.' )  THEN


         IF ( I < LEN_TRIM( GivenFil ) ) THEN                   ! Make sure the index I is okay
            IF ( INDEX( '\/', GivenFil(I+1:I+1)) == 0 ) THEN    ! Make sure we don't have the RootName in a different directory
               RootName = GivenFil(:I-1)
            ELSE
               RootName = GivenFil                              ! This does not have a file extension
            END IF
         ELSE
            IF ( I == 1 ) THEN
               RootName = ''
            ELSE
               RootName = GivenFil(:I-1)
            END IF
         END IF

         RETURN

      END IF
   END DO ! I

   RootName =  GivenFil


   RETURN
   END SUBROUTINE GetRoot
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

!=======================================================================
!> This routine returns the next unit number greater than 9 that is not currently in use.
!! If it cannot find any unit between 10 and 99 that is available, it either aborts or returns an appropriate error status/message.   
   SUBROUTINE GetNewUnit ( UnIn, ErrVar )



      ! Argument declarations.

   INTEGER,        INTENT(OUT)            :: UnIn                                         !< Logical unit for the file.                           !< The error message, if an error occurred
   TYPE(ErrorVariables), INTENT(INOUT)             :: ErrVar


      ! Local declarations.

   INTEGER                                :: Un                                           ! Unit number
   LOGICAL                                :: Opened                                       ! Flag indicating whether or not a file is opened.
   INTEGER(IntKi), PARAMETER              :: StartUnit = 10                               ! Starting unit number to check (numbers less than 10 reserved)
   ! NOTE: maximum unit numbers in fortran 90 and later is 2**31-1.  However, there are limits within the OS.
   !     macos -- 256  (change with ulimit -n)
   !     linux -- 1024 (change with ulimit -n)
   !     windows -- 512 (not sure how to change -- ADP)
   INTEGER(IntKi), PARAMETER              :: MaxUnit   = 1024                             ! The maximum unit number available (or 10 less than the number of files you want to have open at a time)

      ! Initialize subroutine outputs

   Un = StartUnit

      ! See if unit is connected to an open file. Check the next largest number until it is not opened.

   DO

      INQUIRE ( UNIT=Un , OPENED=Opened )

      IF ( .NOT. Opened )  EXIT
      Un = Un + 1

      IF ( Un > MaxUnit ) THEN

         ErrVar%aviFAIL = -1
         ErrVar%ErrMsg  = 'GetNewUnit() was unable to find an open file unit specifier between '//TRIM(Int2LStr(StartUnit)) &
                                                                            //' and '//TRIM(Int2LStr(MaxUnit))//'.'

         EXIT           ! stop searching now

      END IF


   END DO

   UnIn = Un

   RETURN
   END SUBROUTINE GetNewUnit

!=======================================================================
!> This function returns a character string encoded with the time in the form "hh:mm:ss".
    FUNCTION CurTime( )

    ! Function declaration.

    CHARACTER(8)                 :: CurTime                                      !< The current time in the form "hh:mm:ss".


    ! Local declarations.

    CHARACTER(10)                :: CTime                                        ! String to hold the returned value from the DATE_AND_TIME subroutine call.



    CALL DATE_AND_TIME ( TIME=CTime )

    CurTime = CTime(1:2)//':'//CTime(3:4)//':'//CTime(5:6)


    RETURN
    END FUNCTION CurTime

!=======================================================================
! This function checks whether an array is non-decreasing
    LOGICAL Function NonDecreasing(Array)

    IMPLICIT NONE

    REAL(DbKi), DIMENSION(:)            :: Array
    INTEGER(IntKi)         :: I_DIFF

    NonDecreasing = .TRUE.
    ! Is Array non decreasing
    DO I_DIFF = 1, size(Array) - 1
        IF (Array(I_DIFF + 1) - Array(I_DIFF) <= 0) THEN
            NonDecreasing = .FALSE.
            RETURN
        END IF
    END DO

    RETURN
    END FUNCTION NonDecreasing

!=======================================================================
!> This routine converts all the text in a string to upper case.
    SUBROUTINE Conv2UC ( Str )

        ! Argument declarations.
  
     CHARACTER(*), INTENT(INOUT)  :: Str                                          !< The string to be converted to UC (upper case).
  
  
        ! Local declarations.
  
     INTEGER                      :: IC                                           ! Character index
  
  
  
     DO IC=1,LEN_TRIM( Str )
  
        IF ( ( Str(IC:IC) >= 'a' ).AND.( Str(IC:IC) <= 'z' ) )  THEN
           Str(IC:IC) = CHAR( ICHAR( Str(IC:IC) ) - 32 )
        END IF
  
     END DO ! IC
  
  
     RETURN
     END SUBROUTINE Conv2UC

!=======================================================================
     !> This function returns a left-adjusted string representing the passed numeric value. 
    !! It eliminates trailing zeroes and even the decimal point if it is not a fraction. \n
    !! Use Num2LStr (nwtc_io::num2lstr) instead of directly calling a specific routine in the generic interface.   
    FUNCTION Int2LStr ( Num )

        CHARACTER(11)                :: Int2LStr                                     !< string representing input number.
    
    
        ! Argument declarations.
    
        INTEGER, INTENT(IN)          :: Num                                          !< The number to convert to a left-justified string.
    
    
    
        WRITE (Int2LStr,'(I11)')  Num
    
        Int2Lstr = ADJUSTL( Int2LStr )
    
    
        RETURN
        END FUNCTION Int2LStr

!=======================================================================

    subroutine AddToList(list, element)
        ! Credit to: https://stackoverflow.com/questions/28048508/how-to-add-new-element-to-dynamical-array-in-fortran-90
        ! This is set up for integers, will need to make interface for other types
          IMPLICIT NONE

          integer :: i, isize
          Integer(IntKi), intent(in) :: element
          Integer(IntKi), dimension(:), allocatable, intent(inout) :: list
          Integer(IntKi), dimension(:), allocatable :: clist


          if(allocated(list)) then
              isize = size(list)
              allocate(clist(isize+1))
              do i=1,isize          
              clist(i) = list(i)
              end do
              clist(isize+1) = element

              deallocate(list)
              call move_alloc(clist, list)

          else
              allocate(list(1))
              list(1) = element
          end if


      end subroutine AddToList

    !-------------------------------------------------------------------------------------------------------------------------------
    ! Copied from NWTC_IO.f90
    !> This function returns a character string encoded with today's date in the form dd-mmm-ccyy.
    FUNCTION CurDate( )

    ! Function declaration.

    CHARACTER(11)                :: CurDate                                      !< 'dd-mmm-yyyy' string with the current date


    ! Local declarations.

    CHARACTER(8)                 :: CDate                                        ! String to hold the returned value from the DATE_AND_TIME subroutine call.



    !  Call the system date function.

    CALL DATE_AND_TIME ( CDate )


    !  Parse out the day.

    CurDate(1:3) = CDate(7:8)//'-'


    !  Parse out the month.

    SELECT CASE ( CDate(5:6) )
    CASE ( '01' )
        CurDate(4:6) = 'Jan'
    CASE ( '02' )
        CurDate(4:6) = 'Feb'
    CASE ( '03' )
        CurDate(4:6) = 'Mar'
    CASE ( '04' )
        CurDate(4:6) = 'Apr'
    CASE ( '05' )
        CurDate(4:6) = 'May'
    CASE ( '06' )
        CurDate(4:6) = 'Jun'
    CASE ( '07' )
        CurDate(4:6) = 'Jul'
    CASE ( '08' )
        CurDate(4:6) = 'Aug'
    CASE ( '09' )
        CurDate(4:6) = 'Sep'
    CASE ( '10' )
        CurDate(4:6) = 'Oct'
    CASE ( '11' )
        CurDate(4:6) = 'Nov'
    CASE ( '12' )
        CurDate(4:6) = 'Dec'
    END SELECT


    !  Parse out the year.

    CurDate(7:11) = '-'//CDate(1:4)


    RETURN
    END FUNCTION CurDate


END MODULE ROSCO_Helpers
