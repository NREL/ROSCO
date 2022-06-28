!**********************************************************************************************************************************
! LICENSING
! Copyright (C) 2021  National Renewable Energy Laboratory
!
!    This file is part of ROSCO.
!
! Licensed under the Apache License, Version 2.0 (the "License");
! you may not use this file except in compliance with the License.
! You may obtain a copy of the License at
!
!     http://www.apache.org/licenses/LICENSE-2.0
!
! Unless required by applicable law or agreed to in writing, software
! distributed under the License is distributed on an "AS IS" BASIS,
! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
! See the License for the specific language governing permissions and
! limitations under the License.
!**********************************************************************************************************************************
MODULE SysSubs

    USE ROSCO_Types
    USE Constants


    IMPLICIT NONE





! This module contains routines with system-specific logic and references, including all references to the console unit, CU.
! It also contains standard (but not system-specific) routines it uses.
! SysGnuLinux.f90 is specifically for the GNU Fortran (gfortran) compiler on Linux and macOS.

    INTEGER, PARAMETER            :: ConRecL     = 120                               ! The record length for console output.
    INTEGER, PARAMETER            :: CU          = 6                                 ! The I/O unit for the console.  Unit 6 causes ADAMS to crash.
    INTEGER, PARAMETER            :: MaxWrScrLen = 98                                ! The maximum number of characters allowed to be written to a line in WrScr
    LOGICAL, PARAMETER            :: KBInputOK   = .TRUE.                            ! A flag to tell the program that keyboard input is allowed in the environment.
    CHARACTER(*),  PARAMETER      :: NewLine     = ACHAR(10)                         ! The delimiter for New Lines [ Windows is CHAR(13)//CHAR(10); MAC is CHAR(13); Unix is CHAR(10) {CHAR(13)=\r is a line feed, CHAR(10)=\n is a new line}]
    CHARACTER(*),  PARAMETER      :: OS_Desc     = 'GNU Fortran for Windows'         ! Description of the language/OS
    CHARACTER( 1), PARAMETER      :: PathSep     = '\'                               ! The path separator.
    CHARACTER( 1), PARAMETER      :: SwChar      = '/'                               ! The switch character for command-line options.
    CHARACTER(11), PARAMETER      :: UnfForm     = 'UNFORMATTED'                     ! The string to specify unformatted I/O files.
 

    CONTAINS
!=======================================================================
    SUBROUTINE LoadDynamicLib ( DLL, ErrStat, ErrMsg )

        ! This SUBROUTINE is used to dynamically load a DLL.
     
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be loaded.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
     
        INTERFACE  ! Definitions of Windows API routines
     
           !...........................
           !bjj: I have been unable to find a solution that works with both IVF and gfortran...
           !bjj: note that "Intel Fortran does not support use of STDCALL with BIND(C) at this time"
           !     See this link: http://software.intel.com/en-us/articles/replacing-intel-fortran-attributes-with-c-interoperability-features
           !bjj: Until this is fixed, Intel uses kernel32.f90 definitions instead of the interface below:
           !...........................
     
           FUNCTION LoadLibrary(lpFileName) BIND(C,NAME='LoadLibraryA')
              USE, INTRINSIC :: ISO_C_BINDING
              IMPLICIT NONE
              !GCC$ ATTRIBUTES STDCALL :: LoadLibrary
              INTEGER(C_INTPTR_T)        :: LoadLibrary
              CHARACTER(KIND=C_CHAR)     :: lpFileName(*)
           END FUNCTION LoadLibrary
     
        END INTERFACE
     
        ErrStat = ErrID_None
        ErrMsg = ''
     
        ! Load the DLL and get the file address:
        DLL%FileAddr = LoadLibrary( TRIM(DLL%FileName)//C_NULL_CHAR )  !the "C_NULL_CHAR" converts the Fortran string to a C-type string (i.e., adds //CHAR(0) to the end)
        IF ( DLL%FileAddr == INT(0,C_INTPTR_T) ) THEN
           ErrStat = ErrID_Fatal
         !   WRITE(ErrMsg,'(I2)') BITS_IN_ADDR
           ErrMsg  = 'The dynamic library '//TRIM(DLL%FileName)//' could not be loaded. Check that the file '// &
                    'exists in the specified location and that it is compiled for '//TRIM(ErrMsg)//'-bit applications.'
           RETURN
        END IF
     
        ! Get the procedure address:
        CALL LoadDynamicLibProc ( DLL, ErrStat, ErrMsg )
     
        RETURN
     END SUBROUTINE LoadDynamicLib
     !=======================================================================
     SUBROUTINE LoadDynamicLibProc ( DLL, ErrStat, ErrMsg )
     
        ! This SUBROUTINE is used to dynamically load a procedure in a DLL.
     
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be loaded.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
        INTEGER(IntKi)                            :: i
     
        INTERFACE  ! Definitions of Windows API routines
     
           !...........................
           !bjj: I have been unable to find a solution that works with both IVF and gfortran...
           !bjj: note that "Intel Fortran does not support use of STDCALL with BIND(C) at this time"
           !     See this link: http://software.intel.com/en-us/articles/replacing-intel-fortran-attributes-with-c-interoperability-features
           !bjj: Until this is fixed, Intel uses kernel32.f90 definitions instead of the interface below:
           !...........................
     
           FUNCTION GetProcAddress(hModule, lpProcName) BIND(C, NAME='GetProcAddress')
              USE, INTRINSIC :: ISO_C_BINDING
              IMPLICIT NONE
              !GCC$ ATTRIBUTES STDCALL :: GetProcAddress
              TYPE(C_FUNPTR)             :: GetProcAddress
              INTEGER(C_INTPTR_T),VALUE  :: hModule
              CHARACTER(KIND=C_CHAR)     :: lpProcName(*)
           END FUNCTION GetProcAddress
     
        END INTERFACE
     
        ErrStat = ErrID_None
        ErrMsg = ''
     
        ! Get the procedure addresses:
        do i=1,NWTC_MAX_DLL_PROC
           if ( len_trim( DLL%ProcName(i) ) > 0 ) then
              DLL%ProcAddr(i) = GetProcAddress( DLL%FileAddr, TRIM(DLL%ProcName(i))//C_NULL_CHAR )  !the "C_NULL_CHAR" converts the Fortran string to a C-type string (i.e., adds //CHAR(0) to the end)
              IF(.NOT. C_ASSOCIATED(DLL%ProcAddr(i))) THEN
                 ErrStat = ErrID_Fatal + i - 1
                 ErrMsg  = 'The procedure '//TRIM(DLL%ProcName(i))//' in file '//TRIM(DLL%FileName)//' could not be loaded.'
                 RETURN
              END IF
           end if
        end do
     
        RETURN
     END SUBROUTINE LoadDynamicLibProc
     !=======================================================================
     SUBROUTINE FreeDynamicLib ( DLL, ErrStat, ErrMsg )
     
        ! This SUBROUTINE is used to free a dynamically loaded DLL (loaded in LoadDynamicLib).
     
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be freed.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
     
        ! Local variable:
        INTEGER(C_INT)                            :: Success     ! Whether or not the call to FreeLibrary was successful
        INTEGER(C_INT), PARAMETER                 :: FALSE  = 0
     
        INTERFACE  ! Definitions of Windows API routines
     
           FUNCTION FreeLibrary(hLibModule) BIND(C, NAME='FreeLibrary')
              USE, INTRINSIC :: ISO_C_BINDING
              IMPLICIT NONE
              !GCC$ ATTRIBUTES STDCALL :: FreeLibrary
              INTEGER(C_INT)             :: FreeLibrary ! BOOL
              INTEGER(C_INTPTR_T),VALUE  :: hLibModule ! HMODULE hLibModule
           END FUNCTION
     
        END INTERFACE
     
        ! Free the DLL:
        IF ( DLL%FileAddr == INT(0,C_INTPTR_T) ) RETURN
     
        Success = FreeLibrary( DLL%FileAddr ) !If the function succeeds, the return value is nonzero. If the function fails, the return value is zero.
     
        IF ( Success == FALSE ) THEN !BJJ: note that this isn't the same as the Fortran LOGICAL .FALSE.
           ErrStat = ErrID_Fatal
           ErrMsg  = 'The dynamic library could not be freed.'
           RETURN
        ELSE
           ErrStat = ErrID_None
           ErrMsg = ''
           DLL%FileAddr = INT(0,C_INTPTR_T)
        END IF
     
        RETURN
     END SUBROUTINE FreeDynamicLib
     !=======================================================================

END MODULE SysSubs