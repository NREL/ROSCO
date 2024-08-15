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
   CHARACTER(*),  PARAMETER      :: OS_Desc     = 'LLVM Fortran for Linux'           ! Description of the language/OS
   CHARACTER( 1), PARAMETER      :: PathSep     = '/'                               ! The path separator.
   CHARACTER( 1), PARAMETER      :: SwChar      = '-'                               ! The switch character for command-line options.
   CHARACTER(11), PARAMETER      :: UnfForm     = 'UNFORMATTED'                     ! The string to specify unformatted I/O files.


    CONTAINS
!=================================================================================================================
    SUBROUTINE LoadDynamicLib ( DLL, ErrStat, ErrMsg )

        ! This SUBROUTINE is used to dynamically load a DLL.
    
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be loaded.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
    
    
    !bjj: these are values I found on the web; I have no idea if they actually work...
    !bjj: hopefully we can find them pre-defined in a header somewhere
        INTEGER(C_INT), PARAMETER :: RTLD_LAZY=1            ! "Perform lazy binding. Only resolve symbols as the code that references them is executed. If the symbol is never referenced, then it is never resolved. (Lazy binding is only performed for function references; references to variables are always immediately bound when the library is loaded.) "
        INTEGER(C_INT), PARAMETER :: RTLD_NOW=2             ! "If this value is specified, or the environment variable LD_BIND_NOW is set to a nonempty string, all undefined symbols in the library are resolved before dlopen() returns. If this cannot be done, an error is returned."
        INTEGER(C_INT), PARAMETER :: RTLD_GLOBAL=256        ! "The symbols defined by this library will be made available for symbol resolution of subsequently loaded libraries"
        INTEGER(C_INT), PARAMETER :: RTLD_LOCAL=0           ! "This is the converse of RTLD_GLOBAL, and the default if neither flag is specified. Symbols defined in this library are not made available to resolve references in subsequently loaded libraries."
    
        INTERFACE !linux API routines
        !bjj see http://linux.die.net/man/3/dlopen
        !    and https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/dlopen.3.html
    
        FUNCTION dlOpen(filename,mode) BIND(C,NAME="dlopen")
        ! void *dlopen(const char *filename, int mode);
            USE ISO_C_BINDING
            IMPLICIT NONE
            TYPE(C_PTR)                   :: dlOpen
            CHARACTER(C_CHAR), INTENT(IN) :: filename(*)
            INTEGER(C_INT), VALUE         :: mode
        END FUNCTION
    
        END INTERFACE
    
        ErrStat = 0
        ErrMsg = ''
    
        ! Load the DLL and get the file address:
    
        DLL%FileAddrX = dlOpen( TRIM(DLL%FileName)//C_NULL_CHAR, RTLD_LAZY )  !the "C_NULL_CHAR" converts the Fortran string to a C-type string (i.e., adds //CHAR(0) to the end)
    
        IF( .NOT. C_ASSOCIATED(DLL%FileAddrX) ) THEN
        ErrStat = -1
        ! WRITE(ErrMsg,'(I2)') BITS_IN_ADDR
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
    
        ! This SUBROUTINE is used to dynamically load a procedure from a DLL.
    
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be loaded.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
        INTEGER(IntKi)                            :: i
    
   
        INTERFACE !linux API routines
    
        !bjj see http://linux.die.net/man/3/dlsym
        !    and https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/dlsym.3.html
        
        FUNCTION dlSym(handle,name) BIND(C,NAME="dlsym")
        ! void *dlsym(void *handle, const char *name);
            USE ISO_C_BINDING
            IMPLICIT NONE
            TYPE(C_FUNPTR)                :: dlSym ! A function pointer
            TYPE(C_PTR), VALUE            :: handle
            CHARACTER(C_CHAR), INTENT(IN) :: name(*)
        END FUNCTION
    
        END INTERFACE
    
        ErrStat = ErrID_None
        ErrMsg = ''
    
        do i=1,NWTC_MAX_DLL_PROC
        if ( len_trim( DLL%ProcName(i) ) > 0 ) then
        
            DLL%ProcAddr(i) = dlSym( DLL%FileAddrX, TRIM(DLL%ProcName(i))//C_NULL_CHAR )  !the "C_NULL_CHAR" converts the Fortran string to a C-type string (i.e., adds //CHAR(0) to the end)
    
            IF(.NOT. C_ASSOCIATED(DLL%ProcAddr(i))) THEN
                ErrStat = ErrID_Fatal
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
        INTEGER(C_INT)                            :: Success     ! Whether or not the call to dlClose was successful
        INTEGER(C_INT), PARAMETER                 :: TRUE  = 0
    
    !bjj: note that this is not tested.
    
        INTERFACE !linux API routine
        !bjj see http://linux.die.net/man/3/dlclose
        !    and https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/dlclose.3.html
    
        FUNCTION dlClose(handle) BIND(C,NAME="dlclose")
        ! int dlclose(void *handle);
            USE ISO_C_BINDING
            IMPLICIT NONE
            INTEGER(C_INT)       :: dlClose
            TYPE(C_PTR), VALUE   :: handle
        END FUNCTION
    
        END INTERFACE
    
        ! Close the library:
    
        IF( .NOT. C_ASSOCIATED(DLL%FileAddrX) ) RETURN
        Success = dlClose( DLL%FileAddrX ) !The function dlclose() returns 0 on success, and nonzero on error.
    
        IF ( Success /= TRUE ) THEN !bjj: note that this is not the same as LOGICAL .TRUE.
        ErrStat = ErrID_Fatal
        ErrMsg  = 'The dynamic library could not be freed.'
        RETURN
        ELSE
        ErrStat = ErrID_None
        ErrMsg = ''
        DLL%FileAddrX = C_NULL_PTR
        END IF
        
    
        RETURN
    END SUBROUTINE FreeDynamicLib
    !=======================================================================

END MODULE SysSubs
