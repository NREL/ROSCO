! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.



MODULE Constants
    USE, INTRINSIC  :: ISO_C_Binding
    
    Character(*), PARAMETER     :: rosco_version = 'v2.4.1'             ! ROSCO version
    INTEGER, PARAMETER                  :: DbKi             = C_DOUBLE            !< Default kind for double floating-point numbers
    INTEGER, PARAMETER                  :: ReKi             = C_FLOAT             !< Default kind for single floating-point numbers
    INTEGER, PARAMETER                  :: IntKi            = C_INT               !< Default kind for integer numbers
    
    REAL(DbKi), PARAMETER               :: RPS2RPM          = 9.5492966           ! Factor to convert radians per second to revolutions per minute.
    REAL(DbKi), PARAMETER               :: R2D              = 57.295780           ! Factor to convert radians to degrees.
    REAL(DbKi), PARAMETER               :: D2R              = 0.0175              ! Factor to convert degrees to radians.
    REAL(DbKi), PARAMETER               :: PI               = 3.14159265359       ! Mathematical constant pi
    INTEGER(IntKi), PARAMETER           :: NP_1             = 1                   ! First rotational harmonic
    INTEGER(IntKi), PARAMETER           :: NP_2             = 2                   ! Second rotational harmonic
    CHARACTER(*),  PARAMETER            :: NewLine          = ACHAR(10)           ! The delimiter for New Lines [ Windows is CHAR(13)//CHAR(10); MAC is CHAR(13); Unix is CHAR(10) {CHAR(13)=\r is a line feed, CHAR(10)=\n is a new line}]
END MODULE Constants