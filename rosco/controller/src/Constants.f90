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
    
    INTEGER, PARAMETER                  :: DbKi             = C_DOUBLE            !< Default kind for double floating-point numbers
    INTEGER, PARAMETER                  :: ReKi             = C_FLOAT             !< Default kind for single floating-point numbers
    INTEGER, PARAMETER                  :: IntKi            = C_INT               !< Default kind for integer numbers
    
    REAL(DbKi), PARAMETER               :: RPS2RPM          = 9.5492966           ! Factor to convert radians per second to revolutions per minute.
    REAL(DbKi), PARAMETER               :: R2D              = 57.2957795130       ! Factor to convert radians to degrees.
    REAL(DbKi), PARAMETER               :: D2R              = 0.01745329251       ! Factor to convert degrees to radians.
    REAL(DbKi), PARAMETER               :: PI               = 3.14159265359       ! Mathematical constant pi
    INTEGER(IntKi), PARAMETER           :: NP_1             = 1                   ! First rotational harmonic
    INTEGER(IntKi), PARAMETER           :: NP_2             = 2                   ! Second rotational harmonic

    ! NWTC Constants
    INTEGER(IntKi), PARAMETER     :: NWTC_MAX_DLL_PROC    = 3
    INTEGER(IntKi), PARAMETER     :: ErrID_None   = 0                              !< ErrStat parameter indicating "no error"
    INTEGER(IntKi), PARAMETER     :: ErrID_Info   = 1                              !< ErrStat parameter indicating "informational message"
    INTEGER(IntKi), PARAMETER     :: ErrID_Warn   = 2                              !< ErrStat parameter indicating "warning"
    INTEGER(IntKi), PARAMETER     :: ErrID_Severe = 3                              !< ErrStat parameter indicating "severe error"; 
    INTEGER(IntKi), PARAMETER     :: ErrID_Fatal  = 4                              !< ErrStat parameter indicating "fatal error"; simulation should end
    INTEGER, PARAMETER            :: BITS_IN_ADDR  = C_INTPTR_T*8                  !< The number of bits in an address (32-bit or 64-bit).

    CHARACTER(1), PARAMETER       :: Tab      = CHAR( 9 ) 

    ! Control Modes

    ! VS_ControlMode
    INTEGER(IntKi), PARAMETER     :: VS_Mode_Disabled   = 0
    INTEGER(IntKi), PARAMETER     :: VS_Mode_KOmega     = 1
    INTEGER(IntKi), PARAMETER     :: VS_Mode_WSE_TSR    = 2
    INTEGER(IntKi), PARAMETER     :: VS_Mode_Power_TSR  = 3
    INTEGER(IntKi), PARAMETER     :: VS_Mode_Torque_TSR = 4

    ! VS_ConstPower
    INTEGER(IntKi), PARAMETER     :: VS_Mode_ConstTrq = 0
    INTEGER(IntKi), PARAMETER     :: VS_Mode_ConstPwr = 1

    ! VS_FBP
    INTEGER(IntKi), PARAMETER     :: VS_FBP_Variable_Pitch  = 0
    INTEGER(IntKi), PARAMETER     :: VS_FBP_Power_Overspeed = 1
    INTEGER(IntKi), PARAMETER     :: VS_FBP_WSE_Ref         = 2
    INTEGER(IntKi), PARAMETER     :: VS_FBP_Torque_Ref      = 3

    ! VS_State
    INTEGER(IntKi), PARAMETER     :: VS_State_Error             = 0
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_1_5        = 1
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_2          = 2
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_2_5        = 3
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_3_ConstTrq = 4
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_3_ConstPwr = 5
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_3_FBP      = 6
    INTEGER(IntKi), PARAMETER     :: VS_State_PI                = 7

    ! PC_State
    INTEGER(IntKi), PARAMETER     :: PC_State_Disabled = 0
    INTEGER(IntKi), PARAMETER     :: PC_State_Enabled  = 1
    
    ! PRC Mode constants
    INTEGER(IntKi), PARAMETER      :: PRC_Comm_Constant    = 0     ! Constant based on discon input
    INTEGER(IntKi), PARAMETER      :: PRC_Comm_OpenLoop    = 1     ! Based on open loop input
    INTEGER(IntKi), PARAMETER      :: PRC_Comm_ZMQ         = 2     ! Determined using ZMQ input


END MODULE Constants
