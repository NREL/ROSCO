! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! This module contains basic functions used by the controller

! Functions:
!       AeroDynTorque: Calculate aerodynamic torque
!       ColemanTransform: Perform Colemant transform
!       ColemanTransformInverse: Perform inverse Colemant transform
!       CPFunction: Find Cp using parameterized surface
!       Debug: Debug the controller
!       DFController: DF control
!       identity: Make identity matrix
!       interp1d: 1-d interpolation
!       interp2d: 2-d interpolation
!       matinv3: 3x3 matrix inverse
!       PIController: implement a PI controller
!       ratelimit: Rate limit signal
!       saturate: Saturate signal

MODULE Functions

USE Constants
USE Filters

IMPLICIT NONE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION saturate(inputValue, minValue, maxValue)
    ! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue

        IMPLICIT NONE

        REAL(DbKi), INTENT(IN)     :: inputValue
        REAL(DbKi), INTENT(IN)     :: minValue
        REAL(DbKi), INTENT(IN)     :: maxValue

        saturate = REAL(MIN(MAX(inputValue,minValue), maxValue),DbKi)

    END FUNCTION saturate
    
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION ratelimit(inputSignal, inputSignalPrev, minRate, maxRate, DT)
    ! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
        IMPLICIT NONE

        REAL(DbKi), INTENT(IN)     :: inputSignal
        REAL(DbKi), INTENT(IN)     :: inputSignalPrev
        REAL(DbKi), INTENT(IN)     :: minRate
        REAL(DbKi), INTENT(IN)     :: maxRate
        REAL(DbKi), INTENT(IN)     :: DT
        ! Local variables
        REAL(DbKi)                 :: rate

        rate = (inputSignal - inputSignalPrev)/DT                       ! Signal rate (unsaturated)
        rate = saturate(rate, minRate, maxRate)                 ! Saturate the signal rate
        ratelimit = inputSignalPrev + rate*DT                       ! Saturate the overall command using the rate limit

    END FUNCTION ratelimit

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, reset, inst)
    ! PI controller, with output saturation

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi), INTENT(IN)         :: error
        REAL(DbKi), INTENT(IN)         :: kp
        REAL(DbKi), INTENT(IN)         :: ki
        REAL(DbKi), INTENT(IN)         :: minValue
        REAL(DbKi), INTENT(IN)         :: maxValue
        REAL(DbKi), INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)   :: inst
        REAL(DbKi), INTENT(IN)         :: I0
        LOGICAL, INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term
        REAL(DbKi), DIMENSION(99), SAVE    :: ITerm = (/ (real(9999.9), i = 1,99) /)       ! Integral term, current.
        REAL(DbKi), DIMENSION(99), SAVE    :: ITermLast = (/ (real(9999.9), i = 1,99) /)   ! Integral term, the last time this controller was called. Supports 99 separate instances.
        INTEGER(IntKi), DIMENSION(99), SAVE :: FirstCall = (/ (1, i=1,99) /)                ! First call of this function?
        
        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF ((FirstCall(inst) == 1) .OR. reset) THEN
            ITerm(inst) = I0
            ITermLast(inst) = I0
            
            FirstCall(inst) = 0
            PIController = I0
        ELSE
            PTerm = kp*error
            ITerm(inst) = ITerm(inst) + DT*ki*error
            ITerm(inst) = saturate(ITerm(inst), minValue, maxValue)
            PIController = saturate(PTerm + ITerm(inst), minValue, maxValue)
        
            ITermLast(inst) = ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIController

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, reset, inst)
    ! PI controller, with output saturation. 
    ! Added error2 term for additional integral control input

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi), INTENT(IN)         :: error
        REAL(DbKi), INTENT(IN)         :: error2
        REAL(DbKi), INTENT(IN)         :: kp
        REAL(DbKi), INTENT(IN)         :: ki2
        REAL(DbKi), INTENT(IN)         :: ki
        REAL(DbKi), INTENT(IN)         :: minValue
        REAL(DbKi), INTENT(IN)         :: maxValue
        REAL(DbKi), INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)   :: inst
        REAL(DbKi), INTENT(IN)         :: I0
        LOGICAL, INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term
        REAL(DbKi), DIMENSION(99), SAVE    :: ITerm = (/ (real(9999.9), i = 1,99) /)       ! Integral term, current.
        REAL(DbKi), DIMENSION(99), SAVE    :: ITermLast = (/ (real(9999.9), i = 1,99) /)   ! Integral term, the last time this controller was called. Supports 99 separate instances.
        REAL(DbKi), DIMENSION(99), SAVE    :: ITerm2 = (/ (real(9999.9), i = 1,99) /)       ! Second Integral term, current.
        REAL(DbKi), DIMENSION(99), SAVE    :: ITermLast2 = (/ (real(9999.9), i = 1,99) /)   ! Second Integral term, the last time this controller was called. Supports 99 separate instances.
        INTEGER(IntKi), DIMENSION(99), SAVE :: FirstCall = (/ (1, i=1,99) /)                ! First call of this function?
        
        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF ((FirstCall(inst) == 1) .OR. reset) THEN
            ITerm(inst) = I0
            ITermLast(inst) = I0
            ITerm2(inst) = I0
            ITermLast2(inst) = I0
            
            FirstCall(inst) = 0
            PIIController = I0
        ELSE
            PTerm = kp*error
            ITerm(inst) = ITerm(inst) + DT*ki*error
            ITerm2(inst) = ITerm2(inst) + DT*ki2*error2
            ITerm(inst) = saturate(ITerm(inst), minValue, maxValue)
            ITerm2(inst) = saturate(ITerm2(inst), minValue, maxValue)
            PIIController = PTerm + ITerm(inst) + ITerm2(inst)
            PIIController = saturate(PIIController, minValue, maxValue)
        
            ITermLast(inst) = ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIIController

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION interp1d(xData, yData, xq, ErrVar)
    ! interp1d 1-D interpolation (table lookup), xData should be strictly increasing
        
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE

        ! Inputs
        REAL(DbKi), DIMENSION(:), INTENT(IN)       :: xData        ! Provided x data (vector), to be interpolated
        REAL(DbKi), DIMENSION(:), INTENT(IN)       :: yData        ! Provided y data (vector), to be interpolated
        REAL(DbKi), INTENT(IN)                     :: xq           ! x-value for which the y value has to be interpolated
        INTEGER(IntKi)                              :: I            ! Iteration index

        ! Error Catching
        TYPE(ErrorVariables), INTENT(INOUT)     :: ErrVar
        INTEGER(IntKi)                              :: I_DIFF

        CHARACTER(*), PARAMETER                 :: RoutineName = 'interp1d'

        
        ! Catch Errors
        ! Are xData and yData the same size?
        IF (SIZE(xData) .NE. SIZE(yData)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = ' xData and yData are not the same size'
            WRITE(ErrVar%ErrMsg,"(A,I2,A,I2,A)") " SIZE(xData) =", SIZE(xData), & 
            ' and SIZE(yData) =', SIZE(yData),' are not the same'
        END IF

        ! Is xData non decreasing
        DO I_DIFF = 1, size(xData) - 1
            IF (xData(I_DIFF + 1) - xData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' xData is not strictly increasing'
                EXIT 
            END IF
        END DO
        
        ! Interpolate
        IF (xq <= MINVAL(xData)) THEN
            interp1d = yData(1)
        ELSEIF (xq >= MAXVAL(xData)) THEN
            interp1d = yData(SIZE(xData))
        ELSE
            DO I = 1, SIZE(xData)
                IF (xq <= xData(I)) THEN
                    interp1d = yData(I-1) + (yData(I) - yData(I-1))/(xData(I) - xData(I-1))*(xq - xData(I-1))
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        END IF

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION interp1d

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION interp2d(xData, yData, zData, xq, yq, ErrVar)
    ! interp2d 2-D interpolation (table lookup). Query done using bilinear interpolation. 
    ! Note that the interpolated matrix with associated query vectors may be different than "standard", - zData should be formatted accordingly
    ! - xData follows the matrix from left to right
    ! - yData follows the matrix from top to bottom
    ! A simple case: xData = [1 2 3], yData = [4 5 6]
    !        | 1    2   3
    !       -------------
    !       4| a    b   c
    !       5| d    e   f
    !       6| g    H   i

        USE ROSCO_Types, ONLY : ErrorVariables
        USE ieee_arithmetic
        
        IMPLICIT NONE
    
        ! Inputs
        REAL(DbKi), DIMENSION(:),   INTENT(IN)     :: xData        ! Provided x data (vector), to find query point (should be strictly increasing)
        REAL(DbKi), DIMENSION(:),   INTENT(IN)     :: yData        ! Provided y data (vector), to find query point (should be strictly increasing)
        REAL(DbKi), DIMENSION(:,:), INTENT(IN)     :: zData        ! Provided z data (vector), to be interpolated
        REAL(DbKi),                 INTENT(IN)     :: xq           ! x-value for which the z value has to be interpolated
        REAL(DbKi),                 INTENT(IN)     :: yq           ! y-value for which the z value has to be interpolated

        ! Allocate variables
        INTEGER(IntKi)                              :: i            ! Iteration index & query index, x-direction
        INTEGER(IntKi)                              :: ii           ! Iteration index & second que .  ry index, x-direction
        INTEGER(IntKi)                              :: j            ! Iteration index & query index, y-direction
        INTEGER(IntKi)                              :: jj           ! Iteration index & second query index, y-direction
        REAL(DbKi), DIMENSION(2,2)                 :: fQ           ! zData value at query points for bilinear interpolation            
        REAL(DbKi), DIMENSION(1)                   :: fxy           ! Interpolated z-data point to be returned
        REAL(DbKi)                                 :: fxy1          ! zData value at query point for bilinear interpolation
        REAL(DbKi)                                 :: fxy2          ! zData value at query point for bilinear interpolation       
        LOGICAL                                 :: edge     

        ! Error Catching
        TYPE(ErrorVariables), INTENT(INOUT)     :: ErrVar
        INTEGER(IntKi)                              :: I_DIFF

        CHARACTER(*), PARAMETER                 :: RoutineName = 'interp2d'
        
        ! Error catching
        ! Are xData and zData(:,1) the same size?
        IF (SIZE(xData) .NE. SIZE(zData,2)) THEN
            ErrVar%aviFAIL = -1
            WRITE(ErrVar%ErrMsg,"(A,I4,A,I4,A)") " SIZE(xData) =", SIZE(xData), & 
            ' and SIZE(zData,1) =', SIZE(zData,2),' are not the same'
        END IF

        ! Are yData and zData(1,:) the same size?
        IF (SIZE(yData) .NE. SIZE(zData,1)) THEN
            ErrVar%aviFAIL = -1
            WRITE(ErrVar%ErrMsg,"(A,I4,A,I4,A)") " SIZE(yData) =", SIZE(yData), & 
            ' and SIZE(zData,2) =', SIZE(zData,1),' are not the same'
        END IF

        ! Is xData non decreasing
        DO I_DIFF = 1, size(xData) - 1
            IF (xData(I_DIFF + 1) - xData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' xData is not strictly increasing'
                EXIT 
            END IF
        END DO

        ! Is yData non decreasing
        DO I_DIFF = 1, size(yData) - 1
            IF (yData(I_DIFF + 1) - yData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' yData is not strictly increasing'
                EXIT 
            END IF
        END DO

        ! ---- Find corner indices surrounding desired interpolation point -----
            ! x-direction
        IF (xq <= MINVAL(xData) .OR. (ieee_is_nan(xq))) THEN       ! On lower x-bound, just need to find zData(yq)
            j = 1
            jj = 1
            interp2d = interp1d(yData,zData(:,j),yq,ErrVar)     
            RETURN
        ELSEIF (xq >= MAXVAL(xData)) THEN   ! On upper x-bound, just need to find zData(yq)
            j = size(xData)
            jj = size(xData)
            interp2d = interp1d(yData,zData(:,j),yq,ErrVar)
            RETURN
        ELSE
            DO j = 1,size(xData)            
                IF (xq == xData(j)) THEN ! On axis, just need 1d interpolation
                    jj = j
                    interp2d = interp1d(yData,zData(:,j),yq,ErrVar)  
                    RETURN
                ELSEIF (xq < xData(j)) THEN
                    jj = j
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        ENDIF
        j = j-1 ! Move j back one
            ! y-direction
        IF (yq <= MINVAL(yData) .OR. (ieee_is_nan(yq))) THEN       ! On lower y-bound, just need to find zData(xq)
            i = 1
            ii = 1
            interp2d = interp1d(xData,zData(i,:),xq,ErrVar)     
            RETURN
        ELSEIF (yq >= MAXVAL(yData)) THEN   ! On upper y-bound, just need to find zData(xq)
            i = size(yData)
            ii = size(yData)
            interp2d = interp1d(xData,zData(i,:),xq,ErrVar)      
            RETURN
        ELSE
            DO i = 1,size(yData)
                IF (yq == yData(i)) THEN    ! On axis, just need 1d interpolation
                    ii = i
                    interp2d = interp1d(xData,zData(i,:),xq,ErrVar)        
                    RETURN
                ELSEIF (yq < yData(i)) THEN
                    ii = i
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        ENDIF
        i = i-1 ! move i back one
        
        ! ---- Do bilinear interpolation ----
        ! Find values at corners 
        fQ(1,1) = zData(i,j)
        fQ(2,1) = zData(ii,j)
        fQ(1,2) = zData(i,jj)
        fQ(2,2) = zData(ii,jj)
        ! Interpolate
        fxy1 = (xData(jj) - xq)/(xData(jj) - xData(j))*fQ(1,1) + (xq - xData(j))/(xData(jj) - xData(j))*fQ(1,2)
        fxy2 = (xData(jj) - xq)/(xData(jj) - xData(j))*fQ(2,1) + (xq - xData(j))/(xData(jj) - xData(j))*fQ(2,2)
        fxy = (yData(ii) - yq)/(yData(ii) - yData(i))*fxy1 + (yq - yData(i))/(yData(ii) - yData(i))*fxy2

        interp2d = fxy(1)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END FUNCTION interp2d

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION matinv3(A) RESULT(B)
    ! Performs a direct calculation of the inverse of a 3×3 matrix.
    ! Source: http://fortranwiki.org/fortran/show/Matrix+inversion
        REAL(DbKi), INTENT(IN) :: A(3,3)   !! Matrix
        REAL(DbKi)             :: B(3,3)   !! Inverse matrix
        REAL(DbKi)             :: detinv

        ! Calculate the inverse determinant of the matrix
        detinv = 1/(A(1,1)*A(2,2)*A(3,3) - A(1,1)*A(2,3)*A(3,2)&
                - A(1,2)*A(2,1)*A(3,3) + A(1,2)*A(2,3)*A(3,1)&
                + A(1,3)*A(2,1)*A(3,2) - A(1,3)*A(2,2)*A(3,1))

        ! Calculate the inverse of the matrix
        B(1,1) = +detinv * (A(2,2)*A(3,3) - A(2,3)*A(3,2))
        B(2,1) = -detinv * (A(2,1)*A(3,3) - A(2,3)*A(3,1))
        B(3,1) = +detinv * (A(2,1)*A(3,2) - A(2,2)*A(3,1))
        B(1,2) = -detinv * (A(1,2)*A(3,3) - A(1,3)*A(3,2))
        B(2,2) = +detinv * (A(1,1)*A(3,3) - A(1,3)*A(3,1))
        B(3,2) = -detinv * (A(1,1)*A(3,2) - A(1,2)*A(3,1))
        B(1,3) = +detinv * (A(1,2)*A(2,3) - A(1,3)*A(2,2))
        B(2,3) = -detinv * (A(1,1)*A(2,3) - A(1,3)*A(2,1))
        B(3,3) = +detinv * (A(1,1)*A(2,2) - A(1,2)*A(2,1))
    END FUNCTION matinv3

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION identity(n) RESULT(A)
    ! Produces an identity matrix of size n x n

        INTEGER, INTENT(IN)         :: n
        REAL(DbKi), DIMENSION(n, n)    :: A
        INTEGER                     :: i
        INTEGER                     :: j

        ! Build identity matrix 
        DO i=1,n  
            DO j = 1,n
                IF (i == j) THEN 
                    A(i,j) = 1.0
                ELSE
                    A(i,j) = 0.0
                ENDIF
            ENDDO
        ENDDO
    
    END FUNCTION identity

!-------------------------------------------------------------------------------------------------------------------------------  
    REAL(DbKi) FUNCTION DFController(error, Kd, Tf, DT, inst)
    ! DF controller, with output saturation
    
        IMPLICIT NONE
        ! Inputs
        REAL(DbKi), INTENT(IN)     :: error
        REAL(DbKi), INTENT(IN)     :: kd
        REAL(DbKi), INTENT(IN)     :: tf
        REAL(DbKi), INTENT(IN)     :: DT
        INTEGER(IntKi), INTENT(IN)  :: inst
        ! Local
        REAL(DbKi)                         :: B                                    ! 
        INTEGER(IntKi)                      :: i                                    ! Counter for making arrays
        REAL(DbKi), DIMENSION(99), SAVE    :: errorLast = (/ (0, i=1,99) /)        ! 
        REAL(DbKi), DIMENSION(99), SAVE    :: DFControllerLast = (/ (0, i=1,99) /) ! 
        INTEGER(IntKi), DIMENSION(99), SAVE :: FirstCall = (/ (1, i=1,99) /)        ! First call of this function?
        
        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        ! IF (FirstCall(inst) == 1) THEN
            ! FirstCall(inst) = 0
        ! END IF
        
        B = 2.0/DT
        DFController = (Kd*B)/(B*Tf+1.0)*error - (Kd*B)/(B*Tf+1.0)*errorLast(inst) - (1.0-B*Tf)/(B*Tf+1.0)*DFControllerLast(inst)

        errorLast(inst) = error
        DFControllerLast(inst) = DFController
    END FUNCTION DFController

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ColemanTransform(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
    ! The Coleman or d-q axis transformation transforms the root out of plane bending moments of each turbine blade
    ! to a direct axis and a quadrature axis

        IMPLICIT NONE
        ! Inputs
        REAL(DbKi), INTENT(IN)     :: rootMOOP(3)                      ! Root out of plane bending moments of each blade
        REAL(DbKi), INTENT(IN)     :: aziAngle                         ! Rotor azimuth angle
        INTEGER(IntKi), INTENT(IN)  :: nHarmonic                        ! The harmonic number, nP
        ! Outputs
        REAL(DbKi), INTENT(OUT)    :: axTOut, axYOut               ! Direct axis and quadrature axis outputted by this transform
        ! Local
        REAL(DbKi), PARAMETER      :: phi2 = 2.0/3.0*PI                ! Phase difference from first to second blade
        REAL(DbKi), PARAMETER      :: phi3 = 4.0/3.0*PI                ! Phase difference from first to third blade

        ! Body
        axTOut  = 2.0/3.0 * (cos(nHarmonic*(aziAngle))*rootMOOP(1) + cos(nHarmonic*(aziAngle+phi2))*rootMOOP(2) + cos(nHarmonic*(aziAngle+phi3))*rootMOOP(3))
        axYOut  = 2.0/3.0 * (sin(nHarmonic*(aziAngle))*rootMOOP(1) + sin(nHarmonic*(aziAngle+phi2))*rootMOOP(2) + sin(nHarmonic*(aziAngle+phi3))*rootMOOP(3))
        
    END SUBROUTINE ColemanTransform

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ColemanTransformInverse(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC)
    ! The inverse Coleman or d-q axis transformation transforms the direct axis and quadrature axis
    ! back to root out of plane bending moments of each turbine blade
        IMPLICIT NONE
        ! Inputs
        REAL(DbKi), INTENT(IN)     :: axTIn, axYIn         ! Direct axis and quadrature axis
        REAL(DbKi), INTENT(IN)     :: aziAngle                     ! Rotor azimuth angle
        REAL(DbKi), INTENT(IN)     :: aziOffset                    ! Phase shift added to the azimuth angle
        INTEGER(IntKi), INTENT(IN)  :: nHarmonic                    ! The harmonic number, nP
        ! Outputs
        REAL(DbKi), INTENT(OUT)    :: PitComIPC(3)                 ! Root out of plane bending moments of each blade
        ! Local
        REAL(DbKi), PARAMETER      :: phi2 = 2.0/3.0*PI                ! Phase difference from first to second blade
        REAL(DbKi), PARAMETER      :: phi3 = 4.0/3.0*PI                ! Phase difference from first to third blade

        ! Body
        PitComIPC(1) = cos(nHarmonic*(aziAngle+aziOffset))*axTIn + sin(nHarmonic*(aziAngle+aziOffset))*axYIn
        PitComIPC(2) = cos(nHarmonic*(aziAngle+aziOffset+phi2))*axTIn + sin(nHarmonic*(aziAngle+aziOffset+phi2))*axYIn
        PitComIPC(3) = cos(nHarmonic*(aziAngle+aziOffset+phi3))*axTIn + sin(nHarmonic*(aziAngle+aziOffset+phi3))*axYIn

    END SUBROUTINE ColemanTransformInverse

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION CPfunction(CP, lambda)
    ! Paremeterized Cp(lambda) function for a fixed pitch angle. Circumvents the need of importing a look-up table
        IMPLICIT NONE
        
        ! Inputs
        REAL(DbKi), INTENT(IN) :: CP(4)    ! Parameters defining the parameterizable Cp(lambda) function
        REAL(DbKi), INTENT(IN) :: lambda    ! Estimated or measured tip-speed ratio input
        
        ! Lookup
        CPfunction = exp(-CP(1)/lambda)*(CP(2)/lambda-CP(3))+CP(4)*lambda
        CPfunction = saturate(CPfunction, 0.001D0, 1.0D0)
        
    END FUNCTION CPfunction

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION AeroDynTorque(LocalVar, CntrPar, PerfData, ErrVar)
    ! Function for computing the aerodynamic torque, divided by the effective rotor torque of the turbine, for use in wind speed estimation
        
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, PerformanceData, ErrorVariables
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN) :: CntrPar
        TYPE(LocalVariables), INTENT(IN) :: LocalVar
        TYPE(PerformanceData), INTENT(IN) :: PerfData
        TYPE(ErrorVariables), INTENT(INOUT) :: ErrVar
            
        ! Local
        REAL(DbKi) :: RotorArea
        REAL(DbKi) :: Cp
        REAL(DbKi) :: Lambda

        CHARACTER(*), PARAMETER                 :: RoutineName = 'AeroDynTorque'

        ! Find Torque
        RotorArea = PI*CntrPar%WE_BladeRadius**2
        Lambda = LocalVar%RotSpeedF*CntrPar%WE_BladeRadius/LocalVar%WE_Vw

        ! Compute Cp
        Cp = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, LocalVar%PC_PitComT*R2D, Lambda, ErrVar)
        
        AeroDynTorque = 0.5*(CntrPar%WE_RhoAir*RotorArea)*(LocalVar%WE_Vw**3/LocalVar%RotSpeedF)*Cp
        AeroDynTorque = MAX(AeroDynTorque, 0.0)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION AeroDynTorque

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
        IF (LocalVar%iStatus == 0)  THEN  ! .TRUE. if we're on the first call to the DLL
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
            IF (MODULO(LocalVar%Time, 10.0) == 0) THEN
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

END MODULE Functions
