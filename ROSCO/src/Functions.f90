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
    REAL(DbKi) FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst)
        USE ROSCO_Types, ONLY : piParams

    ! PI controller, with output saturation

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)      :: inst
        REAL(DbKi),    INTENT(IN)         :: I0
        TYPE(piParams), INTENT(INOUT)  :: piP
        LOGICAL,    INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term

        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            
            PIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIController

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst)
    ! PI controller, with output saturation. 
    ! Added error2 term for additional integral control input
        USE ROSCO_Types, ONLY : piParams
        
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
        TYPE(piParams), INTENT(INOUT) :: piP
        LOGICAL, INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term

        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            piP%ITerm2(inst) = I0
            piP%ITermLast2(inst) = I0
            
            PIIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm2(inst) = piP%ITerm2(inst) + DT*ki2*error2
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            piP%ITerm2(inst) = saturate(piP%ITerm2(inst), minValue, maxValue)
            PIIController = PTerm + piP%ITerm(inst) + piP%ITerm2(inst)
            PIIController = saturate(PIIController, minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
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
        REAL(DbKi), INTENT(OUT)    :: PitComIPC(3)                   ! Commanded individual pitch (deg)
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
        CPfunction = saturate(CPfunction, 0.001_DbKi, 1.0_DbKi)
        
    END FUNCTION CPfunction

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION AeroDynTorque(RotSpeed, BldPitch, LocalVar, CntrPar, PerfData, ErrVar)
    ! Function for computing the aerodynamic torque, divided by the effective rotor torque of the turbine, for use in wind speed estimation
        
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, PerformanceData, ErrorVariables
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN) :: CntrPar
        TYPE(LocalVariables), INTENT(IN) :: LocalVar
        TYPE(PerformanceData), INTENT(IN) :: PerfData
        TYPE(ErrorVariables), INTENT(INOUT) :: ErrVar

        REAL(DbKi), INTENT(IN)  :: RotSpeed
        REAL(DbKi), INTENT(IN)  :: BldPitch
            
        ! Local
        REAL(DbKi) :: RotorArea
        REAL(DbKi) :: Cp
        REAL(DbKi) :: Lambda

        CHARACTER(*), PARAMETER                 :: RoutineName = 'AeroDynTorque'

        ! Find Torque
        RotorArea = PI*CntrPar%WE_BladeRadius**2
        Lambda = RotSpeed*CntrPar%WE_BladeRadius/LocalVar%WE_Vw

        ! Compute Cp
        Cp = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, BldPitch*R2D, Lambda, ErrVar)
        
        AeroDynTorque = 0.5*(CntrPar%WE_RhoAir*RotorArea)*(LocalVar%WE_Vw**3/RotSpeed)*Cp
        AeroDynTorque = MAX(AeroDynTorque, 0.0_DbKi)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION AeroDynTorque


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
