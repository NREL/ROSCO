! This module contains basic functions
MODULE FunctionToolbox

IMPLICIT NONE

CONTAINS
	!-------------------------------------------------------------------------------------------------------------------------------
	! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
	REAL FUNCTION saturate(inputValue, minValue, maxValue)
	!

		IMPLICIT NONE

		REAL(4), INTENT(IN)		:: inputValue
		REAL(4), INTENT(IN)		:: minValue
		REAL(4), INTENT(IN)		:: maxValue

		saturate = MIN(MAX(inputValue,minValue), maxValue)

	END FUNCTION saturate
	!-------------------------------------------------------------------------------------------------------------------------------
	! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
	REAL FUNCTION ratelimit(refSignal, measSignal, minRate, maxRate, DT)
	!
		IMPLICIT NONE

		REAL(4), INTENT(IN)		:: refSignal
		REAL(4), INTENT(IN)		:: measSignal
		REAL(4), INTENT(IN)		:: minRate
		REAL(4), INTENT(IN)		:: maxRate
		REAL(4), INTENT(IN)		:: DT
		
		! Local variables
		REAL(4)					:: rate

		rate = (refSignal - measSignal)/DT						! Signal rate (unsaturated)
		rate = saturate(rate, minRate, maxRate)					! Saturate the signal rate
		ratelimit = measSignal + rate*DT						! Saturate the overall command using the rate limit

	END FUNCTION ratelimit
	!-------------------------------------------------------------------------------------------------------------------------------
	! PI controller, with output saturation
	REAL FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, reset, inst)
	!
		IMPLICIT NONE

			! Inputs
		REAL(4), INTENT(IN)		:: error
		REAL(4), INTENT(IN)		:: kp
		REAL(4), INTENT(IN)		:: ki
		REAL(4), INTENT(IN)		:: minValue
		REAL(4), INTENT(IN)		:: maxValue
		REAL(4), INTENT(IN)		:: DT
		INTEGER(4), INTENT(IN)	:: inst
		REAL(4), INTENT(IN)		:: I0
		LOGICAL, INTENT(IN)		:: reset
		
			! Local
		INTEGER(4)						:: i											! Counter for making arrays
		REAL(4)							:: PTerm										! Proportional term
		REAL(4), DIMENSION(99), SAVE	:: ITerm = (/ (real(9999.9), i = 1,99) /)		! Integral term, current.
		REAL(4), DIMENSION(99), SAVE	:: ITermLast = (/ (real(9999.9), i = 1,99) /)	! Integral term, the last time this controller was called. Supports 99 separate instances.
		INTEGER(4), DIMENSION(99), SAVE	:: FirstCall = (/ (1, i=1,99) /)				! First call of this function?
		
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
			PIController = PTerm + ITerm(inst)
			PIController = saturate(PIController, minValue, maxValue)
		
			ITermLast(inst) = ITerm(inst)
		END IF
		
	END FUNCTION PIController
	!-------------------------------------------------------------------------------------------------------------------------------
	! interp1 1-D interpolation (table lookup), xData and yData should be monotonically increasing
	REAL FUNCTION interp1d(xData, yData, xq)
	!
		IMPLICIT NONE
			! Inputs
		REAL(4), DIMENSION(:), INTENT(IN)		:: xData		! Provided x data (vector), to be interpolated
		REAL(4), DIMENSION(:), INTENT(IN)		:: yData		! Provided y data (vector), to be interpolated
		REAL(4), INTENT(IN)						:: xq			! x-value for which the y value has to be interpolated
		INTEGER(4)								:: I			! Iteration index
		
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
		
	END FUNCTION interp1d
	!-------------------------------------------------------------------------------------------------------------------------------
	! DF controller, with output saturation
	REAL FUNCTION DFController(error, Kd, Tf, DT, inst)
	!
		IMPLICIT NONE

			! Inputs
		REAL(4), INTENT(IN)		:: error
		REAL(4), INTENT(IN)		:: kd
		REAL(4), INTENT(IN)		:: tf
		REAL(4), INTENT(IN)		:: DT
		INTEGER(4), INTENT(IN)	:: inst
		
			! Local
		REAL(4)							:: B									! 
		INTEGER(4)						:: i									! Counter for making arrays
		REAL(4), DIMENSION(99), SAVE	:: errorLast = (/ (0, i=1,99) /)		! 
		REAL(4), DIMENSION(99), SAVE	:: DFControllerLast = (/ (0, i=1,99) /)	! 
		INTEGER(4), DIMENSION(99), SAVE	:: FirstCall = (/ (1, i=1,99) /)		! First call of this function?
		
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
	! PRBS identification signal generator function
	!REAL FUNCTION PRBSgen(mean, amplitude, cycleTime, seed, initValue, reset, inst)
	!!
	!	IMPLICIT NONE
    !
	!		! Inputs
	!	REAL(4), INTENT(IN)		:: mean
	!	REAL(4), INTENT(IN)		:: amplitude
	!	INTEGER(4), INTENT(IN)	:: cycleTime
	!	INTEGER(4), INTENT(IN)	:: seed
	!	LOGICAL, INTENT(IN)		:: reset
	!	REAL(4), INTENT(IN)		:: initValue
	!	
	!		! Local
	!	INTEGER(4)				:: i											! Counter for making arrays
	!	REAL(4)					:: randomNumber
	!	INTEGER(4), DIMENSION(99), SAVE	:: FirstCall = (/ (1, i=1,99) /)
	!	
	!	IF ((FirstCall(inst) == 1) .OR. reset) THEN
	!		RANDOM_NUMBER(1)
	!		RAND(seed)
	!		
	!		FirstCall(inst) = 0
	!		PRBSgen = initValue
	!	ELSE
	!		randomNumber = RAND()
	!		
	!		IF randomNumber > 0.5 THEN
	!			randomNumber = 1
	!		ELSE
	!			randomNumber = 0
	!		END IF
	!		
	!		randomNumber = randomNumber - 0.5
	!		randomNumber = randomNumber*amplitude*2 + mean
	!		PRBSgen = randomNumber
	!	END IF
	!	
	!END FUNCTION PRBSgen
	!-------------------------------------------------------------------------------------------------------------------------------
END MODULE FunctionToolbox
