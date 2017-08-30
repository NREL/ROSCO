! This module contains basic functions
MODULE FunctionToolbox

    IMPLICIT NONE

CONTAINS
	!-------------------------------------------------------------------------------------------------------------------------------
	! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
	REAL FUNCTION saturate(inputValue,minValue,maxValue)
	!

		IMPLICIT NONE

		REAL(4), INTENT(IN)		:: inputValue
		REAL(4), INTENT(IN)		:: minValue
		REAL(4), INTENT(IN)		:: maxValue

		saturate = MIN(MAX(inputValue,minValue),maxValue)

	END FUNCTION saturate
	!-------------------------------------------------------------------------------------------------------------------------------
	! PI controller, with output saturation
	REAL FUNCTION PI(error,kp,ki,minValue,maxValue,DT,I0,inst)
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
		
			! Local
		INTEGER(4)						:: i					! Counter for making arrays
		REAL(4)                         :: PTerm				! Proportional term
		REAL(4), DIMENSION(99), SAVE    :: ITerm				! Integral term, current.
		REAL(4), DIMENSION(99), SAVE    :: ITermLast			! Integral term, the last time this controller was called. Supports 99 separate instances.
		INTEGER(4), DIMENSION(99), SAVE :: FirstCall = (/ (1, i=1,99) /)		! First call of this function?
		
			! Initialize persistent variables/arrays, and set inital condition for integrator term
		IF ( FirstCall(inst) == 1 ) THEN
			ITerm(1:99) = (/ (real(9999.9), i = 1,99) /)
			ITermLast(1:99) = (/ (real(9999.9), i = 1,99) /)
			
			ITerm(inst) = I0
			ITermLast(inst) = I0
			
			FirstCall(inst) = 0
		END IF
		
		PTerm = kp*error
		ITerm(inst) = ITerm(inst) + DT*ki*error
		ITerm(inst) = saturate(ITerm(inst),maxValue,minValue)
		PI = PTerm + ITerm(inst)
		PI = saturate(PI,maxValue,minValue)

		ITermLast(inst) = ITerm(inst)
	END FUNCTION PI
	!-------------------------------------------------------------------------------------------------------------------------------
END MODULE FunctionToolbox
