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
		REAL(4), INTENT(IN)		:: inst
		REAL(4), INTENT(IN)		:: I0
		
			! Local
		REAL(4)                      :: PTerm			! Proportional term
		REAL(4), DIMENSION(99), SAVE :: ITermLast		! Integral term signal the last time this controller was called. Supports 99 separate instances.
		
		
		PTerm = kp*error
		ITerm(inst) = ITerm(inst) + DT*ki*error
		ITerm(inst) = saturate(ITerm(inst),maxValue,minValue)
		PI = PTerm + ITerm(inst)
		PI = saturate(PI,maxValue,minValue)

	END FUNCTION saturate
	!-------------------------------------------------------------------------------------------------------------------------------
END MODULE FunctionToolbox
