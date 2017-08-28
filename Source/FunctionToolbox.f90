! This module contains basic functions
MODULE FunctionToolbox

    IMPLICIT NONE

CONTAINS
	!-------------------------------------------------------------------------------------------------------------------------------
	! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
	REAL FUNCTION saturate(inputValue,minValue,maxValue)
	!...............................................................................................................................

		IMPLICIT NONE

		REAL(4), INTENT(IN)		:: inputValue
		REAL(4), INTENT(IN)		:: minValue
		REAL(4), INTENT(IN)		:: maxValue

		saturate = MIN(MAX(inputValue,minValue),maxValue)

	END FUNCTION saturate
	!-------------------------------------------------------------------------------------------------------------------------------
END MODULE FunctionToolbox
