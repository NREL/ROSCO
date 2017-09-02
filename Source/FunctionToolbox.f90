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
	REAL FUNCTION PI(error, kp, ki, minValue, maxValue, DT, I0, inst)
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
		INTEGER(4)						:: i									! Counter for making arrays
		REAL(4)							:: PTerm								! Proportional term
		REAL(4), DIMENSION(99), SAVE	:: ITerm								! Integral term, current.
		REAL(4), DIMENSION(99), SAVE	:: ITermLast							! Integral term, the last time this controller was called. Supports 99 separate instances.
		INTEGER(4), DIMENSION(99), SAVE	:: FirstCall = (/ (1, i=1,99) /)		! First call of this function?
		
			! Debugging
		! INTEGER(4), PARAMETER			:: UnDb = 90									! I/O unit for the debugging information
		! CHARACTER(LEN=6), PARAMETER	:: RootName = "PI_DBG"							! 
		! CHARACTER(1), PARAMETER		:: Tab = CHAR(9)								! The tab character.
		! CHARACTER(25), PARAMETER		:: FmtDat = "(F8.3,99('"//Tab//"',ES10.3E2,:))"	! The format of the debugging data
		
			! Initialize persistent variables/arrays, and set inital condition for integrator term
		IF ( FirstCall(inst) == 1 ) THEN
			ITerm(1:99) = (/ (real(9999.9), i = 1,99) /)
			ITermLast(1:99) = (/ (real(9999.9), i = 1,99) /)
			
			ITerm(inst) = I0
			ITermLast(inst) = I0
			
			! OPEN ( UnDb, FILE=TRIM( RootName )//'.dbg', STATUS='REPLACE' )
			! WRITE (UnDb,'(/////)')
			! WRITE (UnDb,'(A)')  'FirstCall(inst)'  //Tab//'ITerm(inst)' //Tab//'error ' //Tab//'kp  ' //Tab//'ki ' //Tab//'DT'
			
			FirstCall(inst) = 0
		END IF
		
		PTerm = kp*error
		ITerm(inst) = ITerm(inst) + DT*ki*error
		ITerm(inst) = saturate(ITerm(inst), minValue, maxValue)
		PI = PTerm + ITerm(inst)
		PI = saturate(PI, minValue, maxValue)

		ITermLast(inst) = ITerm(inst)
		
		! WRITE (UnDb,FmtDat)  real(FirstCall(inst)),	ITerm(inst),	error,	kp,	ki,	DT
	END FUNCTION PI
	!-------------------------------------------------------------------------------------------------------------------------------
	
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
			interp1d = MINVAL(yData)
		ELSEIF (xq >= MAXVAL(xData)) THEN
			interp1d = MAXVAL(yData)
		ELSE
			DO I = 1, SIZE(xData)
				IF (xData(I) >= xq) THEN
					interp1d = yData(I-1) + (yData(I) - yData(I-1))/(xData(I) - xData(I-1))*(xq - xData(I-1))
					EXIT
				ELSE
					CONTINUE
				END IF
			END DO
		END IF
		
	END FUNCTION interp1d
	
END MODULE FunctionToolbox
