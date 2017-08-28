!-------------------------------------------------------------------------------------------------------------------------------
! This module contains all the filters
MODULE Filters
!...............................................................................................................................

    IMPLICIT NONE

CONTAINS

    !-------------------------------------------------------------------------------------------------------------------------------
    ! Discrete time Low-Pass Filter
    REAL FUNCTION LPFilter( InputSignal, DT, CornerFreq, iStatus, inst)
    !...............................................................................................................................

        IMPLICIT NONE

			! Inputs

        REAL(4), INTENT(IN)     :: InputSignal
		REAL(4), INTENT(IN)     :: DT						! time step [s]
		REAL(4), INTENT(IN)     :: CornerFreq				! corner frequency [rad/s]
        INTEGER, INTENT(IN)		:: iStatus					! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(IN)		:: inst						! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.

			! Local

        REAL(4), DIMENSION(99), SAVE :: InputSignalLast		! Input signal the last time this filter was called. Supports 99 separate instances.
        REAL(4), DIMENSION(99), SAVE :: OutputSignalLast	! Output signal the last time this filter was called. Supports 99 separate instances.

			! Initialization

        IF ( iStatus == 0 )  THEN
           OutputSignalLast (inst) = InputSignal
           InputSignalLast (inst) = InputSignal
        ENDIF

			! Body

        LPFilter = (DT*CornerFreq*InputSignal + DT*CornerFreq*InputSignalLast(inst) - (DT*CornerFreq-2.0)*OutputSignalLast(inst))/(DT*CornerFreq+2.0)

			! Save signals for next time step

        InputSignalLast (inst)  = InputSignal
        OutputSignalLast (inst) = LPFilter

    END FUNCTION LPFilter
    !-------------------------------------------------------------------------------------------------------------------------------
    ! Discrete time second order Low-Pass Filter
    REAL FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, iStatus, inst)
    !...............................................................................................................................

        IMPLICIT NONE

			! Inputs

        REAL(4), INTENT(IN)     :: InputSignal
		REAL(4), INTENT(IN)     :: DT						! time step [s]
		REAL(4), INTENT(IN)     :: CornerFreq				! corner frequency [rad/s]
		REAL(4), INTENT(IN)     :: Damp						! Dampening constant
        INTEGER, INTENT(IN)		:: iStatus					! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(IN)		:: inst						! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.

			! Local

        REAL(4), DIMENSION(99), SAVE :: InputSignalLast1	! Input signal the last time this filter was called. Supports 99 separate instances.
		REAL(4), DIMENSION(99), SAVE :: InputSignalLast2	! Input signal the next to last time this filter was called. Supports 99 separate instances.
        REAL(4), DIMENSION(99), SAVE :: OutputSignalLast1	! Output signal the last time this filter was called. Supports 99 separate instances.
        REAL(4), DIMENSION(99), SAVE :: OutputSignalLast2	! Output signal the next to last time this filter was called. Supports 99 separate instances.

			! Initialization

        IF ( iStatus == 0 )  THEN
            OutputSignalLast1(inst)  = InputSignal
            OutputSignalLast2(inst)  = InputSignal
            InputSignalLast1(inst)   = InputSignal
            InputSignalLast2(inst)   = InputSignal
        ENDIF

			! Body

        SecLPFilter = 1/(4+4*DT*Damp*CornerFreq+DT**2*CornerFreq**2) * ( (8-2*DT**2*CornerFreq**2)*OutputSignalLast1(inst) &
						+ (-4+4*DT*Damp*CornerFreq-DT**2*CornerFreq**2)*OutputSignalLast2(inst) + (DT**2*CornerFreq**2)*InputSignal &
							+ (2*DT**2*CornerFreq**2)*InputSignalLast1(inst) + (DT**2*CornerFreq**2)*InputSignalLast2(inst) )

			! Save signals for next time step

        InputSignalLast2(inst)   = InputSignalLast1 (inst)
        InputSignalLast1(inst)   = InputSignal
        OutputSignalLast2(inst)  = OutputSignalLast1 (inst)
        OutputSignalLast1(inst)  = SecLPFilter

    END FUNCTION SecLPFilter
    !-------------------------------------------------------------------------------------------------------------------------------
    ! Discrete time High-Pass Filter
    REAL FUNCTION HPFilter( InputSignal, DT, CornerFreq, iStatus, inst)
    !...............................................................................................................................

        IMPLICIT NONE

			! Inputs

        REAL(4), INTENT(IN)     :: InputSignal
		REAL(4), INTENT(IN)     :: DT						! time step [s]
		REAL(4), INTENT(IN)     :: CornerFreq				! corner frequency [rad/s]
        INTEGER, INTENT(IN)		:: iStatus					! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(IN)		:: inst						! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.

			! Local

		REAL(4)                 :: K						! Constant gain
        REAL(4), DIMENSION(99), SAVE :: InputSignalLast		! Input signal the last time this filter was called. Supports 99 separate instances.
        REAL(4), DIMENSION(99), SAVE :: OutputSignalLast	! Output signal the last time this filter was called. Supports 99 separate instances.

			! Initialization

        IF ( iStatus == 0 )  THEN
            OutputSignalLast(inst)    = InputSignal
            InputSignalLast(inst)     = InputSignal
        ENDIF

        K = 2.0 / DT

			! Body

        HPFilter = K/(CornerFreq + K)*InputSignal - K/(CornerFreq + K)*InputSignalLast(inst) - (CornerFreq - K)/(CornerFreq + K)*OutputSignalLast(inst)

			! Save signals for next time step

        InputSignalLast(inst)   = InputSignal
        OutputSignalLast(inst)  = HPFilter

    END FUNCTION HPFilter
    !-------------------------------------------------------------------------------------------------------------------------------
    ! Discrete time inverted Notch Filter
    REAL FUNCTION NotchFilter(InputSignal, DT, K, CornerFreq, Damp, iStatus, inst)
    !...............................................................................................................................

        IMPLICIT NONE

			! Inputs

        REAL(4), INTENT(IN)     :: InputSignal
		REAL(4), INTENT(IN)     :: DT						! time step [s]
		REAL(4), INTENT(IN)     :: CornerFreq				! corner frequency [rad/s]
		REAL(4), INTENT(IN)     :: Damp						! Dampening constant
		REAL(4), INTENT(IN)     :: K						! Constant gain
        INTEGER, INTENT(IN)		:: iStatus					! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER, INTENT(IN)		:: inst						! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.

			! Local

        REAL(4), DIMENSION(99), SAVE :: InputSignalLast1	! Input signal the last time this filter was called. Supports 99 separate instances.
		REAL(4), DIMENSION(99), SAVE :: InputSignalLast2	! Input signal the next to last time this filter was called. Supports 99 separate instances.
        REAL(4), DIMENSION(99), SAVE :: OutputSignalLast1	! Output signal the last time this filter was called. Supports 99 separate instances.
        REAL(4), DIMENSION(99), SAVE :: OutputSignalLast2	! Output signal the next to last time this filter was called. Supports 99 separate instances.

			! Initialization

        IF ( iStatus == 0 )  THEN
            OutputSignalLast1(inst)  = InputSignal
            OutputSignalLast2(inst)  = InputSignal
            InputSignalLast1(inst)   = InputSignal
            InputSignalLast2(inst)   = InputSignal
        ENDIF

			! Body

        NotchFilter = 1.0/(4.0+2.0*DT*Damp*CornerFreq+DT**2.0*CornerFreq**2.0) * ( (8.0-2.0*DT**2.0*CornerFreq**2.0)*OutputSignalLast1(inst) &
						+ (-4.0+2.0*DT*Damp*CornerFreq-DT**2.0*CornerFreq**2.0)*OutputSignalLast2(inst) + &
							(2.0*DT*Damp*CornerFreq*K)*InputSignal + (-2.0*DT*Damp*CornerFreq*K)*InputSignalLast2(inst) )

			! Save signals for next time step

        InputSignalLast2(inst)   = InputSignalLast1(inst)
        InputSignalLast1(inst)   = InputSignal			!Save input signal for next time step
        OutputSignalLast2(inst)  = OutputSignalLast1(inst)		!Save input signal for next time step
        OutputSignalLast1(inst)  = NotchFilter

    END FUNCTION NotchFilter
    !-------------------------------------------------------------------------------------------------------------------------------
    END MODULE Filters
