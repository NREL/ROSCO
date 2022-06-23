function [SecLPFilter_value] = SecLPFilter(InputSignal, DT, CornerFreq, Damp, iStatus, reset, inst)
% Discrete time Low-Pass Filter of the form:
%     Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
%     Discrete Time From:     H(z) = (b2*z^2 + b1*z + b0) / (a2*z^2 + a1*z + a0)

%     REAL(8), INTENT(IN)         :: InputSignal
%     REAL(8), INTENT(IN)         :: DT                       % time step [s]
%     REAL(8), INTENT(IN)         :: CornerFreq               % corner frequency [rad/s]
%     REAL(8), INTENT(IN)         :: Damp                     % Dampening constant
%     INTEGER(4), INTENT(IN)      :: iStatus                  % A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
%     INTEGER(4), INTENT(INOUT)   :: inst                     % Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
%     LOGICAL(4), INTENT(IN)      :: reset                    % Reset the filter to the input signal

    % Local
%     REAL(8), DIMENSION(99), SAVE    :: a2                   % Denominator coefficient 2
%     REAL(8), DIMENSION(99), SAVE    :: a1                   % Denominator coefficient 1
%     REAL(8), DIMENSION(99), SAVE    :: a0                   % Denominator coefficient 0
%     REAL(8), DIMENSION(99), SAVE    :: b2                   % Numerator coefficient 2
%     REAL(8), DIMENSION(99), SAVE    :: b1                   % Numerator coefficient 1
%     REAL(8), DIMENSION(99), SAVE    :: b0                   % Numerator coefficient 0 
%     REAL(8), DIMENSION(99), SAVE    :: InputSignalLast1     % Input signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: InputSignalLast2     % Input signal the next to last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast1    % Output signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast2    % Output signal the next to last time this filter was called. Supports 99 separate instances.

persistent a0 a1 a2 b0 b1 b2 InputSignalLast1 InputSignalLast2 
persistent OutputSignalLast1 OutputSignalLast2

    % Initialization
    if ((iStatus == 0) || reset )
        OutputSignalLast1  = InputSignal;
        OutputSignalLast2  = InputSignal;
        InputSignalLast1   = InputSignal;
        InputSignalLast2   = InputSignal;

        % Coefficients
        a2 = DT^2.0*CornerFreq^2.0 + 4.0 + 4.0*Damp*CornerFreq*DT;
        a1 = 2.0*DT^2.0*CornerFreq^2.0 - 8.0;
        a0 = DT^2.0*CornerFreq^2.0 + 4.0 - 4.0*Damp*CornerFreq*DT;
        b2 = DT^2.0*CornerFreq^2.0;
        b1 = 2.0*DT^2.0*CornerFreq^2.0;
        b0 = DT^2.0*CornerFreq^2.0;
    end

    % Filter
    SecLPFilter_value = 1.0/a2 * (b2*InputSignal + b1*InputSignalLast1 + b0*InputSignalLast2 - a1*OutputSignalLast1 - a0*OutputSignalLast2);

%     SecLPFilter = 1/(4+4*DT*Damp*CornerFreq+DT^2*CornerFreq^2) * ((8-2*DT^2*CornerFreq^2)*OutputSignalLast1)+...
%                     (-4+4*DT*Damp*CornerFreq-DT^2*CornerFreq^2)*OutputSignalLast2 + (DT^2*CornerFreq^2)*InputSignal...
%                         + (2*DT^2*CornerFreq^2)*InputSignalLast1 + (DT^2*CornerFreq^2)*InputSignalLast2 )

    % Save signals for next time step
    InputSignalLast2   = InputSignalLast1;
    InputSignalLast1   = InputSignal;
    OutputSignalLast2  = OutputSignalLast1;
    OutputSignalLast1  = SecLPFilter_value;

end