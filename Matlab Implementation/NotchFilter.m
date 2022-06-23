function [NotchFilter_value] = NotchFilter(InputSignal, DT, omega, BetaNum, BetaDen, iStatus, reset)
% Discrete time Notch Filter 
%          Continuous Time Form: G(s) = (s^2 + 2*omega*betaNum*s + omega^2)/(s^2 + 2*omega*betaDen*s + omega^2)
%          Discrete Time Form:   H(z) = (b2*z^2 +b1*z^2 + b0*z)/((z^2 +a1*z^2 + a0*z))

%     REAL(8), INTENT(IN)     :: InputSignal
%     REAL(8), INTENT(IN)     :: DT                       % time step [s]
%     REAL(8), INTENT(IN)     :: omega                    % corner frequency [rad/s]
%     REAL(8), INTENT(IN)     :: betaNum                  % Dampening constant in numerator of filter transfer function
%     REAL(8), INTENT(IN)     :: betaDen                  % Dampening constant in denominator of filter transfer function
%     INTEGER, INTENT(IN)     :: iStatus                  % A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
%     INTEGER, INTENT(INOUT)  :: inst                     % Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
%     LOGICAL(4), INTENT(IN)  :: reset                    % Reset the filter to the input signal
    % Local
%     REAL(8), DIMENSION(99), SAVE    :: K, b2, b1, b0, a1, a0    % Constant gain
%     REAL(8), DIMENSION(99), SAVE    :: InputSignalLast1         % Input signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: InputSignalLast2         % Input signal the next to last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast1        % Output signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast2        % Output signal the next to last time this filter was called. Supports 99 separate instances.


persistent a0 a1 b0 b1 b2 K InputSignalLast1 InputSignalLast2 OutputSignalLast1 OutputSignalLast2

    % Initialization
    if ((iStatus == 0) || reset)
        OutputSignalLast1  = InputSignal;
        OutputSignalLast2  = InputSignal;
        InputSignalLast1   = InputSignal;
        InputSignalLast2   = InputSignal;
        K = 2.0/DT;
        b2 = (K^2.0 + 2.0*omega*BetaNum*K + omega^2.0)/(K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
        b1 = (2.0*omega^2.0 - 2.0*K^2.0)  / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
        b0 = (K^2.0 - 2.0*omega*BetaNum*K + omega^2.0) / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
        a1 = (2.0*omega^2.0 - 2.0*K^2.0)  / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
        a0 = (K^2.0 - 2.0*omega*BetaDen*K + omega^2.0)/ (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
    end

    % Body
    NotchFilter_value = b2*InputSignal + b1*InputSignalLast1 + b0*InputSignalLast2 - a1*OutputSignalLast1 - a0*OutputSignalLast2;

    % Save signals for next time step
    InputSignalLast2   = InputSignalLast1;
    InputSignalLast1   = InputSignal;                  % Save input signal for next time step
    OutputSignalLast2  = OutputSignalLast1;      % Save input signal for next time step
    OutputSignalLast1  = NotchFilter_value;

end
