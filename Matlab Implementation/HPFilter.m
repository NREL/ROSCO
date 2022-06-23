function [HPFilter_value] = HPFilter( InputSignal, DT, CornerFreq, iStatus, reset)
% Discrete time High-Pass Filter

%     REAL(8), INTENT(IN)     :: InputSignal
%     REAL(8), INTENT(IN)     :: DT                       % time step [s]
%     REAL(8), INTENT(IN)     :: CornerFreq               % corner frequency [rad/s]
%     INTEGER, INTENT(IN)     :: iStatus                  % A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
%     INTEGER, INTENT(INOUT)  :: inst                     % Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
%     LOGICAL(4), INTENT(IN)  :: reset                    % Reset the filter to the input signal
    % Local
%     REAL(8)                         :: K                        % Constant gain
%     REAL(8), DIMENSION(99), SAVE    :: InputSignalLast_HPF      % Input signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast_HPF % Output signal the last time this filter was called. Supports 99 separate instances.

persistent InputSignalLast_HPF OutputSignalLast_HPF
    % Initialization
    if ((iStatus == 0) || reset)
        OutputSignalLast_HPF = InputSignal;
        InputSignalLast_HPF = InputSignal;
    end
    K = 2.0 / DT;

    % Body
    HPFilter_value = K/(CornerFreq + K)*InputSignal - K/(CornerFreq + K)*InputSignalLast_HPF - (CornerFreq - K)/(CornerFreq + K)*OutputSignalLast_HPF;

    % Save signals for next time step
    InputSignalLast_HPF   = InputSignal;
    OutputSignalLast_HPF  = HPFilter_value;
%     inst = inst + 1;

end
