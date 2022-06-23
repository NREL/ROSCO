function [LPFilter_value] = LPFilter_WE(InputSignal, DT, CornerFreq, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%    Continuous Time Form:   H(s) = CornerFreq/(1 + CornerFreq)
%    Discrete Time Form:     H(z) = (b1z + b0_LPF) / (a1_LPF*z + a0_LPF)
%
%     REAL(8), INTENT(IN)         :: InputSignal
%     REAL(8), INTENT(IN)         :: DT                       % time step [s]
%     REAL(8), INTENT(IN)         :: CornerFreq               % corner frequency [rad/s]
%     INTEGER(4), INTENT(IN)      :: iStatus                  % A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
%     INTEGER(4), INTENT(INOUT)   :: inst                     % Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
%     LOGICAL(4), INTENT(IN)      :: reset                    % Reset the filter to the input signal

        % Local
%     REAL(8), DIMENSION(99), SAVE    :: a1_LPF                   % Denominator coefficient 1
%     REAL(8), DIMENSION(99), SAVE    :: a0_LPF                   % Denominator coefficient 0
%     REAL(8), DIMENSION(99), SAVE    :: b1_LPF                    % Numerator coefficient 1
%     REAL(8), DIMENSION(99), SAVE    :: b0_LPF                    % Numerator coefficient 0 
% 
%     REAL(8), DIMENSION(99), SAVE    :: InputSignalLast_LPF      % Input signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast_LPF % Output signal the last time this filter was called. Supports 99 separate instances.


persistent a0_LPF_WE a1_LPF_WE b0_LPF_WE b1_LPF_WE InputSignalLast_LPF_WE OutputSignalLast_LPF_WE
        % Initialization
    if ((iStatus == 0) || reset)   
        OutputSignalLast_LPF_WE = InputSignal;
        InputSignalLast_LPF_WE = InputSignal;
        a1_LPF_WE = 2 + CornerFreq*DT;
        a0_LPF_WE = CornerFreq*DT - 2;
        b1_LPF_WE = CornerFreq*DT;
        b0_LPF_WE = CornerFreq*DT;
    end

    % Define coefficients

    % Filter
    LPFilter_value = 1.0/a1_LPF_WE * (-a0_LPF_WE*OutputSignalLast_LPF_WE + b1_LPF_WE*InputSignal + b0_LPF_WE*InputSignalLast_LPF_WE);

    % Save signals for next time step
    InputSignalLast_LPF_WE  = InputSignal;
    OutputSignalLast_LPF_WE = LPFilter_value;

end
