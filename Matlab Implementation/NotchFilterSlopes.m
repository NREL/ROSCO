function [NotchFilterSlopes_value] = NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, iStatus, reset, inst)
% Discrete time inverted Notch Filter with descending slopes, G = CornerFreq*s/(Damp*s^2+CornerFreq*s+Damp*CornerFreq^2)

%     REAL(8), INTENT(IN)     :: InputSignal
%     REAL(8), INTENT(IN)     :: DT                       % time step [s]
%     REAL(8), INTENT(IN)     :: CornerFreq               % corner frequency [rad/s]
%     REAL(8), INTENT(IN)     :: Damp                     % Dampening constant
%     INTEGER, INTENT(IN)     :: iStatus                  % A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
%     INTEGER, INTENT(INOUT)  :: inst                     % Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
%     LOGICAL(4), INTENT(IN)  :: reset                    % Reset the filter to the input signal
    % Local
%     REAL(8), DIMENSION(99), SAVE :: b2, b0, a2, a1, a0    % Input signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE :: InputSignalLast1    % Input signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE :: InputSignalLast2    % Input signal the next to last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE :: OutputSignalLast1   % Output signal the last time this filter was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE :: OutputSignalLast2   % Output signal the next to last time this filter was called. Supports 99 separate instances.

persistent a0 a1 a2 b0 b2 InputSignalLast1 InputSignalLast2 OutputSignalLast1 OutputSignalLast2

    % Initialization
    if ((iStatus == 0) || reset)
        OutputSignalLast1(inst)  = InputSignal;
        OutputSignalLast2(inst)  = InputSignal;
        InputSignalLast1(inst)   = InputSignal;
        InputSignalLast2(inst)   = InputSignal;
        b2(inst) = 2.0 * DT * CornerFreq;
        b0(inst) = -b2(inst);
        a2(inst) = Damp*(DT^2.0)*CornerFreq^2.0 + 2.0*DT*CornerFreq + 4.0*Damp;
        a1(inst) = 2.0*Damp*(DT^2.0)*CornerFreq^2.0 - 8.0*Damp;
        a0(inst) = Damp*(DT^2.0)*CornerFreq^2.0 - 2*DT*CornerFreq + 4.0*Damp;
    end

    NotchFilterSlopes_value = 1.0/a2(inst) * (b2(inst)*InputSignal + b0(inst)*InputSignalLast2(inst)...
                        - a1(inst)*OutputSignalLast1(inst)  - a0(inst)*OutputSignalLast2(inst));
    % Body
%     NotchFilterSlopes = 1.0/(4.0+2.0*DT*Damp*CornerFreq+DT^2.0*CornerFreq^2.0) * ( (8.0-2.0*DT^2.0*CornerFreq^2.0)*OutputSignalLast1(inst)...
%                     + (-4.0+2.0*DT*Damp*CornerFreq-DT^2.0*CornerFreq^2.0)*OutputSignalLast2(inst) +...
%                         (2.0*DT*Damp*CornerFreq)*InputSignal + (-2.0*DT*Damp*CornerFreq)*InputSignalLast2(inst));

    % Save signals for next time step
    InputSignalLast2(inst)   = InputSignalLast1(inst);
    InputSignalLast1(inst)   = InputSignal;          %Save input signal for next time step
    OutputSignalLast2(inst)  = OutputSignalLast1(inst);      %Save input signal for next time step
    OutputSignalLast1(inst)  = NotchFilterSlopes_value;
    inst = inst + 1;

end