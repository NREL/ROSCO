function [SecLPFilter_value] = SecLPFilter_Gen(InputSignal, DT, CornerFreq, Damp, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%     Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
%     Discrete Time From:     H(z) = (b2_gen*z^2 + b1_gen*z + b0_gen) / (a2_gen*z^2 + a1_gen*z + a0_gen)

persistent a0_gen a1_gen a2_gen b0_gen b1_gen b2_gen InputSignalLast1_gen InputSignalLast2_gen 
persistent OutputSignalLast1_gen OutputSignalLast2_gen

% InputSignal = InputSignal*pi/30;

    % Initialization
    if ((iStatus == 0) || reset )
        OutputSignalLast1_gen  = InputSignal;
        OutputSignalLast2_gen  = InputSignal;
        InputSignalLast1_gen   = InputSignal;
        InputSignalLast2_gen   = InputSignal;

        % Coefficients
        a2_gen = DT^2.0*CornerFreq^2.0 + 4.0 + 4.0*Damp*CornerFreq*DT;
        a1_gen = 2.0*DT^2.0*CornerFreq^2.0 - 8.0;
        a0_gen = DT^2.0*CornerFreq^2.0 + 4.0 - 4.0*Damp*CornerFreq*DT;
        b2_gen = DT^2.0*CornerFreq^2.0;
        b1_gen = 2.0*DT^2.0*CornerFreq^2.0;
        b0_gen = DT^2.0*CornerFreq^2.0;
    end

    % Filter
    SecLPFilter_value = 1.0/a2_gen * (b2_gen*InputSignal + b1_gen*InputSignalLast1_gen + b0_gen*InputSignalLast2_gen - a1_gen*OutputSignalLast1_gen - a0_gen*OutputSignalLast2_gen);
    
%     SecLPFilter_value = SecLPFilter_value*30/pi;

%     SecLPFilter = 1/(4+4*DT*Damp*CornerFreq+DT^2*CornerFreq^2) * ((8-2*DT^2*CornerFreq^2)*OutputSignalLast1_gen)+...
%                     (-4+4*DT*Damp*CornerFreq-DT^2*CornerFreq^2)*OutputSignalLast2_gen + (DT^2*CornerFreq^2)*InputSignal...
%                         + (2*DT^2*CornerFreq^2)*InputSignalLast1_gen + (DT^2*CornerFreq^2)*InputSignalLast2_gen )

    % Save signals for next time step
    InputSignalLast2_gen   = InputSignalLast1_gen;
    InputSignalLast1_gen   = InputSignal;
    OutputSignalLast2_gen  = OutputSignalLast1_gen;
    OutputSignalLast1_gen  = SecLPFilter_value;

end