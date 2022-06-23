function [SecLPFilter_value] = SecLPFilter_qdGeAz(InputSignal, DT, CornerFreq, Damp, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%     Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
%     Discrete Time From:     H(z) = (b2_gen*z^2 + b1_gen*z + b0_gen) / (a2_gen*z^2 + a1_gen*z + a0_gen)

persistent a0_temp a1_temp a2_temp b0_temp b1_temp b2_temp InputSignalLast1_temp InputSignalLast2_temp 
persistent OutputSignalLast1_temp OutputSignalLast2_temp

% InputSignal = InputSignal*pi/30;

    % Initialization
    if ((iStatus == 0) || reset )
        OutputSignalLast1_temp  = InputSignal;
        OutputSignalLast2_temp  = InputSignal;
        InputSignalLast1_temp   = InputSignal;
        InputSignalLast2_temp   = InputSignal;

        % Coefficients
        a2_temp = DT^2.0*CornerFreq^2.0 + 4.0 + 4.0*Damp*CornerFreq*DT;
        a1_temp = 2.0*DT^2.0*CornerFreq^2.0 - 8.0;
        a0_temp = DT^2.0*CornerFreq^2.0 + 4.0 - 4.0*Damp*CornerFreq*DT;
        b2_temp = DT^2.0*CornerFreq^2.0;
        b1_temp = 2.0*DT^2.0*CornerFreq^2.0;
        b0_temp = DT^2.0*CornerFreq^2.0;
    end

    % Filter
    SecLPFilter_value = 1.0/a2_temp * (b2_temp*InputSignal + b1_temp*InputSignalLast1_temp + b0_temp*InputSignalLast2_temp - a1_temp*OutputSignalLast1_temp - a0_temp*OutputSignalLast2_temp);
    
    % Save signals for next time step
    InputSignalLast2_temp   = InputSignalLast1_temp;
    InputSignalLast1_temp   = InputSignal;
    OutputSignalLast2_temp  = OutputSignalLast1_temp;
    OutputSignalLast1_temp  = SecLPFilter_value;

end