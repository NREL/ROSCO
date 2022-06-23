function [SecLPFilter_value] = SecLPFilter_GenTrq(InputSignal, DT, CornerFreq, Damp, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%     Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
%     Discrete Time From:     H(z) = (b2_GT*z^2 + b1_GT*z + b0_GT) / (a2_GT*z^2 + a1_GT*z + a0_GT)

persistent a0_GT a1_GT a2_GT b0_GT b1_GT b2_GT InputSignalLast1_GT InputSignalLast2_GT 
persistent OutputSignalLast1_GT OutputSignalLast2_GT

    % Initialization
    if ((iStatus == 0) || reset )
        OutputSignalLast1_GT  = InputSignal;
        OutputSignalLast2_GT  = InputSignal;
        InputSignalLast1_GT   = InputSignal;
        InputSignalLast2_GT   = InputSignal;

        % Coefficients
        a2_GT = DT^2.0*CornerFreq^2.0 + 4.0 + 4.0*Damp*CornerFreq*DT;
        a1_GT = 2.0*DT^2.0*CornerFreq^2.0 - 8.0;
        a0_GT = DT^2.0*CornerFreq^2.0 + 4.0 - 4.0*Damp*CornerFreq*DT;
        b2_GT = DT^2.0*CornerFreq^2.0;
        b1_GT = 2.0*DT^2.0*CornerFreq^2.0;
        b0_GT = DT^2.0*CornerFreq^2.0;
    end

    % Filter
    SecLPFilter_value = 1.0/a2_GT * (b2_GT*InputSignal + b1_GT*InputSignalLast1_GT + b0_GT*InputSignalLast2_GT - a1_GT*OutputSignalLast1_GT - a0_GT*OutputSignalLast2_GT);

%     SecLPFilter = 1/(4+4*DT*Damp*CornerFreq+DT^2*CornerFreq^2) * ((8-2*DT^2*CornerFreq^2)*OutputSignalLast1_GT)+...
%                     (-4+4*DT*Damp*CornerFreq-DT^2*CornerFreq^2)*OutputSignalLast2_GT + (DT^2*CornerFreq^2)*InputSignal...
%                         + (2*DT^2*CornerFreq^2)*InputSignalLast1_GT + (DT^2*CornerFreq^2)*InputSignalLast2_GT )

    % Save signals for next time step
    InputSignalLast2_GT   = InputSignalLast1_GT;
    InputSignalLast1_GT   = InputSignal;
    OutputSignalLast2_GT  = OutputSignalLast1_GT;
    OutputSignalLast1_GT  = SecLPFilter_value;

end