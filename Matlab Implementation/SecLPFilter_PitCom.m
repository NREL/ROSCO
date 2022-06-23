function [SecLPFilter_value] = SecLPFilter_PitCom(InputSignal, DT, CornerFreq, Damp, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%     Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
%     Discrete Time From:     H(z) = (b2_PC*z^2 + b1_PC*z + b0_PC) / (a2_PC*z^2 + a1_PC*z + a0_PC)

persistent OutputSignalLast1_PC OutputSignalLast2_PC InputSignalLast1_PC InputSignalLast2_PC 

    % Initialization
    if ((iStatus == 0) || reset )
        OutputSignalLast1_PC  = InputSignal;
        OutputSignalLast2_PC  = InputSignal;
        InputSignalLast1_PC   = InputSignal;
        InputSignalLast2_PC   = InputSignal;
        
    end
        % Coefficients
        a2_PC = DT^2.0*CornerFreq^2.0 + 4.0 + 4.0*Damp*CornerFreq*DT;
        a1_PC = 2.0*DT^2.0*CornerFreq^2.0 - 8.0;
        a0_PC = DT^2.0*CornerFreq^2.0 + 4.0 - 4.0*Damp*CornerFreq*DT;
        b2_PC = DT^2.0*CornerFreq^2.0;
        b1_PC = 2.0*DT^2.0*CornerFreq^2.0;
        b0_PC = DT^2.0*CornerFreq^2.0;
    

    % Filter
    SecLPFilter_value = 1.0/a2_PC * (b2_PC*InputSignal + b1_PC*InputSignalLast1_PC + b0_PC*InputSignalLast2_PC - a1_PC*OutputSignalLast1_PC - a0_PC*OutputSignalLast2_PC);

    % Save signals for next time step
    InputSignalLast2_PC   = InputSignalLast1_PC;
    InputSignalLast1_PC   = InputSignal;
    OutputSignalLast2_PC  = OutputSignalLast1_PC;
    OutputSignalLast1_PC  = SecLPFilter_value;

end
