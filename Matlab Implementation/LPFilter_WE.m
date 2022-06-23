function [LPFilter_value] = LPFilter_WE(InputSignal, DT, CornerFreq, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%    Continuous Time Form:   H(s) = CornerFreq/(1 + CornerFreq)
%    Discrete Time Form:     H(z) = (b1z + b0_LPF) / (a1_LPF*z + a0_LPF)

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
