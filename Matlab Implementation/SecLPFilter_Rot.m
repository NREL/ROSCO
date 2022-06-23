function [SecLPFilter_value] = SecLPFilter_Rot(InputSignal, DT, CornerFreq, Damp, iStatus, reset)
% Discrete time Low-Pass Filter of the form:
%     Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
%     Discrete Time From:     H(z) = (b2_rot*z^2 + b1_rot*z + b0_rot) / (a2_rot*z^2 + a1_rot*z + a0_rot)


persistent a0_rot a1_rot a2_rot b0_rot b1_rot b2_rot InputSignalLast1_rot InputSignalLast2_rot 
persistent OutputSignalLast1_rot OutputSignalLast2_rot

% InputSignal = InputSignal*pi/30;
    % Initialization
    if ((iStatus == 0) || reset )
        OutputSignalLast1_rot  = InputSignal;
        OutputSignalLast2_rot  = InputSignal;
        InputSignalLast1_rot   = InputSignal;
        InputSignalLast2_rot   = InputSignal;

        % Coefficients
        a2_rot = DT^2.0*CornerFreq^2.0 + 4.0 + 4.0*Damp*CornerFreq*DT;
        a1_rot = 2.0*DT^2.0*CornerFreq^2.0 - 8.0;
        a0_rot = DT^2.0*CornerFreq^2.0 + 4.0 - 4.0*Damp*CornerFreq*DT;
        b2_rot = DT^2.0*CornerFreq^2.0;
        b1_rot = 2.0*DT^2.0*CornerFreq^2.0;
        b0_rot = DT^2.0*CornerFreq^2.0;
    end

    % Filter
    SecLPFilter_value = 1.0/a2_rot * (b2_rot*InputSignal + b1_rot*InputSignalLast1_rot + b0_rot*InputSignalLast2_rot - a1_rot*OutputSignalLast1_rot - a0_rot*OutputSignalLast2_rot);
    
%     SecLPFilter_value = SecLPFilter_value*30/pi;

%     SecLPFilter = 1/(4+4*DT*Damp*CornerFreq+DT^2*CornerFreq^2) * ((8-2*DT^2*CornerFreq^2)*OutputSignalLast1_rot)+...
%                     (-4+4*DT*Damp*CornerFreq-DT^2*CornerFreq^2)*OutputSignalLast2_rot + (DT^2*CornerFreq^2)*InputSignal...
%                         + (2*DT^2*CornerFreq^2)*InputSignalLast1_rot + (DT^2*CornerFreq^2)*InputSignalLast2_rot )

    % Save signals for next time step
    InputSignalLast2_rot   = InputSignalLast1_rot;
    InputSignalLast1_rot   = InputSignal;
    OutputSignalLast2_rot  = OutputSignalLast1_rot;
    OutputSignalLast1_rot  = SecLPFilter_value;

end