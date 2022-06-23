function [PIController_term] = PIController_BrTq(error, kp, ki, minValue, maxValue, DT, I0, reset)
% PI controller, with output saturation


persistent ITerm_PIC_Br  FirstCall_PIC_Br

if isempty(FirstCall_PIC_Br)
    FirstCall_PIC_Br = 1;
end
    % Initialize persistent variables/arrays, and set inital condition for integrator term
    
    if ((FirstCall_PIC_Br == 1) || logical(reset))
        ITerm_PIC_Br = I0;
%         ITermLast_PIC_Br = I0;

        FirstCall_PIC_Br = 0;
        PIController_term = I0;
    else
        PTerm = kp*error;
        ITerm_PIC_Br = ITerm_PIC_Br + DT*ki*error;
        ITerm_PIC_Br = saturate(ITerm_PIC_Br, minValue, maxValue);
        PIController_term = saturate(PTerm + ITerm_PIC_Br, minValue, maxValue);

%         ITermLast_PIC_Br = ITerm_PIC_Br;
    end
end