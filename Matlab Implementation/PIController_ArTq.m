function [PIController_term] = PIController_ArTq(error, kp, ki, minValue, maxValue, DT, I0, reset)
% PI controller, with output saturation

persistent ITerm_PIC_Ar  FirstCall_PIC_Ar

if isempty(FirstCall_PIC_Ar)
    FirstCall_PIC_Ar = 1;
end
    % Initialize persistent variables/arrays, and set inital condition for integrator term
    
    if ((FirstCall_PIC_Ar == 1) || logical(reset))
        ITerm_PIC_Ar = I0;
        ITermLast_PIC_Ar = I0;

        FirstCall_PIC_Ar = 0;
        PIController_term = I0;
    else
        PTerm = kp*error;
        ITerm_PIC_Ar = ITerm_PIC_Ar + DT*ki*error;
        ITerm_PIC_Ar = saturate(ITerm_PIC_Ar, minValue, maxValue);
        PIController_term = saturate(PTerm + ITerm_PIC_Ar, minValue, maxValue);

        ITermLast_PIC_Ar = ITerm_PIC_Ar;
    end

end