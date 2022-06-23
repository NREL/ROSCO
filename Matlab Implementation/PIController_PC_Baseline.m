function [PIController_term] = PIController_PC_Baseline(error, kp, ki, minValue, maxValue, DT, I0, reset)
% PI controller, with output saturation

persistent ITerm_PIC_PC  FirstCall_PIC_PC ITermLast_PIC_PC

if isempty(FirstCall_PIC_PC)
    FirstCall_PIC_PC = 1;
end
    % Initialize persistent variables/arrays, and set inital condition for integrator term
    
    if ((FirstCall_PIC_PC == 1) || logical(reset))
        ITerm_PIC_PC = I0;
        ITermLast_PIC_PC = I0;

        FirstCall_PIC_PC = 0;
        PIController_term = I0;
    else
        PTerm = kp*error;
        ITerm_PIC_PC = ITermLast_PIC_PC + DT*ki*error;
        ITerm_PIC_PC = saturate(ITerm_PIC_PC, minValue, maxValue);
        PIController_term = saturate(PTerm + ITerm_PIC_PC, minValue, maxValue);

        ITermLast_PIC_PC = ITerm_PIC_PC;
    end

end