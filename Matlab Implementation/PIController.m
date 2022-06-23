function [PIController_term] = PIController(error, kp, ki, minValue, maxValue, DT, I0, reset)
% PI controller, with output saturation

persistent ITerm_VS  FirstCall_VS ITermLast_VS

if isempty(FirstCall_VS)
    FirstCall_VS = 1;
end
    % Initialize persistent variables/arrays, and set inital condition for integrator term
    
    if ((FirstCall_VS == 1) || logical(reset))
        ITerm_VS = I0;
        ITermLast_VS = I0;

        FirstCall_VS = 0;
        PIController_term = I0;
    else
        PTerm = kp*error;
        ITerm_VS = ITermLast_VS + DT*ki*error;
        ITerm_VS = saturate(ITerm_VS, minValue, maxValue);
        PIController_term = saturate(PTerm + ITerm_VS, minValue, maxValue);

        ITermLast_VS = ITerm_VS;
    end

end