function [PIIController_term,inst] = PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, reset, inst)
% PI controller, with output saturation. 
% Added error2 term for additional integral control input

%     IMPLICIT NONE
%     % Allocate Inputs
%     REAL(8), INTENT(IN)         :: error
%     REAL(8), INTENT(IN)         :: error2
%     REAL(8), INTENT(IN)         :: kp
%     REAL(8), INTENT(IN)         :: ki2
%     REAL(8), INTENT(IN)         :: ki
%     REAL(8), INTENT(IN)         :: minValue
%     REAL(8), INTENT(IN)         :: maxValue
%     REAL(8), INTENT(IN)         :: DT
%     INTEGER(4), INTENT(INOUT)   :: inst
%     REAL(8), INTENT(IN)         :: I0
%     LOGICAL, INTENT(IN)         :: reset     
%     % Allocate local variables
%     INTEGER(4)                      :: i                                            % Counter for making arrays
%     REAL(8)                         :: PTerm                                        % Proportional term
%     REAL(8), DIMENSION(99), SAVE    :: ITerm = (/ (real(9999.9), i = 1,99) /)       % Integral term, current.
%     REAL(8), DIMENSION(99), SAVE    :: ITermLast = (/ (real(9999.9), i = 1,99) /)   % Integral term, the last time this controller was called. Supports 99 separate instances.
%     REAL(8), DIMENSION(99), SAVE    :: ITerm2 = (/ (real(9999.9), i = 1,99) /)       % Second Integral term, current.
%     REAL(8), DIMENSION(99), SAVE    :: ITermLast2 = (/ (real(9999.9), i = 1,99) /)   % Second Integral term, the last time this controller was called. Supports 99 separate instances.
%     INTEGER(4), DIMENSION(99), SAVE :: FirstCall = (/ (1, i=1,99) /)                % First call of this function?

% persistent ITerm ITerm2 ItermLast ITermLast2 FirstCall
% ITerm = 9999.9.*ones(1,1000);
% ITerm2 = 9999.9.*ones(1,1000);
% ITermLast = 9999.9.*ones(1,1000);
% ITermLast2 = 9999.9.*ones(1,1000);
% FirstCall = ones(1,1000);
    % Initialize persistent variables/arrays, and set inital condition for integrator term
    if ((FirstCall(inst) == 1) || reset)
        ITerm(inst) = I0;
        ITermLast(inst) = I0;
        ITerm2(inst) = I0;
        ITermLast2(inst) = I0;

        FirstCall(inst) = 0;
        PIIController_term = I0;
    else
        PTerm = kp*error;
        ITerm(inst) = ITerm(inst) + DT*ki*error;
        ITerm2(inst) = ITerm2(inst) + DT*ki2*error2;
        ITerm(inst) = saturate(ITerm(inst), minValue, maxValue);
        ITerm2(inst) = saturate(ITerm2(inst), minValue, maxValue);
        PIIController_term = PTerm + ITerm(inst) + ITerm2(inst);
        PIIController_term = saturate(PIIController_term, minValue, maxValue);

        ITermLast(inst) = ITerm(inst);
    end 
    inst = inst + 1;

end
