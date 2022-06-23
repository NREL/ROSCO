function [FloatingFeedback_out] = FloatingFeedback(LocalVar, CntrPar, objInst) 
% FloatingFeedback defines a minimum blade pitch angle based on a lookup table provided by DISON.IN
%       Fl_Mode = 0, No feedback
%       Fl_Mode = 1, Proportional feedback of nacelle velocity
%     % Inputs
%     TYPE(ControlParameters), INTENT(IN)     :: CntrPar
%     TYPE(LocalVariables), INTENT(IN)     :: LocalVar 
%     TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
%     % Allocate Variables 
%     REAL(8)                      :: NacIMU_FA_vel % Tower fore-aft velocity

    % Calculate floating contribution to pitch command
    NacIMU_FA_vel = PIController(LocalVar.NacIMU_FA_AccF, 0.0, 1.0, -100.0 , 100.0 ,LocalVar.DT, 0.0, false, objInst.instPI); % NJA: should never reach saturation limits....
    FloatingFeedback_out = (0.0 - NacIMU_FA_vel) * CntrPar.Fl_Kp; %* LocalVar%PC_KP/maxval(CntrPar%PC_GS_KP)

end