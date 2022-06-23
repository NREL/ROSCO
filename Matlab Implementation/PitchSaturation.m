function [Pitch_Saturation] = PitchSaturation(LocalVar, CntrPar) 
% PitchSaturation defines a minimum blade pitch angle based on a lookup table provided by DISCON.IN
%       SS_Mode = 0, No setpoint smoothing
%       SS_Mode = 1, Implement pitch saturation
    % Inputs
%     TYPE(ControlParameters), INTENT(IN)     :: CntrPar
%     TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
%     TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
%     TYPE(DebugVariables), INTENT(INOUT)     :: DebugVar

    % Define minimum blade pitch angle as a function of estimated wind speed
    Pitch_Saturation = interp1d(CntrPar.PS_WindSpeeds, CntrPar.PS_BldPitchMin, LocalVar.WE_Vw_F);

end