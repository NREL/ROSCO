function [LocalVar] = SetpointSmoother(LocalVar, CntrPar)
% Setpoint smoother modifies controller reference in order to separate generator torque and blade pitch control actions
%       SS_Mode  = 0, No setpoint smoothing
%       SS_Mode  = 1, Implement setpoint smoothing
%       DelOmega = Reference generator speed shift [rad/s].

    % ------ Setpoint Smoothing ------
    if ( CntrPar.SS_Mode == 1)
        % Find setpoint shift amount
        DelOmega = ((LocalVar.PC_PitComT - CntrPar.PC_MinPit)/0.524) * CntrPar.SS_VSGain - ((LocalVar.VS_GenPwr - LocalVar.VS_LastGenTrq))/CntrPar.VS_RtPwr * CntrPar.SS_PCGain; % Normalize to 30 degrees for now
        DelOmega = DelOmega * CntrPar.PC_RefSpd;
        % Filter
        LocalVar.SS_DelOmegaF = LPFilter(DelOmega, LocalVar.DT, CntrPar.F_SSCornerFreq, LocalVar.iStatus, false); 
    else
        LocalVar.SS_DelOmegaF = 0; % No setpoint smoothing
    end

end
