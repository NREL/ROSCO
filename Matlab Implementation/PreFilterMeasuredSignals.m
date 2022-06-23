function [LocalVar] = PreFilterMeasuredSignals(CntrPar, LocalVar, LastTime,t,Servo)


% Prefilter measured wind turbine signals to separate the filtering from the actual control actions

    % Filter the HSS (generator) and LSS (rotor) speed measurement:
    if (CntrPar.F_LPFType == 1) 
        LocalVar.GenSpeedF = LPFilter(LocalVar.GenSpeed, LocalVar.DT, CntrPar.F_LPFCornerFreq, LocalVar.iStatus, false);
        LocalVar.RotSpeedF = LPFilter(LocalVar.RotSpeed, LocalVar.DT, CntrPar.F_LPFCornerFreq, LocalVar.iStatus, false);
    elseif (CntrPar.F_LPFType == 2)    % Second-Order LPF
        LocalVar.GenSpeedF = SecLPFilter_Gen(LocalVar.GenSpeed, LocalVar.DT, CntrPar.F_LPFCornerFreq, CntrPar.F_LPFDamping, LocalVar.iStatus, false); % Second-order low-pass filter on generator speed
        LocalVar.RotSpeedF = SecLPFilter_Rot(LocalVar.RotSpeed, LocalVar.DT, CntrPar.F_LPFCornerFreq, CntrPar.F_LPFDamping, LocalVar.iStatus, false); % Second-order low-pass filter on rotor speed
% %         LocalVar.RotSpeedF = LocalVar.GenSpeedF;
% %         if LocalVar.iStatus == 0
% %             LocalVar.GenSpeedF = LocalVar.GenSpeed;
% %         end
% %         Alpha = exp((LastTime-t)*Servo.CornerFreq);
% %         LocalVar.GenSpeedF = (1.0 - Alpha)*LocalVar.GenSpeed + Alpha*LocalVar.GenSpeedF;
% %         LocalVar.RotSpeedF = LocalVar.GenSpeedF; 
    end
    
    % Filter Wind Speed Estimator Signal
    LocalVar.WE_Vw_F = LPFilter_WE(LocalVar.WE_Vw, LocalVar.DT, 0.209, LocalVar.iStatus, false);
    % Control commands (used by WSE, mostly)
    LocalVar.VS_LastGenTrqF = SecLPFilter_GenTrq(LocalVar.VS_LastGenTrq, LocalVar.DT, CntrPar.F_LPFCornerFreq, 0.7, LocalVar.iStatus, false);
    LocalVar.PC_PitComTF    = SecLPFilter_PitCom(LocalVar.PC_PitComT, LocalVar.DT, CntrPar.F_LPFCornerFreq*0.25, 0.7, LocalVar.iStatus, false);
end


%% To use the Baseline Filter for generator and rotor speed (Discarded)
%         if LocalVar.iStatus == 0
%             LocalVar.GenSpeedF = LocalVar.GenSpeed;
%         end
%         Alpha = exp((LastTime-t)*Servo.CornerFreq);
%         LocalVar.GenSpeedF = (1.0 - Alpha)*LocalVar.GenSpeed + Alpha*LocalVar.GenSpeedF;
%         LocalVar.RotSpeedF = LocalVar.GenSpeedF;
