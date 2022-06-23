function [LocalVar] = PitchControl(CntrPar, LocalVar)
% Blade pitch controller, generally maximizes rotor speed below rated (region 2) and regulates rotor speed above rated (region 3)
%       PC_State = 0, fix blade pitch to fine pitch angle (PC_FinePit)
%       PC_State = 1, is gain scheduled PI controller 
% Additional loops/methods (enabled via switches in DISCON.IN):
%       Individual pitch control
%       Tower fore-aft damping 
%       Sine excitation on pitch    


persistent PitComT_Last

if isempty(PitComT_Last)
    PitComT_Last = 0;
end

    % ------- Blade Pitch Controller --------
    % Load PC State
    if (LocalVar.PC_State == 1)  % PI BldPitch control
        LocalVar.PC_MaxPit = CntrPar.PC_MaxPit;
    else % debug mode, fix at fine pitch
        LocalVar.PC_MaxPit = CntrPar.PC_FinePit;
    end

    % Compute (interpolate) the gains based on previously commanded blade pitch angles and lookup table:
%     LocalVar.PC_KP = max(CntrPar.PC_GS_KP(1),min(CntrPar.PC_GS_KP(end),interp1(CntrPar.PC_GS_angles, CntrPar.PC_GS_KP, LocalVar.PC_PitComTF))); % Proportional gain
%     LocalVar.PC_KI = max(CntrPar.PC_GS_KI(1),min(CntrPar.PC_GS_KI(end),interp1(CntrPar.PC_GS_angles, CntrPar.PC_GS_KI, LocalVar.PC_PitComTF))); % Integral gain
    
    %Manual interpolation using Fortran source code
    LocalVar.PC_KP = interp1d(CntrPar.PC_GS_angles, CntrPar.PC_GS_KP, LocalVar.PC_PitComTF); % Proportional gain
    LocalVar.PC_KI = interp1d(CntrPar.PC_GS_angles, CntrPar.PC_GS_KI, LocalVar.PC_PitComTF); % Integral gain
    
   

    % Compute the collective pitch command associated with the proportional and integral gains:
    if (LocalVar.iStatus == 0) 
        LocalVar.PC_PitComT = PIController_PC_Baseline(LocalVar.PC_SpdErr, LocalVar.PC_KP, LocalVar.PC_KI, CntrPar.PC_FinePit, LocalVar.PC_MaxPit, LocalVar.DT, LocalVar.PitCom(1), true(1));
    else
        LocalVar.PC_PitComT = PIController_PC_Baseline(LocalVar.PC_SpdErr, LocalVar.PC_KP, LocalVar.PC_KI, LocalVar.PC_MinPit, LocalVar.PC_MaxPit, LocalVar.DT, LocalVar.BlPitch(1),false(1));
    end

    % Find individual pitch control contribution
    if ((CntrPar.IPC_ControlMode >= 1) || (CntrPar.Y_ControlMode == 2)) 
        LocalVar = IPC(CntrPar, LocalVar, 1);
    else
        LocalVar.IPC_PitComF = [0.0,0.0,0.0]; % THIS IS AN ARRAY!%
    end

    % Include tower fore-aft tower vibration damping control
    if ((CntrPar.FA_KI > 0.0) || (CntrPar.Y_ControlMode == 2)) 
        LocalVar = ForeAftDamping(CntrPar, LocalVar, 1);
    else
        LocalVar.FA_PitCom = [0.0, 0.0, 0.0]; % THIS IS AN ARRAY!%
    end

    % Pitch Saturation
    if (CntrPar.PS_Mode == 1) 
        LocalVar.PC_MinPit = min(CntrPar.PS_BldPitchMin(1),max(0,interp1d(CntrPar.PS_WindSpeeds, CntrPar.PS_BldPitchMin, LocalVar.WE_Vw_F)));
        LocalVar.PC_MinPit = max(LocalVar.PC_MinPit, CntrPar.PC_FinePit);
    else
        LocalVar.PC_MinPit = CntrPar.PC_FinePit;
    end
    
    if LocalVar.PC_MinPit > CntrPar.PS_BldPitchMin(1)
        fprintf('Check')
    end

    % Shutdown
    if (CntrPar.SD_Mode == 1) 
        LocalVar.PC_PitComT = Shutdown(LocalVar, CntrPar, 1);
    end

    % FloatingFeedback
    if (CntrPar.Fl_Mode == 1) 
        LocalVar.Fl_PitCom = FloatingFeedback(LocalVar, CntrPar, 1);
        LocalVar.PC_PitComT = LocalVar.PC_PitComT + LocalVar.Fl_PitCom;
    end

    % Saturate collective pitch commands:
    LocalVar.PC_PitComT = saturate(LocalVar.PC_PitComT, LocalVar.PC_MinPit, CntrPar.PC_MaxPit);                    % Saturate the overall command using the pitch angle limits
    LocalVar.PC_PitComT = ratelimit(LocalVar.PC_PitComT, PitComT_Last, CntrPar.PC_MinRat, CntrPar.PC_MaxRat, LocalVar.DT); % Saturate the overall command of blade K using the pitch rate limit
    PitComT_Last = LocalVar.PC_PitComT;

    % Combine and saturate all individual pitch commands:
    % Filter to emulate pitch actuator
    for K = 1:LocalVar.NumBl % Loop through all blades, add IPC contribution and limit pitch rate
        LocalVar.PitCom(K) = LocalVar.PC_PitComT + LocalVar.IPC_PitComF(K) + LocalVar.FA_PitCom(K); 
        LocalVar.PitCom(K) = saturate(LocalVar.PitCom(K), LocalVar.PC_MinPit, CntrPar.PC_MaxPit);                    % Saturate the overall command using the pitch angle limits
        LocalVar.PitCom(K) = ratelimit(LocalVar.PitCom(K), LocalVar.BlPitch(K), CntrPar.PC_MinRat, CntrPar.PC_MaxRat, LocalVar.DT); % Saturate the overall command of blade K using the pitch rate limit
    end
end
