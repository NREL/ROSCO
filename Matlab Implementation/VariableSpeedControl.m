function [LocalVar] = VariableSpeedControl(CntrPar, LocalVar,WindNom,Wind,t)
% Generator torque controller
%       VS_State = 0, Error state, for debugging purposes, GenTq = VS_RtTq
%       VS_State = 1, Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
%       VS_State = 2, Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
%       VS_State = 3, Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
%       VS_State = 4, above-rated operation using pitch control (constant torque mode)
%       VS_State = 5, above-rated operation using pitch and torque control (constant power mode)
%       VS_State = 6, Tip-Speed-Ratio tracking PI controller


    % -------- Variable-Speed Torque Controller --------
    % Define max torque
       LocalVar.VS_MaxTq = CntrPar.VS_RtTq;
       
    % Optimal Tip-Speed-Ratio tracking controller
    if ((CntrPar.VS_ControlMode == 2) || (CntrPar.VS_ControlMode == 3)) 
        % Constant Power, update VS_MaxTq
        if (CntrPar.VS_ControlMode == 3) 
            LocalVar.VS_MaxTq = min((CntrPar.VS_RtPwr/(CntrPar.VS_GenEff/100.0))/LocalVar.GenSpeedF, CntrPar.VS_MaxTq);
        end
        % PI controller
        LocalVar.GenTq = PIController(LocalVar.VS_SpdErr, CntrPar.VS_KP(1), CntrPar.VS_KI(1), CntrPar.VS_MinTq, LocalVar.VS_MaxTq, LocalVar.DT, LocalVar.VS_LastGenTrq, false(1));
        LocalVar.GenTq = saturate(LocalVar.GenTq, CntrPar.VS_MinTq, LocalVar.VS_MaxTq);
        
    else 
        % Update PI loops for region 1.5 and 2.5 PI control
        LocalVar.GenArTq = PIController_ArTq(LocalVar.VS_SpdErrAr, CntrPar.VS_KP(1), CntrPar.VS_KI(1), CntrPar.VS_MaxOMTq, CntrPar.VS_ArSatTq, LocalVar.DT, CntrPar.VS_MaxOMTq, false);
        LocalVar.GenBrTq = PIController_BrTq(LocalVar.VS_SpdErrBr, CntrPar.VS_KP(1), CntrPar.VS_KI(1), CntrPar.VS_MinTq, CntrPar.VS_MinOMTq, LocalVar.DT, CntrPar.VS_MinOMTq, false);
       
        % The action
        if (LocalVar.VS_State == 1)  % Region 1.5
            LocalVar.GenTq = LocalVar.GenBrTq;
        elseif (LocalVar.VS_State == 2)  % Region 2
            LocalVar.GenTq = CntrPar.VS_Rgn2K*LocalVar.GenSpeedF*LocalVar.GenSpeedF;
        elseif (LocalVar.VS_State == 3)  % Region 2.5
            LocalVar.GenTq = LocalVar.GenArTq;
        elseif (LocalVar.VS_State == 4)  % Region 3, constant torque
            LocalVar.GenTq = CntrPar.VS_RtTq;
        elseif (LocalVar.VS_State == 5)  % Region 3, constant power
            LocalVar.GenTq = CntrPar.VS_RtPwr/((CntrPar.VS_GenEff/100.0)*LocalVar.GenSpeedF);
        end
        
        % Saturate
        LocalVar.GenTq = saturate(LocalVar.GenTq, CntrPar.VS_MinTq, CntrPar.VS_MaxTq);
    end


    % Saturate the commanded torque using the maximum torque limit:
    LocalVar.GenTq = min(LocalVar.GenTq, CntrPar.VS_MaxTq);

    % Saturate the commanded torque using the torque rate limit:
    LocalVar.GenTq = ratelimit(LocalVar.GenTq, LocalVar.VS_LastGenTrq, -CntrPar.VS_MaxRat, CntrPar.VS_MaxRat, LocalVar.DT);

    % Reset the value of LocalVar.VS_LastGenTrq to the current values:
    LocalVar.VS_LastGenTrq = LocalVar.GenTq;

end
