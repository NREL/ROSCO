function [LocalVar] = StateMachine(CntrPar, LocalVar)
% State machine, determines the state of the wind turbine to specify the corresponding control actions
% PC States:
%       PC_State = 0, No pitch control active, BldPitch = PC_MinPit
%       PC_State = 1, Active PI blade pitch control enabled
% VS States
%       VS_State = 0, Error state, for debugging purposes, GenTq = VS_RtTq
%       VS_State = 1, Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
%       VS_State = 2, Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
%       VS_State = 3, Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
%       VS_State = 4, above-rated operation using pitch control (constant torque mode)
%       VS_State = 5, above-rated operation using pitch and torque control (constant power mode)
%       VS_State = 6, Tip-Speed-Ratio tracking PI controller



    % Initialize State machine if first call
    if (LocalVar.iStatus == 0) 
        if (LocalVar.PitCom(1) >= LocalVar.VS_Rgn3Pitch)  % We are in region 3
            if (CntrPar.VS_ControlMode == 1)  % Constant power tracking
                LocalVar.VS_State = 5;
                LocalVar.PC_State = 1;
            else % Constant torque tracking
                LocalVar.VS_State = 4;
                LocalVar.PC_State = 1;
            end
        else % We are in Region 2
            LocalVar.VS_State = 2;
            LocalVar.PC_State = 0;
        end

    % Operational States
    else
        % --- Pitch controller state machine ---
        if (CntrPar.PC_ControlMode == 1) 
            LocalVar.PC_State = 1;
        else 
            LocalVar.PC_State = 0;
        end

        % --- Torque control state machine ---
        if (LocalVar.PC_PitComT >= LocalVar.VS_Rgn3Pitch)        

            if (CntrPar.VS_ControlMode == 1)                    % Region 3
                LocalVar.VS_State = 5; % Constant power tracking
            else 
                LocalVar.VS_State = 4 ;% Constant torque tracking
            end
        else
            if CntrPar.VS_ControlMode ~= 2 %SBT - Not given in ROSCO, added to avoid error as LocalVar.VS_State is not used for VSMode = 2
                if (LocalVar.GenArTq >= CntrPar.VS_MaxOMTq*1.01)        % Region 2 1/2 - active PI torque control
                        LocalVar.VS_State = 3;                 
                elseif ((LocalVar.GenSpeedF < CntrPar.VS_RefSpd) &&...
                            (LocalVar.GenBrTq >= CntrPar.VS_MinOMTq))        % Region 2 - optimal torque is proportional to the square of the generator speed
                        LocalVar.VS_State = 2;
                elseif (LocalVar.GenBrTq < CntrPar.VS_MinOMTq)    % Region 1 1/2
                        LocalVar.VS_State = 1;
                else                                                        % Error state, Debug
                        LocalVar.VS_State = 0;
                end
            end
        end
    end
end