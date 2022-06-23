    % Calculate setpoints for primary control actions    
function [LocalVar] = ComputeVariablesSetpoints(CntrPar, LocalVar)
% 
%   % ----------Variables Definitions----------------------
% CntrPar
%   .Y_MErrSet		 - Yaw alignment error, set point [rad]
%   .VS_RefSpd		 - Rated generator speed [rad/s]
%   .PC_RefSpd       - Desired (reference) HSS speed for pitch controller, [rad/s]
%   .VS_RtPwr        - Wind turbine rated power [W]
%   .VS_ControlMode	 - Generator torque control mode in above rated conditions {0: constant torque, 1: constant power, 2: TSR tracking PI control}
%   .VS_TSRopt		 - Power-maximizing region 2 tip-speed-ratio [rad]
%   .WE_BladeRadius	 - Blade length (distance from hub center to blade tip), [m]
%   .WE_GearboxRatio - Gearbox ratio [>=1],  [-]
%   .VS_MinOMSpd	 - Optimal mode minimum speed, cut-in speed towards optimal mode gain path, [rad/s]
%   .PC_Switch		 - Angle above lowest minimum pitch angle for switch, [rad]



    % ----- Calculate yaw misalignment error -----
    LocalVar.Y_MErr = LocalVar.Y_M + CntrPar.Y_MErrSet; % Yaw-alignment error

    % ----- Pitch controller speed and power error -----
    % Implement setpoint smoothing
    if (LocalVar.SS_DelOmegaF < 0)
        PC_RefSpd = CntrPar.PC_RefSpd - LocalVar.SS_DelOmegaF;
    else
        PC_RefSpd = CntrPar.PC_RefSpd;
    end
    
    LocalVar.PC_SpdErr = PC_RefSpd - LocalVar.GenSpeedF;            % Speed error
    LocalVar.PC_PwrErr = CntrPar.VS_RtPwr - LocalVar.VS_GenPwr;      % Power error
    

    % ----- Torque controller reference errors -----
    % Define VS reference generator speed [rad/s]
    if ((CntrPar.VS_ControlMode == 2) || (CntrPar.VS_ControlMode == 3))
        VS_RefSpd = (CntrPar.VS_TSRopt * LocalVar.WE_Vw_F / CntrPar.WE_BladeRadius) * CntrPar.WE_GearboxRatio;
        VS_RefSpd = saturate(VS_RefSpd,CntrPar.VS_MinOMSpd, CntrPar.VS_RefSpd);
    else
        VS_RefSpd = CntrPar.VS_RefSpd;
    end 

    % Implement setpoint smoothing
    if (LocalVar.SS_DelOmegaF > 0)
        VS_RefSpd = VS_RefSpd - LocalVar.SS_DelOmegaF;
    end

    % Force zero torque in shutdown mode
    if (LocalVar.SD)
        VS_RefSpd = CntrPar.VS_MinOMSpd;
    end

    % Force minimum rotor speed
    VS_RefSpd = max(VS_RefSpd, CntrPar.VS_MinOMSpd);

    % TSR-tracking reference error
    if ((CntrPar.VS_ControlMode == 2) || (CntrPar.VS_ControlMode == 3))
        LocalVar.VS_SpdErr = VS_RefSpd - LocalVar.GenSpeedF;
    end

    % Define transition region setpoint errors
    LocalVar.VS_SpdErrAr = VS_RefSpd - LocalVar.GenSpeedF;               % Current speed error - Region 2.5 PI-control (Above Rated)
    LocalVar.VS_SpdErrBr = CntrPar.VS_MinOMSpd - LocalVar.GenSpeedF;     % Current speed error - Region 1.5 PI-control (Below Rated)

    % Region 3 minimum pitch angle for state machine
    LocalVar.VS_Rgn3Pitch = LocalVar.PC_MinPit + CntrPar.PC_Switch;

end  