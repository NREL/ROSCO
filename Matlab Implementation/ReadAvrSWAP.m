% Read avrSWAP array passed from ServoDyn    
function [LocalVar] = ReadAvrSWAP(avrSWAP)
% USE ROSCO_Types, ONLY : LocalVariables
% 
% REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)   % The swap array, used to pass data to, and receive data from, the DLL controller.
% TYPE(LocalVariables), INTENT(INOUT) :: LocalVar

% Load variables from calling program (See Appendix A of Bladed User's Guide):
LocalVar.iStatus = avrSWAP(1);    % The status flag {0:First call at time zero, 1:All subsequent calls, -1:Last call}
LocalVar.Time = avrSWAP(2);             % Current time
LocalVar.DT = avrSWAP(3);               % Communication interval
LocalVar.VS_MechGenPwr = avrSWAP(14);   % Measured shaft power (variable speed case only) [W]
LocalVar.VS_GenPwr = avrSWAP(15);       % Measured electrical power output [W]
LocalVar.GenSpeed = avrSWAP(20);        % Measured generator speed [rad/s]
LocalVar.RotSpeed = avrSWAP(21);        % Measured rotor speed [rad/s]
LocalVar.GenTqMeas = avrSWAP(23);       % Measured generator torque (variable speed case only) [Nm]
LocalVar.Y_M = avrSWAP(24);             % Measured yaw error (based on free wind at hub position) [rad]
LocalVar.HorWindV = avrSWAP(27);        % Hub wind speed [m/s]
LocalVar.rootMOOP(1) = avrSWAP(30);     % Blade-1 root out of plane BM [Nm]
LocalVar.rootMOOP(2) = avrSWAP(31);     % Blade-2 root out of plane BM [Nm]
LocalVar.rootMOOP(3) = avrSWAP(32);     % Blade-3 root out of plane BM [Nm]
LocalVar.FA_Acc = avrSWAP(53);          % Tower top fore-aft acceleration [m/s^2]
LocalVar.NacIMU_FA_Acc = avrSWAP(83);   % Nacelle nodding acceleration [rad/s^2]
LocalVar.Azimuth = avrSWAP(60);         % Rotor azimuth angle [rad]
LocalVar.NumBl = avrSWAP(61);     % No. of blades

  % --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does not...
if (LocalVar.iStatus == 0)
    LocalVar.BlPitch(1) = avrSWAP(4);   % Blade-1 pitch angle [rad]
    LocalVar.BlPitch(2) = avrSWAP(33);  % Blade-2 pitch angle [rad]
    LocalVar.BlPitch(3) = avrSWAP(34);  % Blade-3 pitch angle [rad]
else
    LocalVar.BlPitch(1) = LocalVar.PitCom(1);
    LocalVar.BlPitch(2) = LocalVar.PitCom(2);
    LocalVar.BlPitch(3) = LocalVar.PitCom(3) ;     
end

end
