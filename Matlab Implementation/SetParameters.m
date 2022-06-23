% Define parameters for control actions
function [LocalVar,abc,aviFAIL] = SetParameters(CntrPar, LocalVar)

persistent PerfData

% Set aviFAIL to 0 in each iteration:
aviFAIL = 0;


 

if (LocalVar.iStatus == 0)  % .TRUE. if we're on the first call to the DLL

    % Inform users that we are using this user-defined routine:
    aviFAIL = 1;
     disp('Running ROSCO-A wind turbine controller framework for public use in the scientific field')
     disp('Developed in collaboration: National Renewable Energy Laboratory Delft University of Technology, The Netherlands') 
     

    if (CntrPar.WE_Mode > 0) 
        [PerfData] = ReadCpFile(CntrPar);
    else
        PerfData = 0;   % Just to avoid the error, redundant step
    end
    
    % Initialize testValue (debugging variable)
    LocalVar.TestType = 0;

    % Initialize the SAVED variables:
    LocalVar.PitCom = LocalVar.BlPitch; % This will ensure that the variable speed controller picks the correct 
                                        % control region and the pitch controller picks the correct gain on the first call
    
    LocalVar.Y_AccErr = 0.0;  % This will ensure that the accumulated yaw error starts at zero
    LocalVar.Y_YawEndT = -1.0; % This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing

    % Wind speed estimator initialization
    LocalVar.WE_Vw = LocalVar.HorWindV;
    LocalVar.WE_VwI = LocalVar.WE_Vw - CntrPar.WE_Gamma*LocalVar.RotSpeed;

    % Setpoint Smoother initialization to zero
    LocalVar.SS_DelOmegaF = 0;

    % Generator Torque at K omega^2 or rated
    if (LocalVar.GenSpeed > 0.98 * CntrPar.PC_RefSpd) 
        LocalVar.GenTq = CntrPar.VS_RtTq;
    else
        LocalVar.GenTq = min(CntrPar.VS_RtTq, CntrPar.VS_Rgn2K*LocalVar.GenSpeed*LocalVar.GenSpeed);
    end            
    LocalVar.VS_LastGenTrq = LocalVar.GenTq;       
    LocalVar.VS_MaxTq      = CntrPar.VS_MaxTq;
    
    
    if LocalVar.iStatus == 0
        LocalVar.PC_PitComT = 0; %SBT- Assigned to zero, since this value is originally not initialized before it's call in PreFilterMeasuredSignal.m, line 49
        LocalVar.SD = 0;         %SBT- Assigned to zero, since this value is originally not initialized before it's call in ComputeVariablesSetpoints.m, line 47
        LocalVar.PC_MinPit = CntrPar.PC_MinPit; %SBT- Assigned to zero, since this value is originally not initialized before it's call in ComputeVariablesSetpoints.m, line 64
    end
    
    % Check validity of input parameters:
%     [aviFAIL,~] =  Assert(LocalVar, CntrPar, avrSWAP);
end
[aviFAIL,~] =  Assert(LocalVar, CntrPar);
abc = PerfData;
end
