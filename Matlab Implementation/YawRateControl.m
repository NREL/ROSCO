function [LocalVar,avrSWAP] = YawRateControl(avrSWAP, CntrPar, LocalVar, objInst)
    % Yaw rate controller
    %       Y_ControlMode = 0, No yaw control
    %       Y_ControlMode = 1, Simple yaw rate control using yaw drive
    %       Y_ControlMode = 2, Yaw by IPC (accounted for in IPC subroutine)

    %..............................................................................................................................
    % Yaw control
    %..............................................................................................................................

    if (CntrPar.Y_ControlMode == 1) 
        avrSWAP(29) = 0;                                      % Yaw control parameter: 0 = yaw rate control
        if (LocalVar.Time >= LocalVar.Y_YawEndT)         % Check if the turbine is currently yawing
            avrSWAP(48) = 0.0;                                % Set yaw rate to zero

            LocalVar.Y_ErrLPFFast = LPFilter(LocalVar.Y_MErr, LocalVar.DT, CntrPar.Y_omegaLPFast, LocalVar.iStatus, false, objInst.instLPF);        % Fast low pass filtered yaw error with a frequency of 1
            LocalVar.Y_ErrLPFSlow = LPFilter(LocalVar.Y_MErr, LocalVar.DT, CntrPar.Y_omegaLPSlow, LocalVar.iStatus, false, objInst.instLPF);        % Slow low pass filtered yaw error with a frequency of 1/60
            
            if LocalVar.Y_ErrLPFFast<0
                LocalVar.Y_AccErr = LocalVar.Y_AccErr + LocalVar.DT*(-LocalVar.Y_ErrLPFFast^2);    % Integral of the fast low pass filtered yaw error
            else
                LocalVar.Y_AccErr = LocalVar.Y_AccErr + LocalVar.DT*(LocalVar.Y_ErrLPFFast^2);    % Integral of the fast low pass filtered yaw error
            end
            

            if (abs(LocalVar.Y_AccErr) >= CntrPar.Y_ErrThresh)                                    % Check if accumulated error surpasses the threshold
                LocalVar.Y_YawEndT = abs(LocalVar.Y_ErrLPFSlow/CntrPar.Y_Rate) + LocalVar.Time;        % Yaw to compensate for the slow low pass filtered error
            end
        else
            if LocalVar.Y_MErr<0
                avrSWAP(48) = -CntrPar.Y_Rate;        % Set yaw rate to predefined yaw rate, the sign of the error is copied to the rate
            else
                avrSWAP(48) = CntrPar.Y_Rate;
            end
            LocalVar.Y_ErrLPFFast = LPFilter(LocalVar.Y_MErr, LocalVar.DT, CntrPar.Y_omegaLPFast, LocalVar.iStatus, true, objInst.instLPF);        % Fast low pass filtered yaw error with a frequency of 1
            LocalVar.Y_ErrLPFSlow = LPFilter(LocalVar.Y_MErr, LocalVar.DT, CntrPar.Y_omegaLPSlow, LocalVar.iStatus, true, objInst.instLPF);       % Slow low pass filtered yaw error with a frequency of 1/60
            LocalVar.Y_AccErr = 0.0;    % "
        end
    end
end