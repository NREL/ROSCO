function [Shutdown_out] = Shutdown(LocalVar, CntrPar, objInst) 
% PeakShaving defines a minimum blade pitch angle based on a lookup table provided by DISON.IN
%       SS_Mode = 0, No setpoint smoothing
%       SS_Mode = 1, Implement setpoint smoothing

%     % Inputs
%     TYPE(ControlParameters), INTENT(IN)     :: CntrPar
%     TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
%     TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
%     % Allocate Variables 
%     REAL(8)                      :: SD_BlPitchF

    % Initialize Shutdown Varible
    if (LocalVar.iStatus == 0) 
        LocalVar.SD = false;
    end

    % See if we should shutdown
    if (~ LocalVar.SD ) 
        % Filter pitch signal
        SD_BlPitchF = LPFilter(LocalVar.PC_PitComT, LocalVar.DT, CntrPar.SD_CornerFreq, LocalVar.iStatus, false, objInst.instLPF);

        % Go into shutdown if above max pit
        if (SD_BlPitchF > CntrPar.SD_MaxPit) 
            LocalVar.SD  = true;
        else
            LocalVar.SD  = false;
        end 
    end

    % Pitch Blades to 90 degrees at max pitch rate if in shutdown mode
    if (LocalVar.SD) 
        Shutdown_out = LocalVar.BlPitch(1) + CntrPar.PC_MaxRat*LocalVar.DT;
        if (mod(LocalVar.Time, 10.0) == 0) 
             disp(' ** SHUTDOWN MODE **')
        end
    else
        Shutdown_out = LocalVar.PC_PitComT;
    end


end