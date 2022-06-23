% Check for errors before any execution
function [aviFAIL,aviFAIL_array] = Assert(LocalVar, CntrPar)



    %..............................................................................................................................
    % Check validity of input parameters:
    %..............................................................................................................................

    aviFAIL_array = zeros(26,1);
    
    if ((CntrPar.F_LPFType > 2.0) || (CntrPar.F_LPFType < 1.0)) 
        aviFAIL_array(1) = -1;
        ErrMsg  = 'F_LPFType must be 1 or 2';
        disp(ErrMsg);
    end

    if ((CntrPar.F_LPFDamping > 1.0) || (CntrPar.F_LPFDamping < 0.0)) 
        aviFAIL_array(2) = -1;
        ErrMsg  = 'Filter damping coefficient must be between [0, 1]';
        disp(ErrMsg);
    end

    if (CntrPar.IPC_CornerFreqAct < 0.0) 
        aviFAIL_array(3) = -1;
        ErrMsg  = 'Corner frequency of IPC actuator model must be positive, or set to 0 to disable.';
        disp(ErrMsg);
    end

    if (CntrPar.F_LPFCornerFreq <= 0.0) 
        aviFAIL_array(4) = -1;
        ErrMsg  = 'CornerFreq must be greater than zero.';
        disp(ErrMsg);
    end

    if ((CntrPar.IPC_ControlMode > 0) && (CntrPar.Y_ControlMode > 1)) 
        aviFAIL_array(5) = -1;
        ErrMsg  = 'IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously';
        disp(ErrMsg);
    end

    if (LocalVar.DT <= 0.0) 
        aviFAIL_array(6) = -1;
        ErrMsg  = 'DT must be greater than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.VS_MaxRat <= 0.0) 
        aviFAIL_array(7) =  -1;
        ErrMsg  = 'VS_MaxRat must be greater than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.VS_RtTq < 0.0) 
        aviFAIL_array(8) = -1;
        ErrMsg  = 'VS_RtTq must not be negative.';
        disp(ErrMsg);
    end

    if (CntrPar.VS_Rgn2K < 0.0) 
        aviFAIL_array(9) = -1;
        ErrMsg  = 'VS_Rgn2K must not be negative.';
        disp(ErrMsg);
    end

    if (CntrPar.VS_MaxTq < CntrPar.VS_RtTq) 
        aviFAIL_array(10) = -1;
        ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.';
        disp(ErrMsg);
    end

    if (CntrPar.VS_KP(1) > 0.0) 
        aviFAIL_array(11) = -1;
        ErrMsg  = 'VS_KP must be less than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.VS_KI(1) > 0.0) 
        aviFAIL_array(12) = -1;
        ErrMsg  = 'VS_KI must be less than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.PC_RefSpd <= 0.0) 
        aviFAIL_array(13) = -1;
        ErrMsg  = 'PC_RefSpd must be greater than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.PC_MaxRat <= 0.0) 
        aviFAIL_array(14) = -1;
        ErrMsg  = 'PC_MaxRat must be greater than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.PC_MinPit >= CntrPar.PC_MaxPit)  
        aviFAIL_array(15) = -1;
        ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.';
        disp(ErrMsg);
    end

    if (CntrPar.IPC_KI(1) < 0.0)  
        aviFAIL_array(16) = -1;
        ErrMsg  = 'IPC_KI(1) must be zero or greater than zero.';
        disp(ErrMsg);
    end

    if (CntrPar.IPC_KI(2) < 0.0)  
        aviFAIL_array(17) = -1;
        ErrMsg  = 'IPC_KI(2) must be zero or greater than zero.';
        disp(ErrMsg);
    end

    % ---- Yaw Control ----
    if (CntrPar.Y_ControlMode > 0) 
        if (CntrPar.Y_IPC_omegaLP <= 0.0)  
            aviFAIL_array(18) = -1;
            ErrMsg  = 'Y_IPC_omegaLP must be greater than zero.';
            disp(ErrMsg);
        end

        if (CntrPar.Y_IPC_zetaLP <= 0.0)  
            aviFAIL_array(19) = -1;
            ErrMsg  = 'Y_IPC_zetaLP must be greater than zero.';
            disp(ErrMsg);
        end

        if (CntrPar.Y_ErrThresh <= 0.0)  
            aviFAIL_array(20) = -1;
            ErrMsg  = 'Y_ErrThresh must be greater than zero.';
            disp(ErrMsg);
        end

        if (CntrPar.Y_Rate <= 0.0)  
            aviFAIL_array(21) = -1;
            ErrMsg  = 'CntrPar.Y_Rate must be greater than zero.';
            disp(ErrMsg);
        end

        if (CntrPar.Y_omegaLPFast <= 0.0)  
            aviFAIL_array(22) = -1;
            ErrMsg  = 'Y_omegaLPFast must be greater than zero.';
            disp(ErrMsg);
        end

        if (CntrPar.Y_omegaLPSlow <= 0.0)  
            aviFAIL_array(23) = -1;
            ErrMsg  = 'Y_omegaLPSlow must be greater than zero.';
            disp(ErrMsg);
        end
    end

    % --- Floating Control ---
    if (CntrPar.Fl_Mode > 0) 
        if (CntrPar.F_NotchType <= 1 || CntrPar.F_NotchCornerFreq == 0.0) 
            aviFAIL_array(24) = -1;
            ErrMsg = 'F_NotchType and F_NotchCornerFreq must be specified for Fl_Mode greater than zero.';
            disp(ErrMsg);
        end
    end

    
    aviFAIL = min(aviFAIL_array);

end