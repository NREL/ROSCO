function [LocalVar,avrSWAP] = FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
    % Yaw rate controller
    %       Y_ControlMode = 0, No yaw control
    %       Y_ControlMode = 1, Simple yaw rate control using yaw drive
    %       Y_ControlMode = 2, Yaw by IPC (accounted for in IPC subroutine)

%     REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*) % The swap array, used to pass data to, and receive data from, the DLL controller.
% 
%     TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
%     TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
%     TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
%     % Internal Variables
%     Integer(4)                                :: K
%     REAL(8)                                   :: rootMOOP_F(3)
%     REAL(8)                                   :: RootMyb_Vel(3)
%     REAL(8), SAVE                             :: RootMyb_Last(3)
%     REAL(8)                                   :: RootMyb_VelErr(3)


       R2D = 57.295780;        % Factor to convert radians to degrees

    % Flap control
    if (CntrPar.Flp_Mode >= 1) 
        if ((LocalVar.iStatus == 0) && (CntrPar.Flp_Mode >= 1)) 
            RootMyb_Last(1) = 0 - LocalVar.rootMOOP(1);
            RootMyb_Last(2) = 0 - LocalVar.rootMOOP(2);
            RootMyb_Last(3) = 0 - LocalVar.rootMOOP(3);
            % Initial Flap angle
            LocalVar.Flp_Angle(1) = CntrPar.Flp_Angle;
            LocalVar.Flp_Angle(2) = CntrPar.Flp_Angle;
            LocalVar.Flp_Angle(3) = CntrPar.Flp_Angle;
            % Initialize filter
            RootMOOP_F(1) = SecLPFilter(LocalVar.rootMOOP(1),LocalVar.DT, CntrPar.F_FlpCornerFreq, CntrPar.F_FlpDamping, LocalVar.iStatus, false,objInst.instSecLPF);
            RootMOOP_F(2) = SecLPFilter(LocalVar.rootMOOP(2),LocalVar.DT, CntrPar.F_FlpCornerFreq, CntrPar.F_FlpDamping, LocalVar.iStatus, false,objInst.instSecLPF);
            RootMOOP_F(3) = SecLPFilter(LocalVar.rootMOOP(3),LocalVar.DT, CntrPar.F_FlpCornerFreq, CntrPar.F_FlpDamping, LocalVar.iStatus, false,objInst.instSecLPF);
            % Initialize controller
            if (CntrPar.Flp_Mode == 2) 
                LocalVar.Flp_Angle(K) = PIIController(RootMyb_VelErr(K), 0 - LocalVar.Flp_Angle(K), CntrPar.Flp_Kp, CntrPar.Flp_Ki, 0.05, -CntrPar.Flp_MaxPit , CntrPar.Flp_MaxPit , LocalVar.DT, 0.0, true, objInst.instPI);
            end

        % Steady flap angle
        elseif (CntrPar.Flp_Mode == 1) 
            LocalVar.Flp_Angle(1) = LocalVar.Flp_Angle(1);
            LocalVar.Flp_Angle(2) = LocalVar.Flp_Angle(2);
            LocalVar.Flp_Angle(3) = LocalVar.Flp_Angle(3);
            % if (MOD(LocalVar.Time,10.0) == 0) 
            %     LocalVar.Flp_Angle(1) = LocalVar.Flp_Angle(1) + 1*D2R
            %     LocalVar.Flp_Angle(2) = LocalVar.Flp_Angle(2) + 1*D2R
            %     LocalVar.Flp_Angle(3) = LocalVar.Flp_Angle(3) + 1*D2R
            % end

        % PII flap control
        elseif (CntrPar.Flp_Mode == 2) 
            for K = 1:LocalVar.NumBl
                % LPF Blade root bending moment
                RootMOOP_F(K) = SecLPFilter(LocalVar.rootMOOP(K),LocalVar.DT, CntrPar.F_FlpCornerFreq, CntrPar.F_FlpDamping, LocalVar.iStatus, false,objInst.instSecLPF);

                % Find derivative and derivative error of blade root bending moment
                RootMyb_Vel(K) = (RootMOOP_F(K) - RootMyb_Last(K))/LocalVar.DT;
                RootMyb_VelErr(K) = 0 - RootMyb_Vel(K);

                % Find flap angle command - includes an integral term to encourage zero flap angle
                LocalVar.Flp_Angle(K) = PIIController(RootMyb_VelErr(K), 0 - LocalVar.Flp_Angle(K), CntrPar.Flp_Kp, CntrPar.Flp_Ki, 0.05, -CntrPar.Flp_MaxPit , CntrPar.Flp_MaxPit , LocalVar.DT, 0.0, false, objInst.instPI);
                % Saturation Limits
                LocalVar.Flp_Angle(K) = saturate(LocalVar.Flp_Angle(K), -CntrPar.Flp_MaxPit, CntrPar.Flp_MaxPit);

                % Save some data for next iteration
                RootMyb_Last(K) = RootMOOP_F(K);
            end
        end

        % Send to AVRSwap
        avrSWAP(120) = LocalVar.Flp_Angle(1) * R2D;   % Needs to be sent to openfast in degrees
        avrSWAP(121) = LocalVar.Flp_Angle(2) * R2D;   % Needs to be sent to openfast in degrees
        avrSWAP(122) = LocalVar.Flp_Angle(3) * R2D;   % Needs to be sent to openfast in degrees
    end
end
