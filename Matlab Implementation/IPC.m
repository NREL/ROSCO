function [LocalVar] = IPC(CntrPar, LocalVar, objInst)
    % Individual pitch control subroutine
    %   - Calculates the commanded pitch angles for IPC employed for blade fatigue load reductions at 1P and 2P
    %   - Includes yaw by IPC

%     % Local variables
%     REAL(8)                  :: PitComIPC(3), PitComIPCF(3), PitComIPC_1P(3), PitComIPC_2P(3)
%     INTEGER(4)               :: K                                       % Integer used to loop through turbine blades
%     REAL(8)                  :: axisTilt_1P, axisYaw_1P, axisYawF_1P    % Direct axis and quadrature axis outputted by Coleman transform, 1P
%     REAL(8), SAVE            :: IntAxisTilt_1P, IntAxisYaw_1P           % Integral of the direct axis and quadrature axis, 1P
%     REAL(8)                  :: axisTilt_2P, axisYaw_2P, axisYawF_2P    % Direct axis and quadrature axis outputted by Coleman transform, 1P
%     REAL(8), SAVE            :: IntAxisTilt_2P, IntAxisYaw_2P           % Integral of the direct axis and quadrature axis, 1P
%     REAL(8)                  :: IntAxisYawIPC_1P                        % IPC contribution with yaw-by-IPC component
%     REAL(8)                  :: Y_MErrF, Y_MErrF_IPC                    % Unfiltered and filtered yaw alignment error [rad]
% 
%     TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
%     TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
%     TYPE(ObjectInstances), INTENT(INOUT)    :: objInst

    % Body
    % Initialization
    % Set integrals to be 0 in the first time step
    if (LocalVar.iStatus==0) 
        IntAxisTilt_1P  = 0.0;
        IntAxisYaw_1P   = 0.0;
        IntAxisTilt_2P  = 0.0;
        IntAxisYaw_2P   = 0.0;
    end

    % Pass rootMOOPs through the Coleman transform to get the tilt and yaw moment axis
    [axisTilt_1P, axisYaw_1P] = ColemanTransform(LocalVar.rootMOOP, LocalVar.Azimuth, NP_1);
    [axisTilt_2P, axisYaw_2P] = ColemanTransform(LocalVar.rootMOOP, LocalVar.Azimuth, NP_2);

    % High-pass filter the MBC yaw component and filter yaw alignment error, and compute the yaw-by-IPC contribution
    if (CntrPar.Y_ControlMode == 2) 
        Y_MErrF = SecLPFilter(LocalVar.Y_MErr, LocalVar.DT, CntrPar.Y_IPC_omegaLP, CntrPar.Y_IPC_zetaLP, LocalVar.iStatus, false, objInst.instSecLPF);
        Y_MErrF_IPC = PIController(Y_MErrF, CntrPar.Y_IPC_KP(1), CntrPar.Y_IPC_KI(1), -CntrPar.Y_IPC_IntSat, CntrPar.Y_IPC_IntSat, LocalVar.DT, 0.0, false, objInst.instPI);
    else
        axisYawF_1P = axisYaw_1P;
        Y_MErrF = 0.0;
        Y_MErrF_IPC = 0.0;
    end

    % Integrate the signal and multiply with the IPC gain
    if ((CntrPar.IPC_ControlMode >= 1) && (CntrPar.Y_ControlMode ~= 2)) 
        IntAxisTilt_1P = IntAxisTilt_1P + LocalVar.DT * CntrPar.IPC_KI(1) * axisTilt_1P;
        IntAxisYaw_1P = IntAxisYaw_1P + LocalVar.DT * CntrPar.IPC_KI(1) * axisYawF_1P;
        IntAxisTilt_1P = saturate(IntAxisTilt_1P, -CntrPar.IPC_IntSat, CntrPar.IPC_IntSat);
        IntAxisYaw_1P = saturate(IntAxisYaw_1P, -CntrPar.IPC_IntSat, CntrPar.IPC_IntSat);

        if (CntrPar.IPC_ControlMode >= 2) 
            IntAxisTilt_2P = IntAxisTilt_2P + LocalVar.DT * CntrPar.IPC_KI(2) * axisTilt_2P;
            IntAxisYaw_2P = IntAxisYaw_2P + LocalVar.DT * CntrPar.IPC_KI(2) * axisYawF_2P;
            IntAxisTilt_2P = saturate(IntAxisTilt_2P, -CntrPar.IPC_IntSat, CntrPar.IPC_IntSat);
            IntAxisYaw_2P = saturate(IntAxisYaw_2P, -CntrPar.IPC_IntSat, CntrPar.IPC_IntSat);
        end
    else
        IntAxisTilt_1P = 0.0;
        IntAxisYaw_1P = 0.0;
        IntAxisTilt_2P = 0.0;
        IntAxisYaw_2P = 0.0;
    end

    % Add the yaw-by-IPC contribution
    IntAxisYawIPC_1P = IntAxisYaw_1P + Y_MErrF_IPC;

    % Pass direct and quadrature axis through the inverse Coleman transform
    % to get the commanded pitch angles
    [CntrPar.IPC_aziOffset(1), PitComIPC_1P] = ColemanTransformInverse(IntAxisTilt_1P, IntAxisYawIPC_1P, LocalVar.Azimuth, NP_1);
    [CntrPar.IPC_aziOffset(2), PitComIPC_2P] = ColemanTransformInverse(IntAxisTilt_2P, IntAxisYaw_2P, LocalVar.Azimuth, NP_2);

    % Sum nP IPC contributions and store to LocalVar data type
    for K = 1:LocalVar.NumBl
        PitComIPC(K) = PitComIPC_1P(K) + PitComIPC_2P(K);

        % Optionally filter the resulting signal to induce a phase delay
        if (CntrPar.IPC_CornerFreqAct > 0.0) 
            PitComIPCF(K) = LPFilter(PitComIPC(K), LocalVar.DT, CntrPar.IPC_CornerFreqAct, LocalVar.iStatus, false, objInst.instLPF);
        else
            PitComIPCF(K) = PitComIPC(K)
        end

        LocalVar.IPC_PitComF(K) = PitComIPCF(K);
    end
end
