function [LocalVar,DebugVar] = WindSpeedEstimator(LocalVar, CntrPar, PerfData)
% Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
%       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
%       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 

    R2D = 57.295780;        % Factor to convert radians to degrees
    persistent om_r v_t v_m v_h P xh K 
    
    % ---- Debug Inputs ------
    DebugVar.WE_b   = LocalVar.PC_PitComTF*R2D;
    DebugVar.WE_w   = LocalVar.RotSpeedF;
    DebugVar.WE_t   = LocalVar.VS_LastGenTrqF;

    % ---- Define wind speed estimate ---- 

    % Inversion and Invariance Filter implementation
    if (CntrPar.WE_Mode == 1)       
        % Removed in latest patch
%         LocalVar.WE_VwIdot = CntrPar.WE_Gamma/CntrPar.WE_Jtot*(LocalVar.VS_LastGenTrq*CntrPar.WE_GearboxRatio - AeroDynTorque(LocalVar, CntrPar, PerfData));
        
        % Version 2.4 Patch
        Tau_r = AeroDynTorque(LocalVar, CntrPar, PerfData);
        LocalVar.WE_VwIdot = CntrPar.WE_Gamma/CntrPar.WE_Jtot*(LocalVar.VS_LastGenTrq*CntrPar.WE_GearboxRatio - Tau_r);
        
        LocalVar.WE_VwI = LocalVar.WE_VwI + LocalVar.WE_VwIdot*LocalVar.DT;
        LocalVar.WE_Vw = LocalVar.WE_VwI + CntrPar.WE_Gamma*LocalVar.RotSpeedF;

    % Extended Kalman Filter (EKF) implementation
    elseif (CntrPar.WE_Mode == 2) 
        % Define contant values
        L = 6.0 * CntrPar.WE_BladeRadius;
        Ti = 0.18;
        R_m = 0.02;
        H = [1 0 0];
        % Define matrices to be filled
        F = zeros(3,3);
        Q = zeros(3,3);
        
        if (LocalVar.iStatus == 0) 
            % Initialize recurring values
            om_r = LocalVar.RotSpeedF;
            v_t = 0.0;
            v_m = LocalVar.HorWindV;
            v_h = LocalVar.HorWindV;
            lambda = LocalVar.RotSpeed * CntrPar.WE_BladeRadius/v_h;
            xh = [om_r; v_t; v_m];
            P = [0.01 0.0 0.0;0.0 0.01 0.0;0.0 0.0 1.0];
            K = [0.0;0.0;0.0];
            Cp_op   = 0.25;  % initialize so debug output doesn't give *****
        else
            % Find estimated operating Cp and system pole
            A_op = interp1d(CntrPar.WE_FOPoles_v,CntrPar.WE_FOPoles,v_h);

            % TEST INTERP2D
            lambda = LocalVar.RotSpeed * CntrPar.WE_BladeRadius/v_h;
            Cp_op = interp2d(PerfData.Beta_vec,PerfData.TSR_vec,PerfData.Cp_mat, LocalVar.PitCom(1)*R2D, lambda );
            Cp_op = max(0.0,Cp_op);

            % Update Jacobian
            F(1,1) = A_op;
            F(2,1) = 1.0/(2.0*CntrPar.WE_Jtot) * CntrPar.WE_RhoAir * pi *CntrPar.WE_BladeRadius^2.0 * 1/om_r * 3.0 * Cp_op * v_h^2.0;
            F(3,1) = 1.0/(2.0*CntrPar.WE_Jtot) * CntrPar.WE_RhoAir * pi *CntrPar.WE_BladeRadius^2.0 * 1/om_r * 3.0 * Cp_op * v_h^2.0;
            F(2,2) = - pi * v_m/(2.0*L);
            F(3,2) = - pi * v_t/(2.0*L);

            % Update process noise covariance
            Q(1,1) = 0.00001;
            Q(2,2) =(pi * (v_m^3.0) * (Ti^2.0)) / L;
            Q(3,3) = (2.0^2.0)/600.0;

            % Prediction update
            Tau_r = AeroDynTorque(LocalVar,CntrPar,PerfData);
            a = pi * v_m/(2.0*L);
            dxh(1,1) = 1.0/CntrPar.WE_Jtot * (Tau_r - CntrPar.WE_GearboxRatio * LocalVar.VS_LastGenTrqF);
            dxh(1,2) = -a*v_t;
            dxh(1,3) = 0.0;
        
            xh = xh + LocalVar.DT * dxh; % state update
            P = P + LocalVar.DT*(mtimes(F,P) + mtimes(P,transpose(F)) + Q - mtimes(K * R_m, transpose(K))) ;

            % Measurement update
            S = mtimes(H,mtimes(P,transpose(H))) + R_m;        % NJA: (H*T*H') \approx 0
            K = mtimes(P,transpose(H))/S(1,1);
            xh = xh + K*(LocalVar.RotSpeedF - om_r);
            P = mtimes(eye(3) - mtimes(K,H),P);

            % Wind Speed Estimate
            om_r = xh(1,1);
            v_t = xh(1,2);
            v_m = xh(1,3);
            v_h = v_t + v_m;
            LocalVar.WE_Vw = v_m + v_t;
%             disp(LocalVar.WE_Vw)
        end
        % Debug Outputs
        DebugVar.WE_Cp = Cp_op;
        DebugVar.WE_Vm = v_m;
        DebugVar.WE_Vt = v_t;
        DebugVar.WE_lambda = lambda;
    else
%         LocalVar.WE_Vw = LocalVar.HorWindV;
        % Define Variables
        F_WECornerFreq = 0.20944;  % Fix to 30 second time constant for now    

        % Filter wind speed at hub height as directly passed from OpenFAST
        LocalVar.WE_Vw_F = LPFilter_WE(LocalVar.HorWindV, LocalVar.DT, F_WECornerFreq, LocalVar.iStatus, false);
%         disp(LocalVar.WE_Vw_F)
    end
     

end
