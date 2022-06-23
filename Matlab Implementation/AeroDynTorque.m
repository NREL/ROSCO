function [AeroDynTq] = AeroDynTorque(LocalVar, CntrPar, PerfData)
% Function for computing the aerodynamic torque, divided by the effective rotor torque of the turbine, for use in wind speed estimation

    R2D = 57.295780;                 % Factor to convert radians to degrees

    % Find Torque
    RotorArea = pi*CntrPar.WE_BladeRadius^2;
    Lambda = (LocalVar.RotSpeedF)*(CntrPar.WE_BladeRadius)/LocalVar.WE_Vw;
%     Cp = CPfunction(CntrPar.WE_CP, Lambda);
    Cp = interp2d(PerfData.Beta_vec,PerfData.TSR_vec,PerfData.Cp_mat, LocalVar.PitCom(1)*R2D, Lambda);
    AeroDynTq = 0.5*(CntrPar.WE_RhoAir*RotorArea)*(LocalVar.WE_Vw^3/LocalVar.RotSpeedF)*Cp;
    AeroDynTq = max(AeroDynTq, 0.0);

end