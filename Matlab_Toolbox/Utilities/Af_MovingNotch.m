function [dNF_LPV,dNF_LPV_ss] = Af_MovingNotch(frequencyRange,zeta,beta,DT)
%% Example code
% Notch at 2P
% LPV filter with rotor speed as scheduling parameter

% om_1P       = 2*pi*RotorSpeedDomain/60;
% om_2P       = 2*om_1P;
% 
% zeta_2P     = 0.8;
% beta_2P     = zeta_2P/10;
% 
% %populate range of 2P/4P filters
% dNF_2P_lpv = ss;
% for iom = 1:1:length(om_2P)
%     NF_2P = tf([1,2*om_2P(iom)*beta_2P,(om_2P(iom))^2],[1,2*om_2P(iom)*zeta_2P,(om_2P(iom))^2]);
%     dNF_2P_lpv(:,:,iom) = c2d(ss(NF_2P),Control.DT);
% end
% 
% %sampling grid (scheduling parameter)
% dNF_2P_lpv.SamplingGrid = struct('Omega_2P',om_2P);

%% 
NF      = tf;
dNF_LPV = tf;
dNF_LPV_ss = ss;

for iFreq = 1:1:length(frequencyRange)
    NF = tf([1,2*frequencyRange(iFreq)*beta,(frequencyRange(iFreq))^2],[1,2*frequencyRange(iFreq)*zeta,(frequencyRange(iFreq))^2]);
    dNF_LPV(iFreq) = c2d((NF),DT);
    dNF_LPV_ss(:,:,iFreq) = c2d(ss(NF),DT);
end

%sampling grid (scheduling parameter)
dNF_LPV_ss.SamplingGrid = struct('Omega',frequencyRange);
