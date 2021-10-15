function [dLPF,LPF] = Af_LPF(omega,zeta,DT,varargin)
%% Example Code
% omega = 2*pi*6;  %rad/s
% LPF = tf(om^2,[1,2*zeta*omega,omega^2]);
% dLFP = c2d(LPF,DT);
%
% varargin to specify order

if isempty(varargin)
    order = 2;
else
    order = varargin{1};
end

if order == 2
    LPF = tf(omega^2,[1,2*zeta*omega,omega^2]);
elseif order > 2
    [b,a] = butter(order,omega,'s');
    LPF = tf(b,a);
else
    LPF = tf(omega,[1,omega]);
end

dLPF = c2d(LPF,DT,'tustin');

if 0
    figure(881);
    bode(LPF);
end