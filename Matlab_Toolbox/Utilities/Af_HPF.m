function [dHPF,HPF] = Af_HPF(omega,zeta,DT,varargin)
%% Example Code
% omega = 2*pi*6;  %rad/s
% HPF = tf(om^2,[1,2*zeta*omega,omega^2]);
% dHFP = c2d(LPF,DT);
%
% varargin to specify order

if isempty(varargin)
    order = 2;
else
    order = varargin{1};
end

if order == 2
    HPF = tf([1,0,0],[1,2*zeta*omega,omega^2]);
elseif order > 2
    [b,a] = butter(order,omega,'high','s');
    HPF = tf(b,a);
else
    HPF = tf([1,0],[1,omega]);
end

dHPF = c2d(HPF,DT,'tustin');

if 0
    figure(881);
    bode(HPF);
end