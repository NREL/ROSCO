%% ROSCO Filters

clear;

%% Notch Filter

DT = 0.01;

omega = 2.4;
BetaNum = 0.1;
BetaDen = 0.65;

K = 2.0/DT;
b2 = (K^2.0 + 2.0*omega*BetaNum*K + omega^2.0)/(K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
b1 = (2.0*omega^2.0 - 2.0*K^2.0)  / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
b0 = (K^2.0 - 2.0*omega*BetaNum*K + omega^2.0) / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
a1 = (2.0*omega^2.0 - 2.0*K^2.0)  / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
a0 = (K^2.0 - 2.0*omega*BetaDen*K + omega^2.0)/ (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);

notch_filt = tf([b2,b1,b0],[1,a1,a0],DT)


figure(1);
bode(notch_filt)
