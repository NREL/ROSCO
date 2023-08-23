%% ROSCO Filters

clear;

dbg = ROSCOout2Matlab('/Users/dzalkind/Tools/ROSCO1/Examples/examples_out/26_MHK/5_low_ws_debug/RM1_MHK/power_curve/base/RM1_MHK_0.RO.dbg2');


%% 



%%

num2 = 0.001;
den2 = 0.1;

num1 = 0.001;
den1 = 0.1;

NF1 = notch(1,num1,den1,0.01);
NF2 = notch(2.42,num2,den2,0.01);

figure(1);
bode(NF1)
bode(NF2)
bode(NF1*NF2)

% BS = fdesign.bandstop(




y = lsim(NF1*NF2,dbg.GenSpeed,dbg.Time);
% % 
% % 
figure(2);
plot(dbg.Time,dbg.GenSpeed)
hold on;
plot(dbg.Time,dbg.GenSpeedF)
plot(dbg.Time,y)
% % 
% % 
hold off

%% Notch Filter

function notch_filt = notch(omega,BetaNum,BetaDen,DT)

%     DT = 0.01;
% 
%     omega = 1;
%     BetaNum = 0.01;
%     BetaDen = 0.25;

    K = 2.0/DT;
    b2 = (K^2.0 + 2.0*omega*BetaNum*K + omega^2.0)/(K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
    b1 = (2.0*omega^2.0 - 2.0*K^2.0)  / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
    b0 = (K^2.0 - 2.0*omega*BetaNum*K + omega^2.0) / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
    a1 = (2.0*omega^2.0 - 2.0*K^2.0)  / (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);
    a0 = (K^2.0 - 2.0*omega*BetaDen*K + omega^2.0)/ (K^2.0 + 2.0*omega*BetaDen*K + omega^2.0);

    notch_filt = tf([b2,b1,b0],[1,a1,a0],DT);

end

