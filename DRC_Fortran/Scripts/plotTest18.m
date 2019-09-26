%% plotTest18
% Plot results from Test18.out. Data is imported in a structure data. Also
% has the option to run Test18 from the matlab script

%% Cleaning
close all
clearvars
clc

%% Settings
totalTime = tic;
testFile = 'Test18.out';        % Name of the test file to read
runCmdFromHere = false;         % Run RunTest18.cmd from this matlab script

%% Loading
debugFolder = 'CertTest\';      % Directory were .out file is located

if(runCmdFromHere)              % Run Test18 first if true
    dos('RunTest18.cmd');
end       

% Import data from Test18.out and put in the data structure
dataRaw = dlmread([debugFolder testFile],'\t',8,0);
[~,vars] = size(dataRaw);
fid = fopen([debugFolder testFile]);
header = textscan(fid,'%s','delimiter','\t');
fclose(fid);

header = strtrim(header{1,1}(4:vars+3));
for i = 1:vars
    data.(header{i}) = dataRaw(:,i);
end

clearvars vars

%% Plotting

% calculate total wind velocity
Wind1VelTot = sqrt(data.Wind1VelX.^2+data.Wind1VelY.^2+data.Wind1VelZ.^2);

% Plot different wind velocities
figure;
title('Wind Velocity')
hold on
plot(data.Time,data.Wind1VelX)
plot(data.Time,data.Wind1VelY)
plot(data.Time,data.Wind1VelZ)
plot(data.Time,Wind1VelTot)
ylabel('Wind velocity [m/s]')
legend('x','y','z','total')

% Plot rotor and generator speed
figure;
s(1) = subplot(2,2,1);
title('Rotor and Generator Speed')
hold on
plot(data.Time,data.RotSpeed)
ylabel('Rotor speed [rpm]')
yyaxis right
plot(data.Time,data.GenSpeed)
ylabel('Generator speed [rpm]')

% Plot generator torque
s(2) = subplot(2,2,2);
title('Generator Torque')
hold on
plot(data.Time,data.GenTq)
ylabel('Torque [kN*m]')

% Plot pitch of blade 1
s(3) = subplot(2,2,3);
title('BldPitch1')
hold on
plot(data.Time,data.BldPitch1)
ylabel('Pitch [deg]')

% Calculate RootMtotb1
RootMtotb1 = sqrt(data.RootMxb1.^2 + data.RootMyb1.^2 + data.RootMzb1.^2);

% Plot the root bending moments of blade 1
s(4) = subplot(2,2,4);
title('Blade 1 root bending moments')
hold on
plot(data.Time,data.RootMxb1)
plot(data.Time,data.RootMyb1)
plot(data.Time,data.RootMzb1)
plot(data.Time,RootMtotb1)
ylabel('Moment [kN*m]')
legend('x','y','z','total')

% Link the time axes of the subplots
linkaxes(s,'x')

toc(totalTime)
