function [CntrPar] = ReadControlParameterFileSub(fileName)

fileID = fopen(fileName,'r');

% Header
fgetl(fileID);
fgetl(fileID);
fgetl(fileID);
fgetl(fileID);

% Debug
line = fgetl(fileID);
CntrPar.LoggingLevel = str2double(sscanf(line,'%s',1));
fgetl(fileID);

% Controller Flags
fgetl(fileID);
line = fgetl(fileID);
CntrPar.F_LPFType = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.F_NotchType = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.IPC_ControlMode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.VS_ControlMode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.PC_ControlMode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.Y_ControlMode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.SS_Mode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.WE_Mode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.PS_Mode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.SD_Mode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.Fl_Mode = str2double(sscanf(line,'%s',1));
line = fgetl(fileID);
CntrPar.Flp_Mode = str2double(sscanf(line,'%s',1));
fgetl(fileID);

% Filter Constants
fgetl(fileID);
line = fgetl(fileID);
CntrPar.F_LPFCornerFreq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.F_LPFDamping = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.F_NotchCornerFreq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.F_NotchBetaNumDen = cell2mat(temp)';

line = fgetl(fileID);
CntrPar.F_SSCornerFreq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.F_FlCornerFreq = temp{1}(1,1);
CntrPar.F_FlDamping = temp{1}(2,1);

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.F_FlpCornerFreq = temp{1}(1,1);
CntrPar.F_FlpDamping = temp{1}(2,1);

fgetl(fileID);

% Blade Pitch Controller Constants

fgetl(fileID);

line = fgetl(fileID);
CntrPar.PC_GS_n = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PC_GS_angles = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PC_GS_KP = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PC_GS_KI = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PC_GS_KD = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PC_GS_TF = cell2mat(temp)';

line = fgetl(fileID);
CntrPar.PC_MaxPit = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.PC_MinPit = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.PC_MaxRat = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.PC_MinRat = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.PC_RefSpd = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.PC_FinePit = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.PC_Switch = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Individual Pitch Control

fgetl(fileID);

line = fgetl(fileID);
CntrPar.IPC_IntSat = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.IPC_KI = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.IPC_aziOffset = cell2mat(temp)';

line = fgetl(fileID);
CntrPar.IPC_CornerFreqAct = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% VS Torque Control

fgetl(fileID);

line = fgetl(fileID);
CntrPar.VS_GenEff = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_ArSatTq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_MaxRat = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_MaxTq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_MinTq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_MinOMSpd = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_Rgn2K = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_RtPwr = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_RtTq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_RefSpd = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_n = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_KP = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_KI = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.VS_TSRopt = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Setpoint Smoother
fgetl(fileID);

line = fgetl(fileID);
CntrPar.SS_VSGain = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.SS_PCGain = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Wind Speed Estimator
fgetl(fileID);

line = fgetl(fileID);
CntrPar.WE_BladeRadius = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.WE_CP_n = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.WE_CP = cell2mat(temp)';

line = fgetl(fileID);
CntrPar.WE_Gamma = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.WE_GearboxRatio = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.WE_Jtot = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.WE_RhoAir = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = strtok(line);
temp = strsplit(temp,'"');
CntrPar.PerfFileName = temp{2};

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PerfTableSize = cell2mat(temp)';

line = fgetl(fileID);
CntrPar.WE_FOPoles_N = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.WE_FOPoles_v = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.WE_FOPoles = cell2mat(temp)';

fgetl(fileID);

% Yaw Controller Constants

fgetl(fileID);

line = fgetl(fileID);
CntrPar.Y_ErrThresh = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_IPC_IntSat = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_IPC_n = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_IPC_KP = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_IPC_KI = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_IPC_omegaLP = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_IPC_zetaLP = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_MErrSet = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_omegaLPFast = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_omegaLPSlow = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Y_Rate = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Fore-Aft Tower Damper Constants
fgetl(fileID);

line = fgetl(fileID);
CntrPar.FA_KI = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.FA_HPFCornerFreq = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.FA_IntSat = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Peak Shaving

fgetl(fileID);

line = fgetl(fileID);
CntrPar.PS_BldPitchMin_N = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PS_WindSpeeds = cell2mat(temp)';

line = fgetl(fileID);
temp = textscan(line,'%f');
CntrPar.PS_BldPitchMin = cell2mat(temp)';

fgetl(fileID);

% Shutdown 
fgetl(fileID);

line = fgetl(fileID);
CntrPar.SD_MaxPit = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.SD_CornerFreq = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Floating
fgetl(fileID);

line = fgetl(fileID);
CntrPar.Fl_Kp = str2double(sscanf(line,'%s',1));

fgetl(fileID);

% Flaps
fgetl(fileID);

line = fgetl(fileID);
CntrPar.Flp_Angle = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Flp_Kp = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Flp_Ki = str2double(sscanf(line,'%s',1));

line = fgetl(fileID);
CntrPar.Flp_MaxPit = str2double(sscanf(line,'%s',1));
fid = fclose(fileID);
% End of Input File

% Calculated Constants
CntrPar.RtTq99 = CntrPar.VS_RtTq*0.99;
CntrPar.VS_MinOMTq = (CntrPar.VS_Rgn2K)*(CntrPar.VS_MinOMSpd)^2;
CntrPar.VS_MaxOMTq = (CntrPar.VS_Rgn2K)*(CntrPar.VS_RefSpd)^2;