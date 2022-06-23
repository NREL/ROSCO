% Read all constant control parameters from DISCON.IN parameter file
function PerfData = ReadCpFile(CntrPar)


fileID = fopen(CntrPar.PerfFileName,'r'); % Read performance file from DISCON.IN

% ----------------------- Axis Definitions ------------------------
fgetl(fileID);
fgetl(fileID);
fgetl(fileID);
fgetl(fileID);

line = fgetl(fileID);
temp = textscan(line,'%f');
PerfData.Beta_vec = cell2mat(temp)';

fgetl(fileID); 

line = fgetl(fileID);
temp = textscan(line,'%f');
PerfData.TSR_vec = cell2mat(temp)';

% ----------------------- Read Cp, Ct, Cq, Tables ------------------------
fgetl(fileID); 
fgetl(fileID); % Input file should contains wind speed information here - unneeded for now
fgetl(fileID); 
fgetl(fileID); 
fgetl(fileID);

PerfData.Cp_mat = zeros(CntrPar.PerfTableSize(2),CntrPar.PerfTableSize(1));
for ii = 1:CntrPar.PerfTableSize(2)
    line = fgetl(fileID);
    temp = textscan(line,'%f');
    PerfData.Cp_mat(ii,:) = cell2mat(temp)';
end

fgetl(fileID); 
fgetl(fileID); 
fgetl(fileID);
fgetl(fileID);

PerfData.Ct_mat = zeros(CntrPar.PerfTableSize(2),CntrPar.PerfTableSize(1));
for ii = 1:CntrPar.PerfTableSize(2)
    line = fgetl(fileID);
    temp = textscan(line,'%f');
    PerfData.Ct_mat(ii,:) = cell2mat(temp)';
end

fgetl(fileID); 
fgetl(fileID); 
fgetl(fileID); 
fgetl(fileID);

PerfData.Cq_mat = zeros(CntrPar.PerfTableSize(2),CntrPar.PerfTableSize(1));
for ii = 1:CntrPar.PerfTableSize(2)
    line = fgetl(fileID);
    temp = textscan(line,'%f');
    PerfData.Cq_mat(ii,:) = cell2mat(temp)';
end
fid = fclose(fileID);
end
