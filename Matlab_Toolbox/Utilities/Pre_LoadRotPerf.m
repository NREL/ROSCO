function RotPerf = Pre_LoadRotPerf(RotPerfFile)
% Load rotor performance information from a text file. 
% Loads OpenFast model output into a MATLAB structure to be post processed
%
% Inputs: RotPerfFile - Cp_Ct_Cq.txt file from CCBlade/ROSCO
% Outputs: RotorPerformance - Structure containing rotor performance data
%
% Nikhar Abbas


fid = fopen(RotPerfFile, 'r');
if fid == -1, error('Error loading file'), end

% Read first line to start tline
tline = fgetl(fid);
while ~feof(fid)
    tline = fgetl(fid);
    l_str = strsplit(tline);
    if tline
        % Find pitch angle vector
        if strcmpi(l_str{2},'Pitch')
            tline = fgetl(fid);
            BlPitch = str2num(tline);    
        % Find TSR vector
        elseif strcmpi(l_str{2},'TSR')
            tline = fgetl(fid);
            TSR = str2num(tline);   
        % Find Wind speed vector
        elseif strcmpi(l_str{2},'Wind')
            tline = fgetl(fid);
            Wind = str2num(tline);   

         % Read Cp table
        elseif strcmpi(l_str{2},'Power')
            tline = fgetl(fid);
            Cpmat = zeros(length(TSR),length(BlPitch));
            for pind = 1:length(TSR)
                tline = fgetl(fid);
                CpMat(pind,:) = str2num(tline);
            end
            
        % Read Ct table
        elseif strcmpi(l_str{2},'Thrust')
            tline = fgetl(fid);
            Ctmat = zeros(length(TSR),length(BlPitch));
            for pind = 1:length(TSR)
                tline = fgetl(fid);
                CtMat(pind,:) = str2num(tline);
            end
            
        % Read Cq table
        elseif strcmpi(l_str{2},'Torque')
            tline = fgetl(fid);
            Cqmat = zeros(length(TSR),length(BlPitch));
            for pind = 1:length(TSR)
                tline = fgetl(fid);
                CqMat(pind,:) = str2num(tline);
            end
        end
    end
end
    
    


% Save Structure
RotPerf.BlPitch = BlPitch;
RotPerf.TSR = TSR;
RotPerf.Wind = Wind;
RotPerf.Cpmat = CpMat;
RotPerf.Ctmat = CtMat;
RotPerf.Cqmat = CqMat;

end