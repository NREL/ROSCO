function ROSCO = ROSCOout2Matlab(FileName)

if exist(FileName,'file')
    
    %% file read
    [Channels, ChanName, ChanUnit] = ReadFASTtext(FileName,[],2);
    
    %% trim channel names and units, assign to struct
    
    ROSCO       = struct;
    ROSCO.Time  = Channels(:,1);
    
    for iChan = 1:length(ChanName)
        ROSCO.ChanName{iChan} = strtrim(ChanName{iChan});
        ROSCO.ChanUnit{iChan} = strtrim(ChanUnit{iChan});
        
        ROSCO.(ROSCO.ChanName{iChan}) = Channels(:,iChan);
        
    end
    
else
    ROSCO = [];
    disp('No .RO.dbg file')
end
