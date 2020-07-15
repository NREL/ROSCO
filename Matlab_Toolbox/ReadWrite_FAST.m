function [P,Cx] = ReadWrite_FAST(fast)
% fast should have the following fields:
%  .FAST_directory - where files come from
%  .FAST_InputFile - main input file (.fst)
%  .FAST_runDirectory - where new files will be run from
%
%  Outputs:
%   P - parameter structure
%   F - filename structure
% 
% This is a simplified version of a similar file at https://github.com/dzalkind/matlab-toolbox/tree/master/Simulations

%% Read
FP = FAST2Matlab(fullfile(fast.FAST_directory,fast.FAST_InputFile),2); %FP are Fast Parameters, specify 2 lines of header (FAST 8)

[EDP, EDFile]   = GetFASTPar_Subfile(FP, 'EDFile', fast.FAST_directory, fast.FAST_directory);
[IWP, IWFile]   = GetFASTPar_Subfile(FP, 'InflowFile', fast.FAST_directory, fast.FAST_directory);
[ADP, ADFile]   = GetFASTPar_Subfile(FP, 'AeroFile', fast.FAST_directory, fast.FAST_directory);
[SvDP, SvDFile] = GetFASTPar_Subfile(FP, 'ServoFile', fast.FAST_directory, fast.FAST_directory);
[HDP, HDFile]   = GetFASTPar_Subfile(FP, 'HydroFile', fast.FAST_directory, fast.FAST_directory,true);
% [SbDP, SbDFile] = GetFASTPar_Subfile(FP, 'SubFile', fast.FAST_directory, fast.FAST_directory);
% [MDP, MDFile]   = GetFASTPar_Subfile(FP, 'MooringFile', fast.FAST_directory, fast.FAST_directory);
% [IDP, IDFile] = GetFASTPar_Subfile(FP, 'IceFile', fast.FAST_directory, fast.FAST_directory);

% Control Parameters
[~,SD_dllFile]          = GetFASTPar_Subfile(SvDP,'DLL_InFile',fast.FAST_directory, fast.FAST_directory);
SD_dllP                 = ROSCO2Matlab(SD_dllFile,2);
PerfFileName            = GetFASTPar(SD_dllP,'PerfFileName');

% Cp Surface
Cx                      = Pre_LoadRotPerf(fullfile(fast.FAST_directory,PerfFileName(2:end-1)));


%% Write Outputs

% parameters
P.FP        = FP;
P.EDP       = EDP;
P.IWP       = IWP;
P.ADP       = ADP;
P.HDP       = HDP;
P.SvDP      = SvDP;
P.SD_dllP   = SD_dllP;


