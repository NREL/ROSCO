% Post_TestCases.m
% Script to load and plot data from test cases in the WTC_Toolbox

%% Define and load test cases

% Define test case names (lan
testcases = {   '5MW_Step_Legacy';...
                '5MW_Step_Baseline';...
                '5MW_BR_Legacy';...
                '5MW_BR_Baseline';...
                '5MW_NR_Legacy';...   
                '5MW_NR_Baseline';...
                '5MW_AR_Legacy';...   
                '5MW_AR_Baseline';...
                '5MW_OC4_ARsteady_Legacy';...
                '5MW_OC4_ARsteady_Baseline';...
                };
        
   
% Load OpenFAST output data
for i = 1:length(testcases)
    % Define filepaths
    fastdir = ['../Test_Cases/',testcases{i}];
    infilename = dir([fastdir filesep '*.fst']);
    outfilename = [infilename.name(1:end-3) 'out'];
    outfile = [fastdir filesep,outfilename];

    % Load to data structure
    if strcmp(testcases{i}(1:3),'5MW')
        fo.(testcases{i}(5:end)) = Post_LoadFastOut(outfile);
    else
        fo.(testcases{i}) = Post_LoadFastOut(outfile);
    end
end
 
 
 %% Plot Data
 % Will want to (un)comment desired cases to plot

% Below Rated
% Pl_FastPlots(fo.Step_Legacy, fo.Step_Baseline)
 
% Below Rated
% Pl_FastPlots(fo.BR_Legacy, fo.BR_Baseline)

% Near Rated
% Pl_FastPlots(fo.NR_Legacy, fo.NR_Baseline)

% % Above Rated
% Pl_FastPlots(fo.AR_Legacy, fo.AR_Baseline)
 
% % Floating, Above Rated steady
Pl_FastPlots(fo.OC4_ARsteady_Legacy, fo.OC4_ARsteady_Baseline)
 
 
 
 