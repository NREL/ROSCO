% Post_TestCases.m
% Script to load and plot data from test cases in the ROSCO_Toolbox

%% Define and load test cases

% Define test case names (lan
testcases = {   '5MW_Baseline';...
                '5MW_Land';...
                '5MW_OC4_NR';...
                '5MW_Ramp';...
                '5MW_Step';...
                '5MW_Turb_NR';...
                '5MW_Turb_NR_ps'
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

 % Usable Plot types: 
 %  - Step, Below Rated, Near Rated, Above Rated, Floating Steady, Floating Near Rated, Floating Mexican Hat, 'Floating Above Rated
plottype = 'Floating Steady';


close all
% Below Rated
switch plottype
    case 'Step'
        Pl_FastPlots(fo.Step_Legacy, fo.Step_Baseline)
    case 'Below Rated'
        Pl_FastPlots(fo.BR_Legacy, fo.BR_Baseline)
    case 'Near Rated'
        Pl_FastPlots(fo.NR_Legacy, fo.NR_Baseline)
    case 'Above Rated'
        Pl_FastPlots(fo.AR_Legacy, fo.AR_Baseline)
    case 'Floating Steady'
        Pl_FastPlots(fo.OC4_ARsteady_Legacy, fo.OC4_ARsteady_Baseline)
    case 'Floating Near Rated'
        Pl_FastPlots_present(fo.OC4_NR_Legacy, fo.OC4_NR_Baseline)
    case 'Floating Above Rated'
        Pl_FastPlots_present(fo.OC4_AR_Legacy, fo.OC4_AR_Baseline)
    case 'Floating Mexican Hat'
        Pl_FastPlots(fo.OC4_MH_Legacy, fo.OC4_MH_Baseline)
end
 
 
 
 