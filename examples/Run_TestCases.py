# Run_TestCases
# Run the simulation cases in the Test_Cases folder
##
#################  OUTDATED #################
##
import os
from ROSCO_toolbox import utilities as wtc_Utilities
util = wtc_Utilities.FAST_IO()

# define call for OpenFAST
fastcall = 'openfast_dev'

# define folder names in Test_Cases to run
testcases = [
                '5MW_Step_Legacy',     # Below Rated
                '5MW_Step_Baseline',
                '5MW_BR_Legacy',     # Below Rated
                '5MW_BR_Baseline',
                '5MW_NR_Legacy',    # Near Rated
                '5MW_NR_Baseline',
                '5MW_AR_Legacy',    # Above Rated
                '5MW_AR_Baseline',
                '5MW_OC4_ARsteady_Legacy', # OC4 Hywind
                '5MW_OC4_ARsteady_Baseline', # OC4 Hywind
                '5MW_OC4_NR_Legacy', # OC4 Hywind
                '5MW_OC4_NR_Baseline', # OC4 Hywind
                '5MW_OC4_AR_Legacy', # OC4 Hywind
                '5MW_OC4_AR_Baseline', # OC4 Hywind
                '5MW_OC4_MH_Legacy', # OC4 Hywind
                '5MW_OC4_MH_Baseline', # OC4 Hywind
            ]

# Run test cases
for case  in testcases:
    OpenFAST_dir = os.path.join(os.getcwd(),'../Test_Cases',case)
    util.run_openfast(OpenFAST_dir,fastcall=fastcall,fastfile='test.fst')

