# Example_01
# Load a turbine model
# In this example load a turbine model from FAST using AeroelasticSE
# Use the NREL baseline included with open FAST
#
# Some notes on dependencies
# https://github.com/WISDEM/AeroelasticSE
#  https://github.com/OpenFAST/openfast
# Be sure to clone the repo with the --recursive flag or execute git submodule update --init --recursive after cloning.

from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import sim as wtc_sim

# PARAMETERS

# (USES AERODYN 15, a problem for now)
# FAST_InputFile = '5MW_OC3Spar_DLL_WTurb_WavesIrr.fst'
# FAST_directory = '/Users/pfleming/Desktop/git_tools/floating/OpenFAST/reg_tests/r-test/glue-codes/openfast/5MW_OC3Spar_DLL_WTurb_WavesIrr'

FAST_InputFile = '5MW_ITIBarge_DLL_WTurb_WavesIrr.fst'
FAST_directory = 'example_fast_input/5MW_ITIBarge_DLL_WTurb_WavesIrr'



# Initialiize a turbine class
turbine = wtc_turbine.Turbine()

# Load the turbine model from a FAST input folder
turbine.load_from_fast(FAST_InputFile,FAST_directory,dev_branch=True)