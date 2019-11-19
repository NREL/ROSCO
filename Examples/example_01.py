# ----------- Example_01 --------------
# Load a turbine model
# In this example load a turbine model from FAST using AeroelasticSE. CCBlade is also run to find the rotor performance properties
# Use the NREL baseline included with open FAST

# Python Modules
import yaml
import os
# ROSCO Modules
from ROSCO_toolbox import turbine as wtc_turbine
from ROSCO_toolbox import controller as wtc_controller
from ROSCO_toolbox import sim as wtc_sim

os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/ROSCO_toolbox/Examples')
# Point to openfast files
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '../Test_Cases/5MW_Land'

# Point to yaml and openfast file
parameter_filename = '../Tune_Cases/NREL5MW.yaml'
inps = yaml.safe_load(open(parameter_filename))
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']


# Initialize turbine and controller classes
turbine = wtc_turbine.Turbine(turbine_params)
turbine.load_from_fast(path_params['FAST_InputFile'],path_params['FAST_directory'],dev_branch=True,rot_source='txt',txt_filename=path_params['rotor_performance_filename'])

# Load the turbine model from a FAST input folder
turbine.load_from_fast(FAST_InputFile,FAST_directory, FAST_ver='OpenFAST',dev_branch=True,rot_source='txt', txt_filename='Cp_Ct_Cq.txt')
# Display a little about the turbine
print(turbine)

# Save the turbine model
turbine.save('saved_turbine.p')