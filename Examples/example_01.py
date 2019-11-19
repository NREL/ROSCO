# ----------- Example_01 --------------
# Load and save a turbine model
# -------------------------------------
# In this example:
# - Read .yaml input file
# - Load an openfast turbine model
# - Run CCBlade to find the rotor performance properties
# - Print some basic turbine properties
# - Save the turbine as a picklle
# 
# Note: Uses the NREL 5MW included in the Test Cases and is a part of the OpenFAST distribution

# Python Modules
import yaml
import os
# ROSCO Modules
from ROSCO_toolbox import turbine as wtc_turbine
from ROSCO_toolbox import controller as wtc_controller
from ROSCO_toolbox import sim as wtc_sim

# Point to yaml and openfast file
parameter_filename = '../Tune_Cases/NREL5MW.yaml'
inps = yaml.safe_load(open(parameter_filename))
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Load turbine data from openfast model
turbine = wtc_turbine.Turbine(turbine_params)
turbine.load_from_fast(path_params['FAST_InputFile'],path_params['FAST_directory'],dev_branch=True,rot_source='txt',txt_filename=path_params['rotor_performance_filename'])

# Print some basic turbine info
print(turbine)

# Save the turbine model
turbine.save('saved_turbine.p')