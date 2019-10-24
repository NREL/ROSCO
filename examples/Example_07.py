# Example_07
# Load gain schedules, write_paramter input file, plot gains

#%%
import numpy as np
from scipy import interpolate 
import yaml

from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim
import os

# Ensure propper working directory
os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/WTC_toolbox/examples')

# parameter filename
parameter_filename = 'NREL5MW.yaml'         # Name of .yaml input file for the specific turbine

# Load input file contents, put them in some dictionaries to keep things cleaner
inps = yaml.safe_load(open(parameter_filename))
path_params = inps['path_params']
turbine_params = inps['turbine_params']
controller_params = inps['controller_params']

# Initialiize turbine, controller, and file processing classes
turbine         = wtc_turbine.Turbine(turbine_params)
controller      = wtc_controller.Controller(controller_params)
file_processing = wtc_controller.FileProcessing()

# Fast input file and Cp surface text file
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'
txt_filename = 'Cp_Ct_Cq.txt'

turbine.load_from_fast(path_params['FAST_InputFile'],path_params['FAST_directory'],dev_branch=True,rot_source='txt',txt_filename=path_params['rotor_performance_filename'])

# Load controller
controller.tune_controller(turbine)

# Write parameter input file
param_file = 'DISCON_TEST.IN'
file_processing.write_param_file(param_file,turbine,controller,new_file=True)

# Plot
import matplotlib.pyplot as pl 

pl.figure(0)
pl.plot(controller.v[len(controller.vs_gain_schedule.Kp):], controller.pc_gain_schedule.Kp)
pl.xlabel('Wind Speed')
pl.ylabel('Proportional Gain')

pl.figure(1)
pl.plot(controller.v[len(controller.vs_gain_schedule.Ki):], controller.pc_gain_schedule.Ki)
pl.xlabel('Wind Speed')
pl.ylabel('Integral Gain')

pl.show()

