# example_09.py
# Include peak shaving

%##
import numpy as np
from scipy import interpolate 

from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim
import os

os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/WTC_toolbox/examples')

# Initialiize turbine and controller classes
turbine = wtc_turbine.Turbine()
controller = wtc_controller.Controller()
file_processing = wtc_controller.FileProcessing()

# Fast input file and Cp surface text file
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'
txt_filename = 'Cp_Ct_Cq.txt'

drivetrain_inertia = 40469564.444
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source='txt', txt_filename=txt_filename)

# Load controller
controller.tune_controller(turbine)

turbine.Ct.interp_surface