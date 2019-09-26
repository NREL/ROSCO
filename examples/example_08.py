# Example_08
# Load gain schedule, write parameter file, run test OpenFAST simulation, 

#%%
import numpy as np
from scipy import interpolate 
from shutil import copyfile
from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim
import os

# os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/WTC_toolbox/examples')

# Initialiize turbine and controller classes
turbine = wtc_turbine.Turbine()
controller = wtc_controller.Controller()
file_processing = wtc_controller.FileProcessing()

# Initialize Filenames and Directories
os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/WTC_toolbox/examples')
FAST_InputFile = '5MW_Land.fst'
FAST_directory = os.path.join(os.getcwd(),'../Test_Cases/5MW_Land')
txt_filename = 'Cp_Ct_Cq.txt'
DISCON_source = os.path.join(os.getcwd(),'../DRC_Fortran/DISCON/DISCON_glin64.so')
DISCON_dest = os.path.join(os.getcwd(),'../Test_Cases/5MW_Baseline/DISCON/DISCON_glin64.so')

# Copy controller from DRC_Fortran directory if necessary
copyfile(DISCON_source,DISCON_dest)

# Load turbine and tune controller
drivetrain_inertia = 40469564.444
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source='txt', txt_filename=txt_filename)
controller.tune_controller(turbine)

# Write parameter input file
param_file_source = os.path.join(FAST_directory,'DISCON.IN')
file_processing.write_param_file(param_file_source,turbine,controller,new_file=True)

# Run OpenFAST
# --- May need to be changed for specific user's call for OpenFAST
os.system('openfast_dev %s' % os.path.join(FAST_directory,FAST_InputFile))