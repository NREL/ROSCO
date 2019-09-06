# Controller Tuning Example
# Load ccblade array outputs, run controller tuning script, make sure it all looks good. 

#%%
import numpy as np
from scipy import interpolate 

from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim

# Initialiize a turbine class

turbine = wtc_turbine.Turbine()

# Fast input file 
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'

# Load quick from python
# turbine.load('examples/saved_turbine.p')

txt_filename = 'examples/Cp_Ct_Cq.txt'

drivetrain_inertia = 40469564.444
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source='txt', txt_filename=txt_filename)


#%% Run controller
from WTC_toolbox import controller as wtc_controller
controller = wtc_controller.Controller()

controller.turbine_params


#%%
