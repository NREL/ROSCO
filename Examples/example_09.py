# example_09.py
# Plot expected shaved rotor thrust 

#%%
# Python Modules
import yaml
import os
import matplotlib.pyplot as plt
# ROSCO Modules
from ROSCO_toolbox import turbine as wtc_turbine
from ROSCO_toolbox import controller as wtc_controller
from ROSCO_toolbox import sim as wtc_sim

os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/ROSCO_toolbox/Examples')

# Point to yaml and openfast file
parameter_filename = '../Tune_Cases/IEA15MW.yaml'
inps = yaml.safe_load(open(parameter_filename))
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']


# Initialize a turbine class
turbine = wtc_turbine.Turbine(turbine_params)
controller = wtc_controller.Controller(controller_params)

# Load turbine and tune controller
turbine.load_from_fast(path_params['FAST_InputFile'],path_params['FAST_directory'],dev_branch=True,rot_source='txt',txt_filename='../Tune_Cases/Cp_Ct_Cq.IEA15MW.txt')
controller.tune_controller(turbine)
#%%

plt.plot(controller.v, controller.pitch_op)
plt.plot(controller.v, controller.ps_min_bld_pitch)

plt.show()

#%%
