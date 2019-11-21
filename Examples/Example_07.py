# ----------- Example_07 --------------
# Load saved turbine, tune controller, plot minimum pitch schedule
# -------------------------------------
#
# In this example:
#   - Load a yaml file
#   - Load a turbien from openfast
#   - Tune a controller
#   - Plot minimum pitch schedule


# Python modules
import matplotlib.pyplot as plt 
import yaml 
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as wtc_controller
from ROSCO_toolbox import turbine as wtc_turbine
from ROSCO_toolbox import sim as wtc_sim
from ROSCO_toolbox import utilities as wtc_utilities

# Load yaml file 
parameter_filename = 'NREL5MW_example.yaml'
inps = yaml.safe_load(open(parameter_filename))
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Ensure minimum generator speed at 50 rpm (for example's sake), turn on peak shaving and cp-maximizing min pitch
controller_params['vs_minspd'] = 50
controller_params['PS_Mode'] = 3

# Instantiate turbine, controller, and file processing classes
turbine         = wtc_turbine.Turbine(turbine_params)
controller      = wtc_controller.Controller(controller_params)
file_processing = wtc_utilities.FileProcessing()

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(path_params['FAST_InputFile'],path_params['FAST_directory'],dev_branch=True,rot_source='txt',txt_filename=path_params['rotor_performance_filename'])

# Tune controller 
controller.tune_controller(turbine)

# Plot minimum pitch schedule
plt.plot(controller.v, controller.pitch_op,label='Steady State Operation')
plt.plot(controller.v, controller.ps_min_bld_pitch, label='Minimum Pitch Schedule')
plt.legend()
plt.xlabel('Wind speed (m/s)')
plt.ylabel('Blade pitch (rad)')
plt.show()
