'''
----------- Example_04 --------------
Load a turbine model and tune the controller
-------------------------------------

In this example:
  - Read a .yaml file
  - Load a turbine model from OpenFAST
  - Tune a controller
  - Write a controller input file
  - Plot gain schedule
'''
# Python modules
import matplotlib.pyplot as plt 
import yaml, os 
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox.utilities import write_DISCON

# Load yaml file 
parameter_filename = os.path.join(os.path.dirname(__file__),'NREL5MW_example.yaml')
inps = yaml.safe_load(open(parameter_filename))
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Instantiate turbine, controller, and file processing classes
turbine         = ROSCO_turbine.Turbine(turbine_params)
controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(
    path_params['FAST_InputFile'],
    os.path.join(os.path.dirname(__file__),path_params['FAST_directory']),
    dev_branch=True,
    rot_source='txt',txt_filename=os.path.join(os.path.dirname(__file__),path_params['rotor_performance_filename'])
    )

# Tune controller 
controller.tune_controller(turbine)

# Write parameter input file
param_file = 'DISCON.IN'   
write_DISCON(turbine,controller,param_file=param_file, txt_filename=path_params['rotor_performance_filename'])

# Plot gain schedule
fig, ax = plt.subplots(1,2,constrained_layout=True)
ax[0].plot(controller.v[len(controller.vs_gain_schedule.Kp):], controller.pc_gain_schedule.Kp)
ax[0].set_xlabel('Wind Speed')
ax[0].set_ylabel('Proportional Gain')

ax[1].plot(controller.v[len(controller.vs_gain_schedule.Ki):], controller.pc_gain_schedule.Ki)
ax[1].set_xlabel('Wind Speed')
ax[1].set_ylabel('Integral Gain')

plt.suptitle('Pitch Controller Gains')
plt.show()