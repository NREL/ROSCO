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
import os 
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.utilities import write_DISCON
from ROSCO_toolbox.inputs.validation import load_rosco_yaml


# Load yaml file 
this_dir = os.path.dirname(os.path.abspath(__file__))
tune_dir =  os.path.join(this_dir,'../Tune_Cases')
parameter_filename = os.path.join(tune_dir,'NREL5MW.yaml')
inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Instantiate turbine, controller, and file processing classes
turbine         = ROSCO_turbine.Turbine(turbine_params)
controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(
    path_params['FAST_InputFile'],
    os.path.join(tune_dir,path_params['FAST_directory']),
    dev_branch=True,
    rot_source='txt',txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
    )

# Tune controller 
controller.tune_controller(turbine)

# Write parameter input file
param_file = os.path.join(this_dir,'DISCON.IN')
write_DISCON(turbine,controller,param_file=param_file, txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename']))

# Plot gain schedule
fig, ax = plt.subplots(2,2,constrained_layout=True,sharex=True)
ax = ax.flatten()
ax[0].plot(controller.v[len(controller.v_below_rated)+1:], controller.omega_pc_U)
ax[0].set_ylabel('omega_pc')

ax[1].plot(controller.v[len(controller.v_below_rated)+1:], controller.zeta_pc_U)
ax[1].set_ylabel('zeta_pc')

ax[2].plot(controller.v[len(controller.v_below_rated)+1:], controller.pc_gain_schedule.Kp)
ax[2].set_xlabel('Wind Speed')
ax[2].set_ylabel('Proportional Gain')

ax[3].plot(controller.v[len(controller.v_below_rated)+1:], controller.pc_gain_schedule.Ki)
ax[3].set_xlabel('Wind Speed')
ax[3].set_ylabel('Integral Gain')

plt.suptitle('Pitch Controller Gains')

example_out_dir = os.path.join(this_dir,'examples_out')
if not os.path.isdir(example_out_dir):
  os.makedirs(example_out_dir)

if False:
  plt.show()
else:
  plt.savefig(os.path.join(example_out_dir,'04_GainSched.png'))