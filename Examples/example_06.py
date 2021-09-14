'''
----------- Example_06 --------------
Load a turbine, tune a controller, run OpenFAST simulation 
-------------------------------------

In this example:
  - Load a turbine from OpenFAST
  - Tune a controller
  - Run an OpenFAST simulation

Note - you will need to have a compiled controller in ROSCO/build/ 
'''
# Python Modules
import yaml
import os
import numpy as np
import matplotlib.pyplot as plt
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.utilities import write_DISCON, run_openfast
from ROSCO_toolbox.inputs.validation import load_rosco_yaml


this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir,'examples_out')

# Load yaml file 
parameter_filename = os.path.join(os.path.dirname(this_dir), 'Tune_Cases', 'IEA15MW_MultiOmega.yaml') 
inps = load_rosco_yaml(parameter_filename)
path_params         = inps['path_params']
turbine_params      = inps['turbine_params']
controller_params   = inps['controller_params']

# Instantiate turbine, controller, and file processing classes
turbine         = ROSCO_turbine.Turbine(turbine_params)
controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(path_params['FAST_InputFile'], \
  os.path.join(this_dir,path_params['FAST_directory']), \
    dev_branch=True,rot_source='txt',\
      txt_filename=os.path.join(this_dir,path_params['FAST_directory'],path_params['rotor_performance_filename']))

# Tune controller 
controller.tune_controller(turbine)

# Now double Kp_float and check that it's passed through
Kp_float = -18
controller_params['Kp_float'] = Kp_float
controller_params['tune_Fl'] = False
controller      = ROSCO_controller.Controller(controller_params)
controller.tune_controller(turbine)
np.testing.assert_almost_equal(Kp_float,controller.Kp_float)

# Write parameter input file
param_file = os.path.join(this_dir,'DISCON.IN')   # This must be named DISCON.IN to be seen by the compiled controller binary. 
write_DISCON(turbine,controller,param_file=param_file, txt_filename=path_params['rotor_performance_filename'])

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

if False:
  plt.show()
else:
  plt.savefig(os.path.join(example_out_dir,'06_GainSched.png'))

# Run OpenFAST
# --- May need to change fastcall if you use a non-standard command to call openfast
fastcall = 'openfast'
run_openfast(
  os.path.join(this_dir,path_params['FAST_directory']),
  fastcall=fastcall, 
  fastfile=path_params['FAST_InputFile'], 
  chdir=True
  )




