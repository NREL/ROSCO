'''
----------- Example_05 --------------
Run and plot a simple simple step wind simulation
-------------------------------------

In this example:
  - Load turbine from saved pickle
  - Tune a controller
  - Run and plot a simple step wind simulation

Notes - You will need to have a compiled controller in ROSCO, and 
        properly point to it in the `lib_name` variable.
      - Using wind speed estimators in this simple simulation is 
        known to cause problems. We suggesting using WE_Mode = 0 in DISCON.IN 
        or increasing sampling rate of simulation
'''
# Python modules
import matplotlib.pyplot as plt 
import numpy as np
import os, platform
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox import control_interface as ROSCO_ci
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

# Specify controller dynamic library path and name
this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir,'examples_out')
if not os.path.isdir(example_out_dir):
  os.makedirs(example_out_dir)

if platform.system() == 'Windows':
    lib_name = os.path.join(this_dir, '../ROSCO/build/libdiscon.dll')
elif platform.system() == 'Darwin':
    lib_name = os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib')
else:
    lib_name = os.path.join(this_dir, '../ROSCO/build/libdiscon.so')

# # Load turbine model from saved pickle
turbine         = ROSCO_turbine.Turbine
turbine         = turbine.load(os.path.join(example_out_dir,'01_NREL5MW_saved.p'))
# controller      = ROSCO_controller.Controller(controller_params)

# Load turbine data from OpenFAST and rotor performance text file
turbine.load_from_fast(
    path_params['FAST_InputFile'],
    os.path.join(tune_dir,path_params['FAST_directory']),
    dev_branch=True,
    rot_source='txt',txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
    )

# Tune controller 
controller      = ROSCO_controller.Controller(controller_params)
controller.tune_controller(turbine)

# Write parameter input file
param_filename = os.path.join(this_dir,'DISCON.IN')
write_DISCON(turbine,controller,param_file=param_filename, txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename']))


# Load controller library
controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename,sim_name='sim1')

# Load the simulator
sim_1 = ROSCO_sim.Sim(turbine,controller_int)

# Define a wind speed history
dt = 0.025
tlen = 1000      # length of time to simulate (s)
ws0 = 7         # initial wind speed (m/s)
t= np.arange(0,tlen,dt) 
ws = np.ones_like(t) * ws0
# add steps at every 100s
for i in range(len(t)):
    ws[i] = ws[i] + t[i]//100

# Run simulator and plot results
sim_1.sim_ws_series(t,ws,rotor_rpm_init=4)

# Load controller library again to see if we deallocated properly
controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename,sim_name='sim_2')

# Run simulator again and plot results
sim_2 = ROSCO_sim.Sim(turbine,controller_int)
sim_2.sim_ws_series(t,ws,rotor_rpm_init=4)

# Check if simulations are equal
np.testing.assert_almost_equal(sim_1.gen_speed,sim_2.gen_speed)

if False:
  plt.show()
else:
  plt.savefig(os.path.join(example_out_dir,'05_NREL5MW_SimpSim.png'))

