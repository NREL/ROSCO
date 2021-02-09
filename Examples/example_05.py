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
        known to cause problems. We suggesting using WE_Mode = 0 in DISCON.IN.
'''
# Python modules
import matplotlib.pyplot as plt 
import numpy as np
import yaml, os, platform
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox import control_interface as ROSCO_ci

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

param_filename = os.path.join(this_dir,'DISCON.IN')

# Load turbine model from saved pickle
turbine = ROSCO_turbine.Turbine
turbine = turbine.load(os.path.join(example_out_dir,'01_NREL5MW_saved.p'))

# Load controller library
controller_int = ROSCO_ci.ControllerInterface(lib_name,param_filename=param_filename)

# Load the simulator
sim = ROSCO_sim.Sim(turbine,controller_int)

# Define a wind speed history
dt = 0.1
tlen = 1000      # length of time to simulate (s)
ws0 = 7         # initial wind speed (m/s)
t= np.arange(0,tlen,dt) 
ws = np.ones_like(t) * ws0
# add steps at every 100s
for i in range(len(t)):
    ws[i] = ws[i] + t[i]//100

# Run simulator and plot results
sim.sim_ws_series(t,ws,rotor_rpm_init=4)

if False:
  plt.show()
else:
  plt.savefig(os.path.join(example_out_dir,'05_NREL5MW_SimpSim.png'))

