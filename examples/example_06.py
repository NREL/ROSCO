# Example_06
# Step wind simulation, and plot

from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim
from WTC_toolbox import control_interface as ci
import numpy as np
import matplotlib.pyplot as plt
import os

# ensure proper directory location 
# os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/WTC_toolbox/examples')

# Load turbine model
# Initialiize a turbine class
turbine = wtc_turbine.Turbine()

# Load turbine from OpenFAST and *.txt file
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'
txt_filename = 'Cp_Ct_Cq.txt'
drivetrain_inertia = 40469564.444
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source='txt', txt_filename=txt_filename)

# # Load turbine quick from python
# turbine.load('saved_turbine.p')

# Load controller 
lib_name = 'test_controller/DISCON.dll'
# lib_name = '/Users/pfleming/Desktop/git_tools/floating/DRC_Fortran/DISCON//DISCON_glin64.so'
controller_int = ci.ConInt(lib_name)

# Load the simulator
sim = wtc_sim.Sim(turbine,controller_int)

# Define a wind speed history
dt = 0.1
tlen = 400      # length of time to simulate (s)
ws0 = 9         # initial wind speed (m/s)
t= np.arange(0,tlen,dt) 
ws = np.ones_like(t) * ws0
# add steps at every 100s
for i in range(len(t)):
    ws[i] = ws[i] + t[i]//100


# Run simulator
sim.sim_ws_series(t,ws)
plt.show()

