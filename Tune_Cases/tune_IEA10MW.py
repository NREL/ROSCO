# IEA WindTask 37 - 10MW
# Example_07
# Load gain schedules, write_paramter input file, plot gains

#%%
import numpy as np
from scipy import interpolate 
import matplotlib.pyplot as plt 

from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim

import os

os.chdir('/Users/nabbas/Documents/WindEnergyToolbox/WTC_toolbox/Tune_Cases')

# Initialiize turbine and controller classes
turbine = wtc_turbine.Turbine()
controller = wtc_controller.Controller()
file_processing = wtc_controller.FileProcessing()

# Fast input file and Cp surface text file
FAST_InputFile = 'RotorSE_FAST_IEA37_offshore_RWT.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/IEA-10.0-198-RWT/openfast'
# txt_filename = 'Cp_Ct_Cq.txt'

# FAST_InputFile = '5MW_Land.fst'
# FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'
txt_filename = 'Cp_Ct_Cq.txt'

rotor_inertia = 176658470.56  # Available from Elastodyn I/O
generator_inertia = 1500.5
gb_ratio = 1
drivetrain_inertia = rotor_inertia + generator_inertia* gb_ratio**2

# drivetrain_inertia = 40469564.444

# turbine.load('NREL15MW_turbine.p')

# Load Turbine
# turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source='txt',txt_filename=txt_filename)
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source=None,txt_filename=txt_filename)
# Tune controller
controller.tune_controller(turbine)
# Save turbine
# turbine.save('NREL15MW_turbine.p')


#%%
turbine.write_rotorperformance(txt_filename='Cp_Ct_Cq.txt')

# Write parameter input file
param_file = 'DISCON.IN'
file_processing.write_param_file(param_file,turbine,controller,new_file=True)

#%% Load Saved Turbine
# turbine.load('NREL15MW_turbine.p')

#%% Tiny sim
from WTC_toolbox import control_interface as ci

# Load controller 
lib_name = '../DRC_Fortran/DISCON/DISCON_glin64.so'
# lib_name = '/Users/pfleming/Desktop/git_tools/floating/DRC_Fortran/DISCON//DISCON_glin64.so'
controller_int = ci.ConInt(lib_name)

# Load the simulator
sim = wtc_sim.Sim(turbine,controller_int)

# Define a wind speed history
dt = 0.1
tlen = 500      # length of time to simulate (s)
ws0 = 8      # initial wind speed (m/s)
t= np.arange(0,tlen,dt) 
ws = np.ones_like(t) * ws0
# add steps at every 100s
for i in range(len(t)):
    ws[i] = ws[i] + t[i]//100


# Run simulator
sim.sim_ws_series(t,ws,rotor_rpm_init=turbine.RRspeed)
plt.show()

# #%%
# plt.figure(2)
# plt.plot(controller.pitch_op_pc, controller.pc_gain_schedule.Kp)
# plt.show()
# print(controller.v_rated)
# print(controller.pc_gain_schedule)