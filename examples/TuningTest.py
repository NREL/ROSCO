# Controller Tuning Example
# Load gain schedules, plot blade pitch proportional gain

import numpy as np
from scipy import interpolate 

from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import sim as wtc_sim

# Initialiize turbine and controller classes
turbine = wtc_turbine.Turbine()
controller = wtc_controller.Controller()

# Fast input file and Cp surface text file
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'
txt_filename = 'examples/Cp_Ct_Cq.txt'

drivetrain_inertia = 40469564.444
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia,dev_branch=True,rot_source='txt', txt_filename=txt_filename)

# Load controller
controller.tune_controller(turbine)


# Plot
import matplotlib.pyplot as pl 

pl.plot(controller.v[len(controller.vs_gain_schedule.Kp):], controller.pc_gain_schedule.Kp)
pl.xlabel('Wind Speed')
pl.ylabel('Proportional Gain')
