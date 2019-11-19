# Example_05
# make some test controller calls
# This example is not so useful and can perhaps be deleted when simulator example is working


from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import controller as wtc_controller
# from WTC_toolbox import sim as wtc_sim
from WTC_toolbox import control_interface as ci
import numpy as np

# Some useful constants
degRad = np.pi/180.
rpmRadSec = 2.0*(np.pi)/60.0

# Initialiize a controller interface class
lib_name = 'test_controller/DISCON_glin64.so'
# lib_name = '/Users/pfleming/Desktop/git_tools/floating/DRC_Fortran/DISCON//DISCON_glin64.so'
controller = ci.ConInt(lib_name)

GBRatio = 97.
t = 0
dt = 0.1
pitch=0
rotspeed = 10 * rpmRadSec
genspeed = GBRatio * rotspeed
ws = 8.0

for i in range(10):
    controller.show_control_values()
    controller.call_controller(t,dt,pitch,genspeed,rotspeed,ws)
