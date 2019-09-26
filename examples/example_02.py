# Example_04
# Load a turbine model from saved pickle, make a quick cp plot


from WTC_toolbox import turbine as wtc_turbine
from WTC_toolbox import controller as wtc_controller
from WTC_toolbox import sim as wtc_sim
import numpy as np
import matplotlib.pyplot as plt

# Initialiize a turbine class
turbine = wtc_turbine.Turbine()

# Load quick from python
# turbine.load('saved_turbine.p')

# ## Or Load the turbine model from a FAST input folder
FAST_InputFile = '5MW_Land.fst'
FAST_directory = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Land'
txt_filename = 'Cp_Ct_Cq.txt'
drivetrain_inertia = 40469564.444
turbine.load_from_fast(FAST_InputFile,FAST_directory,drivetrain_inertia=drivetrain_inertia,dev_branch=True,rot_source=None,txt_filename='Cp_Ct_Cq.txt')

# Sweep TSR and fix pitch in two positions
fixed_rpm = 10. # RPM
fixed_pitch = 0.

tsr = np.arange(1,15,0.1)


cp_0 = np.array([turbine.cp_interp(t,0) for t in tsr])
cp_3 = np.array([turbine.cp_interp(t,3) for t in tsr])

fig, ax = plt.subplots()
ax.plot(tsr,cp_0,label='Pitch=0')
ax.plot(tsr,cp_3,label='Pitch=3')
ax.set_xlabel('TSR')
ax.set_ylabel('Cp')
ax.grid(True)
ax.legend()
plt.show()