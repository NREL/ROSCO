'''
----------- Example_02 --------------
Load a turbine model from saved pickle, make a quick cp plot 
-------------------------------------

In this example:
  - Load a turbine from a saved pickle
  - Plot Cp Surface
'''

# Python modules
import os
import matplotlib.pyplot as plt
# ROSCO toolbox modules 
from ROSCO_toolbox import turbine as ROSCO_turbine

this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir,'examples_out')
if not os.path.isdir(example_out_dir):
  os.makedirs(example_out_dir)

# Initialize a turbine class -- Don't need to instantiate!
turbine = ROSCO_turbine.Turbine

# Load quick from python pickle
turbine = turbine.load(os.path.join(example_out_dir,'01_NREL5MW_saved.p'))

# plot rotor performance 
print('Plotting Cp data')
turbine.Cp.plot_performance()



if False:
  plt.show()
else:
  plt.savefig(os.path.join(example_out_dir,'02_NREL5MW_Cp.png'))