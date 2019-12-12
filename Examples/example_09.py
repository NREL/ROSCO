# ----------- Example_09 --------------
# Run TurbSim to create binary
# -------------------------------------
#
# In this example:
#   - Leverage the run_openfast functionality to compile a turbsim binary

# Python Modules
import numpy as np
import matplotlib.pyplot as plt 
# ROSCO toolbox modules 
from ROSCO_toolbox import utilities as wtc_utilities

# Instantiate fast_IO
fast_io = wtc_utilities.FAST_IO()

# Define openfast output filenames
wind_directory = '../Test_Cases/Wind/'
turbsim_infile = '90m_12mps.inp'

fast_io.run_openfast(wind_directory, fastcall='turbsim', fastfile=turbsim_infile, chdir=False)

