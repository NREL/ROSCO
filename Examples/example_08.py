# ----------- Example_08 --------------
# Plot some OpenFAST output data
# -------------------------------------
#
# In this example:
#   - Load openfast output data
#   - Plot some available channels

# Python Modules
import numpy as np
import matplotlib.pyplot as plt 
# ROSCO toolbox modules 
from ROSCO_toolbox import utilities as wtc_utilities

# Instantiate fast_IO
fast_io = wtc_utilities.FAST_IO()

# Define openfast output filenames
filenames = ["../Test_Cases/5MW_Land/5MW_Land.outb"]

# ---- Note: Could plot multiple cases, and binaries...
# filenames = ["../Test_Cases/5MW_Land/5MW_Land.out",
#              "../Test_Cases/5MW_Land/5MW_Land.outb"]

# Load output info and data
allinfo, alldata = fast_io.load_output(filenames)

#  Define Plot cases 
#  --- Comment,uncomment, create, and change these as desired...
cases = {}
cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed']
cases['Rotor'] = ['BldPitch1', 'GenTq', 'GenPwr']
cases['Rotor Performance'] = ['RtVAvgxh', 'RtTSR', 'RtAeroCp']

# Plot, woohoo!
fast_io.plot_fast_out(cases, allinfo, alldata)