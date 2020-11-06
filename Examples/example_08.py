'''
----------- Example_08 --------------
Plot some OpenFAST output data
-------------------------------------

In this example:
  - Load openfast output data
  - Trim the time series
  - Plot some available channels

Note: need to run openfast model in '../Test_Cases/5MW_Land_DLL_WTurb/' to plot
'''

# Python Modules
import numpy as np
import matplotlib.pyplot as plt 
# ROSCO toolbox modules 
from ROSCO_toolbox import utilities as ROSCO_utilities

# Instantiate fast_IO
fast_io = ROSCO_utilities.FAST_IO()
fast_pl = ROSCO_utilities.FAST_Plots()

# Define openfast output filenames
filenames = ["../Test_Cases/NREL-5MW/NREL-5MW.outb"]

# ---- Note: Could plot multiple cases, textfiles, and binaries...
# filenames = ["../Test_Cases/NREL-5MW/NREL-5MW.outb",
#             "../Test_Cases/NREL-5MW/NREL-5MW_ex8.outb"]

# Load output info and data
fastout = fast_io.load_fast_out(filenames, tmin=10)


#  Define Plot cases 
#  --- Comment,uncomment, create, and change these as desired...
cases = {}
cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed']
cases['Rotor'] = ['BldPitch1', 'GenTq', 'GenPwr']

# Plot, woohoo!
fast_pl.plot_fast_out(cases, fastout, showplot=True)
