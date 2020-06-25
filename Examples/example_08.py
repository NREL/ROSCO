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
fast_plots = ROSCO_utilities.FAST_Plots()

# Define openfast output filenames
# filenames = ["../Test_Cases/5MW_Land/5MW_Land.outb"]

# ---- Note: Could plot multiple cases, textfiles, and binaries...
filenames = ["/Users/dzalkind/Tools/WISDEM/temp/OpenFAST/testing_0.outb"]

# Load output info and data
alldata = fast_io.load_FAST_out(filenames)

# Trim time series
for i,(data) in enumerate(alldata):
    alldata[i] = fast_io.trim_output(data, tmin=0, tmax=630)

#  Define Plot cases 
#  --- Comment,uncomment, create, and change these as desired...
cases = {}
cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'GenSpeed','GenPwr','RootMyb1','TwrBsMyt','PtfmPitch']
cases['Platform'] = ['PtfmHeave','PtfmPitch','PtfmRoll','PtfmSurge','PtfmSway','PtfmYaw']
# cases['Rotor Performance'] = ['RtVAvgxh', 'RtTSR', 'RtAeroCp']

# Plot, woohoo!
fast_plots.plot_fast_out(cases, alldata,showplot=True)
