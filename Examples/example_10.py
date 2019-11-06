# example_10.py
# Plot OpenFAST output data

import numpy as np
import matplotlib.pyplot as plt 
import os
from ROSCO_toolbox import utilities as wtc_utilities

# Instantiate fast_IO
FAST_IO = wtc_utilities.FAST_IO()

# Define openfast output filenames
filenames = ["/Users/nabbas/Documents/TurbineModels/IEA-15-240-RWT/OpenFAST/IEA-15-240-RWT.out",
            "/Users/nabbas/Documents/TurbineModels/DTU_10MW/DTU10MWRWT/Baseline/DTU_10MW_RWT.out"]
            # "/Users/nabbas/Documents/TurbineModels/IEA-15-240-RWT/OpenFAST/IEA-15-240-RWT.outb"]

# Load output info and data
allinfo, alldata = FAST_IO.load_output(filenames)

#  Define Plot cases 
#       - Comment,uncomment, create, and change these as desired...
cases = {}
cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed', 'GenPwr']
cases['RotPerf'] = ['RtTSR', 'RtAeroCp']

# Plot, woohoo!
FAST_IO.plot_fast_out(cases, allinfo, alldata)