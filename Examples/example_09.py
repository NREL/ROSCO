'''
----------- Example_09 --------------
Run TurbSim to create wind field binary
-------------------------------------

In this example:
  - Leverage the run_openfast functionality to compile a turbsim binary
'''

# Python Modules
import numpy as np
import matplotlib.pyplot as plt 
# ROSCO toolbox modules 
from ROSCO_toolbox.utilities import run_openfast
import os

this_dir = os.path.dirname(os.path.abspath(__file__))

# Define openfast output filenames
wind_directory = os.path.join(this_dir,'../Test_Cases/Wind/')
turbsim_infile = '90m_12mps_twr.inp'

run_openfast(
  wind_directory, 
  fastcall='/Users/dzalkind/Tools/WEIS-1/build/temp.macosx-10.9-x86_64-3.8_rosco_openfast/modules/turbsim/turbsim', 
  fastfile=turbsim_infile, 
  chdir=False
  )

