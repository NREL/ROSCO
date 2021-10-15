'''
----------- Example_09 --------------
Run TurbSim to create wind field binary
-------------------------------------

In this example:
  - Leverage the run_openfast functionality to compile a turbsim binary
'''

# ROSCO toolbox modules 
from ROSCO_toolbox.utilities import run_openfast
import os

this_dir = os.path.dirname(os.path.abspath(__file__))

# Define openfast output filenames
wind_directory = os.path.join(this_dir,'../Test_Cases/Wind/')
turbsim_infile = '90m_12mps_twr.inp'

run_openfast(wind_directory, fastcall='turbsim',
             fastfile=os.path.join(wind_directory, turbsim_infile), chdir=False)

print('test')
