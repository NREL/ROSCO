'''
----------- Example_18 ------------------------
Run openfast with ROSCO and pitch offset faults
-----------------------------------------------

Set up and run simulation with tower resonance avoidance

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.fast_io import output_processing
import numpy as np

rpm2RadSec = 2.0*(np.pi)/60.0


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))


def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'19_TRA_2')
    os.makedirs(run_dir,exist_ok=True)
    
    # Set DISCON input dynamically through yaml/dict
    controller_params = {}
    controller_params['Twr_Mode'] = 2    # Set pitch fault mode to pitch offsets
    controller_params['vs_minspd'] = 0.    # Reduce minimum rotor speed so that saturation does not interfere with exclusion
    
    controller_params['DISCON'] = {}
    controller_params['DISCON']['Twr_ExclSpeed'] = 5.5 * rpm2RadSec
    controller_params['DISCON']['Twr_ExclBand'] = 1 * rpm2RadSec

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.ramp_up_down  # single step wind input
    r.wind_case_opts    = {
        'U_start': 4,  # from 10 to 15 m/s
        'U_end': 12,
        'T_ramp': 600,
        }

    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()



if __name__=="__main__":
    main()