'''
----------- 21_optional_inputse_discon_version -----------------
Test and demonstrate update_discon_version() function for converting an old ROSCO input
to the current version
'''

import os, platform
from ROSCO_toolbox import control_interface as ROSCO_ci
from ROSCO_toolbox import sim as ROSCO_sim
from ROSCO_toolbox import turbine as ROSCO_turbine
import numpy as np


this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)

example_out_dir     = os.path.join(this_dir,'examples_out')
example_in_dir      = os.path.join(this_dir,'example_inputs')


def main():

    # Set up rosco_dll
    if platform.system() == 'Windows':
        rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.dll')
    elif platform.system() == 'Darwin':
        rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.dylib')
    else:
        rosco_dll = os.path.join(rosco_dir, 'ROSCO/build/libdiscon.so')

    # Load turbine model from saved pickle
    turbine         = ROSCO_turbine.Turbine
    turbine         = turbine.load(os.path.join(example_out_dir,'01_NREL5MW_saved.p'))
        
    # Not an extensive list, but can add if issues arise
    param_filenames = [
        os.path.join(example_in_dir, 'minimal_DISCON.IN'),        # pass
        os.path.join(example_in_dir, 'minimal_DISCON_err.IN'),    # fail
    ]

    avi_fail = []
    for param_filename in param_filenames:
        controller_int = ROSCO_ci.ControllerInterface(rosco_dll,param_filename=param_filename,sim_name='sim1')
        controller_int.kill_discon()
        avi_fail.append(controller_int.aviFAIL.value)

    # Check whether controller call failed
    assert avi_fail == [0,-1], "Unexpected pass/fail"

if __name__ == "__main__":
    main()