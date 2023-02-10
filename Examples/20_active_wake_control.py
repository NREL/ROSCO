'''
----------- 20_active_wake_control ------------
Run openfast with ROSCO and active wake control
-----------------------------------------------

Set up and run simulation with AWC, check outputs

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.utilities import read_DISCON, DISCON_dict
import numpy as np


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
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/NREL2p8.yaml')  # will be dummy and overwritten with SNL DISCON params
    run_dir = os.path.join(example_out_dir,'20_active_wake_control/setup_4')
    os.makedirs(run_dir,exist_ok=True)

    # Read all DISCON inputs
    rosco_vt = read_DISCON(os.path.join(rosco_dir,'TestCases','NREL_2p8_127/NREL-2p8-127_DISCON.IN'))

    # Could change discon parameters here


    # Apply all discon variables as case inputs
    control_base_case = {}
    for discon_input in rosco_vt:
        control_base_case[('DISCON_in',discon_input)] = {'vals': [rosco_vt[discon_input]], 'group': 0}
    

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve  # single step wind input
    r.wind_case_opts    = {
        'U': [6],  # from 10 to 15 m/s
        'TMax': 100,
        }
    r.case_inputs = control_base_case
    r.case_inputs[("ServoDyn","Ptch_Cntrl")] = {'vals':[1], 'group':0}  # Individual pitch control must be enabled in ServoDyn
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()

    # # Check AWC here
    # filenames = [os.path.join(run_dir,'IEA15MW/simp_step/base/IEA15MW_0.outb')]
    # fast_out = output_processing.output_processing()

    # # Load and plot
    # fastout = fast_out.load_fast_out(filenames)
    # offset_2 = fastout[0]['BldPitch2'] - fastout[0]['BldPitch1']
    # offset_3 = fastout[0]['BldPitch3'] - fastout[0]['BldPitch1']

    # # check that offset (min,max) is very close to prescribed values
    # np.testing.assert_almost_equal(offset_2.max(),pitch2_offset,decimal=3)
    # np.testing.assert_almost_equal(offset_2.min(),pitch2_offset,decimal=3)
    # np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=3)
    # np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=3)



if __name__=="__main__":
    main()