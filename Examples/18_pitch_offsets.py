"""
18_pitch_offsets
----------------
Run openfast with ROSCO and pitch offset faults
Set up and run simulation with pitch offsets, check outputs
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing
import numpy as np


def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'18_PitchFaults')
    os.makedirs(run_dir,exist_ok=True)
    
    # Set DISCON input dynamically through yaml/dict
    controller_params = {}
    controller_params['PF_Mode'] = 1    # Set pitch fault mode to pitch offsets
    controller_params['DISCON'] = {}

    pitch2_offset = 1       # deg
    pitch3_offset = -2      # deg
    controller_params['DISCON']['PF_Offsets'] =  [0.,float(np.radians(pitch2_offset)),float(np.radians(pitch3_offset))]

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_opts    = {
        'U_start': [10],  # from 10 to 15 m/s
        'U_end': [15],
        'wind_dir': run_dir,
        'T_step': 50,   # step at 50 sec
        'T_Max': 100    # simulation is 100 sec
        }
    r.case_inputs = {}
    r.case_inputs[("ServoDyn","Ptch_Cntrl")] = {'vals':[1], 'group':0}  # Individual pitch control must be enabled in ServoDyn
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()


    # Check pitch offsets
    filenames = [os.path.join(run_dir,'IEA15MW/simp_step/base/IEA15MW_0.outb')]
    fast_out = output_processing.output_processing()

    # Load and plot
    fastout = fast_out.load_fast_out(filenames)
    offset_2 = fastout[0]['BldPitch2'] - fastout[0]['BldPitch1']
    offset_3 = fastout[0]['BldPitch3'] - fastout[0]['BldPitch1']

    # check that offset (min,max) is very close to prescribed values
    # Note that some OpenFAST configurations (e.g., fixed bottom) do not apply offet on
    # first timestep and this example may fail
    np.testing.assert_almost_equal(offset_2.max(),pitch2_offset,decimal=3)
    np.testing.assert_almost_equal(offset_2.min(),pitch2_offset,decimal=3)
    np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=3)
    np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=3)



if __name__=="__main__":
    main()
