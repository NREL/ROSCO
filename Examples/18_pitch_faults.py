"""
18_pitch_offsets
----------------
Demosntrate two kinds of pitch faults using ROSCO:

1. Pitch offsets
^^^^^^^^^^^^^^^^^^^
When ``PF_Mode`` is set to 1, the pitch controller will apply a constant offset to the pitch angles of the blades.
The offsets are set in the ``PF_Offsets`` array in the DISCON file.

.. image:: ../images/examples/18_pitch_offsets.png


2. Stuck pitch actuator
^^^^^^^^^^^^^^^^^^^^^^^^^

When ``PF_Mode`` is set to 2, the pitch actuator will become stuck at its current position at time ``PF_TimeStuck``.

.. image:: ../images/examples/18_pitch_stuck.png


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
    run_dir = os.path.join(example_out_dir,'18_PitchOffset')
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
        'TStep': 5,   # step at 5 sec
        'TMax': 10    # simulation is 10 sec
        }
    r.case_inputs = {}
    r.case_inputs[("ServoDyn","Ptch_Cntrl")] = {'vals':[1], 'group':0}  # Individual pitch control must be enabled in ServoDyn
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()


    # Check pitch offsets
    filenames = [os.path.join(run_dir,'IEA15MW_0.outb')]
    fast_out = output_processing.output_processing()

    # Load and plot
    fastout = fast_out.load_fast_out(filenames)
    offset_2 = fastout[0]['BldPitch2'] - fastout[0]['BldPitch1']
    offset_3 = fastout[0]['BldPitch3'] - fastout[0]['BldPitch1']

    # check that offset (min,max) is very close to prescribed values
    # Note that some OpenFAST configurations (e.g., fixed bottom) do not apply offet on
    # first timestep and this example may fail
    np.testing.assert_almost_equal(offset_2.max(),pitch2_offset,decimal=2)
    np.testing.assert_almost_equal(offset_2.min(),pitch2_offset,decimal=2)
    np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=2)
    np.testing.assert_almost_equal(offset_3.max(),pitch3_offset,decimal=2)


    # Stuck pitch actuator:

    time_stuck = 7.5

    controller_params['PF_Mode'] = 2   # Stuck pitch actuator
    controller_params['DISCON']['PF_TimeStuck'] = [time_stuck,time_stuck+1,time_stuck+2]   # time at which the actuator becomes stuck
    run_dir = os.path.join(example_out_dir,'18_PitchStuck')
    r.save_dir      = run_dir
    r.run_FAST()

    # Check pitch stays the same
    filenames = [os.path.join(run_dir,'IEA15MW_0.outb')]
    fast_out2 = output_processing.output_processing()

    # Load output and check that the pitch angle is constant after time_stuck (last value = value at time_stuck)
    fastout = fast_out2.load_fast_out(filenames)
    ind_stuck = fastout[0]['Time'] == time_stuck
    np.testing.assert_almost_equal(fastout[0]['BldPitch1'][ind_stuck],fastout[0]['BldPitch1'][-1])
    fastout[0]['BldPitch1'][ind_stuck] == fastout[0]['BldPitch1'][-1]



if __name__=="__main__":
    main()
