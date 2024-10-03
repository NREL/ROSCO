"""
26_marine_hydro
---------------
Run MHK turbine in OpenFAST with ROSCO torque controller
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#from rosco.toolbox.ofTools.fast_io import output_processing
#from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
#from rosco.toolbox.inputs.validation import load_rosco_yaml
#import matplotlib.pyplot as plt
#from rosco.toolbox.controller import OpenLoopControl

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/RM1_MHK.yaml')
    run_dir = os.path.join(example_out_dir,'26_MHK/0_baseline')
    os.makedirs(run_dir,exist_ok=True)


    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    # r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [2.5],
        'TMax': 100.0,
        }
    r.case_inputs = {}
    # r.fst_vt        = reader.fst_vt
    # r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()

    print('here')


    # op = output_processing.output_processing()
    # op2 = output_processing.output_processing()

    # md_out = op.load_fast_out([os.path.join(run_dir,'IEA15MW_cable/power_curve/base/IEA15MW_cable_0.MD.Line1.out')], tmin=0)
    # local_vars = op2.load_fast_out([os.path.join(run_dir,'IEA15MW_cable/power_curve/base/IEA15MW_cable_0.RO.dbg2')], tmin=0)

    # fig, axs = plt.subplots(4,1)
    # axs[0].plot(local_vars[0]['Time'],local_vars[0]['CC_DesiredL'],label='CC_DesiredL')
    # axs[1].plot(local_vars[0]['Time'],local_vars[0]['CC_ActuatedL'],label='CC_ActuatedL')
    # axs[2].plot(local_vars[0]['Time'],local_vars[0]['CC_ActuatedDL'],label='CC_ActuatedDL')
    # axs[3].plot(md_out[0]['Time'],md_out[0]['Seg20Lst'],label='Seg20Lst')

    # [a.legend() for a in axs]
    # [a.grid() for a in axs]

    # if False:
    #     plt.show()
    # else:
    #     plt.savefig(os.path.join(example_out_dir,'22_cable_control.png'))

    # Check that the last segment of line 1 shrinks by 10 m
    # np.testing.assert_almost_equal(md_out[0]['Seg20Lst'][-1] - md_out[0]['Seg20Lst'][0], line_ends[0], 2)



if __name__=="__main__":
    main()
