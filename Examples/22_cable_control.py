"""
22_cable_control
----------------
Run openfast with ROSCO and cable control
Set up and run simulation with pitch offsets, check outputs

ROSCO currently supports user-defined hooks for cable control actuation, if CC_Mode = 1.
The control logic can be determined in Controllers.f90 with the CableControl subroutine.
The CableControl subroutine takes an array of CC_DesiredL (length) equal to the ChannelIDs set in MoorDyn and
determines the length and change in length needed for MoorDyn using a 2nd order actuator model (CC\_ActTau). 
In the DISCON input, users must specify CC\_GroupIndex relating to the deltaL of each control ChannelID.  
These indices can be found in the ServoDyn summary file (\*SrvD.sum)

In the example below (and hard-coded in ROSCO) a step change of -10 m on line 1 is applied at 50 sec.
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing
import numpy as np
from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
from rosco.toolbox.inputs.validation import load_rosco_yaml
import matplotlib.pyplot as plt
from rosco.toolbox.controller import OpenLoopControl

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW_cable.yaml')
    run_dir = os.path.join(example_out_dir,'22_cable_control')
    os.makedirs(run_dir,exist_ok=True)

    # Read initial input file
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']

    # Change inputs programatically, read first
    reader = InputReader_OpenFAST()
    reader.FAST_InputFile = path_params['FAST_InputFile']
    reader.FAST_directory = os.path.join(this_dir,'Tune_Cases',path_params['FAST_directory'])
    reader.execute()

    # Set control line mapping (ChannelID -> Line(s))
    reader.fst_vt['MoorDyn']['ChannelID'] = [1, 2, 3]
    reader.fst_vt['MoorDyn']['Lines_Control'] = [['1'], ['2'], ['3']]

    # Make segments longer
    reader.fst_vt['MoorDyn']['NumSegs'] = [20,20,20]

    # Outputs
    reader.fst_vt['MoorDyn']['Outputs'] = ['l', 'l', 'l']

    # Set up ServoDyn for cable control
    reader.fst_vt['ServoDyn']['CCmode'] = 5

    # Set heading
    heading = 40 # deg
    reader.fst_vt['ServoDyn']['YawNeut'] = heading
    reader.fst_vt['ElastoDyn']['NacYaw'] = heading
    reader.fst_vt['InflowWind']['PropagationDir'] = -heading

    t_trans = 100
    t_sigma = 20
    t_max = 200

    line_ends = [-14.51, 1.58, 10.33]

    olc = OpenLoopControl(t_max=t_max)
    olc.interp_timeseries(
        'cable_control_1', 
        [0,t_trans,t_trans+t_sigma], 
        [0,0,line_ends[0]] , 
        'sigma'
        )
    
    olc.interp_timeseries(
        'cable_control_2', 
        [0,t_trans,t_trans+t_sigma], 
        [0,0,line_ends[1]] , 
        'sigma'
        )
    
    olc.interp_timeseries(
        'cable_control_3', 
        [0,t_trans,t_trans+t_sigma], 
        [0,0,line_ends[2]] , 
        'sigma'
        )

    
    ol_params = olc.write_input(os.path.join(run_dir,'open_loop_cable.dat'))

    controller_params = {}
    controller_params['open_loop'] = ol_params
    controller_params['CC_Mode'] = 2

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [8],
        'TMax': t_max,
        }
    r.case_inputs = {}
    r.fst_vt        = reader.fst_vt
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.run_FAST()


    op = output_processing.output_processing()
    op2 = output_processing.output_processing()

    md_out = op.load_fast_out([os.path.join(run_dir,'IEA15MW_cable/power_curve/base/IEA15MW_cable_0.MD.Line1.out')], tmin=0)
    local_vars = op2.load_fast_out([os.path.join(run_dir,'IEA15MW_cable/power_curve/base/IEA15MW_cable_0.RO.dbg2')], tmin=0)

    fig, axs = plt.subplots(4,1)
    axs[0].plot(local_vars[0]['Time'],local_vars[0]['CC_DesiredL'],label='CC_DesiredL')
    axs[1].plot(local_vars[0]['Time'],local_vars[0]['CC_ActuatedL'],label='CC_ActuatedL')
    axs[2].plot(local_vars[0]['Time'],local_vars[0]['CC_ActuatedDL'],label='CC_ActuatedDL')
    axs[3].plot(md_out[0]['Time'],md_out[0]['Seg20Lst'],label='Seg20Lst')

    [a.legend() for a in axs]
    [a.grid() for a in axs]

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir,'22_cable_control.png'))

    # Check that the last segment of line 1 shrinks by 10 m
    np.testing.assert_almost_equal(md_out[0]['Seg20Lst'][-1] - md_out[0]['Seg20Lst'][0], line_ends[0], 2)



if __name__=="__main__":
    main()
