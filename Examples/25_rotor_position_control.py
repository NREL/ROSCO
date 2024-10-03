"""
25_rotor_position_control
-------------------------
Run ROSCO with rotor position control
Run a steady simulation, use the azimuth output as an input to the next steady simulation, with different ICs 
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.controller import OpenLoopControl
import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt


def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)


    # Set up paths
    parameter_filename = os.path.join(this_dir,'Tune_Cases/NREL2p8.yaml')
    run_dir = os.path.join(example_out_dir,'25_rotor_position_control')
    os.makedirs(run_dir,exist_ok=True)
   
    # Steady simualtion with initial RotSpeed of 5 rpm
    r = run_FAST_ROSCO()
    r.wind_case_fcn = cl.power_curve
    r.tuning_yaml   = parameter_filename
    r.wind_case_opts    = {
        'U': [14],
        'TMax': 100,
        }
    r.save_dir      = run_dir
    r.run_FAST()

    # Gather azimuth, blade pitch, generator torque output, apply as open loop inputs to ROSCO
    op = output_processing.output_processing()
    fast_out = op.load_fast_out(os.path.join(run_dir,'NREL2p8/power_curve/base/NREL2p8_0.outb'), tmin=0)

    olc = OpenLoopControl()
    olc.ol_timeseries['time'] = fast_out[0]['Time']
    olc.ol_timeseries['blade_pitch1'] = np.radians(fast_out[0]['BldPitch1'])
    olc.ol_timeseries['blade_pitch2'] = np.radians(fast_out[0]['BldPitch2'])
    olc.ol_timeseries['blade_pitch3'] = np.radians(fast_out[0]['BldPitch3'])
    olc.ol_timeseries['generator_torque'] = fast_out[0]['GenTq'] * 1000
    olc.ol_timeseries['azimuth'] = np.radians(fast_out[0]['Azimuth'])

    # Save initial RotSpeed, Azimuth for later
    RotSpeed_0 = fast_out[0]['RotSpeed'][0]
    Azimuth_0 = fast_out[0]['Azimuth'][0]
    
    # set up control_params for next run
    open_loop = olc.write_input(os.path.join(run_dir,'ol_input.dat'))
    controller_params = {}
    controller_params['open_loop'] = open_loop
    controller_params['OL_Mode'] = 2        # Azimuth tracking open loop
    controller_params['PA_Mode'] = 0        # No pitch actuator
    controller_params['DISCON'] = {}
    
    gains = -1800 * np.array([12,1.2,120])  # PID gains of rotor position control
    controller_params['DISCON']['RP_Gains'] = np.r_[gains, 2]  # Add Tf
    controller_params['DISCON']['PC_MinPit'] = np.radians(-20)  # Remove lower limit of pitch so it doesn't interfere with open loop input


    # run again with different IC and rotor position control
    r.base_name     = 'rpc'
    r.tuning_yaml   = parameter_filename
    # Set initial conditions
    r.case_inputs = {}
    r.case_inputs[("ElastoDyn","RotSpeed")]      = {'vals':[RotSpeed_0 + 1], 'group':0}
    r.case_inputs[("ElastoDyn","Azimuth")]      = {'vals':[Azimuth_0 + 30], 'group':0}
    r.case_inputs[("ServoDyn","Ptch_Cntrl")]      = {'vals':[1], 'group':0}

    r.controller_params = controller_params
    r.run_FAST()

    # Plot relevant outputs
    op = output_processing.output_processing()
    op_dbg2 = output_processing.output_processing()

    out_files = [os.path.join(run_dir,f'NREL2p8/power_curve/base/NREL2p8_0.out'),
                os.path.join(run_dir,f'rpc/power_curve/base/rpc_0.out')]
    dbg2_files = [out.split('.out')[0] + '.RO.dbg2' for out in out_files]

    fst_out = op.load_fast_out(out_files, tmin=0)
    local_vars = op_dbg2.load_fast_out(dbg2_files, tmin=0)

    comb_out = [None] * len(fst_out)
    for i, (r_out2, f_out) in enumerate(zip(local_vars,fst_out)):
        r_out2.update(f_out)
        comb_out[i] = r_out2

    cases = {}
    cases['Fl Sigs.'] = ['BldPitch1','GenTq','GenSpeed', 'Azimuth', 'AzError','GenTqAz']
    fig, ax = op.plot_fast_out(comb_out,cases, showplot=True)

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(run_dir,'25_rotor_position_control.png'))



if __name__=="__main__":
    main()
