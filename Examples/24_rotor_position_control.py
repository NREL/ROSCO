'''
----------- 24_rotor_position_control --------------
Run ROSCO with rotor position control
-------------------------------------

Run a steady simulation, use the azimuth output as an input to the next steady simulation, with different ICs 

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.controller import OpenLoopControl
import numpy as np
import pandas as pd
import pickle

#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir         = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))

def main():


    # Ensure external control paths are okay
    parameter_filename = '/Users/dzalkind/Projects/RAAW/RAAW_OpenFAST/ROSCO/RAAW_rosco_BD.yaml'
    run_dir = os.path.join(example_out_dir,'24_rotor_position_control_0')
    gains = -1800 * np.array([12,1.2,120])
    os.makedirs(run_dir,exist_ok=True)

    # Case input for RotSpeed IC
    case_inputs = {}
    case_inputs[("ElastoDyn","RotSpeed")]    = {'vals':[5], 'group':0}
    case_inputs[("ElastoDyn","PtfmSgDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmSwDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmHvDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmRDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmPDOF")]    = {'vals':['False'], 'group':0}
    case_inputs[("ElastoDyn","PtfmYDOF")]    = {'vals':['False'], 'group':0}
    
    # Steady simualtion with initial RotSpeed of 5 rpm
    r = run_FAST_ROSCO()
    r.wind_case_fcn = cl.power_curve
    r.tuning_yaml   = parameter_filename
    r.wind_case_opts    = {
        'U': [200],
        'TMax': 100,
        }
    r.save_dir      = run_dir
    r.openfast_exe = '/Users/dzalkind/opt/anaconda3/envs/rosco-env2/bin/openfast'
    r.case_inputs   = case_inputs
    r.run_FAST()

    # Gather azimuth, blade pitch, generator torque output
    op = output_processing.output_processing()
    # fast_out = op.load_fast_out(os.path.join(example_out_dir,'19_RotPos_Turb_0/IEA15MW/turb_bts/base/IEA15MW_0.outb'), tmin=0)
    bl_out = '/Users/dzalkind/Tools/ROSCO2/outputs/RPC_Sweeps/Baseline_Cases/RAAW_rosco_BD/turb_bts/base/RAAW_rosco_BD_0.outb'
    fast_out = op.load_fast_out(bl_out)

    olc = OpenLoopControl()
    olc.ol_timeseries['time'] = fast_out[0]['Time']
    olc.ol_timeseries['blade_pitch1'] = np.radians(fast_out[0]['BldPitch1'])
    olc.ol_timeseries['blade_pitch2'] = np.radians(fast_out[0]['BldPitch2'])
    olc.ol_timeseries['blade_pitch3'] = np.radians(fast_out[0]['BldPitch3'])
    olc.ol_timeseries['generator_torque'] = fast_out[0]['GenTq'] * 1000
    olc.ol_timeseries['azimuth'] = np.radians(fast_out[0]['Azimuth'])

    RotSpeed_0 = fast_out[0]['RotSpeed'][0]
    Azimuth_0 = fast_out[0]['Azimuth'][0]
    BldPitch_0 = fast_out[0]['BldPitch1'][0]

    # Save olc
    olc.RotSpeed_0 = RotSpeed_0
    olc.Azimuth_0 = Azimuth_0
    olc.BldPitch_0 = BldPitch_0
    case_ind = bl_out.split('.out')[0][-1]
    case_dir = os.path.split(bl_out)[0]
    with open(os.path.join(case_dir,f'olc_{case_ind}.p'),'wb') as f:
        pickle.dump(olc,f)
    
    # set up control_params for next run
    open_loop = olc.write_input(os.path.join(run_dir,'ol_input.dat'))
    controller_params = {}
    controller_params['open_loop'] = open_loop
    controller_params['OL_Mode'] = 2
    controller_params['PA_Mode'] = 0
    controller_params['DISCON'] = {}
    controller_params['DISCON']['RP_Gains'] = gains
    controller_params['DISCON']['PC_MinPit'] = np.radians(-20)


    # run again with slower IC and rotor position control
    # case_inputs[("ElastoDyn","RotSpeed")]    = {'vals':[4], 'group':0}
    r.case_inputs   = case_inputs
    r.base_name     = 'rpc'
    r.tuning_yaml   = parameter_filename
    # Set initial conditions
    r.case_inputs = {}
    r.case_inputs[("ElastoDyn","RotSpeed")]      = {'vals':[RotSpeed_0 + 1], 'group':0}
    r.case_inputs[("ElastoDyn","Azimuth")]      = {'vals':[Azimuth_0 + 30], 'group':0}
    r.case_inputs[("ServoDyn","Ptch_Cntrl")]      = {'vals':[1], 'group':0}

    r.controller_params = controller_params
    r.run_FAST()



if __name__=="__main__":
    main()