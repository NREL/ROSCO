'''
----------- Example_17 --------------
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
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'19_RotPos')
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
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [16],
        'T_max': 200
        }
    r.save_dir      = run_dir
    r.case_inputs   = case_inputs
    # r.run_FAST()

    # Gather azimuth, blade p
    # itch, generator torque output
    op = output_processing.output_processing()
    fast_out = op.load_fast_out(os.path.join(example_out_dir,'19_RotPos/IEA15MW/power_curve/base/IEA15MW_0.outb'), tmin=0)

    olc = OpenLoopControl()
    olc.ol_timeseries['time'] = fast_out[0]['Time']
    olc.ol_timeseries['blade_pitch1'] = np.radians(fast_out[0]['BldPitch1'])
    olc.ol_timeseries['blade_pitch2'] = np.radians(fast_out[0]['BldPitch2'])
    olc.ol_timeseries['blade_pitch3'] = np.radians(fast_out[0]['BldPitch3'])
    olc.ol_timeseries['generator_torque'] = fast_out[0]['GenTq'] * 1000
    olc.ol_timeseries['azimuth'] = np.radians(fast_out[0]['Azimuth'])

    # set up control_params for next run
    open_loop = olc.write_input(os.path.join(run_dir,'ol_input.dat'))
    controller_params = {}
    controller_params['open_loop'] = open_loop
    controller_params['OL_Mode'] = 2
    controller_params['PA_Mode'] = 0
    controller_params['DISCON'] = {}
    controller_params['DISCON']['RP_Gains'] = [-3e7,-3e6,-3e8]


    # run again with slower IC and rotor position control
    case_inputs[("ElastoDyn","RotSpeed")]    = {'vals':[4], 'group':0}
    r.case_inputs   = case_inputs
    r.base_name     = 'rpc'
    r.controller_params = controller_params
    r.run_FAST()



if __name__=="__main__":
    main()