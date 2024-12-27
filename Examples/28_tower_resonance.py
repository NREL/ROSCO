"""
28_tower_resonance
------------------
Demonstrate tower resonance avoidance controller
Set up and run simulation with tower resonance avoidance
"""

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
from rosco.toolbox.inputs.validation import load_rosco_yaml

import numpy as np

def main():
    rpm2RadSec = 2.0*(np.pi)/60.0

    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'28_TRA_Ramp_0')
    os.makedirs(run_dir,exist_ok=True)

    # Change tower programatically, read first
    # Read initial input file
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']

    reader = InputReader_OpenFAST()
    reader.FAST_InputFile = path_params['FAST_InputFile']
    reader.FAST_directory = os.path.join(this_dir,'Tune_Cases',path_params['FAST_directory'])
    # reader.FAST_directory = '/Users/dzalkind/Tools/ROSCO1/Test_Cases/ptfm_control_archive/IEA-15-240-RWT-UMaineSemi_ballast'
    reader.execute()

    # Reduce stiffness to 1/4 original
    ed_twr = reader.fst_vt['ElastoDynTower']
    ed_twr['FAStTunr1'] = ed_twr['FAStTunr2'] = ed_twr['SSStTunr1'] = ed_twr['SSStTunr2'] = 0.25
    
    # Set DISCON input dynamically through yaml/dict
    controller_params = {}
    controller_params['TRA_Mode'] = 2    
    controller_params['vs_minspd'] = 0.    # Reduce minimum rotor speed so that saturation does not interfere with exclusion
    controller_params['VS_ControlMode'] = 3.   
    
    # TRA parameters
    controller_params['DISCON'] = {}
    controller_params['DISCON']['TRA_ExclSpeed'] = 4.75 * rpm2RadSec
    controller_params['DISCON']['TRA_ExclBand'] = 1 * rpm2RadSec
    controller_params['DISCON']['TRA_RateLimit'] = 0.7916800 / 100

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    
    # Wind case
    # A few different cases highlight TRA
    
    # Ramp: good demo of functionality, short for CI
    r.wind_case_fcn = cl.ramp  
    r.wind_case_opts    = {
        'U_start': 0,  # from 10 to 15 m/s
        'U_end': 10,
        't_start': 100,
        't_end': 300
        }

    # # steady
    # r.wind_case_fcn = cl.power_curve  
    # r.wind_case_opts    = {
    #     'U': 6.5,  # from 10 to 15 m/s
    #     'TMax': 400,
    #     }
    
    # # turbulence
    # r.wind_case_fcn = cl.turb_bts  
    # r.wind_case_opts    = {
    #     'TMax': 400,  # from 10 to 15 m/s
    #     'wind_filenames': ['/Users/dzalkind/Downloads/heavy_test_1ETM_U6.000000_Seed603.0.bts'],
    #     }

    # Do a run with both tower modes
    r.control_sweep_fcn = cl.sweep_yaml_input
    r.control_sweep_opts = {
            'control_param': 'TRA_Mode',
            'param_values': [0,1]
        }
    
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.case_inputs = {}
    r.n_cores = 2
    r.run_FAST()



if __name__=="__main__":
    main()
