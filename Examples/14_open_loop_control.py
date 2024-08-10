"""
14_open_loop_control
--------------------
Load a turbine, tune a controller with open loop control commands
In this example:

* Load a turbine from OpenFAST
* Tune a controller
* Write open loop inputs
* Run simple simulation with open loop control
"""

# Python Modules
import os
import numpy as np
import matplotlib.pyplot as plt

# ROSCO toolbox modules 
from rosco import discon_lib_path
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import utilities as ROSCO_utilities
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox.ofTools.case_gen.CaseLibrary import set_channels
from rosco.toolbox.ofTools.case_gen.runFAST_pywrapper   import runFAST_pywrapper, runFAST_pywrapper_batch
from rosco.toolbox.ofTools.case_gen.CaseGen_General     import CaseGen_General

def main():
    this_dir          = os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')

    rosco_dir         = os.path.dirname(this_dir)
    example_out_dir   = os.path.join(this_dir,'examples_out')
    example_out_dir = os.path.join(this_dir,'examples_out')
    if not os.path.isdir(example_out_dir):
        os.makedirs(example_out_dir)

    # Load yaml file (Open Loop Case)
    parameter_filename = os.path.join(tune_dir, 'IEA15MW_OL.yaml')

    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Set up open loop input
    olc = ROSCO_controller.OpenLoopControl(t_max=20)
    olc.interp_timeseries(
    'blade_pitch', 
    [0,20], 
    [0,0.0873] , 
    'sigma'
    )
    olc.const_timeseries(
    'generator_torque', 
    19624046*.5
    )
    olc.sine_timeseries('nacelle_yaw', 0.0524, 60)

    # Plot open loop timeseries
    fig,ax = olc.plot_timeseries()
    if False:
        plt.show()
    else:
        fig.savefig(os.path.join(example_out_dir,'14_OL_Inputs.png'))

    # Write open loop input, get OL indices
    ol_filename = os.path.join(example_out_dir,'14_OL_Input.dat')
    ol_dict = olc.write_input(ol_filename)
    controller_params['open_loop'] = ol_dict


    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    turbine.load_from_fast(path_params['FAST_InputFile'], \
    os.path.join(tune_dir,path_params['FAST_directory']), \
        rot_source='txt',\
        txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename']))

    # Tune controller 
    controller.tune_controller(turbine)

    # Write parameter input file
    param_file = os.path.join(this_dir,'DISCON.IN')   # This must be named DISCON.IN to be seen by the compiled controller binary. 
    ROSCO_utilities.write_DISCON(turbine,controller,param_file=param_file, txt_filename=path_params['rotor_performance_filename'])

    ### Run OpenFAST using aeroelasticse tools
    case_inputs = {}
    case_inputs[('ServoDyn','DLL_FileName')] = {'vals': [discon_lib_path], 'group': 0}

    # Apply all discon variables as case inputs
    discon_vt = ROSCO_utilities.DISCON_dict(
    turbine, 
    controller, 
    txt_filename=os.path.join(tune_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
    )
    for discon_input in discon_vt:
        case_inputs[('DISCON_in',discon_input)] = {'vals': [discon_vt[discon_input]], 'group': 0}

    case_inputs[('Fst','TMax')] = {'vals': [20], 'group': 0}
    case_inputs[('InflowWind','HWindSpeed')] = {'vals': [10], 'group': 0}
    case_inputs[('ElastoDyn','HWindSpeed')] = {'vals': [5.], 'group': 0}
    case_inputs[('DISCON_in','LoggingLevel')] = {'vals': [3], 'group': 0}

    # Generate cases
    run_dir = os.path.join(example_out_dir,'14_OL_Sim')
    if not os.path.exists(run_dir):
        os.makedirs(run_dir)

    case_list, case_name_list = CaseGen_General(case_inputs, dir_matrix=run_dir, namebase='OL_Example')
    channels = set_channels()

    # Run FAST cases
    fastBatch                   = runFAST_pywrapper_batch()

    fastBatch.FAST_directory    = os.path.realpath(os.path.join(tune_dir,path_params['FAST_directory']))
    fastBatch.FAST_InputFile    = path_params['FAST_InputFile']        
    fastBatch.channels          = channels
    fastBatch.FAST_runDirectory = run_dir
    fastBatch.case_list         = case_list
    fastBatch.case_name_list    = case_name_list
    fastBatch.debug_level       = 2
    fastBatch.FAST_exe          = 'openfast'

    fastBatch.run_serial()


    # #  Define Plot cases 
    cases = {}
    cases['Baseline'] = ['Wind1VelX', 'BldPitch1', 'GenTq', 'RotSpeed','NacYaw']

    out_file = os.path.join(example_out_dir,'14_OL_Sim/OL_Example_0.outb')
    op = output_processing.output_processing()
    fastout = op.load_fast_out(out_file, tmin=0)
    fig, ax = op.plot_fast_out(cases=cases,showplot=False)

    # Check that open loop commands are close to control outputs from OpenFAST
    fo = fastout[0]
    tt = fo['Time']
    valid_ind = tt > 2  # first few timesteps can differ, depending on OpenFAST solve config

    # Compute errors
    nacelle_yaw_diff = fo['NacYaw'][valid_ind] - np.degrees(np.interp(tt[valid_ind],olc.ol_timeseries['time'],olc.ol_timeseries['nacelle_yaw']))
    bld_pitch_diff = fo['BldPitch1'][valid_ind] - np.degrees(np.interp(tt[valid_ind],olc.ol_timeseries['time'],olc.ol_timeseries['blade_pitch']))
    gen_tq_diff = fo['GenTq'][valid_ind] - np.interp(tt[valid_ind],olc.ol_timeseries['time'],olc.ol_timeseries['generator_torque'])/1e3

    # Check diff timeseries
    np.testing.assert_allclose(nacelle_yaw_diff,  0,  atol = 1e-1)   # yaw has dynamics and integration error, tolerance higher
    np.testing.assert_allclose(bld_pitch_diff,    0,  atol = 1e-3)
    np.testing.assert_allclose(gen_tq_diff,       0,  atol = 1e-3)


    if False:
        plt.show()
    else:
        fig[0].savefig(os.path.join(example_out_dir,'14_OL_FAST_Out.png'))

if __name__ == "__main__":
    main()

