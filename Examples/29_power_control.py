'''
29_power_control
----------------
This example demonstrates an advanced method for controlling the power output of a turbine.
Users may want to reduce the power output, or de-rate the turbine to reduce loads, increase wind speeds deeper in the farm, or for grid support functions.
Users might also want to increase the power output when it's safe to do so.
Future advanced control methods can make use of these inputs to directly account for the trade off between power and loads.

There are a few ways to control the turbine power output

The power rating (``R``) is the controlled power relative to the rated power (or available power below rated).
For example R = 0.9, will produce 90% of the rated power (or available power below rated).

* Speed (``R_Speed``) will change the rated speed, or the speed relative to the optimal tip speed ratio (below rated).
* Torque (``R_Torque``): will change the rated torque.  In constant power operation (`VS_ConstPower` of 1), the torque is non-constant, but the rated power used to calcualte the torque is adapted accordingly.
* Pitch (``R_Pitch``): will change the minimum pitch angle of the turbine.  When using peak shaving, the min pitch is the maximum of the minimum pitch for peak shaving (``PS_Min_Pitch``) and the minimum pitch for power control (``PRC_Min_Pitch``). The ROSCO toolbox can generate a lookup table from ``R_Pitch`` to ``PRC_Min_Pitch`` using the Cp surface.

The three methods are compared in the following figure:

.. image:: ../images/29_PRC_Methods.png

The power rating can be controlled with three different "communication" methods (``PRC_Comm``), via:

#. Constant settings in the DISCON: ``R_Speed``, ``R_Torque``, and ``R_Pitch``.
#. Open-loop control inputs with time or wind speed breakpoints.  Two applications will be shown in the example below.
#. The ZeroMQ interface.  A simple example is provided in ``17b_zeromq_multi_openfast.py``.

This example shows users how to set up

#. A constant input with R_Speed of 0.85.
#. A soft start up routine that slowly ramps the rating from 0 to 1 over 60 seconds.
#. A soft cut-out routine for high wind speeds.



'''

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
#from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
from rosco.toolbox.inputs.validation import load_rosco_yaml
from rosco.toolbox import controller as ROSCO_controller


import numpy as np

rpm2RadSec = 2.0*(np.pi)/60.0


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)


def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'29_PRC_Test/12_AWC_Speed')
    os.makedirs(run_dir,exist_ok=True)

    
    # Set DISCON input dynamically through yaml/dict
    controller_params = {}
    controller_params['DISCON'] = {}
    controller_params['DISCON']['Echo'] = 1
    controller_params['DISCON']['PRC_Mode'] = 1
    controller_params['DISCON']['PRC_Comm'] = 1
    controller_params['DISCON']['PRC_R_Torque'] = 1.0
    controller_params['DISCON']['PRC_R_Speed'] = 1.0
    controller_params['DISCON']['PRC_R_Pitch'] = 1.0
    controller_params['DISCON']['OL_BP_FitFreq'] = 2 * np.pi / 30
    

    # Soft start up
    if False:
        olc = ROSCO_controller.OpenLoopControl(t_max=100)
        olc.interp_series(
            'R_speed', 
            [50,70], 
            [1,1.1] , 
            'sigma'
            )
    
    # Soft cut out
    if False:
        olc = ROSCO_controller.OpenLoopControl(
            breakpoint='wind_speed',
            u_min = 20,
            u_max = 30
            )
        olc.interp_series(
            'R_speed', 
            [20,30], 
            [1,0.5] , 
            'cubic'
            )
        
    # AWC above rated via R_Speed/Torque
    if True:
        controller_params['OL_Mode'] = 1
        olc = ROSCO_controller.OpenLoopControl(t_max=600)
        olc.sine_timeseries(
            'R_speed', 
            0.104719,   # amplitude, 1 rpm
            100,        # period, sec
            )
        
    
    fig,ax = olc.plot_series()
    fig.savefig(os.path.join(example_out_dir,'29_OL_Inputs.png'))
    # import matplotlib.pyplot as plt
    # plt.show()
    

    # Write open loop input, get OL indices
    ol_filename = os.path.join(example_out_dir,'29_OL_Input.dat')
    ol_dict = olc.write_input(ol_filename)
    controller_params['open_loop'] = ol_dict
    controller_params['OL_Mode'] = 1
    

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    
    # Wind case
    # A few different cases highlight TRA
    
    # Ramp: good demo of functionality, short for CI
    r.wind_case_fcn = cl.power_curve  
    r.wind_case_opts    = {
            'U': [14],  # from 10 to 15 m/s
            'TMax': 600,
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

    # # Do a run with both tower modes
    # r.control_sweep_fcn = cl.sweep_yaml_input
    # r.control_sweep_opts = {
    #         'control_param': 'TRA_Mode',
    #         'param_values': [0,1]
    #     }
    
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dll     = '/Users/dzalkind/Tools/ROSCO-PRC/rosco/controller/build/libdiscon.dylib'
    # r.rosco_dir     = rosco_dir
    # r.case_inputs = {}
    # r.n_cores = 2
    r.run_FAST()



if __name__=="__main__":
    main()
