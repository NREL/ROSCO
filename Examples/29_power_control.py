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

* Speed (``R_Speed``) will change the rated speed, or the speed setpoint relative to the optimal tip speed ratio (below rated).
* Torque (``R_Torque``): will change the rated torque.  In constant power operation (`VS_ConstPower` of 1), the torque is non-constant, but the rated power used to calcualte the torque demand is adapted accordingly.
* Pitch (``R_Pitch``): will change the minimum pitch angle of the turbine.  When using peak shaving, the min pitch is the maximum of the minimum pitch for peak shaving (``PS_Min_Pitch``) and the minimum pitch for power control (``PRC_Min_Pitch``). The ROSCO toolbox can generate a lookup table from ``R_Pitch`` to ``PRC_Min_Pitch`` using the Cp surface.

The three methods are compared in the following figure:

.. image:: ../images/examples/29_PRC_Methods.png

The power rating can be controlled with three different "communication" methods (``PRC_Comm``), via:

#. Constant settings in the DISCON: ``R_Speed``, ``R_Torque``, and ``R_Pitch``.
#. Open-loop control inputs with time or wind speed breakpoints.  Two applications will be shown in the example below.
#. The ZeroMQ interface.  A simple example is provided in ``17b_zeromq_multi_openfast.py``.

This example shows users how to set up open loop control inputs for

#. A start up routine that ramps the turbine rating using R_Torque in steps over 400 seconds.
#. A soft cut-out routine for high wind speeds.
#. Active wake control for above rated operation, using a sinusoidal input for R_speed.


Start Up Demo
``````````````

.. image:: ../images/examples/29_StartUp.png

The turbine is started in a parked configuration with the blades pitched to 90 degrees.
R_Torque is increased from 0 to 0.2, then from 0.2 to 1.0, simulating a startup routine.
The torque, because it is saturated at the max torque, which is scaled by ``R_Torque`` follows the trajectory of ``PRC_R_Torque``.
``PRC_R_Torque`` is the variable name inside ROSCO and the ``.dbg2`` file.
The blade pitch controller is active throughout the simulation, regulating the generator to the rated speed as usual.


Soft Cut-out Demo
`````````````````
.. image:: ../images/examples/29_Soft_Cutout.png

The turbine starts in above-rated operation and a ramp wind input goes well beyond the normal cut-out wind speed of 25 m/s.
In this demonstration, we use both R_Speed and R_Torque to ramp the turbine rating from 1.0 at 20 m/s to 0.5 at 30 m/s and 0.0 at 40 m/s.
The signals ``PRC_R_Speed`` and ``PRC_R_Torque`` ramp towards 0 following the R versus wind speed table.
Both GenTq and GenSpeed drop to 0 following the reference signals and the power follows.


Active Wake Control Demo
`````````````````````````

.. image:: ../images/examples/29_AWC.png

In active wake control, the goal is to change the rotor thrust through low frequency changes to the blade pitch.
Since the blade pitch must be used to control the rotor speed in above rated operation, we can insteady vary the generator speed reference (via R_Speed) to create similar blade pitch variations.

'''

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox import controller as ROSCO_controller


import numpy as np

rpm2RadSec = 2.0*(np.pi)/60.0


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)


def main():

    FULL_TEST = False       # FULL_TEST for local testing, otherwise shorter for CI

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW.yaml')

    
    # Set DISCON input dynamically through yaml/dict
    controller_params = {}
    controller_params['DISCON'] = {}
    controller_params['DISCON']['Echo'] = 1
    controller_params['DISCON']['PRC_Mode'] = 2
    controller_params['DISCON']['PRC_Comm'] = 1
    controller_params['DISCON']['PRC_R_Torque'] = 1.0
    controller_params['DISCON']['PRC_R_Speed'] = 1.0
    controller_params['DISCON']['PRC_R_Pitch'] = 1.0
    controller_params['DISCON']['OL_BP_FiltFreq'] = 2 * np.pi / 30

    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.case_inputs = {}

    # Disable floating DOFs for clarity
    r.case_inputs[('ElastoDyn','PtfmSgDOF')] = {'vals': ['False'], 'group': 0}
    r.case_inputs[('ElastoDyn','PtfmSwDOF')] = {'vals': ['False'], 'group': 0}
    r.case_inputs[('ElastoDyn','PtfmHvDOF')] = {'vals': ['False'], 'group': 0}
    r.case_inputs[('ElastoDyn','PtfmRDOF')] = {'vals': ['False'], 'group': 0}
    r.case_inputs[('ElastoDyn','PtfmPDOF')] = {'vals': ['False'], 'group': 0}
    r.case_inputs[('ElastoDyn','PtfmYDOF')] = {'vals': ['False'], 'group': 0}
    

    sim_config = 1    

    # 1. Soft start up
    if sim_config == 1:
        if FULL_TEST:
            t_max = 500
        else:   # Shorter for ROSCO CI
            t_max = 100

        run_dir = os.path.join(example_out_dir,'29_PRC_Demo/1_Soft_Start')
        olc = ROSCO_controller.OpenLoopControl(t_max=t_max)
        
        if FULL_TEST:
            olc.interp_series(
                'R_torque', 
                [0,100,200,300,400], 
                [0,0.0,0.2,0.2,1] , 
                'sigma'
                )
        else:
            olc.interp_series(
                'R_torque', 
                [0,100], 
                [0,1.0] , 
                'sigma'
                )
        
        # Wind case
        r.wind_case_fcn = cl.power_curve  
        r.wind_case_opts    = {
                'U': [14],  # from 10 to 15 m/s
                'TMax': t_max,
                }
        r.case_inputs[('ElastoDyn','BlPitch1')] = {'vals': [90.], 'group': 0}
        r.case_inputs[('ElastoDyn','BlPitch2')] = {'vals': [90.], 'group': 0}
        r.case_inputs[('ElastoDyn','BlPitch3')] = {'vals': [90.], 'group': 0}
        r.case_inputs[('ElastoDyn','RotSpeed')] = {'vals': [0.], 'group': 0}

        [("ElastoDyn","BlPitch1")]
    
    # 2. Soft cut out
    if sim_config == 2:
        run_dir = os.path.join(example_out_dir,'29_PRC_Demo/2_Use_Torque')
        olc = ROSCO_controller.OpenLoopControl(
            breakpoint='wind_speed',
            u_min = 20,
            u_max = 40
            )
        olc.interp_series(
            'R_speed', 
            [20,30,40], 
            [1,0.707,0.0] , 
            )
        olc.interp_series(
            'R_torque', 
            [20,30,40], 
            [1,0.707,0.0] , 
            )
        r.wind_case_fcn = cl.ramp  
        r.wind_case_opts    = {
            'U_start': 20,  # from 10 to 15 m/s
            'U_end': 50,
            't_start': 500,
            't_end': 2500,
            }
        
        r.case_inputs[('ElastoDyn','BlPitch1')] = {'vals': [19.], 'group': 0}
        r.case_inputs[('ElastoDyn','BlPitch2')] = {'vals': [19.], 'group': 0}
        r.case_inputs[('ElastoDyn','BlPitch3')] = {'vals': [19.], 'group': 0}
        r.case_inputs[('ElastoDyn','RotSpeed')] = {'vals': [7.], 'group': 0}
        
        controller_params['DISCON']['OL_BP_FiltFreq'] = 1/100 
        controller_params['DISCON']['VS_MinOMSpd'] = 0   # required to force low reference speed
        
    # AWC above rated via R_Speed/Torque
    if sim_config == 3:
        run_dir = os.path.join(example_out_dir,'29_PRC_Demo/3_AWC_Above_Rated')
        t_max = 600
        r.wind_case_fcn = cl.power_curve  
        r.wind_case_opts    = {
                'U': [14],  # from 10 to 15 m/s
                'TMax': t_max,
                }
        olc = ROSCO_controller.OpenLoopControl(t_max=600)
        olc.sine_timeseries(
            'R_speed',  # could use R_speed or R_torque
            0.05,       # amplitude, fraction of rated power
            100,        # period, sec
            1,          # offset, (-)
            )
        
    
    fig,ax = olc.plot_series()
    fig.savefig(os.path.join(example_out_dir,'29_OL_Inputs.png'))

    # Write open loop input, get OL indices
    os.makedirs(run_dir,exist_ok=True)
    ol_filename = os.path.join(run_dir,'29_OL_Input.dat')
    ol_dict = olc.write_input(ol_filename)
    controller_params['open_loop'] = ol_dict
    controller_params['OL_Mode'] = 1
    
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.run_FAST()



if __name__=="__main__":
    main()
