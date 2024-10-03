"""
27_power_ref_control
--------------------
Run openfast with ROSCO and cable control
Demonstrate a simulation with a generator reference speed that changes with estimated wind speed
Set reference rotor speed as a function of wind speed (estimate in ROSCO)
"""

import os
from rosco import discon_lib_path
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.tune import yaml_to_objs
import numpy as np
#from rosco.toolbox.inputs.validation import load_rosco_yaml
import matplotlib.pyplot as plt

FULL_TEST = False

def main():
    #directories
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    rosco_dir           = os.path.dirname(this_dir)
    example_out_dir     = os.path.join(this_dir,'examples_out')
    os.makedirs(example_out_dir,exist_ok=True)
    lib_name = discon_lib_path
    
    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'27_PRC_0')
    os.makedirs(run_dir,exist_ok=True)


    # Create rotor speed table
    controller, _, _ = yaml_to_objs(parameter_filename)


    # plot original ops
    plt.plot(controller.v,controller.omega_op)


    omega = controller.omega_op
    v = controller.v

    # Hold near rated rotor speed, for testing setpoint smoother
    near_rated = np.bitwise_and(v > 9, v < 13)
    first_near_rated_ind = np.where(near_rated)[0][0]
    omega[near_rated] = omega[first_near_rated_ind]

    # add high wind ride-through
    v_hwrt = [50]     # m/s
    omega_hwrt = [0.3]  # rad/s
    v = np.r_[v,v_hwrt]
    omega = np.r_[omega,omega_hwrt]

    # plot new lookup
    plt.plot(v,omega,linestyle='--')

    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.ramp  # single step wind input
    
    if FULL_TEST:
        # Full test
        r.wind_case_opts    = {
            'U_start': 5,  # from 10 to 15 m/s
            'U_end': 35,
            't_start': 100,
            't_end': 2500,
            }
    else:
        # Short test for CI
        r.wind_case_opts    = {
            'U_start': 25,  # from 10 to 15 m/s
            'U_end': 27,
            't_start': 50,
            't_end': 100,
            }
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir
    r.controller_params  = {
        'PRC_Mode': 1,
        'DISCON': {
            'PRC_Mode': 1,
            'PRC_n': len(v),
            'PRC_WindSpeeds': v,
            'PRC_GenSpeeds': omega,

        }}   # Use OutData in control
    r.run_FAST()

if __name__=="__main__":
    main()
