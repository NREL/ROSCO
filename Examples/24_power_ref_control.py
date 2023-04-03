'''
----------- 24_outdata_in_control ------------------------
Run openfast with ROSCO and cable control
-----------------------------------------------

Set up and run simulation with pitch offsets, check outputs

'''

import os, platform
from ROSCO_toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from ROSCO_toolbox.ofTools.case_gen import CaseLibrary as cl
from ROSCO_toolbox.ofTools.fast_io import output_processing
from ROSCO_toolbox.tune import yaml_to_objs
import numpy as np
from ROSCO_toolbox.ofTools.fast_io.FAST_reader import InputReader_OpenFAST
from ROSCO_toolbox.ofTools.fast_io.FAST_writer import InputWriter_OpenFAST
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
import matplotlib.pyplot as plt

'''
Set reference rotor speed as a function of wind speed (estimate in ROSCO)

'''


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir,'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

if platform.system() == 'Windows':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dll'))
elif platform.system() == 'Darwin':
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.dylib'))
else:
    lib_name = os.path.realpath(os.path.join(this_dir, '../ROSCO/build/libdiscon.so'))


def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(rosco_dir,'Tune_Cases/IEA15MW.yaml')
    run_dir = os.path.join(example_out_dir,'24_PRC_0')
    os.makedirs(run_dir,exist_ok=True)


    # Create rotor speed table
    controller, turbine, path_params = yaml_to_objs(parameter_filename)


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


    # simulation set up
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [9],
        'T_max': 100,
        }
    r.case_inputs = {}
    r.save_dir      = run_dir
    # r.controller_params  = {
    #     'OD_Mode': 1,
    #     'DISCON': {
    #         'Echo': True,
    #     }}   # Use OutData in control
    r.rosco_dir     = rosco_dir

    r.run_FAST()



if __name__=="__main__":
    main()