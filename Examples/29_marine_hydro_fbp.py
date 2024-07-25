'''
----------- 29_marine_hydro_fbp ---------------
Run openfast with ROSCO and a MHK turbine with fixed blade pitch control
-----------------------------------------------


'''

import os
from rosco.toolbox.ofTools.case_gen.run_FAST import run_FAST_ROSCO
from rosco.toolbox.ofTools.case_gen import CaseLibrary as cl
from rosco.toolbox.ofTools.fast_io import output_processing
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import utilities as ROSCO_utilities
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml
import matplotlib.pyplot as plt
import numpy as np

'''
Run MHK turbine in OpenFAST with ROSCO torque controller


'''


#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir, 'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

def main():

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir, 'Tune_Cases/RM1_MHK_FBP.yaml')
    tune_dir = os.path.dirname(parameter_filename)
    run_dir = os.path.join(example_out_dir, '29_MHK/0_baseline')
    os.makedirs(run_dir,exist_ok=True)

    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    cp_filename = os.path.join(tune_dir, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir, path_params['FAST_directory']),
        rot_source='txt', txt_filename= cp_filename
        )


    ### Tune controller cases
    # Constant power underspeed (should be the default)
    controller_params_1 = controller_params.copy()
    controller_params_1['VS_FBP'] = 3 # Power reference
    controller_params_1['VS_FBP_speed_mode'] = 0
    controller_params_1['VS_FBP_P'] = [1.0, 1.0]
    controller_1      = ROSCO_controller.Controller(controller_params_1)
    controller_1.tune_controller(turbine)

    # Constant power overspeed
    controller_params_2 = controller_params.copy()
    controller_params_2['VS_FBP'] = 2 # Switch to WSE reference
    controller_params_2['VS_FBP_speed_mode'] = 1
    controller_params_2['VS_FBP_P'] = [1.0, 1.0]
    controller_2      = ROSCO_controller.Controller(controller_params_2)
    controller_2.tune_controller(turbine)

    # Linear increasing power
    controller_params_3 = controller_params.copy()
    controller_params_3['VS_FBP_speed_mode'] = 0
    controller_params_3['VS_FBP_P'] = [1.0, 2.0]
    controller_3      = ROSCO_controller.Controller(controller_params_3)
    controller_3.tune_controller(turbine)

    # Linear increasing power, leveling out
    controller_params_4 = controller_params.copy()
    controller_params_4['VS_FBP_U'] = [2.0, 3.0]
    controller_params_4['VS_FBP_P'] = [1.0, 2.0]
    controller_4      = ROSCO_controller.Controller(controller_params_4)
    controller_4.tune_controller(turbine)

    # Generic numeric function
    controller_params_5 = controller_params.copy()
    controller_params_5['VS_FBP'] = 2 # WSE reference
    controller_params_5['VS_FBP_U'] = [2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0]
    controller_params_5['VS_FBP_P'] = [1.0, 1.3, 1.6, 1.8, 1.9, 2.0, 1.9, 1.8, 1.7, 1.6, 1.5]
    controller_5      = ROSCO_controller.Controller(controller_params_5)
    controller_5.tune_controller(turbine)

    # Constant power overspeed, nonlinear lookup table control
    controller_params_6 = controller_params.copy()
    controller_params_6['VS_FBP'] = 0 # Constant power overspeed
    controller_params_6['VS_FBP_speed_mode'] = 1
    controller_params_6['VS_FBP_P'] = [1.0, 1.0]
    controller_params_6['VS_ControlMode'] = 1
    controller_6      = ROSCO_controller.Controller(controller_params_6)
    controller_6.tune_controller(turbine)


    plot_labels = ['Constant Power Underspeed', 'Constant Power Overspeed', 'Linear Increasing Power', 'Increasing Leveled Power', 'Generic User-Defined']
    fig, axs = plt.subplots(3,1)
    axs[0].plot(controller_1.v, controller_1.power_op, label=plot_labels[0])
    axs[0].plot(controller_2.v, controller_2.power_op, label=plot_labels[1], linestyle='--')
    axs[0].plot(controller_3.v, controller_3.power_op, label=plot_labels[2])
    axs[0].plot(controller_4.v, controller_4.power_op, label=plot_labels[3])
    axs[0].plot(controller_5.v, controller_5.power_op, label=plot_labels[4])
    axs[0].set_ylabel('Gen Power [W]')
    axs[1].plot(controller_1.v, controller_1.omega_gen_op, label=plot_labels[0])
    axs[1].plot(controller_2.v, controller_2.omega_gen_op, label=plot_labels[1], linestyle='--')
    axs[1].plot(controller_3.v, controller_3.omega_gen_op, label=plot_labels[2])
    axs[1].plot(controller_4.v, controller_4.omega_gen_op, label=plot_labels[3])
    axs[1].plot(controller_5.v, controller_5.omega_gen_op, label=plot_labels[4])
    axs[1].set_ylabel('Gen Speed [rad/s]')
    axs[2].plot(controller_1.v, controller_1.tau_op, label=plot_labels[0])
    axs[2].plot(controller_2.v, controller_2.tau_op, label=plot_labels[1], linestyle='--')
    axs[2].plot(controller_3.v, controller_3.tau_op, label=plot_labels[2])
    axs[2].plot(controller_4.v, controller_4.tau_op, label=plot_labels[3])
    axs[2].plot(controller_5.v, controller_5.tau_op, label=plot_labels[4])
    axs[2].set_ylabel('Gen Torque [N m]')
    axs[2].set_xlabel('Flow Speed [m/s]')
    axs[0].legend(loc='upper left')

    if False:
        plt.show()
    else:
        fig_fname = os.path.join(example_out_dir, '29_marine_hydro_fbp_sched.png')
        print('Saving figure ' + fig_fname)
        plt.savefig(fig_fname)

    # Write parameter input file for constant power underspeed controller
    param_file = os.path.join(run_dir,'DISCON.IN')
    write_DISCON(turbine,
                 controller_1,
                 param_file=param_file, 
                 txt_filename=cp_filename
    )


    # simulation set up
    # TODO: simulate multiple controller configurations in parallel
    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    # r.wind_case_fcn = cl.simp_step  # single step wind input
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [3.0],
        'TMax': 60.0,
        }
    r.case_inputs = {}
    # r.fst_vt        = reader.fst_vt
    r.controller_params = controller_params_1
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()

    op = output_processing.output_processing()
    fast_out = op.load_fast_out([os.path.join(run_dir,'RM1_MHK_FBP/power_curve/base/RM1_MHK_FBP_0.out')], tmin=0)
    fig, axs = plt.subplots(4,1)
    axs[0].plot(fast_out[0]['Time'], fast_out[0]['Wind1VelX'],             label='Constant Power Underspeed')
    axs[0].set_ylabel('Flow Speed [m/s]')
    axs[1].plot(fast_out[0]['Time'], fast_out[0]['GenSpeed'] * 2*np.pi/60, label='Constant Power Underspeed')
    axs[1].set_ylabel('Gen Speed [rad/s]')
    axs[2].plot(fast_out[0]['Time'], fast_out[0]['GenTq'] * 1e3,           label='Constant Power Underspeed')
    axs[2].set_ylabel('Gen Torque [N m]')
    axs[3].plot(fast_out[0]['Time'], fast_out[0]['GenPwr'] * 1e3,          label='Constant Power Underspeed')
    axs[3].set_ylabel('Gen Power [W]')
    axs[3].set_xlabel('Time [s]')

    # TODO: Compare result to desired operating schedule

    if False:
        plt.show()
    else:
        fig_fname = os.path.join(example_out_dir, '29_marine_hydro_fbp_sim.png')
        print('Saving figure ' + fig_fname)
        plt.savefig(fig_fname)


if __name__=="__main__":
    main()
