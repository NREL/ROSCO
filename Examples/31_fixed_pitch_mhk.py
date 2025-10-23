'''
31_fixed_pitch_mhk
------------------

This example demonstrates the fixed-pitch control of a marine hydrodkinetic (MHK) turbine.

There are several ways to control the power output of a turbine in above-rated conditions.  
In this example we demonstrate the following control configurations:

#. Constant power underspeed (should be the default)
#. Constant power overspeed
#. Linear increasing power
#. Linear increasing power, leveling out
#. Generic numeric function
#. Constant power overspeed, nonlinear lookup table control

More details about the controller methods can be found in :ref:`marine_hydro`.

The desired power curves of each configuration are as follows:

.. image:: ../images/examples/31_fixed_pitch_mhk_sched.png

In the first case, the reference generator speed is decreased (underspeed) to maintain a constant rated power above rated.
To slow down the generator, a higher torque must be used:

.. image:: ../images/examples/31_fixed_pitch_mhk_sim.png



'''

# Copying images, from docs/:
# cp ../Examples/examples_out/30_fixed_pitch_mhk_sched.png images/
# cp ../Examples/examples_out/30_fixed_pitch_mhk_sim.png images/

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



#directories
this_dir            = os.path.dirname(os.path.abspath(__file__))
rosco_dir           = os.path.dirname(this_dir)
example_out_dir     = os.path.join(this_dir, 'examples_out')
os.makedirs(example_out_dir,exist_ok=True)

def main():

    FULL_TEST = False   # Run a full test locally (True) or a shorter one for CI
    sim_config = 1      # Choose which simulation configuration (1-6)

    # Input yaml and output directory
    parameter_filename = os.path.join(this_dir, 'Tune_Cases/RM1_MHK_FBP.yaml')
    tune_dir = os.path.dirname(parameter_filename)

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
    controller_params_1['VS_FBP_U'] = [2.0, 4.0]
    controller_params_1['VS_FBP_P'] = [1.0, 1.0]
    controller_1      = ROSCO_controller.Controller(controller_params_1)
    controller_1.tune_controller(turbine)

    # Constant power overspeed
    controller_params_2 = controller_params.copy()
    controller_params_2['VS_FBP'] = 2 # Switch to WSE reference
    controller_params_2['VS_FBP_speed_mode'] = 1
    controller_params_2['VS_FBP_U'] = [2.0, 4.0]
    controller_params_2['VS_FBP_P'] = [1.0, 1.0]
    controller_2      = ROSCO_controller.Controller(controller_params_2)
    controller_2.tune_controller(turbine)

    # Linear increasing power
    controller_params_3 = controller_params.copy()
    controller_params_3['VS_FBP_speed_mode'] = 0
    controller_params_2['VS_FBP_U'] = [2.0, 4.0]
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
    controller_params_6['VS_FBP'] = 1 # Constant power overspeed
    controller_params_6['VS_FBP_speed_mode'] = 1
    controller_params_6['VS_FBP_U'] = [2.0, 4.0]
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
    axs[0].legend(loc='upper left', bbox_to_anchor=(.2, 2.35))

    fig.align_ylabels()
    plt.subplots_adjust(hspace=0.5)
                        

    if False:
        plt.show()
    else:
        fig_fname = os.path.join(example_out_dir, '30_fixed_pitch_mhk_sched.png')
        print('Saving figure ' + fig_fname)
        plt.savefig(fig_fname,bbox_inches='tight',)

    # Write parameter input file for constant power underspeed controller
    run_dir = os.path.join(example_out_dir, f'31_MHK/{sim_config}_config')
    os.makedirs(run_dir,exist_ok=True)

    # simulation set up
    if FULL_TEST:
        TMax = 60
    else:
        TMax = 5

    all_controller_params = [
        controller_params_1,
        controller_params_2,
        controller_params_3,
        controller_params_4,
        controller_params_5,
        controller_params_6,
    ]

    if sim_config in range(1,len(all_controller_params)+1):
        controller_params = all_controller_params[sim_config-1]
    else:
        raise Exception(f'Invalid sim_config of {sim_config}.  Note the 1-indexing.')

    r = run_FAST_ROSCO()
    r.tuning_yaml   = parameter_filename
    r.wind_case_fcn = cl.power_curve
    r.wind_case_opts    = {
        'U': [3.0],
        'TMax': TMax,
        }
    r.case_inputs = {}
    r.controller_params = controller_params
    r.save_dir      = run_dir
    r.rosco_dir     = rosco_dir

    r.run_FAST()

    op = output_processing.output_processing()
    fast_out = op.load_fast_out([os.path.join(run_dir,'RM1_MHK_FBP_0.out')], tmin=0)
    fig, axs = plt.subplots(4,1)
    axs[0].plot(fast_out[0]['Time'], fast_out[0]['Wind1VelX'],             label='Constant Power Underspeed')
    axs[0].set_ylabel('Flow Speed [m/s]',rotation=0, labelpad=50)
    axs[1].plot(fast_out[0]['Time'], fast_out[0]['GenSpeed'] * 2*np.pi/60, label='Constant Power Underspeed')
    axs[1].set_ylabel('Gen Speed [rad/s]',rotation=0, labelpad=50)
    axs[2].plot(fast_out[0]['Time'], fast_out[0]['GenTq'] * 1e3,           label='Constant Power Underspeed')
    axs[2].set_ylabel('Gen Torque [N m]',rotation=0, labelpad=50)
    axs[3].plot(fast_out[0]['Time'], fast_out[0]['GenPwr'] * 1e3,          label='Constant Power Underspeed')
    axs[3].set_ylabel('Gen Power [W]',rotation=0, labelpad=50)
    axs[3].set_xlabel('Time [s]')

    plt.subplots_adjust(hspace=0.5)
    fig.align_ylabels()


    # TODO: Compare result to desired operating schedule

    if False:
        plt.show()
    else:
        fig_fname = os.path.join(example_out_dir, '30_fixed_pitch_mhk_sim.png')
        print('Saving figure ' + fig_fname)
        plt.savefig(fig_fname, bbox_inches='tight')


if __name__=="__main__":
    main()
