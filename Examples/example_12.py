'''
----------- Example_12 --------------
Controller tuning to satisfy a robustness criteria
-------------------------------------
NOTE: This example necessitates the mbc3 through either pyFAST or WEIS

In this example:
  - setup ROSCO's robust tuning methods for the IEA15MW on the UMaine Semi-sub
  - run a the standard tuning method to find k_float
  - run robust tuning to find omega_pc schedule satisfy a prescribed stability margin
  - Tune ROSCO's pitch controller using omega_pc schedule
  - Plot gain schedule

The example is put in a function call to show the ability to load linear models in parallel
'''
import os
import numpy as np
import matplotlib.pyplot as plt
from ROSCO_toolbox.inputs.validation import load_rosco_yaml
from ROSCO_toolbox.linear.robust_scheduling import rsched_driver
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox import controller as ROSCO_controller

def run_example():
    # Shorthand directories
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir = os.path.join(this_dir, '../Tune_Cases')
    test_dir = os.path.join(this_dir, '../Test_Cases')

    # ROSCO options
    parameter_filename = os.path.join(tune_dir, 'IEA15MW_robust.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    turbine_params = inps['turbine_params']
    controller_params = inps['controller_params']
    linmodel_tuning = inps['linmodel_tuning']
    ROSCO_options = {
        'path_params': path_params,
        'turbine_params': turbine_params,
        'controller_params': controller_params
    }

    # Path options
    example_out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'examples_out')
    output_name = '12_robust_scheduling'
    path_options = {'output_dir': example_out_dir,
                    'output_name': output_name
                    }

    # Run ROSCO tuning to get k_float (see example 4)
    turbine = ROSCO_turbine.Turbine(turbine_params)
    controller = ROSCO_controller.Controller(controller_params)
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(this_dir, path_params['FAST_directory']),
        dev_branch=True,
        rot_source='txt', txt_filename=os.path.join(this_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
        )

    # Fix path params for robust setup
    path_params['FAST_directory'] = os.path.join(this_dir, path_params['FAST_directory'])

    controller.tune_controller(turbine)
    k_float = controller.Kp_float

    # Scheduling options
    opt_options = {'driver': 'optimization',  # 'design_of_experiments',
                   'windspeed': controller_params['U_pc'],
                   'stability_margin': linmodel_tuning['stability_margin'],
                   'omega': [linmodel_tuning['omega_pc']['min'],
                             linmodel_tuning['omega_pc']['max']],  # two inputs denotes a range for a design variable
                   'k_float': [k_float]}    # one input denotes a set value

    # Collect options
    options = {}
    options['linturb_options'] = linmodel_tuning
    options['ROSCO_options'] = ROSCO_options
    options['path_options'] = path_options
    options['opt_options'] = opt_options

    options['linturb_options']['linfile_path'] = os.path.join(this_dir, options['linturb_options']['linfile_path'])

    # Run robust scheduling
    sd = rsched_driver(options)
    sd.setup()
    sd.execute()

    # Re-define ROSCO tuning parameters
    controller.omega_pc = sd.omegas
    controller.zeta_pc = np.ones(
        len(opt_options['windspeed'])) * controller.zeta_pc

    # Tune ROSCO with to satisfy robust stability margin
    controller.tune_controller(turbine)

    # Plot gain schedule
    fig, ax = plt.subplots(5, 1, constrained_layout=True, sharex=True)
    ax = ax.flatten()
    ax[0].plot(controller.v[len(controller.v_below_rated)+1:], controller.omega_pc_U)
    ax[0].plot(opt_options['windspeed'], sd.omegas, linestyle='--', marker='x', label='Robust Tuning Values')
    ax[0].set_ylabel('omega_pc')
    ax[0].legend()
    ax[0].grid()

    ax[1].plot(controller.v[len(controller.v_below_rated)+1:], controller.zeta_pc_U)
    ax[1].set_ylabel('zeta_pc')
    ax[1].grid()

    ax[2].plot(opt_options['windspeed'], sd.sms)
    ax[2].set_ylabel('stability margins')
    ax[2].grid()

    ax[3].plot(controller.v[len(controller.v_below_rated)+1:], controller.pc_gain_schedule.Kp)
    ax[3].set_ylabel('Proportional Gain')
    ax[3].grid()

    ax[4].plot(controller.v[len(controller.v_below_rated)+1:], controller.pc_gain_schedule.Ki)
    ax[4].set_xlabel('Wind Speed')
    ax[4].set_ylabel('Integral Gain')
    ax[4].grid()


    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir, '12_RobustSched.png'))
if __name__ == '__main__':
    run_example()
