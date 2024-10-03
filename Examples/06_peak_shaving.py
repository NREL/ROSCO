"""
06_peak_shavings
----------------
Load saved turbine, tune controller, plot minimum pitch schedule
In this example:

* Load a yaml file
* Load a turbine from openfast
* Tune a controller
* Plot minimum pitch schedule
"""

# Python modules
import matplotlib.pyplot as plt 
import os
# ROSCO toolbox modules 
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.inputs.validation import load_rosco_yaml

def main():
    this_dir = os.path.dirname(__file__)
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    example_out_dir = os.path.join(this_dir,'examples_out')
    if not os.path.isdir(example_out_dir):
        os.makedirs(example_out_dir)

    # Load yaml file 
    parameter_filename = os.path.join(tune_dir,'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Ensure minimum generator speed at 50 rpm (for example's sake), turn on peak shaving and cp-maximizing min pitch
    controller_params['vs_minspd'] = 50/97
    controller_params['PS_Mode'] = 3

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory']),
        rot_source='txt',txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
        )
    # Tune controller 
    controller.tune_controller(turbine)

    # Plot minimum pitch schedule
    fig, ax = plt.subplots(1,1)
    ax.plot(controller.v, controller.pitch_op,label='Steady State Operation')
    ax.plot(controller.v, controller.ps_min_bld_pitch, label='Minimum Pitch Schedule')
    ax.legend()
    ax.set_xlabel('Wind speed (m/s)')
    ax.set_ylabel('Blade pitch (rad)')

    if False:
        plt.show()
    else:
        plt.savefig(os.path.join(example_out_dir,'06_MinPitch.png'))

if __name__ == "__main__":
    main()
