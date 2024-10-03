"""
05_openfast_sim
---------------
Load a turbine, tune a controller, run OpenFAST simulation 
In this example:

* Load a turbine from OpenFAST
* Tune a controller
* Run an OpenFAST simulation

Note

* you will need to have a compiled controller in ROSCO/build/ 
"""

# Python Modules
#import yaml
import os
#import numpy as np
#import matplotlib.pyplot as plt
# ROSCO toolbox modules 
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.utilities import write_DISCON, run_openfast
from rosco.toolbox.inputs.validation import load_rosco_yaml

def main():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    example_out_dir = os.path.join(this_dir,'examples_out')

    # Load yaml file 
    parameter_filename = os.path.join(tune_dir, 'IEA15MW_MultiOmega.yaml') 
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Instantiate turbine, controller, and file processing classes
    turbine         = ROSCO_turbine.Turbine(turbine_params)
    controller      = ROSCO_controller.Controller(controller_params)

    # Load turbine data from OpenFAST and rotor performance text file
    turbine.load_from_fast(
    path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory']),
        rot_source='txt',
        txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
        )

    # Tune controller 
    controller.tune_controller(turbine)

    # Write parameter input file
    param_file = os.path.join(this_dir,'DISCON.IN')   # This must be named DISCON.IN to be seen by the compiled controller binary. 
    write_DISCON(turbine,controller,param_file=param_file, txt_filename=path_params['rotor_performance_filename'])

    # Run OpenFAST
    # --- May need to change fastcall if you use a non-standard, conda-installed command to call openfast
    # If you run the `fastcall` from the command line where you run this script, it should run OpenFAST
    fastcall = 'openfast'
    run_openfast(
    os.path.join(tune_dir,path_params['FAST_directory']),
    fastcall=fastcall, 
    fastfile=path_params['FAST_InputFile'], 
    chdir=True
    )

if __name__ == "__main__":
    main()