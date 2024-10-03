"""
02_ccblade
-------------- 
Run CCblade, save a rotor performance text file.
In this example:

* Read .yaml input file
* Load an openfast turbine model
* Run ccblade to get rotor performance properties
* Write a text file with rotor performance properties
"""

# Python modules
import os 
# ROSCO toolbox modules 
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox.utilities import write_rotor_performance
from rosco.toolbox.inputs.validation import load_rosco_yaml

def main():
    # Initialize parameter dictionaries
    turbine_params = {}
    control_params = {}

    this_dir = os.path.dirname(os.path.abspath(__file__))
    example_out_dir = os.path.join(this_dir,'examples_out')
    if not os.path.isdir(example_out_dir):
        os.makedirs(example_out_dir)

    # Load yaml file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    tune_dir =  os.path.join(this_dir,'Tune_Cases')
    parameter_filename = os.path.join(tune_dir,'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params         = inps['path_params']
    turbine_params      = inps['turbine_params']
    controller_params   = inps['controller_params']

    # Load turbine data from openfast model
    turbine = ROSCO_turbine.Turbine(turbine_params)
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir,path_params['FAST_directory']),
        rot_source='cc-blade',
        txt_filename=None)

    # Write rotor performance text file
    txt_filename = os.path.join(example_out_dir,'02_Cp_Ct_Cq.Ex03.txt')
    write_rotor_performance(turbine,txt_filename=txt_filename)

if __name__ == "__main__":
    main()

