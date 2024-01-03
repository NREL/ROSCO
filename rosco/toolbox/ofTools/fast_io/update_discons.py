import os
# ROSCO toolbox modules 

from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.tune import yaml_to_objs

def update_discons(tune_to_test_map):
    # Update a set of discon files
    # Input is a dict: each key is the tuning yaml and each value is the discon or list of discons
    for tuning_yaml in tune_to_test_map:

        controller, turbine, path_params = yaml_to_objs(tuning_yaml)

        # Write parameter input file
        if not isinstance(tune_to_test_map[tuning_yaml],list):
            tune_to_test_map[tuning_yaml] = [tune_to_test_map[tuning_yaml]]

        # Handle relative directory to Cp file
        yaml_dir = os.path.dirname(tuning_yaml)
        cp_filename = os.path.relpath(
            os.path.join(yaml_dir,path_params['rotor_performance_filename']),
            os.path.join(yaml_dir,path_params['FAST_directory']))

        discon_in_files = [f for f in tune_to_test_map[tuning_yaml]]
        for discon in discon_in_files: 
            write_DISCON(
                turbine,controller,
                param_file=discon, 
                txt_filename=cp_filename
                )
