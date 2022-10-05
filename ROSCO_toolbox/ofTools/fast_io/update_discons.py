import os
# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.utilities import write_DISCON
from ROSCO_toolbox.inputs.validation import load_rosco_yaml

def update_discons(tune_to_test_map):
    # Update a set of discon files
    # Input is a dict: each key is the tuning yaml and each value is the discon or list of discons
    for tuning_yaml in tune_to_test_map:

        # Load yaml file 
        inps = load_rosco_yaml(tuning_yaml)
        path_params         = inps['path_params']
        turbine_params      = inps['turbine_params']
        controller_params   = inps['controller_params']

        # Instantiate turbine, controller, and file processing classes
        turbine         = ROSCO_turbine.Turbine(turbine_params)
        controller      = ROSCO_controller.Controller(controller_params)

        # Load turbine data from OpenFAST and rotor performance text file
        yaml_dir = os.path.dirname(tuning_yaml)  # files relative to tuning yaml
        turbine.load_from_fast(
            path_params['FAST_InputFile'],
            os.path.join(yaml_dir,path_params['FAST_directory']),
            dev_branch=True,
            rot_source='txt',
            txt_filename=os.path.join(yaml_dir,path_params['FAST_directory'],path_params['rotor_performance_filename'])
            )

        # Tune controller 
        controller.tune_controller(turbine)

        # Write parameter input file
        if not isinstance(tune_to_test_map[tuning_yaml],list):
            tune_to_test_map[tuning_yaml] = [tune_to_test_map[tuning_yaml]]

        discon_in_files = [f for f in tune_to_test_map[tuning_yaml]]
        for discon in discon_in_files: 
            write_DISCON(
                turbine,controller,
                param_file=discon, 
                txt_filename=path_params['rotor_performance_filename']
                )
