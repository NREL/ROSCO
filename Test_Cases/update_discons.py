'''
Update the DISCON.IN examples in the ROSCO repository using the Tune_Case/ .yaml files

'''
import os

# ROSCO toolbox modules 
from ROSCO_toolbox import controller as ROSCO_controller
from ROSCO_toolbox import turbine as ROSCO_turbine
from ROSCO_toolbox.utilities import write_DISCON
from ROSCO_toolbox.inputs.validation import load_rosco_yaml

test_dir = os.path.dirname(os.path.abspath(__file__))
tune_dir = os.path.realpath(os.path.join(test_dir,'../Tune_Cases'))


# Paths are relative to Tune_Case/ and Test_Case/
tune_to_test_map = {
    'NREL5MW.yaml': 'NREL-5MW/DISCON.IN',
    'IEA15MW.yaml': 'IEA-15-240-RWT-UMaineSemi/DISCON-UMaineSemi.IN',
    'BAR.yaml': 'BAR_10/BAR_10_DISCON.IN'
}

for tuning_yaml in tune_to_test_map:

    # Load yaml file 
    inps = load_rosco_yaml(os.path.join(tune_dir,tuning_yaml))
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
        dev_branch=True,
        rot_source='txt',
        txt_filename=os.path.join(tune_dir,path_params['rotor_performance_filename'])
        )

    # Tune controller 
    controller.tune_controller(turbine)

    # Write parameter input file
    discon_in_file = os.path.join(test_dir,tune_to_test_map[tuning_yaml])
    write_DISCON(
        turbine,controller,
        param_file=discon_in_file, 
        txt_filename=path_params['rotor_performance_filename'].split('/')[-1]
        )
